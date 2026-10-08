"""Round-trip support for the ``node_list`` / ``rod_list`` / ``coupler_list``
scaffolding JSON exchanged with the upstream layout generator.

Two Rhino entry points share this module:

* :mod:`rs_import_scaffold_json` -- reads the JSON and bakes one registered
  bar per rod,
* :mod:`rs_export_scaffold_json` -- reads the bars back out and writes the
  same schema, after the designer has run RSCreateBar / RSBarSnap / joint
  placement on them.

The schema itself
-----------------

::

    {"node_list":    [{"point": {"X":…, "Y":…, "Z":…}, "node_id": int}, …],
     "rod_list":     [{"end_node_ids": [int, int], "rod_id": int,
                       "layer_id": int, "grounded": bool}, …],
     "coupler_list": [{"rod_ids": [int, int]}, …]}

``rod_id`` is the bar identity (the Rhino ``bar_id`` is written as
``B<rod_id>`` on import).  Rod-to-rod connectivity lives in
``coupler_list``; the shared ``node_id`` values in ``rod_list`` are only
the *nominal* layout, and they stop being shared the moment RSBarSnap
offsets a bar to its joint contact distance.  :func:`build_node_list`
handles exactly that: a node whose incident bar ends have drifted apart
is split into one node per distinct position, the original ``node_id``
staying with the lowest-numbered rod so ids churn as little as possible.

Everything here is plain Python (no ``Rhino``/``rhinoscriptsyntax``
imports) so it can be exercised by ``tests/test_scaffold_json.py``
outside Rhino.
"""

from __future__ import annotations

import json
import math


# ---------------------------------------------------------------------------
# Schema keys
# ---------------------------------------------------------------------------

NODE_LIST_KEY = "node_list"
ROD_LIST_KEY = "rod_list"
COUPLER_LIST_KEY = "coupler_list"

#: Keys this module understands inside each list entry.  Anything else found
#: in the source file is carried through untouched so a future producer can
#: add fields without them being dropped by a Rhino round trip.
_KNOWN_NODE_KEYS = ("point", "node_id")
_KNOWN_ROD_KEYS = ("end_node_ids", "rod_id", "layer_id", "grounded")
_KNOWN_COUPLER_KEYS = ("rod_ids",)
_KNOWN_TOP_KEYS = (NODE_LIST_KEY, ROD_LIST_KEY, COUPLER_LIST_KEY)


# ---------------------------------------------------------------------------
# Rhino UserText keys (written on each bar centerline curve)
# ---------------------------------------------------------------------------

#: Authoritative rod identity.  Survives ``ensure_bar_id`` re-issuing a
#: ``bar_id`` after a copy/paste and survives RSReorderBarID renumbering the
#: bars, so the export always reads *this* key rather than parsing ``B<n>``.
KEY_ROD_ID = "rod_id"
KEY_ROD_LAYER_ID = "rod_layer_id"
KEY_ROD_GROUNDED = "rod_grounded"
#: ``"<start_node_id>,<end_node_id>"`` as authored in the source file.
KEY_ROD_NODE_IDS = "rod_end_node_ids"
#: Comma-separated rod ids this rod is coupled to (denormalised from
#: ``coupler_list`` purely as a design-time convenience in Rhino).
KEY_ROD_COUPLED = "rod_coupled_rod_ids"
#: As-imported endpoint positions.  The export uses them to tell whether the
#: curve has since been flipped, so ``end_node_ids`` keeps pointing at the
#: right physical end of the bar.
KEY_ROD_START_XYZ = "rod_source_start_xyz"
KEY_ROD_END_XYZ = "rod_source_end_xyz"
#: JSON blob of any rod fields this module does not model.
KEY_ROD_EXTRA = "rod_extra_json"

#: Document-level UserText: everything that is not per-bar.
DOC_KEY_SOURCE = "scaffold_json_source"
DOC_KEY_COUPLERS = "scaffold_json_coupler_list"
DOC_KEY_NODE_EXTRA = "scaffold_json_node_extra"
DOC_KEY_TOP_EXTRA = "scaffold_json_top_extra"


#: ``layer_id`` of the storey that stands on the ground.  A rod there is
#: the one the ground carries, which is exactly what ``grounded`` means, so
#: RSSetRodLayer derives one field from the other instead of asking twice.
GROUND_LAYER_ID = 0


class ScaffoldParseError(ValueError):
    """Raised when the source JSON cannot be interpreted at all."""


def grounded_for_layer(layer_id):
    """Return the ``grounded`` flag implied by *layer_id*.

    Layer 0 is the storey standing on the ground; every layer above it is
    carried by the layer below.  Kept here rather than in the Rhino command
    so the convention is stated once, next to the schema it belongs to.
    """
    return int(layer_id) == GROUND_LAYER_ID


# ---------------------------------------------------------------------------
# Small serialisation helpers
# ---------------------------------------------------------------------------


def point_to_dict(xyz):
    """``(x, y, z)`` -> ``{"X": …, "Y": …, "Z": …}`` (schema point form)."""
    return {"X": float(xyz[0]), "Y": float(xyz[1]), "Z": float(xyz[2])}


def dict_to_point(value):
    """``{"X": …, "Y": …, "Z": …}`` -> ``(x, y, z)`` float tuple."""
    try:
        return (float(value["X"]), float(value["Y"]), float(value["Z"]))
    except (TypeError, KeyError, ValueError) as exc:
        raise ScaffoldParseError(f"Malformed point {value!r}: {exc}") from exc


def format_xyz(xyz):
    """Serialise a point for UserText storage (matches the tube-cache format)."""
    return f"{float(xyz[0]):.6f},{float(xyz[1]):.6f},{float(xyz[2]):.6f}"


def parse_xyz(text):
    """Inverse of :func:`format_xyz`.  Returns ``None`` when unparseable."""
    if not text:
        return None
    parts = str(text).split(",")
    if len(parts) != 3:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def parse_bool(text, default=False):
    """Parse a UserText boolean written by :func:`str` (``"True"`` / ``"False"``)."""
    if text is None or text == "":
        return default
    return str(text).strip().lower() in ("true", "1", "yes")


def parse_int(text, default=None):
    """Parse a UserText integer, returning *default* when absent/invalid."""
    if text is None or text == "":
        return default
    try:
        return int(str(text).strip())
    except ValueError:
        return default


def dumps_compact(value):
    """JSON dump with no gratuitous whitespace (UserText values are strings)."""
    return json.dumps(value, separators=(",", ":"), sort_keys=True)


def loads_or_default(text, default):
    """``json.loads`` that falls back to *default* on empty/invalid input."""
    if not text:
        return default
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return default


def format_id_list(ids):
    """``[3, 1, 2]`` -> ``"1,2,3"`` (sorted, comma separated)."""
    return ",".join(str(int(i)) for i in sorted(set(int(i) for i in ids)))


def parse_id_list(text):
    """Inverse of :func:`format_id_list`; unparseable tokens are skipped."""
    out = []
    for token in str(text or "").split(","):
        token = token.strip()
        if not token:
            continue
        try:
            out.append(int(token))
        except ValueError:
            continue
    return out


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def _extra_keys(entry, known):
    return {k: v for k, v in entry.items() if k not in known}


def parse_scaffold(data):
    """Validate and normalise a parsed scaffolding JSON document.

    Parameters
    ----------
    data : dict
        The object returned by ``json.load`` on a scaffolding file.

    Returns
    -------
    dict
        ``{"nodes": {node_id: {"point": (x,y,z), "extra": {…}}},
           "rods":  [{"rod_id", "end_node_ids", "layer_id", "grounded",
                      "extra"} …]  (sorted by rod_id),
           "couplers": [{"rod_ids": (a, b), "extra": {…}} …],
           "top_extra": {…},
           "warnings": [str, …]}``

    Raises
    ------
    ScaffoldParseError
        If the document is not a dict, or ``node_list`` / ``rod_list`` are
        missing or not lists.  Recoverable problems (a rod pointing at an
        unknown node, a duplicate rod id, a self-coupler) are reported in
        ``warnings`` and the offending entry is dropped.
    """
    if not isinstance(data, dict):
        raise ScaffoldParseError("Top level of the JSON file must be an object.")
    for key in (NODE_LIST_KEY, ROD_LIST_KEY):
        if not isinstance(data.get(key), list):
            raise ScaffoldParseError(f"Missing or non-list '{key}'.")

    warnings = []

    nodes = {}
    for entry in data[NODE_LIST_KEY]:
        if not isinstance(entry, dict) or "node_id" not in entry:
            warnings.append(f"Skipped malformed node entry {entry!r}.")
            continue
        node_id = int(entry["node_id"])
        if node_id in nodes:
            warnings.append(f"Duplicate node_id {node_id}; kept the first one.")
            continue
        nodes[node_id] = {
            "point": dict_to_point(entry.get("point")),
            "extra": _extra_keys(entry, _KNOWN_NODE_KEYS),
        }

    rods = []
    seen_rod_ids = set()
    for entry in data[ROD_LIST_KEY]:
        if not isinstance(entry, dict) or "rod_id" not in entry:
            warnings.append(f"Skipped malformed rod entry {entry!r}.")
            continue
        rod_id = int(entry["rod_id"])
        if rod_id in seen_rod_ids:
            warnings.append(f"Duplicate rod_id {rod_id}; kept the first one.")
            continue
        end_ids = entry.get("end_node_ids") or []
        if len(end_ids) != 2:
            warnings.append(f"Rod {rod_id} has {len(end_ids)} end node(s); skipped.")
            continue
        start_id, end_id = int(end_ids[0]), int(end_ids[1])
        missing = [n for n in (start_id, end_id) if n not in nodes]
        if missing:
            warnings.append(f"Rod {rod_id} references unknown node(s) {missing}; skipped.")
            continue
        seen_rod_ids.add(rod_id)
        rods.append(
            {
                "rod_id": rod_id,
                "end_node_ids": (start_id, end_id),
                "layer_id": int(entry.get("layer_id", 0)),
                "grounded": bool(entry.get("grounded", False)),
                "extra": _extra_keys(entry, _KNOWN_ROD_KEYS),
            }
        )
    rods.sort(key=lambda r: r["rod_id"])

    couplers = []
    seen_pairs = set()
    for entry in data.get(COUPLER_LIST_KEY) or []:
        if not isinstance(entry, dict):
            warnings.append(f"Skipped malformed coupler entry {entry!r}.")
            continue
        pair_ids = entry.get("rod_ids") or []
        if len(pair_ids) != 2:
            warnings.append(f"Coupler {entry!r} does not name exactly 2 rods; skipped.")
            continue
        a, b = int(pair_ids[0]), int(pair_ids[1])
        if a == b:
            warnings.append(f"Coupler {entry!r} couples rod {a} to itself; skipped.")
            continue
        unknown = [r for r in (a, b) if r not in seen_rod_ids]
        if unknown:
            warnings.append(f"Coupler {entry!r} references unknown rod(s) {unknown}; skipped.")
            continue
        key = (min(a, b), max(a, b))
        if key in seen_pairs:
            warnings.append(f"Duplicate coupler {list(key)}; kept the first one.")
            continue
        seen_pairs.add(key)
        couplers.append(
            {"rod_ids": key, "extra": _extra_keys(entry, _KNOWN_COUPLER_KEYS)}
        )
    couplers.sort(key=lambda c: c["rod_ids"])

    return {
        "nodes": nodes,
        "rods": rods,
        "couplers": couplers,
        "top_extra": _extra_keys(data, _KNOWN_TOP_KEYS),
        "warnings": warnings,
    }


def load_scaffold(path):
    """Read *path* and return :func:`parse_scaffold` of its contents."""
    with open(path, "r", encoding="utf-8") as stream:
        return parse_scaffold(json.load(stream))


# ---------------------------------------------------------------------------
# Derived views used by the import command
# ---------------------------------------------------------------------------


def assembly_order(rods):
    """Return *rods* in the order they should be assembled in Rhino.

    Bottom-up: ``layer_id`` first (the source file's storey index), then
    ``rod_id`` so the order is stable and reproducible.  The Rhino bar
    sequence numbers (``bar_seq``, 1..N) follow this order, which is what
    RSJointPlace uses to decide which bar of a pair gets the female half.
    """
    return sorted(rods, key=lambda r: (r["layer_id"], r["rod_id"]))


def coupled_rods_map(couplers):
    """``coupler_list`` -> ``{rod_id: [coupled rod ids, sorted]}``."""
    out = {}
    for coupler in couplers:
        a, b = coupler["rod_ids"]
        out.setdefault(a, set()).add(b)
        out.setdefault(b, set()).add(a)
    return {rod_id: sorted(peers) for rod_id, peers in out.items()}


def merge_couplers(stored, discovered):
    """Union of the imported coupler pairs and pairs found as placed joints.

    Parameters
    ----------
    stored : iterable of (int, int)
        Pairs carried over from the source file.
    discovered : iterable of (int, int)
        Pairs read back from joint block instances placed in Rhino.

    Returns
    -------
    tuple[list[tuple[int, int]], list[tuple[int, int]]]
        ``(merged_pairs_sorted, pairs_that_were_only_discovered)`` -- the
        second element is what the designer added in Rhino beyond the
        source file, which the export prints so the change is visible.
    """
    def _norm(pairs):
        return {(min(int(a), int(b)), max(int(a), int(b))) for a, b in pairs}

    stored_set = _norm(stored)
    discovered_set = _norm(discovered)
    added = sorted(discovered_set - stored_set)
    return sorted(stored_set | discovered_set), added


# ---------------------------------------------------------------------------
# Node reconstruction (the export side)
# ---------------------------------------------------------------------------


def _cluster_endpoints(records, tol):
    """Group endpoint records that occupy the same position.

    Uses a uniform spatial hash with a 3x3x3 neighbour sweep so the cost
    stays linear in the number of endpoints (a naive all-pairs compare gets
    slow on multi-thousand-rod models) while still being exact at the cell
    boundaries.
    """
    cell = max(float(tol), 1e-9)
    grid = {}
    clusters = []
    for record in records:
        point = record["point"]
        key = (
            int(math.floor(point[0] / cell)),
            int(math.floor(point[1] / cell)),
            int(math.floor(point[2] / cell)),
        )
        found = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for index in grid.get((key[0] + dx, key[1] + dy, key[2] + dz), ()):
                        seed = clusters[index]["point"]
                        if (
                            abs(seed[0] - point[0]) <= tol
                            and abs(seed[1] - point[1]) <= tol
                            and abs(seed[2] - point[2]) <= tol
                        ):
                            found = index
                            break
                    if found is not None:
                        break
                if found is not None:
                    break
            if found is not None:
                break
        if found is None:
            clusters.append({"point": point, "records": [record]})
            grid.setdefault(key, []).append(len(clusters) - 1)
        else:
            clusters[found]["records"].append(record)
    return clusters


def build_node_list(records, *, tol=1e-6, node_extra=None, first_new_node_id=None):
    """Rebuild ``node_list`` from the bars' current endpoint positions.

    Parameters
    ----------
    records : list of dict
        One entry per bar end: ``{"rod_id": int, "index": 0|1,
        "point": (x, y, z), "source_node_id": int | None}``.  ``index`` 0 is
        the curve start, 1 the curve end; ``source_node_id`` is the node id
        the end carried in the source file (``None`` for bars drawn in
        Rhino after the import).
    tol : float
        Positions within this distance (per axis, mm) are the same node.
    node_extra : dict, optional
        ``{source_node_id: {extra schema fields}}`` carried through onto
        every node derived from that source node.
    first_new_node_id : int, optional
        Where minted ids start.  Defaults to one past the highest
        ``source_node_id`` seen, so original ids are never reused for a
        different position.

    Returns
    -------
    dict
        ``{"nodes": [{"point": (x,y,z), "node_id": int, "extra": {…}} …],
           "rod_end_node_ids": {(rod_id, index): node_id},
           "split_source_ids": [source ids that now cover >1 position],
           "merged_clusters": [positions that fused >1 source id]}``

    Notes
    -----
    A source node that has been pulled apart by RSBarSnap keeps its id on
    the cluster holding the lowest ``rod_id``; the other clusters get fresh
    ids.  That keeps the diff against the input file readable instead of
    renumbering every node in the model.
    """
    node_extra = node_extra or {}
    clusters = _cluster_endpoints(list(records), tol)

    max_source = -1
    for record in records:
        source_id = record.get("source_node_id")
        if source_id is not None:
            max_source = max(max_source, int(source_id))
    next_id = int(first_new_node_id) if first_new_node_id is not None else max_source + 1

    # Rank the clusters that claim the same source id: the one holding the
    # lowest (rod_id, endpoint index) keeps the id, the rest are re-minted.
    claims = {}
    merged_clusters = []
    for index, cluster in enumerate(clusters):
        source_ids = sorted(
            {
                int(r["source_node_id"])
                for r in cluster["records"]
                if r.get("source_node_id") is not None
            }
        )
        rank = min(
            (int(r["rod_id"]), int(r["index"])) for r in cluster["records"]
        )
        cluster["source_ids"] = source_ids
        cluster["rank"] = rank
        if len(source_ids) > 1:
            # Two originally-distinct nodes now sit at the same point.
            merged_clusters.append((tuple(cluster["point"]), source_ids))
        if source_ids:
            claims.setdefault(source_ids[0], []).append(index)

    split_source_ids = []
    keeper = {}
    for source_id, cluster_indices in claims.items():
        cluster_indices.sort(key=lambda i: clusters[i]["rank"])
        keeper[source_id] = cluster_indices[0]
        if len(cluster_indices) > 1:
            split_source_ids.append(source_id)

    nodes = []
    rod_end_node_ids = {}
    for index, cluster in enumerate(clusters):
        source_ids = cluster["source_ids"]
        primary = source_ids[0] if source_ids else None
        if primary is not None and keeper.get(primary) == index:
            node_id = primary
        else:
            node_id = next_id
            next_id += 1
        # Average the cluster so a node is not biased toward whichever bar
        # happened to be visited first (they agree to within `tol` anyway).
        count = float(len(cluster["records"]))
        centroid = tuple(
            sum(r["point"][axis] for r in cluster["records"]) / count for axis in range(3)
        )
        nodes.append(
            {
                "node_id": node_id,
                "point": centroid,
                "extra": dict(node_extra.get(primary, {})) if primary is not None else {},
            }
        )
        for record in cluster["records"]:
            rod_end_node_ids[(int(record["rod_id"]), int(record["index"]))] = node_id

    nodes.sort(key=lambda n: n["node_id"])
    return {
        "nodes": nodes,
        "rod_end_node_ids": rod_end_node_ids,
        "split_source_ids": sorted(split_source_ids),
        "merged_clusters": merged_clusters,
    }


# ---------------------------------------------------------------------------
# Serialisation (the export side)
# ---------------------------------------------------------------------------


def build_document(nodes, rods, couplers, top_extra=None):
    """Assemble the final JSON document in the source file's schema.

    Parameters
    ----------
    nodes : list of dict
        As returned in ``build_node_list()["nodes"]``.
    rods : list of dict
        ``{"rod_id", "end_node_ids": (a, b), "layer_id", "grounded",
        "extra": {…}}``.
    couplers : iterable of (int, int)
        Rod-id pairs.
    top_extra : dict, optional
        Top-level fields carried over from the source file.

    Returns
    -------
    dict
        Ready for ``json.dump(..., indent=2)``.
    """
    node_entries = []
    for node in sorted(nodes, key=lambda n: n["node_id"]):
        entry = {"point": point_to_dict(node["point"]), "node_id": int(node["node_id"])}
        entry.update(node.get("extra") or {})
        node_entries.append(entry)

    rod_entries = []
    for rod in sorted(rods, key=lambda r: r["rod_id"]):
        entry = {
            "end_node_ids": [int(rod["end_node_ids"][0]), int(rod["end_node_ids"][1])],
            "rod_id": int(rod["rod_id"]),
            "layer_id": int(rod["layer_id"]),
            "grounded": bool(rod["grounded"]),
        }
        entry.update(rod.get("extra") or {})
        rod_entries.append(entry)

    coupler_entries = [
        {"rod_ids": [int(a), int(b)]}
        for a, b in sorted({(min(a, b), max(a, b)) for a, b in couplers})
    ]

    document = {
        NODE_LIST_KEY: node_entries,
        ROD_LIST_KEY: rod_entries,
        COUPLER_LIST_KEY: coupler_entries,
    }
    document.update(top_extra or {})
    return document


def write_document(path, document):
    """Write *document* to *path* with the source file's 2-space indentation."""
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
