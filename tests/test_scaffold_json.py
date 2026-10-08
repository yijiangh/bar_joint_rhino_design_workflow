"""Tests for the node/rod/coupler scaffolding JSON round trip.

Covers :mod:`core.scaffold_json` only -- parsing, the node rebuild that has
to happen once RSBarSnap has offset bars to their joint contact distance,
and coupler merging.  The Rhino entry points that use it
(``rs_import_scaffold_json`` / ``rs_export_scaffold_json``) depend on
``rhinoscriptsyntax`` and are verified manually, per ``tests/conftest.py``.
"""

from __future__ import annotations

import json
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, ".."))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from core import scaffold_json as sj

SAMPLE_PATH = os.path.join(REPO_ROOT, "large_scaffold.json")


# ---------------------------------------------------------------------------
# Fixtures / helpers
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def sample_raw():
    if not os.path.isfile(SAMPLE_PATH):
        pytest.skip(f"sample file {SAMPLE_PATH} is not present")
    with open(SAMPLE_PATH, "r", encoding="utf-8") as stream:
        return json.load(stream)


@pytest.fixture(scope="module")
def sample(sample_raw):
    return sj.parse_scaffold(sample_raw)


def _endpoint_records(parsed, moves=None):
    """Build export-side endpoint records, optionally displacing bar ends.

    *moves* maps ``(rod_id, endpoint_index)`` to an ``(dx, dy, dz)`` offset,
    standing in for what RSBarSnap does to a bar in Rhino.
    """
    moves = moves or {}
    records = []
    for rod in parsed["rods"]:
        for index, node_id in enumerate(rod["end_node_ids"]):
            point = parsed["nodes"][node_id]["point"]
            offset = moves.get((rod["rod_id"], index))
            if offset is not None:
                point = tuple(p + o for p, o in zip(point, offset))
            records.append(
                {
                    "rod_id": rod["rod_id"],
                    "index": index,
                    "point": point,
                    "source_node_id": node_id,
                }
            )
    return records


def _rods_for_export(parsed, built):
    return [
        {
            "rod_id": rod["rod_id"],
            "end_node_ids": (
                built["rod_end_node_ids"][(rod["rod_id"], 0)],
                built["rod_end_node_ids"][(rod["rod_id"], 1)],
            ),
            "layer_id": rod["layer_id"],
            "grounded": rod["grounded"],
            "extra": rod["extra"],
        }
        for rod in parsed["rods"]
    ]


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def test_sample_file_parses_without_warnings(sample, sample_raw):
    assert sample["warnings"] == []
    assert len(sample["nodes"]) == len(sample_raw["node_list"])
    assert len(sample["rods"]) == len(sample_raw["rod_list"])
    assert len(sample["couplers"]) == len(sample_raw["coupler_list"])
    assert sample["top_extra"] == {}


def test_every_node_is_referenced_by_a_rod(sample):
    referenced = set()
    for rod in sample["rods"]:
        referenced.update(rod["end_node_ids"])
    assert referenced == set(sample["nodes"])


def test_parse_drops_bad_entries_with_warnings():
    parsed = sj.parse_scaffold(
        {
            "node_list": [
                {"node_id": 0, "point": {"X": 0.0, "Y": 0.0, "Z": 0.0}},
                {"node_id": 1, "point": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            ],
            "rod_list": [
                {"end_node_ids": [0, 1], "rod_id": 0, "layer_id": 0, "grounded": True},
                # references a node that does not exist
                {"end_node_ids": [0, 7], "rod_id": 1, "layer_id": 0, "grounded": False},
                # duplicate id
                {"end_node_ids": [1, 0], "rod_id": 0, "layer_id": 0, "grounded": False},
            ],
            "coupler_list": [
                {"rod_ids": [0, 0]},  # self-coupling
                {"rod_ids": [0, 9]},  # unknown rod
            ],
        }
    )
    assert [rod["rod_id"] for rod in parsed["rods"]] == [0]
    assert parsed["couplers"] == []
    assert len(parsed["warnings"]) == 4


def test_parse_rejects_a_document_without_rods():
    with pytest.raises(sj.ScaffoldParseError):
        sj.parse_scaffold({"node_list": []})


def test_unmodelled_fields_are_preserved(sample):
    parsed = sj.parse_scaffold(
        {
            "schema_version": 3,
            "node_list": [
                {"node_id": 0, "point": {"X": 0.0, "Y": 0.0, "Z": 0.0}, "fixed": True},
                {"node_id": 1, "point": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            ],
            "rod_list": [
                {
                    "end_node_ids": [0, 1],
                    "rod_id": 0,
                    "layer_id": 0,
                    "grounded": True,
                    "material": "steel",
                }
            ],
            "coupler_list": [],
        }
    )
    assert parsed["top_extra"] == {"schema_version": 3}
    assert parsed["nodes"][0]["extra"] == {"fixed": True}
    assert parsed["rods"][0]["extra"] == {"material": "steel"}

    built = sj.build_node_list(
        _endpoint_records(parsed),
        node_extra={n: d["extra"] for n, d in parsed["nodes"].items()},
    )
    document = sj.build_document(
        built["nodes"],
        _rods_for_export(parsed, built),
        [c["rod_ids"] for c in parsed["couplers"]],
        top_extra=parsed["top_extra"],
    )
    assert document["schema_version"] == 3
    assert document["node_list"][0]["fixed"] is True
    assert document["rod_list"][0]["material"] == "steel"


# ---------------------------------------------------------------------------
# Node rebuild
# ---------------------------------------------------------------------------


def test_untouched_model_round_trips_byte_for_byte(sample, sample_raw):
    """Import then export with nothing moved must reproduce the input."""
    built = sj.build_node_list(_endpoint_records(sample))
    assert built["split_source_ids"] == []
    assert built["merged_clusters"] == []

    document = sj.build_document(
        built["nodes"],
        _rods_for_export(sample, built),
        [c["rod_ids"] for c in sample["couplers"]],
        top_extra=sample["top_extra"],
    )
    assert document == sample_raw


def test_snapping_a_bar_splits_the_shared_node(sample):
    """A node whose incident bar ends drift apart becomes several nodes."""
    # Rods 0 and 12 are coupled and share node 0 (rod 0 end 0, rod 12 end 0).
    shared = 0
    owners = [
        (rod["rod_id"], index)
        for rod in sample["rods"]
        for index, node_id in enumerate(rod["end_node_ids"])
        if node_id == shared
    ]
    assert len(owners) > 1
    moved_rod, moved_index = max(owners)  # not the lowest rod: it loses the id

    n_source_nodes = len(sample["nodes"])
    built = sj.build_node_list(
        _endpoint_records(sample, moves={(moved_rod, moved_index): (0.0, 0.0, 36.0)})
    )

    assert built["split_source_ids"] == [shared]
    # The lowest-numbered rod keeps the original id ...
    kept_rod, kept_index = min(owners)
    assert built["rod_end_node_ids"][(kept_rod, kept_index)] == shared
    # ... and the bar that moved gets a fresh id past the original range.
    new_id = built["rod_end_node_ids"][(moved_rod, moved_index)]
    assert new_id == n_source_nodes
    assert len(built["nodes"]) == n_source_nodes + 1

    moved_node = next(n for n in built["nodes"] if n["node_id"] == new_id)
    original = sample["nodes"][shared]["point"]
    assert moved_node["point"] == pytest.approx((original[0], original[1], original[2] + 36.0))


def test_endpoints_that_meet_after_a_move_share_one_node(sample):
    """Two originally distinct nodes landing on the same point are merged."""
    # Node 24 (0, 600, 0) pulled onto node 0 (0, 0, 0).
    owners = [
        (rod["rod_id"], index)
        for rod in sample["rods"]
        for index, node_id in enumerate(rod["end_node_ids"])
        if node_id == 24
    ]
    moves = {owner: (0.0, -600.0, 0.0) for owner in owners}
    built = sj.build_node_list(_endpoint_records(sample, moves=moves))

    assert len(built["merged_clusters"]) == 1
    _point, source_ids = built["merged_clusters"][0]
    assert source_ids == [0, 24]
    # The lower source id wins, and node 24 disappears from the output.
    assert built["rod_end_node_ids"][owners[0]] == 0
    assert 24 not in {node["node_id"] for node in built["nodes"]}


def test_bars_without_source_nodes_get_new_ids(sample):
    """A bar drawn in Rhino after the import still exports cleanly."""
    records = _endpoint_records(sample)
    new_rod_id = max(rod["rod_id"] for rod in sample["rods"]) + 1
    records.extend(
        {
            "rod_id": new_rod_id,
            "index": index,
            "point": point,
            "source_node_id": None,
        }
        for index, point in enumerate([(0.0, 0.0, 5000.0), (0.0, 1000.0, 5000.0)])
    )
    built = sj.build_node_list(records)

    n_source_nodes = len(sample["nodes"])
    assert built["rod_end_node_ids"][(new_rod_id, 0)] == n_source_nodes
    assert built["rod_end_node_ids"][(new_rod_id, 1)] == n_source_nodes + 1
    assert built["split_source_ids"] == []


def test_node_positions_follow_the_moved_geometry(sample):
    """Exported node points are the bars' current endpoints, not the input's."""
    moves = {(rod["rod_id"], index): (100.0, 0.0, 0.0)
             for rod in sample["rods"] for index in (0, 1)}
    built = sj.build_node_list(_endpoint_records(sample, moves=moves))
    # Everything shifted rigidly, so nothing splits and every node moved by +100 X.
    assert built["split_source_ids"] == []
    for node in built["nodes"]:
        original = sample["nodes"][node["node_id"]]["point"]
        assert node["point"] == pytest.approx((original[0] + 100.0, original[1], original[2]))


# ---------------------------------------------------------------------------
# Ordering and couplers
# ---------------------------------------------------------------------------


def test_assembly_order_is_layer_then_rod(sample):
    ordered = sj.assembly_order(sample["rods"])
    keys = [(rod["layer_id"], rod["rod_id"]) for rod in ordered]
    assert keys == sorted(keys)
    assert len(ordered) == len(sample["rods"])


def test_coupled_rods_map_is_symmetric(sample):
    peers = sj.coupled_rods_map(sample["couplers"])
    for rod_id, neighbours in peers.items():
        for neighbour in neighbours:
            assert rod_id in peers[neighbour]


def test_merge_couplers_reports_pairs_added_in_rhino():
    merged, added = sj.merge_couplers([(0, 12), (12, 0)], [(12, 0), (3, 1)])
    assert merged == [(0, 12), (1, 3)]
    assert added == [(1, 3)]


def test_build_document_normalises_coupler_pairs():
    document = sj.build_document(
        [{"node_id": 0, "point": (0.0, 0.0, 0.0), "extra": {}},
         {"node_id": 1, "point": (1.0, 0.0, 0.0), "extra": {}}],
        [{"rod_id": 0, "end_node_ids": (0, 1), "layer_id": 0,
          "grounded": True, "extra": {}}],
        [(5, 2), (2, 5)],
    )
    assert document["coupler_list"] == [{"rod_ids": [2, 5]}]
    assert document["rod_list"][0]["grounded"] is True
    assert document["node_list"][1]["point"] == {"X": 1.0, "Y": 0.0, "Z": 0.0}


# ---------------------------------------------------------------------------
# UserText codecs
# ---------------------------------------------------------------------------


def test_xyz_user_text_round_trip():
    point = (1234.5678901, -0.000002, 3000.0)
    restored = sj.parse_xyz(sj.format_xyz(point))
    assert restored == pytest.approx(point, abs=1e-6)
    assert sj.parse_xyz("") is None
    assert sj.parse_xyz("1,2") is None


def test_id_list_user_text_round_trip():
    assert sj.format_id_list([3, 1, 2, 1]) == "1,2,3"
    assert sj.parse_id_list("1,2,3") == [1, 2, 3]
    assert sj.parse_id_list("") == []
    assert sj.parse_id_list("1, ,x,4") == [1, 4]


def test_scalar_user_text_codecs():
    assert sj.parse_bool("True") is True
    assert sj.parse_bool("false") is False
    assert sj.parse_bool("", default=True) is True
    assert sj.parse_int("7") == 7
    assert sj.parse_int(None) is None
    assert sj.parse_int("abc", 0) == 0
    assert sj.loads_or_default("", []) == []
    assert sj.loads_or_default("not json", {}) == {}
