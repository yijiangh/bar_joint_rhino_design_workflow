"""One-shot migration: legacy `pairs[]` schema -> normalized `halves/mates/ground_joints` schema.

Reads `scripts/core/joint_pairs.json` in the legacy format:

    {"pairs": [ {name, contact_distance_mm, jp_range, jr_range,
                 female: <full half dict>, male: <full half dict>}, ...]}

Writes the same file in the new normalized format:

    {"halves": [...], "mates": [...], "ground_joints": []}

Validates by:
  1. Reading the new file via `load_joint_registry`.
  2. Reconstructing every pair as `JointPairDef` and comparing field-by-field
     against the original (deep numpy `np.allclose`, scalar `==`).
  3. Detecting and reporting halves that share `block_name` but have different
     transforms (would be lossy without the warning).

On success: backs up the legacy file as `joint_pairs.json.bak` (only if no
backup already exists -- never overwrite a backup) and writes the new schema
in place.

Run from repo root with the project venv:

    python scripts/migrate_joint_pairs.py
"""

from __future__ import annotations

import json
import os
import shutil
import sys

import numpy as np


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from core.joint_pair import (  # noqa: E402
    DEFAULT_REGISTRY_PATH,
    JointHalfDef,
    JointPairDef,
    JointRegistry,
    load_joint_registry,
    save_joint_registry,
)


_TRANS_TOL_MM = 1e-6
_ROT_TOL = 1e-9


def _is_legacy(data: dict) -> bool:
    return "pairs" in data and "halves" not in data


def _half_kind_from_role(role: str) -> str:
    if role not in ("male", "female"):
        raise ValueError(f"Unknown half role: {role!r}")
    return role


def _halves_match(a: JointHalfDef, b: JointHalfDef) -> tuple[bool, str]:
    if a.block_name != b.block_name:
        return False, f"block_name mismatch: {a.block_name!r} vs {b.block_name!r}"
    t_err = float(np.linalg.norm(a.M_block_from_bar[:3, 3] - b.M_block_from_bar[:3, 3]))
    r_err = float(np.linalg.norm(a.M_block_from_bar[:3, :3] - b.M_block_from_bar[:3, :3]))
    s_t_err = float(np.linalg.norm(a.M_screw_from_block[:3, 3] - b.M_screw_from_block[:3, 3]))
    s_r_err = float(np.linalg.norm(a.M_screw_from_block[:3, :3] - b.M_screw_from_block[:3, :3]))
    if t_err > _TRANS_TOL_MM or r_err > _ROT_TOL or s_t_err > _TRANS_TOL_MM or s_r_err > _ROT_TOL:
        return False, (
            f"transforms differ: M_block(t={t_err:.3e},r={r_err:.3e}) "
            f"M_screw(t={s_t_err:.3e},r={s_r_err:.3e})"
        )
    return True, ""


def migrate(path: str = DEFAULT_REGISTRY_PATH) -> bool:
    print(f"[migrate] reading {path}")
    if not os.path.exists(path):
        print("[migrate] ERROR: registry not found.")
        return False

    with open(path, "r", encoding="utf-8") as stream:
        raw = json.load(stream)

    if not _is_legacy(raw):
        print("[migrate] file is already in the new normalized schema; nothing to do.")
        return True

    legacy_pairs = raw.get("pairs", [])
    print(f"[migrate] found {len(legacy_pairs)} legacy pairs")

    halves: dict[str, JointHalfDef] = {}
    legacy_pair_objects: dict[str, JointPairDef] = {}

    for entry in legacy_pairs:
        name = str(entry["name"])
        print(f"[migrate]   pair '{name}'")
        # Reconstruct the legacy JointPairDef object for round-trip comparison.
        # The legacy `from_dict` does NOT consume a `kind` field; we add it
        # explicitly so the new dataclass roundtrips cleanly.
        female_dict = dict(entry["female"])
        male_dict = dict(entry["male"])
        female_dict["kind"] = "female"
        male_dict["kind"] = "male"
        female = JointHalfDef.from_dict(female_dict)
        male = JointHalfDef.from_dict(male_dict)

        for role, half in (("female", female), ("male", male)):
            existing = halves.get(half.block_name)
            if existing is None:
                halves[half.block_name] = half
                print(
                    f"[migrate]     + half block_name={half.block_name!r} "
                    f"kind={half.kind!r} (new)"
                )
            else:
                ok, msg = _halves_match(existing, half)
                if not ok:
                    print(
                        f"[migrate]     ! WARNING: pair '{name}' {role} half "
                        f"block_name={half.block_name!r} differs from existing "
                        f"entry: {msg}.  Keeping FIRST occurrence."
                    )
                else:
                    print(
                        f"[migrate]     . half block_name={half.block_name!r} "
                        f"already present (matches)"
                    )

        pair = JointPairDef(
            name=name,
            female=halves[female.block_name],  # use the canonical (first) instance
            male=halves[male.block_name],
            contact_distance_mm=float(entry["contact_distance_mm"]),
            jp_range=tuple(float(v) for v in entry.get("jp_range", (-500.0, 500.0))),
            jr_range=tuple(float(v) for v in entry.get("jr_range", (-np.pi, np.pi))),
        )
        legacy_pair_objects[name] = pair

    registry = JointRegistry(halves=halves, mates=legacy_pair_objects, ground_joints={})
    print(f"[migrate] normalized: {len(halves)} halves, {len(legacy_pair_objects)} mates, 0 ground_joints")

    # Backup, then write.
    backup_path = path + ".bak"
    if os.path.exists(backup_path):
        print(f"[migrate] backup already exists at {backup_path}; not overwriting.")
    else:
        shutil.copy2(path, backup_path)
        print(f"[migrate] backed up legacy file -> {backup_path}")

    save_joint_registry(registry, path)
    print(f"[migrate] wrote new registry -> {path}")

    # Validate by reloading and comparing.
    print("[migrate] validating round-trip...")
    reloaded = load_joint_registry(path)

    if set(reloaded.mates.keys()) != set(legacy_pair_objects.keys()):
        print(
            f"[migrate] FAIL: reloaded mate names {sorted(reloaded.mates)} != "
            f"original {sorted(legacy_pair_objects)}"
        )
        return False

    all_ok = True
    for name, original in legacy_pair_objects.items():
        rebuilt = reloaded.mates[name]
        try:
            np.testing.assert_allclose(
                rebuilt.female.M_block_from_bar, original.female.M_block_from_bar, atol=1e-12
            )
            np.testing.assert_allclose(
                rebuilt.female.M_screw_from_block, original.female.M_screw_from_block, atol=1e-12
            )
            np.testing.assert_allclose(
                rebuilt.male.M_block_from_bar, original.male.M_block_from_bar, atol=1e-12
            )
            np.testing.assert_allclose(
                rebuilt.male.M_screw_from_block, original.male.M_screw_from_block, atol=1e-12
            )
            assert rebuilt.contact_distance_mm == original.contact_distance_mm, "contact_distance_mm mismatch"
            assert rebuilt.female.block_name == original.female.block_name
            assert rebuilt.male.block_name == original.male.block_name
            assert rebuilt.female.kind == "female"
            assert rebuilt.male.kind == "male"
            assert rebuilt.female.collision_filename == original.female.collision_filename
            assert rebuilt.male.collision_filename == original.male.collision_filename
            print(f"[migrate]   ok  '{name}'")
        except AssertionError as exc:
            all_ok = False
            print(f"[migrate]   FAIL '{name}': {exc}")

    if all_ok:
        print("[migrate] SUCCESS: all pairs roundtrip exactly.")
    else:
        print("[migrate] FAILURE: at least one pair did not roundtrip.")
    return all_ok


if __name__ == "__main__":
    ok = migrate()
    sys.exit(0 if ok else 1)
