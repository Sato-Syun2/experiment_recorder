#!/usr/bin/env python3
"""Validate 18-group, 72-trial recording plan (Python standard library only)."""
import csv
import json
from pathlib import Path
from collections import Counter

ROOT = Path(__file__).resolve().parent

def load_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))

def validate_group(gid, rows, sets):
    assert len(rows) == 72, f"{gid} row count={len(rows)}"
    assert [int(r["trial_no"]) for r in rows] == list(range(1, 73)), f"{gid} trial numbers"
    assert len({r["trial_id"] for r in rows}) == 72, f"{gid} trial_id duplicated"
    assert Counter(r["condition_id"] for r in rows) == Counter({f"E{i:02d}": 4 for i in range(1, 19)}), f"{gid} condition counts"
    assert Counter(r["trial_type"] for r in rows) == {"overtaking": 64, "solo": 8}, f"{gid} trial types"
    assert [rows[i*18]["set_id"] for i in range(4)] == sets, f"{gid} order"
    assert {r["set_id"] for r in rows} == {"A", "B", "C1", "C2"}, f"{gid} set types"
    for sn in range(4):
        block = rows[sn*18:(sn+1)*18]
        assert all(int(r["set_no"]) == sn+1 for r in block), f"{gid} set_no {sn+1}"
        assert [int(r["set_trial_no"]) for r in block] == list(range(1, 19)), f"{gid} set_trial_no {sn+1}"
        assert all(int(r["optional"]) == (1 if sn==3 else 0) for r in block), f"{gid} optional"
        assert {r["condition_id"] for r in block} == {f"E{i:02d}" for i in range(1, 19)}, f"{gid} block IDs"
    for person in ("P1", "P2"):
        assert Counter(r["overtaker"] for r in rows)[person] == 32, f"{gid} {person} overtaking"
        assert Counter(r["overtaken"] for r in rows)[person] == 32, f"{gid} {person} overtaken"
        assert Counter(r["solo_person"] for r in rows)[person] == 4, f"{gid} {person} solo"
    for k in range(1,17):
        c = [r for r in rows if r["condition_id"] == f"E{k:02d}"]
        assert Counter(r["overtaker"] for r in c) == {"P1":2,"P2":2}, f"{gid} E{k:02d} role"
    for i,r in enumerate(rows):
        changed = i > 0 and r["width"] != rows[i-1]["width"]
        assert int(r["width_change_before"]) == int(changed), f"{gid} width flag trial {i+1}"
        if r["trial_type"]=="solo":
            assert r["solo_person"] in {"P1","P2"} and not r["overtaker"] and not r["overtaken"], f"{gid} solo"
        else:
            assert {r["overtaker"],r["overtaken"]} == {"P1","P2"} and not r["solo_person"], f"{gid} roles"
    return sum(int(r["width_change_before"]) for r in rows)

def main():
    with (ROOT/"manifest.json").open(encoding="utf-8") as f:
        manifest=json.load(f)
    all_rows = load_csv(ROOT/"all_trials.csv")
    assert len(all_rows) == 1296, f"global count: {len(all_rows)}"
    count=0
    for gid,sets in manifest["group_order"].items():
        rows=load_csv(ROOT/"groups"/f"{gid}.csv")
        widths=validate_group(gid,rows,sets)
        assert [r for r in all_rows if r["group_id"] == gid] == rows, f"{gid} global mismatch"
        print(f"{gid} 72 trials | {'/'.join(sets)} | width changes {widths} | OK")
        count+=len(rows)
    assert count == 1296
    print(f"PASS: {len(manifest['group_order'])} groups / {count} trials / role balances correct")

if __name__=="__main__":
    main()
