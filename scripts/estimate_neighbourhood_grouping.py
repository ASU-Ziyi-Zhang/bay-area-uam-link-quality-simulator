"""Offline estimate: replay a finished run's policy classifier with another grouping.

Positions and per-aircraft SINR are taken from the archive as flown, so the
estimate holds the trajectories fixed; a rerun is needed for the closed loop.
The replay is first checked against the archive under the grouping the run
actually used, then repeated under the neighbourhood grouping.

Works for the single-entry 3 x 3 study (one origin stream) and the three-stream
3 x 1 study (capacity summed over origin streams, complete snapshots only).
"""
from __future__ import annotations

import argparse
import gzip
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from uam_simulator.bay_area_dispatch import DispatchConfig  # noqa: E402
from uam_simulator.geographic_traffic import grid_reach, neighbourhood_members  # noqa: E402
from uam_simulator.policy_motion import policy_from_exposure  # noqa: E402


def load(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt") as stream:
        return json.load(stream)


def lane_order_members(rows, n):
    groups = {}
    by_lane = defaultdict(list)
    for r in rows:
        by_lane[r["lane"]].append(r)
    for lane_rows in by_lane.values():
        ordered = sorted(lane_rows, key=lambda r: (r["q_m"], r["aircraft_id"]))
        for i, own in enumerate(ordered):
            groups[own["aircraft_id"]] = tuple(r["aircraft_id"] for r in ordered[max(0, i - n):i + n + 1])
    return groups


def neighbourhood(rows, cfg, lateral_reach, vertical_reach):
    ids = [r["aircraft_id"] for r in rows]
    q = np.array([r["q_m"] for r in rows])
    d = np.array([r["offset_m"] for r in rows])
    h = np.array([r["xyz"][2] for r in rows])
    cap = cfg.group_radius_m if cfg.group_radius_m is not None else cfg.neighbors_each_side * cfg.spacing("F", cfg.cruise_mps)
    return {ids[i]: neighbourhood_members(i, ids, q, d, h, count=2 * cfg.neighbors_each_side, along_cap_m=cap,
                                          lateral_reach_m=lateral_reach, vertical_reach_m=vertical_reach)
            for i in range(len(ids))}


def replay(times, positions, sinr, grouping, cfg, active_at_mark):
    """Return per-sample group size and raw policy, and filtered policies at each policy mark.

    Radio samples and policy updates run on separate clocks, as in the runner:
    at a shared timestamp the sample is taken first, then every active aircraft
    updates its persistence filter from its newest raw policy.
    """
    history = defaultdict(list)
    latest_raw = {}
    filters = {}
    samples, policy_at = [], {}
    events = sorted([(t, 0) for t in times] + [(t, 1) for t in active_at_mark])
    for t, kind in events:
        if kind == 0:
            rows = positions[t]
            members = grouping(rows)
            for r in rows:
                a = r["aircraft_id"]
                ids = members[a]
                fraction = float(np.mean([sinr[t][m] < cfg.threshold_db for m in ids]))
                hist = [x for x in history[a] if x[0] >= t - cfg.window_s - 1e-8]
                hist.append((t, fraction))
                history[a] = hist
                raw = policy_from_exposure(float(np.mean([x[1] for x in hist])), cfg.policy_config())
                latest_raw[a] = raw
                samples.append({"t": t, "id": a, "size": len(ids), "raw": raw, "q": r["q_m"]})
            continue
        active = {a for a in active_at_mark[t] if a in latest_raw}
        for a in active:
            rec = filters.setdefault(a, {"state": "C", "cand": "C", "count": 0})
            raw = latest_raw[a]
            if raw == rec["state"]:
                rec["cand"], rec["count"] = rec["state"], 0
            else:
                rec["count"] = rec["count"] + 1 if raw == rec["cand"] else 1
                rec["cand"] = raw
                if rec["count"] >= cfg.persistence_k:
                    rec["state"], rec["count"] = raw, 0
        for a in list(filters):
            if a not in active_at_mark[t]:
                filters.pop(a)
        policy_at[t] = {a: filters[a]["state"] if a in filters else "C" for a in active_at_mark[t]}
    return samples, policy_at


def shares(values):
    c = Counter(values)
    n = sum(c.values())
    return {p: round(100 * c[p] / n, 2) for p in "CRF"}


def capacity(policy_at, cfg, origin=None, origins=None, window=None):
    """Single stream: 3600 v / mean S. Several origin streams: sum of per-stream rates, complete snapshots only."""
    q = []
    for t, pols in sorted(policy_at.items()):
        if not pols or (window and not window[0] <= t <= window[1]):
            continue
        if origin is None:
            q.append(3600 * cfg.cruise_mps / np.mean([cfg.spacing(p) for p in pols.values()]))
            continue
        by = defaultdict(list)
        for a, p in pols.items():
            by[origin(a)].append(cfg.spacing(p))
        if any(not by[o] for o in origins):
            continue
        q.append(sum(3600 * cfg.cruise_mps / np.mean(by[o]) for o in origins))
    if not q:
        return None
    q = np.asarray(q)
    return {"mean": round(float(q.mean()), 2), "q95": round(float(np.quantile(q, 0.05)), 2), "n": len(q)}


def summarise(samples, policy_at, cfg, window, origin, origins):
    sizes = Counter(min(s["size"], 5) for s in samples)
    total = sum(sizes.values())
    bins = defaultdict(list)
    for s in samples:
        bins[int(s["q"] // 5000) * 5].append(s["raw"])
    filtered = [p for pols in policy_at.values() for p in pols.values()]
    return {
        "group_size_share_pct": {("5+" if k == 5 else str(k)): round(100 * v / total, 1) for k, v in sorted(sizes.items())},
        "raw_policy_pct": shares([s["raw"] for s in samples]),
        "policy_after_persistence_pct": shares(filtered),
        "raw_F_pct_by_5km": {f"{k}-{k + 5} km": shares(v)["F"] for k, v in sorted(bins.items())},
        "capacity_full_trace": capacity(policy_at, cfg, origin, origins),
        "capacity_matched_window": capacity(policy_at, cfg, origin, origins, window),
    }


def geometry(spec):
    traffic = spec["traffic"]
    if "flow_points" in traffic:
        offsets = [p["offset_m"] for p in traffic["flow_points"]]
        heights = [p["altitude_m"] for p in traffic["flow_points"]]
        return offsets, heights, None, None
    offsets = [float(x) for x in traffic["lateral_offsets_m"]]
    heights = [float(traffic["altitude_m"])] * len(offsets)
    # Aircraft keep their origin stream: identifiers are "L<lane+1>-UAM<n>".
    return offsets, heights, (lambda a: int(a.split("-")[0][1:]) - 1), list(range(len(offsets)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=ROOT / "research/dynamic-transitions/runs/R0056")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--case", action="append", help="limit to these case ids")
    args = parser.parse_args()
    spec = load(args.run / "resolved_config.json")
    params = {k: v for k, v in spec["parameters"].items() if k != "group_mode"}
    cfg = DispatchConfig(**params)
    offsets, heights, origin, origins = geometry(spec)
    lateral_reach, vertical_reach = grid_reach(offsets), grid_reach(heights)
    cases = [c["id"] for c in spec["cases"] if not args.case or c["id"] in args.case]
    # Compared window: first exit to last entry, common to the cases replayed.
    exits, entries = [], []
    for case in cases:
        ev = load(args.run / case / "events.json")
        ev = ev if isinstance(ev, list) else ev.get("events", [])
        exits.append(min((e["t_s"] for e in ev if e.get("status") == "exit"), default=None))
        entries.append(max((e["t_s"] for e in ev if e.get("status") == "corridor_entry"), default=None))
    window = (max(exits), min(entries)) if None not in exits + entries else None
    result = {"run": args.run.name, "cap_m": cfg.neighbors_each_side * cfg.spacing("F", cfg.cruise_mps),
              "lateral_reach_m": lateral_reach, "vertical_reach_m": vertical_reach, "window_s": window, "cases": {}}
    for case in cases:
        obs = load(args.run / case / "observations.json.gz")
        trace = load(args.run / case / "trace.json.gz")
        sinr = defaultdict(dict)
        recorded = {}
        for o in obs:
            sinr[o["t_s"]][o["aircraft_id"]] = o["sinr_db"]
            recorded[(o["t_s"], o["aircraft_id"])] = o
        positions = defaultdict(list)
        active_at_mark = defaultdict(set)
        trace_policy = {}
        for r in trace:
            t = r["t_s"]
            if t in sinr and r["aircraft_id"] in sinr[t]:
                positions[t].append(r)
            if abs(t / cfg.policy_s - round(t / cfg.policy_s)) < 1e-8:
                active_at_mark[t].add(r["aircraft_id"])
                trace_policy[(t, r["aircraft_id"])] = r["policy"]
        times = sorted(t for t in sinr if len(positions[t]) == len(sinr[t]))
        skipped = len(sinr) - len(times)

        old_samples, old_policy = replay(times, positions, sinr,
                                         lambda rows: lane_order_members(rows, cfg.neighbors_each_side), cfg, active_at_mark)
        pairs = [(t, a, p) for t, pols in old_policy.items() for a, p in pols.items()]
        check = {
            "members_match": sum(tuple(recorded[(s["t"], s["id"])]["members"]) == lane_order_members(positions[s["t"]], cfg.neighbors_each_side)[s["id"]]
                                 for s in old_samples) / len(old_samples),
            "raw_policy_match": sum(recorded[(s["t"], s["id"])]["raw_policy"] == s["raw"] for s in old_samples) / len(old_samples),
            "filtered_policy_match": sum(trace_policy[(t, a)] == p for t, a, p in pairs) / len(pairs),
            "sample_times_skipped": skipped,
        }
        new_samples, new_policy = replay(times, positions, sinr,
                                         lambda rows: neighbourhood(rows, cfg, lateral_reach, vertical_reach), cfg, active_at_mark)
        result["cases"][case] = {
            "replay_check_against_archive": check,
            "archive_grouping_replayed": summarise(old_samples, old_policy, cfg, window, origin, origins),
            "neighbourhood_grouping": summarise(new_samples, new_policy, cfg, window, origin, origins),
        }
        print(case, json.dumps(result["cases"][case], indent=1))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
