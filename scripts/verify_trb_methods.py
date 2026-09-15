"""Optional, read-only parity check against the user's original TRB source.

Not a CI dependency: the public repository remains standalone. No seed files
or paper assets are modified; comparison uses a deterministic synthetic input.
"""
from pathlib import Path
import argparse
import hashlib
import importlib
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capacity_policy.policy import assign_policy, PolicyConfig
from capacity_policy.capacity import snapshot_capacity, CapacityConfig
from uam_simulator.group_runner import reliability_floor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trb-root", type=Path, required=True)
    args = parser.parse_args()
    original = args.trb_root.resolve() / "experiment"
    for name in ("config.py", "pipeline.py", "sinr_policy_pipeline.py", "mixed_capacity.py"):
        if not (original / name).is_file():
            raise FileNotFoundError(original / name)
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(original))
    config = importlib.import_module("config")
    policy = importlib.import_module("sinr_policy_pipeline")
    capacity = importlib.import_module("mixed_capacity")
    times = np.arange(0.0, 3001.0, config.DT_S)
    entry = np.arange(25) * 32.0
    active = (times[None, :] >= entry[:, None]) & (times[None, :] <= entry[:, None] + 1600)
    sinr = 3 * np.sin(times[None, :] / 113 + np.arange(25)[:, None] / 7) - 1.5
    codes, valid, obs, diagnostics = policy.classify_sinr_common_threshold(
        {"t": times, "active": active, "sinr_db": sinr}, -1.5)
    ours = assign_policy({"t": times, "active": active, "link_ok": active & (sinr >= -1.5)},
                         PolicyConfig(time_step_s=config.DT_S, warmup_s=config.WARMUP_S))
    np.testing.assert_array_equal(codes, ours["policy"])
    np.testing.assert_array_equal(valid, ours["valid"])
    np.testing.assert_array_equal(obs, ours["t_obs"])
    np.testing.assert_array_equal(diagnostics["zeta"], ours["exposure"])
    a = capacity.snapshot_capacity(codes, valid, obs)
    b = snapshot_capacity(ours["policy"], ours["valid_after_warmup"], ours["t_obs"], CapacityConfig())
    for key in a:
        np.testing.assert_array_equal(a[key], b[key])
    for rho in (0.9, 0.95, 0.99):
        assert capacity.reliability_qualified_value(a["q_mix_UAM_h"], rho) == reliability_floor(b["q_mix_UAM_h"], rho)
    sources = [original / p for p in ("config.py", "pipeline.py", "sinr_policy_pipeline.py", "mixed_capacity.py")]
    print(json.dumps({"status": "pass", "valid_group_time_observations": int(valid.sum()),
        "comparison": "exact equality of common-threshold policy, exposure, validity, snapshot capacity, and reliability floor",
        "not_compared": "radio assumptions and sampled geometry ensemble differ between the paper and the real corridor",
        "sources": [{"file": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources]}, indent=2))


if __name__ == "__main__":
    main()
