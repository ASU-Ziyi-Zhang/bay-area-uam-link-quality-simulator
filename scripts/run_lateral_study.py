"""Archive the single-altitude Bay Area lateral-response experiment."""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from capacity_policy import load_scenario
from uam_simulator.lateral_study import LateralConfig, SmoothCorridorFrame, field_samples, simulate_isolated
from uam_simulator.research_provenance import execution_metadata
from uam_simulator.minimum_change import MinimumChangeConfig, MinimumChangePlanner


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_cases(spec,cfg):
    spacing=float(spec["candidate_spacing_m"])
    envelopes=list(map(float,spec["envelopes_m"]))
    initial=float(spec["initial_offset_m"])
    if (not np.isfinite([spacing,initial,*envelopes]).all() or spacing<=0 or not envelopes
            or any(w<=0 or abs(w/spacing-round(w/spacing))>1e-9 for w in envelopes)
            or any(b<=a for a,b in zip(envelopes,envelopes[1:]))):
        raise ValueError("positive increasing envelopes on the candidate grid required")
    def grid(w):
        return np.arange(-round(w/spacing),round(w/spacing)+1)*spacing
    static=grid(max(envelopes))
    cases=[]
    if spec.get("study")=="bay_area_minimum_change":
        for case in spec["case_matrix"]:
            mode=case["controller"]
            if mode not in {"none","gain_priority","minimum_change"}:
                raise ValueError("unknown lateral controller")
            width=float(case["envelope_m"])
            if width not in envelopes and not (width==0 and mode=="none"):
                raise ValueError("case envelope must be registered")
            candidates=grid(width)
            if initial not in candidates:
                raise ValueError("initial offset must be a candidate")
            cases.append((case["id"],candidates.tolist(),initial,mode!="none",
                          replace(cfg,**case.get("parameters",{})),mode))
        ids=[case[0] for case in cases]
        if len(ids)!=len(set(ids)) or any(not name.replace("_","").isalnum() for name in ids):
            raise ValueError("unique simple case names required")
        return static,cases
    for offset in static:
        name=f"static_{'minus' if offset<0 else 'plus'}_{abs(offset):g}"
        cases.append((name,[float(offset)],float(offset),False,cfg,"static"))
    for width in envelopes:
        candidates=grid(width)
        if initial not in candidates:
            raise ValueError("common initial offset must be in every envelope")
        cases.append((f"adaptive_{width:g}",candidates.tolist(),initial,True,cfg,"adaptive"))
    for refinement in spec.get("refinements",[]):
        width=float(refinement["envelope_m"])
        if width not in envelopes:
            raise ValueError("refinement must match an existing envelope")
        cases.append((refinement["id"],grid(width).tolist(),initial,True,
                      replace(cfg,**refinement["parameters"]),"refinement"))
    ids=[c[0] for c in cases]
    if len(ids)!=len(set(ids)) or any(not name.replace("_","").isalnum() for name in ids):
        raise ValueError("unique simple case names required")
    return static,cases


def run(config_path,output):
    config_path,output=Path(config_path).resolve(),Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"refusing to reuse any run directory: {output}")
    spec=json.loads(config_path.read_text())
    if spec.get("schema_version")!=1 or spec.get("study") not in {"bay_area_singleton_lateral_response","bay_area_minimum_change"}:
        raise ValueError("unsupported lateral study")
    cfg=LateralConfig(**spec["parameters"])
    scenario_path=(config_path.parent/spec["scenario"]).resolve()
    scenario=load_scenario(scenario_path)
    raw_scenario=json.loads(scenario_path.read_text())
    static,cases=make_cases(spec,cfg)
    frame=SmoothCorridorFrame(scenario.corridor,cfg.center_control_step_m)
    geometry=frame.report(float(np.max(np.abs(static))))
    landscape=field_samples(frame,scenario,cfg,static)
    zero_profile=landscape["sinr_db"][int(np.flatnonzero(static==0)[0])]
    radio_delta=zero_profile-landscape["raw_center_sinr_db"]
    geometry.update(centerline_sinr_max_absolute_change_db=float(np.max(np.abs(radio_delta))),
        centerline_sinr_rms_change_db=float(np.sqrt(np.mean(radio_delta**2))),
        centerline_threshold_classification_disagreements=int(np.sum(
            (zero_profile<cfg.threshold_db)!=(landscape["raw_center_sinr_db"]<cfg.threshold_db))),
        radio_comparison_profile_samples=len(zero_profile))
    inputs=[config_path,scenario_path,scenario_path.parent/raw_scenario["corridor"]["path"],
            scenario_path.parent/raw_scenario["base_stations"]["path"]]
    sources=sorted((ROOT/"src").rglob("*.py"))+[ROOT/"tests/test_lateral_study.py",
        Path(__file__).resolve(),ROOT/"scripts/verify_lateral_study.py",ROOT/"scripts/plot_lateral_study.py",
        ROOT/"pyproject.toml",ROOT/"research/dynamic-transitions/singleton-lateral-protocol.md"]
    planning=MinimumChangeConfig(**spec["planning"]) if spec["study"]=="bay_area_minimum_change" else None
    if planning is not None:
        sources += [ROOT/"tests/test_minimum_change.py",ROOT/"research/dynamic-transitions/minimum-change-protocol.md"]
    def record(path):
        return {"path":path.relative_to(ROOT).as_posix(),"sha256":sha(path)}
    # Resolve/validate all dependencies before reserving the immutable run folder.
    code,input_records=[record(p) for p in sources],[record(p) for p in inputs]
    metadata=execution_metadata(ROOT,[str(Path(__file__).resolve()),"--config",str(config_path),"--output",str(output)])
    metadata["source_snapshot_included"]=True
    metadata["reproducibility_note"]="Exact source and scenario input ZIP retained; runtime versions recorded, not bundled."
    output.mkdir(parents=True)
    resolved={**spec,"parameters":asdict(cfg),"scenario_id":scenario.scenario_id,
              "radio":asdict(scenario.radio),"base_stations":[asdict(s) for s in scenario.base_stations.stations],
              "policy_priority_spacing_m":{p:cfg.policy_config().spacing(p,cfg.cruise_mps) for p in "CRF"},
              "cases":[{"id":name,"offsets_m":offsets,"initial_offset_m":start,
                        "transitions":enabled,"parameters":asdict(local),"kind":kind}
                       for name,offsets,start,enabled,local,kind in cases]}
    write_json(output/"resolved_config.json",resolved)
    write_json(output/"geometry.json",geometry)
    with zipfile.ZipFile(output/"source_snapshot.zip","w",zipfile.ZIP_DEFLATED) as archive:
        for p in sources+inputs:
            archive.write(p,p.relative_to(ROOT).as_posix())
    manifest={"study":spec["study"],"schema_version":1,"status":"running",
              "created_utc":datetime.now(timezone.utc).isoformat(),"execution":metadata,
              "code":code,"inputs":input_records}
    write_json(output/"manifest.json",manifest)
    summaries=[]
    try:
        np.savez_compressed(output/"field.npz",**landscape)
        for name,offsets,start,enabled,local,kind in cases:
            clock=time.monotonic()
            print(f"Running {name}",flush=True)
            local_frame=frame if local.center_control_step_m==cfg.center_control_step_m else SmoothCorridorFrame(
                scenario.corridor,local.center_control_step_m)
            planner=MinimumChangePlanner(local_frame,planning) if kind=="minimum_change" else None
            result=simulate_isolated(local_frame,scenario,local,offsets,start,enabled,planner)
            directory=output/name;directory.mkdir()
            for key in ("trace","observations"):
                with gzip.open(directory/f"{key}.json.gz","wt",encoding="utf-8") as stream:
                    json.dump(result[key],stream,allow_nan=False)
            for key in ("events","terminal"):
                write_json(directory/f"{key}.json",result[key])
            summary={"case_id":name,"kind":kind,"parameters":asdict(local),**result["summary"],
                     "wall_time_s":time.monotonic()-clock}
            if planner is not None:
                summary["planning_parameters"]=asdict(planning)
            write_json(directory/"summary.json",summary)
            summaries.append(summary)
            write_json(output/"summary.json",summaries)
            print(f"  moves={summary['completed_moves']}; F={summary['policy_time_shares']['F']:.4%}; "
                  f"time={summary['corridor_time_s']:.3f}s; wall={summary['wall_time_s']:.2f}s",flush=True)
    except Exception as exc:
        manifest["exception"]={"type":type(exc).__name__,"message":str(exc)}
        raise
    finally:
        drift=[r["path"] for r in code+input_records if sha(ROOT/r["path"])!=r["sha256"]]
        manifest.update(status="completed" if len(summaries)==len(cases) and not drift else "requires_review",
                        changed_during_run=drift,completed_cases=len(summaries),
                        outputs=[{"path":p.relative_to(output).as_posix(),"sha256":sha(p)}
                                 for p in sorted(output.rglob("*")) if p.is_file() and p.name!="manifest.json"])
        write_json(output/"manifest.json",manifest)
    return summaries


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=ROOT/"research/dynamic-transitions/configs/singleton_lateral_100m.json")
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    run(args.config,args.output)
