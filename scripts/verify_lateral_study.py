"""Independently audit archived singleton radio/policy and kinematic records."""
from __future__ import annotations

from collections import Counter
import argparse
import gzip
import hashlib
import json
from pathlib import Path, PurePosixPath
import zipfile

import numpy as np
from pyproj import Transformer


def read(path):
    if path.suffix==".gz":
        with gzip.open(path,"rt") as stream:
            return json.load(stream)
    return json.loads(path.read_text())


def independent_reference(archive,manifest,control_step):
    """Reconstruct cubic geometry from archived GIS inputs, no simulator imports."""
    scenario_path=next(r["path"] for r in manifest["inputs"] if r["path"].endswith("/scenario.json"))
    scenario=json.loads(archive.read(scenario_path))
    config=scenario["corridor"]
    geometry_path=(PurePosixPath(scenario_path).parent/config["path"]).as_posix()
    coordinates=np.array(json.loads(archive.read(geometry_path))["features"][0]["geometry"]["coordinates"])
    transform=Transformer.from_crs(config.get("source_crs","EPSG:4326"),config.get("target_crs","EPSG:26910"),always_xy=True)
    x,y=transform.transform(coordinates[:,0],coordinates[:,1])
    raw=np.c_[x,y]
    cumulative=np.r_[0,np.cumsum(np.linalg.norm(np.diff(raw,axis=0),axis=1))]
    knots=np.linspace(0,cumulative[-1],int(np.ceil(cumulative[-1]/control_step))+1)
    points=np.c_[np.interp(knots,cumulative,raw[:,0]),np.interp(knots,cumulative,raw[:,1])]
    relative=points-points[0]
    matrix=np.eye(len(knots));rhs=np.zeros_like(points)
    for i in range(1,len(knots)-1):
        matrix[i,i-1:i+2]=[1,4,1]
        rhs[i]=6*(relative[i+1]-2*relative[i]+relative[i-1])
    second=np.linalg.solve(matrix,rhs)
    c=np.stack([relative[:-1],np.diff(relative,axis=0)-(2*second[:-1]+second[1:])/6,
                second[:-1]/2,np.diff(second,axis=0)/6],axis=1)
    step=knots[1]-knots[0]
    def frame(q):
        u=np.clip(np.asarray(q)/step,0,len(knots)-1)
        index=np.minimum(u.astype(int),len(knots)-2);u=u-index
        coeff=c[index]
        center=points[0]+coeff[...,0,:]+u[...,None]*(coeff[...,1,:]+u[...,None]*(coeff[...,2,:]+u[...,None]*coeff[...,3,:]))
        first=(coeff[...,1,:]+2*u[...,None]*coeff[...,2,:]+3*u[...,None]**2*coeff[...,3,:])/step
        accel=(2*coeff[...,2,:]+6*u[...,None]*coeff[...,3,:])/step**2
        norm=np.linalg.norm(first,axis=-1)
        tangent=first/norm[...,None]
        normal=np.stack([-tangent[...,1],tangent[...,0]],axis=-1)
        curvature=(first[...,0]*accel[...,1]-first[...,1]*accel[...,0])/norm**3
        return center,tangent,normal,norm,curvature
    return frame


def verify(run):
    run=Path(run).resolve()
    manifest=read(run/"manifest.json")
    if manifest["status"]!="completed":
        raise ValueError("run requires review")
    for entry in manifest["outputs"]:
        if hashlib.sha256((run/entry["path"]).read_bytes()).hexdigest()!=entry["sha256"]:
            raise ValueError(f"output hash mismatch: {entry['path']}")
    with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
        for entry in manifest["code"]+manifest["inputs"]:
            if hashlib.sha256(archive.read(entry["path"])).hexdigest()!=entry["sha256"]:
                raise ValueError(f"archive mismatch: {entry['path']}")
    spec=read(run/"resolved_config.json")
    stations=np.array([[s["x_m"],s["y_m"],s["height_m"] if s["height_m"] is not None
                        else spec["radio"]["assumed_bs_height_m"]] for s in spec["base_stations"]])
    radio=spec["radio"]
    summary_rows=read(run/"summary.json")
    total_observations=total_intervals=0
    maximum_velocity_residual=0.
    maximum_progress_residual=maximum_position_residual=0.
    frames={}
    for summary in summary_rows:
        case=run/summary["case_id"]
        cfg=summary["parameters"]
        if cfg["center_control_step_m"] not in frames:
            with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
                frames[cfg["center_control_step_m"]]=independent_reference(archive,manifest,cfg["center_control_step_m"])
        frame=frames[cfg["center_control_step_m"]]
        trace=read(case/"trace.json.gz")
        rows=read(case/"observations.json.gz")
        events=read(case/"events.json")
        terminal=read(case/"terminal.json")
        t=np.array([r["t_s"] for r in trace]);ends=np.array([r["end_s"] for r in trace])
        xyz=np.array([r["position_m"] for r in trace]);vel=np.array([r["velocity_mps"] for r in trace])
        if not np.allclose(ends[:-1],t[1:],rtol=0,atol=1e-8) or abs(ends[-1]-terminal["t_s"])>1e-8:
            raise ValueError("motion time intervals are not contiguous")
        if t[0]!=0 or np.any(ends<=t):
            raise ValueError("invalid motion clock")
        if not np.allclose(xyz[:,2],cfg["altitude_m"]):
            raise ValueError("height changed")
        expected_speed=np.hypot(cfg["cruise_mps"],[r["offset_velocity_mps"] for r in trace])
        if not np.allclose(np.linalg.norm(vel,axis=1),expected_speed,rtol=0,atol=1e-8):
            raise ValueError("physical speed convention mismatch")
        residual=np.linalg.norm(np.diff(xyz,axis=0)/np.diff(t)[:,None]-(vel[1:]+vel[:-1])/2,axis=1)
        peak=float(residual.max(initial=0));maximum_velocity_residual=max(maximum_velocity_residual,peak)
        # Endpoint velocity averaging has a curvature-dependent trapezoidal
        # error. Report it, but do not mistake that approximation for integration
        # error. Independently reconstruct geometry and use another ODE method.
        progress=np.array([r["q_m"] for r in trace])
        offsets=np.array([r["offset_m"] for r in trace])
        offset_speed=np.array([r["offset_velocity_mps"] for r in trace])
        center,tangent,normal,norm,kappa=frame(progress)
        position_error=float(np.linalg.norm(center+offsets[:,None]*normal-xyz[:,:2],axis=1).max())
        maximum_position_residual=max(maximum_position_residual,position_error)
        if position_error>1e-6:
            raise ValueError("Cartesian position disagrees with archived reference geometry")
        modeled_velocity=cfg["cruise_mps"]*tangent+offset_speed[:,None]*normal
        if np.max(np.linalg.norm(modeled_velocity-vel[:,:2],axis=1))>1e-7:
            raise ValueError("tangential/normal velocity components disagree")
        accepted=[e for e in events if e["status"]=="transition_accepted"]
        def offset_at(times):
            d=np.full(len(times),summary["initial_offset_m"],float)
            for move in accepted:
                u=np.clip((times-move["t_s"])/move["duration_s"],0,1)
                candidate=move["source_m"]+(move["target_m"]-move["source_m"])*(10*u**3-15*u**4+6*u**5)
                d=np.where(times>=move["t_s"],candidate,d)
            return d
        def rate(q,time):
            _,_,_,norm,kappa=frame(q)
            return cfg["cruise_mps"]/(norm*(1-kappa*offset_at(time)))
        # Independent explicit-midpoint integration: 128 substeps per saved
        # interval, batch-vectorized, rather than the simulator's adaptive RK4.
        state=progress.copy();step=(ends-t)/128
        for k in range(128):
            time=t+k*step
            first=rate(state,time)
            state+=step*rate(state+step*first/2,time+step/2)
        expected=np.r_[progress[1:],terminal["q_m"]]
        integration_error=float(np.max(np.abs(state-expected)))
        maximum_progress_residual=max(maximum_progress_residual,integration_error)
        if integration_error>1e-4:
            raise ValueError(f"independent midpoint progress mismatch: {integration_error} m")
        for field,limit in [("offset_velocity_mps",cfg["lateral_speed_limit_mps"]),
                            ("offset_acceleration_mps2",cfg["lateral_accel_limit_mps2"])]:
            if max(abs(r[field]) for r in trace)>limit+1e-8:
                raise ValueError("offset component limit violated")
        by_time={round(r["t_s"],8):r for r in trace}
        history=[]
        for row in rows:
            motion=by_time[round(row["t_s"],8)]
            if abs(row["t_s"]/cfg["policy_s"]-round(row["t_s"]/cfg["policy_s"]))>1e-8:
                raise ValueError("policy clock mismatch")
            point=np.array(motion["position_m"])
            distance=np.linalg.norm(point-stations,axis=1)
            power=radio["eirp_dbm"]+radio["receiver_gain_db"]-28-22*np.log10(distance)-20*np.log10(radio["frequency_ghz"])
            selected=np.argsort(distance)[:radio["served_set_size"]]
            best=int(selected[np.argmax(power[selected])])
            linear=10**(power/10)
            interference=max(float(linear[selected].sum()-linear[best]),np.finfo(float).tiny)
            sinr=10*np.log10(linear[best]/(interference+10**(radio["noise_dbm"]/10)))
            if best!=row["serving_bs"] or abs(sinr-row["sinr_db"])>1e-8:
                raise ValueError("independent radio check failed")
            rsrp=power[best]-10*np.log10(radio["resource_elements"])
            if abs(rsrp-row["rsrp_dbm"])>1e-8:
                raise ValueError("RSRP mismatch")
            history=[h for h in history if h[0]>=row["t_s"]-cfg["window_s"]-1e-8]
            history.append((row["t_s"],int(sinr<cfg["threshold_db"])))
            exposure=sum(b for _,b in history)/len(history)
            policy="C" if exposure<=cfg["exposure_c"]+1e-12 else "R" if exposure<=cfg["exposure_r"]+1e-12 else "F"
            if abs(exposure-row["exposure"])>1e-12 or policy!=row["policy"] or len(history)!=row["history_count"]:
                raise ValueError("available-history policy mismatch")
            if row["group_size"]!=1 or row["policy"]!=motion["policy"]:
                raise ValueError("singleton convention mismatch")
        durations=ends-t
        for policy in "CRF":
            share=sum(dt for dt,r in zip(durations,trace) if r["policy"]==policy)/terminal["t_s"]
            if abs(share-summary["policy_time_shares"][policy])>1e-10:
                raise ValueError("policy time share mismatch")
            if sum(r["policy"]==policy for r in rows)!=summary["policy_observation_counts"][policy]:
                raise ValueError("policy observation count mismatch")
        accepted=[e for e in events if e["status"]=="transition_accepted"]
        completed=[e for e in events if e["status"]=="transition_completed"]
        if len(accepted)!=len(completed) or len(completed)!=summary["completed_moves"]:
            raise ValueError("unfinished/uncounted maneuver")
        for start,end in zip(accepted,completed):
            D=start["duration_s"];delta=end["target_m"]-end["source_m"]
            if abs(end["t_s"]-start["t_s"]-D)>1e-8 or start["target_m"]!=end["target_m"]:
                raise ValueError("maneuver boundaries mismatch")
            if 1.875*abs(delta)/D>cfg["lateral_speed_limit_mps"]+1e-8 or 10/np.sqrt(3)*abs(delta)/D**2>cfg["lateral_accel_limit_mps2"]+1e-8:
                raise ValueError("analytic quintic bound failed")
            if start.get("planner")=="minimum_change":
                if by_time[round(start["t_s"],8)]["policy"]=="C":
                    raise ValueError("minimum-change maneuver initiated while C")
                setting=summary["planning_parameters"]
                if abs(delta/setting["candidate_spacing_m"]-round(delta/setting["candidate_spacing_m"]))>1e-8:
                    raise ValueError("movement endpoint not on candidate grid")
                stay=start["stay_recovery_time_s"]
                if start["recovery_time_s"] is None or (stay is not None and start["recovery_time_s"]>stay-cfg["policy_s"]+1e-8):
                    raise ValueError("minimum-change candidate not earlier than natural recovery")
                if max(start["maneuver_curvature_bound_per_m"],start["remaining_curvature_bound_per_m"])>1/setting["minimum_radius_m"]+1e-12:
                    raise ValueError("accepted candidate fails conservative curvature bound")
                alternatives=[e for e in events if abs(e["t_s"]-start["t_s"])<1e-8
                              and e["status"] in {"eligible","transition_accepted"}]
                key=lambda e:(abs(e["target_m"]-e["source_m"]),e["duration_s"],e["recovery_time_s"],e["target_m"])
                if key(start)!=min(map(key,alternatives)):
                    raise ValueError("selected candidate is not minimum tested displacement/duration")
                after=[r for r in rows if r["t_s"]>=end["t_s"]-1e-8]
                if not after or after[-1]["t_s"]<after[0]["t_s"]+cfg["window_s"]-1e-8:
                    raise ValueError("missing realized post-completion window")
                window=[r for r in after if r["t_s"]<=after[0]["t_s"]+cfg["window_s"]+1e-8]
                if any(r["policy"]!="C" for r in window):
                    raise ValueError("realized post-completion recovery was not maintained")
            elif start["exposure_benefit"]<cfg["minimum_exposure_benefit"]-1e-10 or start["policy_spacing_priority_benefit_m"]< -1e-8:
                raise ValueError("accepted candidate failed communication gate")
        # Verify the realized quintic everywhere, not just the reported extrema.
        for row in trace:
            d=summary["initial_offset_m"];vd=ad=0.
            for move in accepted:
                if row["t_s"]<move["t_s"]-1e-8:
                    break
                u=np.clip((row["t_s"]-move["t_s"])/move["duration_s"],0,1)
                delta=move["target_m"]-move["source_m"]
                d=move["source_m"]+delta*(10*u**3-15*u**4+6*u**5)
                vd=delta*30*u*u*(1-u)**2/move["duration_s"]
                ad=delta*60*u*(1-u)*(1-2*u)/move["duration_s"]**2
            if max(abs(d-row["offset_m"]),abs(vd-row["offset_velocity_mps"]),abs(ad-row["offset_acceleration_mps2"]))>1e-7:
                raise ValueError("realized maneuver not the declared quintic")
        if abs(summary["corridor_time_s"]-terminal["t_s"])>1e-8 or summary["capacity_estimated"]:
            raise ValueError("wrong stage/terminal summary")
        if summary.get("kind")=="minimum_change":
            # Actual curvature from independently reconstructed geometry at
            # every saved interval start, in addition to continuous bounds.
            _,_,_,_,curvature=frame(progress)
            vd=offset_speed;ad=np.array([r["offset_acceleration_mps2"] for r in trace]);V=cfg["cruise_mps"]
            actual=V*curvature/((1-curvature*offsets)*np.sqrt(V*V+vd*vd))+V*ad/(V*V+vd*vd)**1.5
            if np.max(np.abs(actual))>1/summary["planning_parameters"]["minimum_radius_m"]+1e-10:
                raise ValueError("realized trajectory violates provisional radius")
        total_observations+=len(rows);total_intervals+=len(trace)
    return {"status":"passed","run":str(run),"conditions":len(summary_rows),
            "independent_radio_policy_records":total_observations,"motion_intervals":total_intervals,
            "maximum_displacement_velocity_residual_mps":maximum_velocity_residual,
            "velocity_residual_note":"Endpoint trapezoidal average is a diagnostic, not integration error; independent midpoint integration is the acceptance check.",
            "maximum_independent_midpoint_progress_residual_m":maximum_progress_residual,
            "maximum_archived_geometry_position_residual_m":maximum_position_residual,
            "verifier_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "verified_output_hashes":len(manifest["outputs"]),"archived_source_hashes":len(manifest["code"]),
            "archived_input_hashes":len(manifest["inputs"]),
            "note":"Numerical kinematic, formula and provenance checks; not flight-performance, safety or capacity certification."}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    result=verify(args.run)
    text=json.dumps(result,indent=2)+"\n"
    if args.output:
        if args.output.exists():
            raise FileExistsError("refusing to replace an audit record")
        source_snapshot=args.output.with_name(args.output.stem+"-verifier.zip")
        if source_snapshot.exists():
            raise FileExistsError("refusing to replace a verifier snapshot")
        with zipfile.ZipFile(source_snapshot,"w",zipfile.ZIP_DEFLATED) as archive:
            archive.write(Path(__file__),"verify_lateral_study.py")
        args.output.write_text(text)
    print(text,end="")
