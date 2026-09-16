"""Reproduce PNG/SVG Stage-1 research figures using only archived run data."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.ticker import MaxNLocator, PercentFormatter
import matplotlib.patheffects as pe
import numpy as np


INK="#20323D";GRAY="#75838B";TEAL="#087F8C";GOLD="#E3A53B";RUST="#C55343"
COLORS={"C":"#198577","R":GOLD,"F":RUST}


def read(path):
    if path.suffix==".gz":
        with gzip.open(path,"rt") as stream:
            return json.load(stream)
    return json.loads(path.read_text())


def plot(run,output):
    run,output=Path(run).resolve(),Path(output).resolve()
    if output.exists():
        raise FileExistsError("refusing to overwrite a figure bundle; choose a new revision directory")
    manifest=read(run/"manifest.json")
    if manifest["status"]!="completed":
        raise ValueError("cannot plot an incomplete run as results")
    for item in manifest["outputs"]:
        if hashlib.sha256((run/item["path"]).read_bytes()).hexdigest()!=item["sha256"]:
            raise ValueError("run hash mismatch")
    spec=read(run/"resolved_config.json");cfg=spec["parameters"]
    summaries=read(run/"summary.json");by_id={r["case_id"]:r for r in summaries}
    width=max(spec["envelopes_m"])
    adaptive=f"adaptive_{width:g}"
    initial=spec["initial_offset_m"]
    baseline=f"static_{'minus' if initial<0 else 'plus'}_{abs(initial):g}"
    def load(name,kind): return read(run/name/f"{kind}.json.gz")
    obs0,obs1=load(baseline,"observations"),load(adaptive,"observations")
    trace0,trace1=load(baseline,"trace"),load(adaptive,"trace")
    terminal0,terminal1=read(run/baseline/"terminal.json"),read(run/adaptive/"terminal.json")
    events=read(run/adaptive/"events.json")
    moves=[e for e in events if e["status"]=="transition_accepted"]
    output.mkdir(parents=True)
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.titlesize":13,
        "axes.labelsize":11,"axes.labelcolor":INK,"text.color":INK,"xtick.color":INK,"ytick.color":INK,
        "axes.edgecolor":"#BAC3C8","axes.spines.top":False,"axes.spines.right":False,
        "axes.titleweight":"bold","figure.facecolor":"white","axes.facecolor":"white",
        "savefig.facecolor":"white","svg.fonttype":"none","svg.hashsalt":"uam-stage1-v1"})
    footer=(f"{run.name} | Millbrae–Santa Clara access proxies | h={cfg['altitude_m']:g} m | "
            f"V tangent={cfg['cruise_mps']:g} m/s | Θ={cfg['threshold_db']:g} dB | singleton exposure; no capacity")
    provenance=[]
    def save(fig,name,description):
        fig.text(.06,.018,footer,fontsize=8,color=GRAY)
        for ext in ("png","svg"):
            path=output/f"{name}.{ext}"
            fig.savefig(path,dpi=210,bbox_inches="tight",pad_inches=.15,
                        metadata={"Description":description} if ext=="svg" else {"Description":description})
            provenance.append({"path":path.name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"description":description})
        plt.close(fig)

    # 1. Scientific cross-section plot, not an invented geographic basemap.
    field=np.load(run/"field.npz")
    q=field["q_m"]/1000;d=field["offsets_m"];z=field["sinr_db"]
    fig,ax=plt.subplots(figsize=(12.4,5.1))
    fig.subplots_adjust(left=.085,right=.88,bottom=.27,top=.84)
    low=min(float(z.min()),cfg["threshold_db"]-.1)
    high=max(float(z.max()),cfg["threshold_db"]+.1)
    norm=TwoSlopeNorm(vmin=low,vcenter=cfg["threshold_db"],vmax=high)
    mesh=ax.pcolormesh(q,d,z,shading="nearest",cmap="RdBu",norm=norm,rasterized=True)
    ax.contour(q,d,z,levels=[cfg["threshold_db"]],colors="#574741",linewidths=.7,alpha=.55)
    ax.plot([0,q[-1]],[initial,initial],color="#313B44",ls="--",lw=1.6,label="No change: d=0 m")
    x=np.r_[[r["q_m"] for r in trace1],terminal1["q_m"]]/1000
    y=np.r_[[r["offset_m"] for r in trace1],terminal1["offset_m"]]
    ax.plot(x,y,color="#F4BD3C",lw=2.8,label=f"Adaptive: ±{width:g} m",path_effects=[pe.Stroke(linewidth=4.5,foreground=INK),pe.Normal()])
    for k,e in enumerate(moves,1):
        ax.plot(e["q_m"]/1000,e["source_m"],"o",ms=7,color="#F4BD3C",mec=INK)
        # Event index is intentionally short; full coordinates are in the report.
        ax.annotate(str(k),(e["q_m"]/1000,e["source_m"]),xytext=(0,-19 if e["source_m"]>=.8*width else 10),textcoords="offset points",ha="center",fontsize=9,
                    bbox={"facecolor":"white","edgecolor":"none","alpha":.85,"pad":1})
    ax.set(xlabel="Reference-route progress (km)",ylabel="Signed lateral offset (m)",xlim=(0,q[-1]),ylim=(d[0]-50,d[-1]+50))
    ax.set_yticks(d[::2] if len(d)>9 else d)
    ax.set_title("Where the radio map supports lateral movement",loc="left",pad=40)
    ax.legend(loc="lower left",bbox_to_anchor=(0,1.01),ncol=2,frameon=False,fontsize=9)
    cax=fig.add_axes([.90,.28,.018,.55]);cb=fig.colorbar(mesh,cax=cax)
    cb.set_label("SINR (dB); color midpoint = Θ")
    # Omit zero: on this centered nonlinear color scale it is too close to Θ.
    color_ticks=[low,cfg["threshold_db"],*np.arange(5,high,5)]
    cb.set_ticks(color_ticks)
    cb.set_ticklabels([f"{v:.1f}" if k<2 else f"{v:g}" for k,v in enumerate(color_ticks)])
    cb.ax.tick_params(labelsize=9)
    fig.text(.085,.095,"Positive offset = left in the direction of travel. Numbered dots mark accepted move starts; thin contours mark Θ.",fontsize=9,color=GRAY)
    save(fig,"01-radio-landscape","Static SINR at 100 m offset candidates, sampled along the smooth reference; paired adaptive and zero-offset trajectories.")

    # 2. Same geometric coverage, actual (different) motion times retained in run.
    fig,axes=plt.subplots(3,1,figsize=(12.4,8.4),sharex=True,gridspec_kw={"height_ratios":[2,1.7,1.1]})
    fig.subplots_adjust(left=.085,right=.96,bottom=.18,top=.91,hspace=.33)
    for rows,label,color,ls in [(obs0,"No change",GRAY,"--"),(obs1,f"Adaptive ±{width:g} m",TEAL,"-")]:
        x=np.array([r["q_m"] for r in rows])/1000
        axes[0].plot(x,[r["sinr_db"] for r in rows],label=label,color=color,lw=1.5,ls=ls)
        axes[1].step(x,[r["exposure"] for r in rows],where="post",label=label,color=color,lw=1.5,ls=ls)
    axes[0].axhline(cfg["threshold_db"],color=RUST,lw=1,ls=":",label=f"Θ={cfg['threshold_db']:g} dB")
    axes[0].set_ylabel("SINR (dB)");axes[0].set_title("A  Paired communication profiles",loc="left")
    axes[0].legend(loc="upper right",frameon=False,ncol=3,fontsize=9)
    axes[1].axhline(cfg["exposure_c"],color=COLORS["C"],lw=1,ls=":")
    axes[1].axhline(cfg["exposure_r"],color=COLORS["R"],lw=1,ls="--")
    axes[1].set(ylabel="Exposure ζ",ylim=(-.02,1.02));axes[1].set_title("B  Available 30 s history; one-aircraft group",loc="left")
    axes[1].text(.99,.96,"C: ζ ≤ 0.05     R: 0.05 < ζ ≤ 0.10     F: ζ > 0.10",transform=axes[1].transAxes,ha="right",va="top",fontsize=9)
    for y,trace,terminal,label in [(1,trace0,terminal0,"No change"),(0,trace1,terminal1,"Adaptive")]:
        progress=np.r_[[r["q_m"] for r in trace],terminal["q_m"]]/1000
        for p in "CRF":
            ranges=[(progress[k],progress[k+1]-progress[k]) for k,r in enumerate(trace) if r["policy"]==p]
            axes[2].broken_barh(ranges,(y-.28,.56),facecolors=COLORS[p],edgecolors="none",rasterized=True)
    axes[2].set(yticks=[0,1],yticklabels=["Adaptive","No change"],ylim=(-.5,1.5),xlabel="Reference-route progress (km)",xlim=(0,q[-1]))
    axes[2].set_title("C  Realized policy along each flight",loc="left")
    from matplotlib.patches import Patch
    axes[2].legend(handles=[Patch(facecolor=COLORS[p],label=p) for p in "CRF"],loc="upper right",bbox_to_anchor=(1,1.45),ncol=3,frameon=False,fontsize=9)
    for ax in axes[:2]:ax.grid(axis="y",alpha=.18)
    fig.text(.085,.062,"R is absent here: with one aircraft and at most 7 samples, one bad sample already gives ζ ≥ 1/7 > 0.10.",fontsize=9,color=GRAY)
    save(fig,"02-link-exposure-policy","Paired radio, singleton available-history exposure and policy along full corridor; policy shares are time-weighted, not capacity.")

    # 3. Independently optimized local decisions need not be monotone in width.
    adaptive_rows=[r for r in summaries if r["kind"]=="adaptive"]
    W=np.array([max(abs(x) for x in r["allowed_offsets_m"]) for r in adaptive_rows])
    F=np.array([100*r["policy_time_shares"]["F"] for r in adaptive_rows])
    counts=np.array([r["completed_moves"] for r in adaptive_rows])
    move_time=np.array([r["total_maneuver_time_s"] for r in adaptive_rows])
    fig,axes=plt.subplots(1,3,figsize=(12.4,4.6))
    fig.subplots_adjust(left=.07,right=.975,bottom=.22,top=.80,wspace=.34)
    fig.suptitle("What changes when more lateral space is available?",x=.07,ha="left",fontsize=15,fontweight="bold")
    for ax,values,title,unit in zip(axes,[F,counts,move_time],
            ["Fallback time share","Completed transitions","Time spent maneuvering"],["F share (%)","Count","Duration (s)"]):
        ax.plot(W,values,"o-",color=TEAL,lw=1.8,ms=6)
        ax.set(title=title,xlabel="Allowed half-width (m)",ylabel=unit,xticks=W)
        ax.grid(axis="y",alpha=.18)
        ax.margins(x=.12,y=.27)
        ax.set_ylim(bottom=0)
        for x,y in zip(W,values):
            ax.annotate(f"{y:.1f}" if unit!="Count" else f"{int(y)}",(x,y),xytext=(0,7),textcoords="offset points",ha="center",fontsize=9)
    base_f=100*by_id[baseline]["policy_time_shares"]["F"]
    axes[0].axhline(base_f,color=GRAY,ls="--",lw=1.2)
    axes[0].set_ylim(0,max(base_f,float(F.max()))*1.3)
    axes[0].text(.98,.96,f"No change: {base_f:.1f}%",transform=axes[0].transAxes,ha="right",va="top",fontsize=9,color=GRAY)
    axes[1].yaxis.set_major_locator(MaxNLocator(integer=True))
    fig.text(.07,.092,"Every adaptive flight starts at d=0 m; candidates are 100 m apart. More moves are not automatically better.",fontsize=9,color=GRAY)
    save(fig,"03-envelope-tradeoff","Matched zero-offset initialization and fixed 100 m candidate resolution; adaptive F time share, move count and maneuver duration across allowed envelopes.")

    # 4. All static offsets, kept distinct from the matched initial-state pair.
    static=[r for r in summaries if r["kind"]=="static"]
    D=np.array([r["initial_offset_m"] for r in static])
    fig,axes=plt.subplots(2,1,figsize=(12.4,6.4),sharex=True,gridspec_kw={"height_ratios":[1.6,1]})
    fig.subplots_adjust(left=.08,right=.96,bottom=.23,top=.89,hspace=.4)
    bottom=np.zeros(len(static))
    for p in "CRF":
        values=np.array([r["policy_time_shares"][p] for r in static])*100
        axes[0].bar(D,values,bottom=bottom,width=65,color=COLORS[p],label=p)
        if p=="F":
            for x,b,v in zip(D,bottom,values):
                if v>3:axes[0].text(x,b+v/2,f"{v:.1f}",ha="center",va="center",fontsize=9,color="white")
        bottom+=values
    axes[0].set(ylabel="Policy time share (%)",ylim=(0,103));axes[0].set_title("Static-offset context: where would a fixed lane perform better?",loc="left")
    axes[0].legend(frameon=False,ncol=3,loc="upper right",bbox_to_anchor=(1,1.23))
    exposures=[r["time_mean_exposure"] for r in static]
    axes[1].plot(D,exposures,"o-",color=TEAL,lw=1.6)
    axes[1].set(xlabel="Fixed lateral offset (m)",ylabel="Time-mean exposure ζ",xticks=D,ylim=(0,max(exposures)*1.25))
    axes[1].grid(axis="y",alpha=.18)
    axes[1].axvline(initial,color=GRAY,ls=":",lw=1)
    fig.text(.08,.075,"Each bar is a separate singleton flight assigned to that offset. These are not simultaneous lanes or matched departure assignments.",fontsize=9,color=GRAY)
    save(fig,"04-static-offsets","Independent static flights at each offset, C/R/F time shares and time-mean exposure. Not simultaneous traffic or capacity.")
    figure_manifest={"run":str(run),"run_manifest_sha256":hashlib.sha256((run/"manifest.json").read_bytes()).hexdigest(),
        "plot_script_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "matplotlib_version":matplotlib.__version__,"numpy_version":np.__version__,
        "figures":provenance,"note":"All marks derive from archived numerical data; no synthetic illustration or measured-flight claim."}
    with zipfile.ZipFile(output/"plot_source.zip","w",zipfile.ZIP_DEFLATED) as archive:
        archive.write(Path(__file__),"plot_lateral_study.py")
    figure_manifest["plot_source_archive_sha256"]=hashlib.sha256((output/"plot_source.zip").read_bytes()).hexdigest()
    (output/"manifest.json").write_text(json.dumps(figure_manifest,indent=2)+"\n")
    print(json.dumps({"output":str(output),"figures":len(provenance)},indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    plot(args.run,args.output)
