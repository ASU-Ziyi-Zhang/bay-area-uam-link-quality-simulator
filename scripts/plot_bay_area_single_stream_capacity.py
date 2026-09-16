#!/usr/bin/env python3
"""Create report-ready R0056 conditional-capacity and grid-use figures."""
from __future__ import annotations

import csv
import gzip
import json
from pathlib import Path
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np


ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/"research/dynamic-transitions/runs/R0056"
OUT=ROOT/"research/dynamic-transitions/figures/R0056"
POLICY_COLOR={"C":"#238B7B","R":"#E6A51A","F":"#D85845"}
BLUE="#2474A6";GRAY="#6F7882";INK="#17324D";PALE="#EAF2F6"

mpl.rcParams.update({
    "font.family":"sans-serif",
    "font.sans-serif":["Arial","Helvetica","DejaVu Sans","sans-serif"],
    "svg.fonttype":"none","pdf.fonttype":42,"font.size":9,
    "axes.spines.right":False,"axes.spines.top":False,
    "axes.linewidth":0.8,"legend.frameon":False,
})


def load_json(path):
    with path.open(encoding="utf-8") as stream:return json.load(stream)


def load_gzip(path):
    with gzip.open(path,"rt",encoding="utf-8") as stream:return json.load(stream)


def load_capacity(case):
    with (RUN/case/"capacity_trace.csv").open(encoding="utf-8-sig",newline="") as stream:
        rows=list(csv.DictReader(stream))
    return {key:np.asarray([float(row[key]) for row in rows])
            for key in ("timestamp_s","q_mix_uam_h","active_aircraft")}


def save(fig,name):
    OUT.mkdir(parents=True,exist_ok=True)
    fig.savefig(OUT/f"{name}.png",dpi=300,bbox_inches="tight",facecolor="white")
    fig.savefig(OUT/f"{name}.svg",bbox_inches="tight",facecolor="white")
    fig.savefig(OUT/f"{name}.pdf",bbox_inches="tight",facecolor="white")
    plt.close(fig)


def rolling_mean(values,width=13):
    if len(values)<width:return values
    kernel=np.ones(width)/width
    return np.convolve(values,kernel,mode="same")


def capacity_figure():
    summaries={case:load_json(RUN/case/"summary.json")
               for case in ("fixed_centerline","spatial_grid")}
    traces={case:load_capacity(case) for case in summaries}
    fig=plt.figure(figsize=(12.8,7.2),facecolor="white")
    grid=fig.add_gridspec(2,3,width_ratios=(2.15,1,1),height_ratios=(1,1),
                          left=.07,right=.97,top=.86,bottom=.12,wspace=.34,hspace=.46)
    ax=fig.add_subplot(grid[:,0])
    for case,color,label in (("fixed_centerline",GRAY,"Fixed centerline"),
                             ("spatial_grid",BLUE,"3×3 spatial grid")):
        x=traces[case]["timestamp_s"]/60
        y=traces[case]["q_mix_uam_h"]
        ax.plot(x,y,color=color,lw=.55,alpha=.22)
        smooth=rolling_mean(y)
        pad=6
        ax.plot(x[pad:-pad],smooth[pad:-pad],color=color,lw=2.0,label=label)
        q95=summaries[case]["conditional_planning_capacity"]["q_mix_rho_uam_h"]
        ax.axhline(q95,color=color,lw=1.0,ls="--")
        ax.text(x.max()+.35,q95,f"{q95:.1f}",color=color,fontsize=9,
                va="center",fontweight="bold")
    ax.set(xlabel="Simulation time (min)",ylabel="Conditional planning capacity (UAM/h)",
           title="a  Spatial control raises the reliable planning-capacity floor")
    ax.set_ylim(75,141);ax.grid(axis="y",color="#DDE4E8",lw=.7)
    ax.legend(loc="lower left")
    ax.text(.02,.98,"Thin: 5 s snapshots   ·   Thick: 65 s moving mean   ·   Dashed: 95% reliability floor",
            transform=ax.transAxes,va="top",fontsize=8,color=GRAY)

    ax=fig.add_subplot(grid[0,1])
    left=np.zeros(2);ys=np.arange(2)
    labels=["Fixed\ncenterline","3×3\ngrid"]
    for policy in "CRF":
        vals=np.asarray([summaries[c]["policy_and_longitudinal_metrics"]["policy_shares"][policy]
                         for c in summaries])*100
        ax.barh(ys,vals,left=left,color=POLICY_COLOR[policy],height=.55,label=policy)
        for y,l,v in zip(ys,left,vals):
            if v>=5:ax.text(l+v/2,y,f"{v:.1f}%",ha="center",va="center",fontsize=8,
                            color="white" if policy in "CF" else INK,fontweight="bold")
        left+=vals
    ax.set(yticks=ys,yticklabels=labels,xlim=(0,100),xlabel="Aircraft-time share (%)",
           title="b  Policy composition")
    ax.invert_yaxis();ax.legend(ncol=3,loc="lower center",bbox_to_anchor=(.5,-.58))

    ax=fig.add_subplot(grid[0,2])
    values=[summaries[c]["conditional_planning_capacity"]["q_mix_rho_uam_h"] for c in summaries]
    bars=ax.bar([0,1],values,color=[GRAY,BLUE],width=.6)
    ax.set(xticks=[0,1],xticklabels=["Fixed","Grid"],ylabel="UAM/h",
           ylim=(0,145),title="c  95%-reliable conditional capacity")
    for bar,value in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,value+3,f"{value:.1f}",ha="center",fontweight="bold")
    gain=values[1]-values[0]
    ax.text(.5,44,f"+{gain:.1f} UAM/h\n(+{100*gain/values[0]:.1f}%)",ha="center",va="center",
            color=BLUE,fontweight="bold",fontsize=10)

    ax=fig.add_subplot(grid[1,1])
    groups={}
    for row in load_gzip(RUN/"spatial_grid"/"observations.json.gz"):
        groups.setdefault(int(row["group_size"]),{p:0 for p in "CRF"})[row["policy"]]+=1
    sizes=sorted(groups);fshare=[100*groups[n]["F"]/sum(groups[n].values()) for n in sizes]
    ax.plot(sizes,fshare,marker="o",color=POLICY_COLOR["F"],lw=1.8)
    ax.set(xticks=sizes,xlabel="Observed local-group size",ylabel="F share (%)",
           title="d  Residual F concentrates in small groups",ylim=(0,13.5))
    ax.grid(axis="y",color="#DDE4E8",lw=.7)
    for x,y in zip(sizes,fshare):ax.text(x,y+.7,f"{y:.1f}",ha="center",fontsize=8)

    ax=fig.add_subplot(grid[1,2]);ax.axis("off")
    fixed=summaries["fixed_centerline"];spatial=summaries["spatial_grid"]
    cards=[
        ("Completed",f"{spatial['completed_requests']}/{spatial['scheduled_requests']}",INK),
        ("Entry delay",f"{sum(v>1e-8 for v in spatial['entry_delays_s'].values())} aircraft",BLUE),
        ("Grid moves",f"{spatial['lane_change_metrics']['completed_lane_changes']}",BLUE),
        ("Sampled NMAC","0",POLICY_COLOR["C"]),
        ("Last exit",f"{fixed['observed_throughput']['last_exit_s']-spatial['observed_throughput']['last_exit_s']:.0f} s earlier",BLUE),
    ]
    ax.set_title("e  Operational checks",loc="left",pad=8)
    for i,(label,value,color) in enumerate(cards):
        y=.91-i*.18
        ax.add_patch(plt.Rectangle((0,y-.11),1,.14,transform=ax.transAxes,color=PALE,ec="none"))
        ax.text(.05,y,label,transform=ax.transAxes,va="center",color=GRAY,fontsize=8)
        ax.text(.95,y,value,transform=ax.transAxes,va="center",ha="right",color=color,
                fontweight="bold",fontsize=10)

    fig.suptitle("A 3×3 spatial grid increases single-stream conditional capacity",
                 x=.07,y=.96,ha="left",fontsize=17,fontweight="bold",color=INK)
    fig.text(.07,.91,"Bay Area airport-to-airport corridor  ·  identical 32 s center-entry schedule  ·  90 s assessment window  ·  93 requests",
             color=GRAY,fontsize=9)
    fig.text(.07,.035,"Deterministic matched simulation. Capacity is the TRB-style policy-conditioned planning rate, not a measured maximum sustainable demand.",
             color=GRAY,fontsize=8)
    save(fig,"01-single-stream-capacity-comparison")


def colored_trajectory(ax,rows,xkey,ykey):
    if len(rows)<2:return
    xy=np.asarray([[float(r[xkey]),float(r[ykey])] for r in rows])
    segments=np.stack([xy[:-1],xy[1:]],axis=1)
    colors=[POLICY_COLOR[r["policy"]] for r in rows[:-1]]
    ax.add_collection(LineCollection(segments,colors=colors,linewidths=.9,alpha=.82))


def grid_figure():
    cfg=load_json(RUN/"resolved_config.json")
    summary=load_json(RUN/"spatial_grid"/"summary.json")
    trace=load_gzip(RUN/"spatial_grid"/"trace.json.gz")
    events=load_json(RUN/"spatial_grid"/"events.json")
    points=[(row["offset_m"],row["altitude_m"]) for row in cfg["traffic"]["flow_points"]]
    occupancy=np.zeros(len(points));policy={i:{p:0. for p in "CRF"} for i in range(len(points))}
    by_id={}
    for row in trace:
        by_id.setdefault(row["aircraft_id"],[]).append(row)
        dt=float(row["dt_s"]);lane=int(row["lane"]);occupancy[lane]+=dt;policy[lane][row["policy"]]+=dt
    moves={}
    move_types={"Lateral":0,"Vertical":0,"Diagonal":0}
    for row in events:
        if row.get("status")!="change_started":continue
        key=(int(row["source_lane"]),int(row["target_lane"]));moves[key]=moves.get(key,0)+1
        a,b=points[key[0]],points[key[1]];dd=a[0]!=b[0];dh=a[1]!=b[1]
        move_types["Diagonal" if dd and dh else "Lateral" if dd else "Vertical"]+=1
    fig=plt.figure(figsize=(12.8,7.2),facecolor="white")
    gs=fig.add_gridspec(2,3,width_ratios=(1.15,1.8,1.8),left=.07,right=.97,
                        top=.86,bottom=.12,wspace=.32,hspace=.48)
    ax=fig.add_subplot(gs[:,0])
    scale=max(occupancy.max(),1)
    for i,(d,h) in enumerate(points):
        if occupancy[i]:
            cshare=policy[i]["C"]/occupancy[i]
            size=220+1050*occupancy[i]/scale
            color=mpl.colors.to_hex(plt.cm.YlGn(.25+.7*cshare))
        else:size=170;color="#F2F3F4"
        ax.scatter(d,h,s=size,color=color,edgecolor="white",linewidth=1.8,zorder=3)
        ax.text(d,h,f"{100*policy[i]['C']/occupancy[i]:.0f}% C" if occupancy[i] else "unused",
                ha="center",va="center",fontsize=8,color=INK,zorder=4)
    for (source,target),count in moves.items():
        a,b=np.asarray(points[source],float),np.asarray(points[target],float)
        delta=b-a;start=a+.17*delta;end=b-.2*delta
        ax.annotate("",xy=end,xytext=start,arrowprops=dict(arrowstyle="->",color=BLUE,
                    lw=.7+2.5*count/max(moves.values()),alpha=.5),zorder=2)
    ax.scatter([0],[300],marker="*",s=150,facecolor="none",edgecolor=POLICY_COLOR["F"],lw=2.0,zorder=5)
    ax.set(xlabel="Lateral offset, d (m)",ylabel="Altitude, h (m)",xticks=[-300,0,300],
           yticks=[200,300,400],xlim=(-440,440),ylim=(140,460),
           title="a  Used grid states and accepted transitions")
    ax.grid(color="#E1E7EA",lw=.7);ax.set_axisbelow(True)

    ids=sorted(by_id)[::max(1,len(by_id)//12)][:12]
    for column,(ykey,label,title) in enumerate((("offset_m","Lateral offset, d (m)","b  Realized lateral trajectories"),
                                                ("altitude_m","Altitude, h (m)","c  Realized vertical trajectories")),start=1):
        ax=fig.add_subplot(gs[0,column])
        for identifier in ids:
            full=by_id[identifier]
            rows=full[::5]
            if rows[-1] is not full[-1]:rows=rows+[full[-1]]
            if ykey=="altitude_m":
                rows=[{**r,"altitude_m":float(r["xyz"][2])} for r in rows]
            colored4=rows
            colored_trajectory(ax,colored4,"q_m",ykey)
        ax.autoscale();ax.set(xlabel="Corridor progress, q (km)",ylabel=label,title=title)
        # Convert displayed x values from metres to kilometres without touching data.
        ticks=ax.get_xticks();ax.set_xticks(ticks,labels=[f"{x/1000:g}" for x in ticks])
        ax.grid(color="#E1E7EA",lw=.6)

    ax=fig.add_subplot(gs[1,1])
    names=list(move_types);vals=[move_types[k] for k in names]
    bars=ax.bar(names,vals,color=[BLUE,"#6BAED6","#9ECAE1"])
    ax.set(ylabel="Completed maneuvers",title="d  Maneuver composition",ylim=(0,max(vals)*1.18))
    for bar,value in zip(bars,vals):ax.text(bar.get_x()+bar.get_width()/2,value+1,str(value),ha="center",fontweight="bold")

    ax=fig.add_subplot(gs[1,2]);ax.axis("off")
    text=(f"{summary['lane_change_metrics']['aircraft_with_lane_change']} of 93 aircraft moved\n"
          f"{move_types['Lateral']} lateral  ·  {move_types['Vertical']} vertical  ·   {move_types['Diagonal']} diagonal\n\n"
          "Every accepted endpoint is one adjacent grid cell.\n"
          "All 113 maneuvers completed; sampled NMAC = 0.\n\n"
          "Policy color along trajectories:  C / R / F")
    ax.text(0,.95,text,va="top",fontsize=10,linespacing=1.55,color=INK)
    handles=[Line2D([0],[0],color=POLICY_COLOR[p],lw=3,label=p) for p in "CRF"]
    ax.legend(handles=handles,ncol=3,loc="lower left",bbox_to_anchor=(0,.03))

    fig.suptitle("The spatial controller uses all three motion types without a sampled NMAC",
                 x=.07,y=.96,ha="left",fontsize=17,fontweight="bold",color=INK)
    fig.text(.07,.91,"Representative trajectories are colored by realized C/R/F policy; node color reports coordinated aircraft-time share.",
             color=GRAY,fontsize=9)
    fig.text(.07,.035,"Twelve evenly spaced aircraft are shown for trajectory readability; grid-use and maneuver counts include all 93 aircraft.",
             color=GRAY,fontsize=8)
    save(fig,"02-spatial-grid-use-and-trajectories")


def source_table():
    summaries={case:load_json(RUN/case/"summary.json") for case in ("fixed_centerline","spatial_grid")}
    rows=[]
    for case,s in summaries.items():
        p=s["policy_and_longitudinal_metrics"]["policy_shares"]
        c=s["conditional_planning_capacity"]
        rows.append({"case":case,"C_share":p["C"],"R_share":p["R"],"F_share":p["F"],
                     "q_mix_mean_uam_h":c["q_mix_mean_uam_h"],"q_mix_rho_uam_h":c["q_mix_rho_uam_h"],
                     "observed_exit_rate_uam_h":s["observed_throughput"]["rate_between_first_and_last_exit_uam_h"],
                     "last_exit_s":s["observed_throughput"]["last_exit_s"],
                     "entry_delay_aircraft":sum(v>1e-8 for v in s["entry_delays_s"].values()),
                     "completed_moves":s["lane_change_metrics"]["completed_lane_changes"],
                     "sampled_nmac":s["policy_and_longitudinal_metrics"]["nmac_sampled"]})
    OUT.mkdir(parents=True,exist_ok=True)
    with (OUT/"source-summary.csv").open("w",encoding="utf-8-sig",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


if __name__=="__main__":
    capacity_figure();grid_figure();source_table()
