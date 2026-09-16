"""Plot archived two-UAM policy recovery, with measured rather than assumed outcomes.

Three panels per direction: speed, actual/instantaneous/equilibrium gaps, and
commanded acceleration. No simulation is performed by this script.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ACC_COLOUR, AKS_COLOUR, GREY = '#b54b3c', '#176b87', '#777777'
TRANSITIONS = [
    ('open_c_to_r', 'C → R', 'R', 750),
    ('open_r_to_f', 'R → F', 'F', 1500),
    ('open_c_to_f', 'C → F', 'F', 2250),
    ('close_r_to_c', 'R → C', 'C', -750),
    ('close_f_to_r', 'F → R', 'R', -1500),
    ('close_f_to_c', 'F → C', 'C', -2250),
]
VARIANTS = [
    ('oversized_gap_hold', 'sufficient gap; no established pair', 'F', 0),
    ('close_without_headroom', 'closing; 50 m/s ceiling', 'C', -2250),
    ('leader_brakes_from_large_gap', 'leader brakes; initially sufficient gap', 'F', 0),
    ('leader_brakes_during_opening', 'leader brakes during opening', 'F', 2250),
    ('open_with_legacy_zero_floor', 'opening; legacy zero-speed floor', 'F', 2250),
    ('open_with_reference_jerk', 'opening; reference jerk bound 0.05', 'F', 2250),
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_trace(run, case, mode, step):
    with gzip.open(run / f'{case}__{mode}__dt{step}.json.gz', 'rt') as stream:
        return json.load(stream)


def series(trace, key):
    return np.array([row[key] for row in trace['trace']], float)


def target_at(speed, spacing, policy):
    return (spacing['d0_m'] + spacing[f'tau_{policy.lower()}_s'] * speed
            + spacing['buffer_s2_per_m'] * speed**2)


def fmt(value, digits=2):
    return 'not reached' if value is None else f'{value:.{digits}f}'


INK = '#1a1a1a'


def style():
    plt.rcParams.update({
        'font.family': 'STIXGeneral', 'mathtext.fontset': 'stix', 'font.size': 11,
        'axes.labelsize': 11.5, 'axes.titlesize': 12.5, 'text.color': INK,
        'axes.labelcolor': INK, 'axes.edgecolor': INK, 'xtick.color': INK,
        'ytick.color': INK, 'axes.linewidth': .7, 'svg.fonttype': 'none',
        'svg.hashsalt': 'uam-two-uam-longitudinal',
        'figure.facecolor': 'white', 'savefig.facecolor': 'white',
        'legend.fontsize': 9,
    })


def mechanism_figure(run, step, case, policy, label, lookup):
    """One policy change, read as second derivative, first derivative, function.

    Panel order is acceleration, speed, gap on purpose: each panel is the
    integral of the one above it, so the reader can follow how a bounded
    acceleration becomes a speed excursion and the speed excursion becomes the
    spacing change.

    The x window is chosen from the dynamics rather than from the observation
    endpoint. Both controllers are observed for T + 1500 s, but the baseline's
    approach is a long exponential tail: plotting all of it compresses every
    curve into the left tenth of the frame. The full timescale is kept, as an
    inset on the gap panel, so nothing is hidden by the choice.
    """
    acc, aks = [read_trace(run, case, mode, step) for mode in ('acc', 'aks')]
    vehicle, spacing = acc['resolved']['vehicle'], acc['resolved']['spacing']
    goal = acc['request']['target_gap_m']
    row_a, row_k = lookup[(case, 'acc')], lookup[(case, 'aks')]
    recovery = acc['resolved']['controller'].get('acc_policy_recovery', False)
    acc_label = 'baseline ACC' + ('' if recovery else ' (cruise limited)')

    t_a, t_k = series(acc, 't_s'), series(aks, 't_s')
    v_a, v_k = series(acc, 'follower_v_mps'), series(aks, 'follower_v_mps')
    g_a, g_k = series(acc, 'gap_m'), series(aks, 'gap_m')
    d_a, d_k = series(acc, 'acceleration_mps2'), series(aks, 'acceleration_mps2')
    opening = acc['request']['initial_gap_m'] < goal

    settle_a, settle_k = row_a['settling_time_s'], row_k['settling_time_s']
    window = max(2.0 * (settle_k or 0.0), 1.5 * row_k['duration_s'])
    window = float(np.ceil(window / 50.0) * 50.0)
    cruise = vehicle['cruise_mps']
    floor, ceiling = vehicle['speed_min_mps'], vehicle['speed_max_mps']
    brake, push = vehicle['deceleration_limit_mps2'], vehicle['acceleration_limit_mps2']

    fig, axes = plt.subplots(3, 1, figsize=(8.8, 11.4), sharex=True,
                             gridspec_kw={'hspace': .34})
    fig.text(.085, .978, label, fontsize=14.5, ha='left', va='top', color=INK)
    fig.text(.085, .955,
             f'{run.name} \u00b7 leader {cruise:g} m/s \u00b7 both controllers under the same '
             f'{floor:g}\u2013{ceiling:g} m/s envelope and \u2212{brake:g}/+{push:g} m/s\u00b2 bounds',
             fontsize=9.5, color=GREY, va='top')

    # ---- (a) acceleration: the second derivative, and the bound each controller
    #      actually spends
    ax = axes[0]
    ax.axhline(0, color=INK, lw=.6)
    # The baseline saturates one of the two bounds at the start of the run. Put
    # that bound's label at the far end of the axis so the two do not collide.
    for bound, name, side in ((-brake, f'braking limit $-{brake:g}$', 'lower'),
                              (push, f'acceleration limit ${push:g}$', 'upper')):
        ax.axhline(bound, color=GREY, ls=(0, (5, 3)), lw=1.0)
        busy = (side == 'lower') == opening
        ax.annotate(name, (window if busy else 0.0, bound),
                    xytext=(-5 if busy else 5, 5 if side == 'lower' else -13),
                    textcoords='offset points', ha='right' if busy else 'left',
                    fontsize=9, color=GREY)
    ax.plot(t_a, d_a, color=ACC_COLOUR, lw=1.7, label=acc_label)
    ax.plot(t_k, d_k, color=AKS_COLOUR, lw=1.7, label='KS2 reference')
    ax.set_ylim(-brake - .28, push + .28)
    ax.set_ylabel('commanded acceleration  (m s$^{-2}$)')
    ax.set_title('(a) what each controller commands', loc='left', pad=7)
    ax.legend(frameon=False, loc='upper right' if opening else 'lower right',
              bbox_to_anchor=(1.0, .88) if opening else (1.0, .12))

    saturated = t_a[(d_a <= -brake + 1e-9) | (d_a >= push - 1e-9)]
    if saturated.size:
        ax.axvspan(saturated[0], saturated[-1], color=ACC_COLOUR, alpha=.12, lw=0)
        held = saturated[-1] - saturated[0]
        edge = -brake if d_a.min() <= -brake + 1e-9 else push
        ax.annotate(f'the baseline spends its whole acceleration budget\n'
                    f'in the first {held:.0f} s, then coasts in on a long tail',
                    (saturated[-1], edge), xytext=(16, 26 if edge < 0 else -34),
                    textcoords='offset points', fontsize=9, color=ACC_COLOUR)
    peak_k = max(abs(d_k.min()), abs(d_k.max()))
    peak_a = max(abs(d_a.min()), abs(d_a.max()))
    index = np.argmin(d_k) if abs(d_k.min()) >= abs(d_k.max()) else np.argmax(d_k)
    budget = brake if d_k[index] < 0 else push
    ax.annotate(f'the reference peaks at {peak_k:.2f}, '
                f'{100 * peak_k / budget:.0f}% of the same bound',
                (t_k[index], d_k[index]), xytext=(14, -18 if d_k[index] < 0 else 16),
                textcoords='offset points', fontsize=9, color=AKS_COLOUR)

    # ---- (b) speed: the first derivative, and the other budget
    ax = axes[1]
    ax.plot(t_a, series(acc, 'leader_v_mps'), color=GREY, ls=(0, (5, 3)), lw=1.1,
            label='leader')
    ax.plot(t_a, v_a, color=ACC_COLOUR, lw=1.7, label=acc_label)
    ax.plot(t_k, v_k, color=AKS_COLOUR, lw=1.7, label='KS2 reference')
    edge = floor if opening else ceiling
    ax.axhline(edge, color=GREY, ls=(0, (1, 2)), lw=1.0)
    ax.annotate(f'speed {"floor" if opening else "ceiling"} {edge:g}',
                (window, edge), xytext=(-4, 5 if opening else -13),
                textcoords='offset points', ha='right', fontsize=9, color=GREY)
    # Scale to what the run actually used, not to the whole declared envelope:
    # an opening case never goes above cruise, and the empty half hides the curve.
    low = min(v_a.min(), v_k.min(), edge)
    high = max(v_a.max(), v_k.max(), cruise)
    ax.set_ylim(low - .16 * (high - low) - 1, high + .22 * (high - low) + 1)
    ax.set_ylabel('speed  (m s$^{-1}$)')
    ax.set_title('(b) the speed excursion that acceleration integrates to',
                 loc='left', pad=7)
    ax.legend(frameon=False, loc='lower right' if opening else 'upper right', ncol=3)
    reach_a = v_a.min() if opening else v_a.max()
    reach_k = v_k.min() if opening else v_k.max()
    margin = abs(cruise - edge)
    ax.annotate(f'reference takes the full margin: {reach_k:.0f} m/s',
                (t_k[np.argmin(v_k) if opening else np.argmax(v_k)], reach_k),
                xytext=(14, 12 if opening else -16), textcoords='offset points',
                fontsize=9, color=AKS_COLOUR)
    # Both curves converge on cruise in the right-hand third, leaving that band
    # free; anchoring on the extremum itself puts the text across the other curve.
    ax.annotate(f'baseline stops at {reach_a:.1f}, using only '
                f'{100 * abs(cruise - reach_a) / margin:.0f}% of the margin',
                (.98, .33), xycoords='axes fraction', ha='right', fontsize=9,
                color=ACC_COLOUR)

    # ---- (c) gap: the function itself
    ax = axes[2]
    ax.axhline(goal, color=GREY, ls=(0, (5, 3)), lw=1.1)
    ax.annotate(f'{policy} target {goal:g} m', (window, goal), xytext=(-6, 6),
                textcoords='offset points', ha='right', fontsize=9, color=GREY)
    ax.plot(t_a, target_at(v_a, spacing, policy), color=ACC_COLOUR, lw=1.1, ls=':',
            label=f'what the baseline aims at,  $S(\\mathrm{{{policy}}},\\,v_F)$')
    ax.plot(t_a, g_a, color=ACC_COLOUR, lw=1.7, label=acc_label)
    ax.plot(t_k, g_k, color=AKS_COLOUR, lw=1.7, label='KS2 reference')
    ax.set_ylabel('gap to leader  (m)')
    ax.set_xlabel('time since the policy change  (s)')
    ax.set_title('(c) the spacing, which the speed excursion integrates to',
                 loc='left', pad=7)
    # The inset occupies the corner the curves leave free, so the legend takes
    # the other one.
    ax.legend(frameon=False, loc='upper left' if opening else 'lower left')
    if settle_k is not None and settle_k <= window:
        ax.axvline(settle_k, color=AKS_COLOUR, lw=.9, ls=':')
        # Bottom of the frame when the gap opens; when it closes the reference
        # already sits there, so the label moves into the band between the curves.
        ax.annotate(f'reference settled {settle_k:.0f} s',
                    (settle_k, 0.0 if opening else .22),
                    xycoords=('data', 'axes fraction'), xytext=(-6, 7),
                    textcoords='offset points', va='bottom', ha='right',
                    fontsize=9, color=AKS_COLOUR)

    # The full observation window, kept rather than cropped away.
    box = [.50, .17, .46, .38] if opening else [.50, .55, .46, .38]
    inset = ax.inset_axes(box)
    inset.plot(t_a, g_a, color=ACC_COLOUR, lw=1.1)
    inset.plot(t_k, g_k, color=AKS_COLOUR, lw=1.1)
    inset.axhline(goal, color=GREY, ls=(0, (4, 3)), lw=.8)
    for moment, colour in ((settle_a, ACC_COLOUR), (settle_k, AKS_COLOUR)):
        if moment is not None:
            inset.axvline(moment, color=colour, lw=.8, ls=':')
    inset.axvspan(0, window, color=INK, alpha=.06, lw=0)
    inset.set_xlim(0, t_a[-1])
    inset.tick_params(labelsize=7.5, length=2.5)
    inset.set_title(f'the whole {t_a[-1]:.0f} s observation; shaded band is the '
                    f'frame above', fontsize=8, loc='left', pad=3, color=INK)
    inset.spines[['top', 'right']].set_visible(False)
    if settle_a is not None:
        # The target line sits at the top of the inset when the gap opens and at
        # the bottom when it closes, so the label goes to the free side.
        inset.annotate(f'baseline settles {settle_a:.0f} s', (settle_a, goal),
                       xytext=(4, -14 if opening else 8), textcoords='offset points',
                       fontsize=7.5, color=ACC_COLOUR)

    for ax in axes:
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', lw=.4, color='#d8d8d8')
        ax.set_axisbelow(True)
        ax.set_xlim(0, window)

    ratio = settle_a / settle_k if settle_a and settle_k else None
    fig.text(.085, .052,
             f'Sampled settling: baseline {fmt(settle_a, 1)} s, reference '
             f'{fmt(settle_k, 1)} s'
             + (f' \u2014 the reference is {ratio:.1f}\u00d7 faster.' if ratio else '.'),
             fontsize=9.5, va='top')
    fig.text(.085, .032,
             'Same settling criterion for both: |gap error| \u2264 1 m and '
             '|speed error| \u2264 0.05 m/s, held for the rest of the trace.',
             fontsize=9, color=GREY, va='top')
    fig.text(.085, .015,
             f'Final gap error: baseline {row_a["final_gap_error_m"]:+.3f} m, '
             f'reference {row_k["final_gap_error_m"]:+.3f} m.  dt = {step} s.  '
             f'Uncalibrated declared gains; two aircraft; no capacity claim.',
             fontsize=9, color=GREY, va='top')
    fig.subplots_adjust(top=.900, bottom=.105, left=.115, right=.965)
    return fig


def results_table(summary, output):
    lookup = {(r['case_id'], r['mode']): r for r in summary['results']}
    rows = []
    for kind, entries in [('transition', TRANSITIONS), ('variant', VARIANTS)]:
        for case, label, policy, delta in entries:
            for mode in ('acc', 'aks'):
                r = lookup[(case, mode)]
                rows.append(dict(group=kind, case=label, controller=mode,
                    delta_spacing_m=delta, initial_request=r['request_status'],
                    reference_duration_s=r['duration_s'],
                    observation_end_s=r['observation_end_s'],
                    sampled_settling_s=r['settling_time_s'],
                    gap_error_at_reference_end_m=r['gap_error_at_reference_end_m'],
                    gap_error_at_T_plus_120_m=r['gap_error_at_legacy_horizon_m'],
                    final_gap_m=r['final_gap_m'],
                    final_equilibrium_error_m=r['final_gap_error_m'],
                    final_dynamic_error_m=r['final_instantaneous_gap_error_m'],
                    final_speed_mps=r['final_speed_mps'],
                    speed_min_mps=r['minimum_speed_mps'], speed_max_mps=r['maximum_speed_mps'],
                    peak_acceleration_mps2=r['maximum_acceleration_mps2'],
                    peak_braking_mps2=r['minimum_acceleration_mps2'], dt_s=r['dt_s']))
    with (output / 'results-table.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    lines = ['# Two-UAM policy recovery results', '',
        'T is the KS2 reference duration, not the ACC completion time. Both methods are observed until T + 1500 s.', '',
        'Sampled settling requires both |gap − S(policy, 50)| ≤ 1 m and |v − 50| ≤ 0.05 m/s for the remainder of the saved trace. Not reached is censored at the observation endpoint.', '',
        'The initial planner request does not describe the complete outcome of a leader-braking case; those cases can invalidate the reference and use feedback fallback.', '']
    for kind in ('transition', 'variant'):
        lines += [f'## {kind.title()} cases', '',
                  '| Case | Method | T (s) | Settling (s) | Gap error at T+120 (m) | Final gap error (m) | Speed min/max (m/s) | Accel min/max (m/s²) |',
                  '|---|---|---:|---:|---:|---:|---|---|']
        for r in rows:
            if r['group'] != kind:
                continue
            lines.append(f'| {r["case"]} | {r["controller"]} | {r["reference_duration_s"]:.1f} | '
                f'{fmt(r["sampled_settling_s"],1)} | {r["gap_error_at_T_plus_120_m"]:+.2f} | '
                f'{r["final_equilibrium_error_m"]:+.3f} | '
                f'{r["speed_min_mps"]:.2f} / {r["speed_max_mps"]:.2f} | '
                f'{r["peak_braking_mps2"]:.3f} / {r["peak_acceleration_mps2"]:.3f} |')
        lines.append('')
    lines += ['The CSV also records the reference-endpoint error, final instantaneous-policy error, final speed and observation endpoint. All final errors above use the common 50 m/s equilibrium target, not the moving ACC target.', '',
              'A sufficient unbound gap correctly stays oversized; the ceiling-limited refusal cannot close. Neither should be counted as a failed nominal recovery. Leader-braking variants are diagnostics, not constant-leader controller rankings.', '']
    (output / 'results-table.md').write_text('\n'.join(lines))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--step', default='0.03125')
    args = parser.parse_args()
    validation = json.loads((args.run / 'validation.json').read_text())
    if validation['status'] != 'passed':
        raise ValueError('refusing to plot failed validation')
    summary = json.loads((args.run / 'summary.json').read_text())
    lookup = {(r['case_id'], r['mode']): r for r in summary['results']}
    if any(r['dt_s'] != float(args.step) for r in lookup.values()):
        raise ValueError('summary and requested plot step differ')
    if args.output.exists():
        raise FileExistsError('use a new figure directory')
    args.output.mkdir(parents=True)
    style()
    captions = {}
    for name, case, policy, title in [
        ('01-policy-degradation-C-to-F', 'open_c_to_f', 'F', 'C → F: open the gap'),
        ('02-policy-upgrade-F-to-C', 'close_f_to_c', 'C', 'F → C: restore the compact gap')]:
        fig = mechanism_figure(args.run, args.step, case, policy, title, lookup)
        ra, rk = lookup[(case,'acc')], lookup[(case,'aks')]
        caption = (f'{args.run.name}, {case}. Both methods have an established follower relationship and the same speed and acceleration bounds. '
                   f'ACC sampled settling {fmt(ra["settling_time_s"],1)} s; AKS {fmt(rk["settling_time_s"],1)} s. '
                   'ACC tracks the instantaneous policy spacing; KS2 tracks a planned trajectory to the same final equilibrium. '
                   'Gains are unchanged and uncalibrated. No capacity or general safety claim.')
        for ext in ('png','svg'):
            fig.savefig(args.output / f'{name}.{ext}', dpi=220, metadata={'Description':caption})
        plt.close(fig)
        captions[name] = caption
    table = results_table(summary,args.output)
    (args.output/'plot_source.py').write_bytes(Path(__file__).read_bytes())
    record = {'run':str(args.run.resolve()), 'step_s':float(args.step), 'table_rows':len(table),
              'run_manifest_sha256':digest(args.run/'manifest.json'),
              'validation_status':validation['status'], 'captions':captions,
              'outputs':[{'path':p.name,'sha256':digest(p)} for p in sorted(args.output.iterdir())]}
    (args.output/'manifest.json').write_text(json.dumps(record,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(record,indent=2,ensure_ascii=False))


if __name__ == '__main__':
    main()
