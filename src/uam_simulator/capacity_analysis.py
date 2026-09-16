"""TRB-style conditional planning capacity from realized policy traces."""
from __future__ import annotations

from collections import defaultdict
import math

import numpy as np

from .group_runner import reliability_floor


def planning_capacity_from_trace(trace, cfg, *, rho=0.95, snapshot_s=None):
    """Evaluate one origin stream; every active aircraft is counted exactly once.

    This is the policy-conditioned planning rate from the TRB spacing law.  It
    is deliberately distinct from measured exit throughput and does not add
    capacity for unoccupied candidate grid cells.
    """
    if not 0 < rho <= 1:
        raise ValueError('rho must lie in (0, 1]')
    period=cfg.policy_s if snapshot_s is None else float(snapshot_s)
    if not np.isfinite(period) or period <= 0:
        raise ValueError('snapshot period must be positive and finite')
    grouped=defaultdict(dict)
    for row in trace:
        timestamp=float(row['t_s'])
        if abs(timestamp/period-round(timestamp/period)) > 1e-8:
            continue
        identifier=row['aircraft_id']
        if identifier in grouped[timestamp]:
            raise ValueError(f'duplicate aircraft at capacity snapshot {timestamp}: {identifier}')
        grouped[timestamp][identifier]=row
    rows=[]
    for timestamp in sorted(grouped):
        active=list(grouped[timestamp].values())
        if not active:
            continue
        policies=[row['policy'] for row in active]
        spacing=np.asarray([cfg.spacing(policy) for policy in policies],float)
        local=3600.*cfg.cruise_mps/spacing
        counts={policy:policies.count(policy) for policy in 'CRF'}
        rows.append({
            'timestamp_s':timestamp,
            'active_aircraft':len(active),
            'n_C':counts['C'],'n_R':counts['R'],'n_F':counts['F'],
            'mean_target_spacing_m':float(spacing.mean()),
            'q_mix_uam_h':float(3600.*cfg.cruise_mps/spacing.mean()),
            'q_bottleneck_uam_h':float(local.min()),
        })
    if not rows:
        raise ValueError('trace contains no eligible capacity snapshots')
    q_mix=np.asarray([row['q_mix_uam_h'] for row in rows])
    q_bottleneck=np.asarray([row['q_bottleneck_uam_h'] for row in rows])
    observations=sum(row['active_aircraft'] for row in rows)
    counts={policy:sum(row[f'n_{policy}'] for row in rows) for policy in 'CRF'}
    summary={
        'definition':'TRB-style policy-conditioned planning rate for the single origin stream; candidate grid cells are not additive lanes',
        'reference_speed_mps':float(cfg.cruise_mps),
        'snapshot_interval_s':period,
        'snapshot_count':len(rows),
        'aircraft_snapshot_observations':observations,
        'policy_counts':counts,
        'policy_shares':{policy:counts[policy]/observations for policy in 'CRF'},
        'target_spacing_m':{policy:float(cfg.spacing(policy)) for policy in 'CRF'},
        'reliability_rho':rho,
        'q_mix_mean_uam_h':float(q_mix.mean()),
        'q_mix_median_uam_h':float(np.median(q_mix)),
        'q_mix_rho_uam_h':reliability_floor(q_mix,rho),
        'q_bottleneck_rho_uam_h':reliability_floor(q_bottleneck,rho),
        'minimum_q_mix_uam_h':float(q_mix.min()),
        'maximum_q_mix_uam_h':float(q_mix.max()),
    }
    return rows,summary


def planning_capacity_by_origin_stream(
        trace, cfg, origin_by_aircraft, *, origin_streams=None, rho=0.95,
        snapshot_s=None):
    """Sum policy-conditioned rates across real origin streams.

    Aircraft retain their origin-stream identity after changing physical lane.
    Only snapshots with at least one active aircraft from every configured
    origin stream are included, preventing empty warm-up or drain streams from
    being treated as zero capacity or invented parallel capacity.
    """
    if not 0 < rho <= 1:
        raise ValueError('rho must lie in (0, 1]')
    period=cfg.policy_s if snapshot_s is None else float(snapshot_s)
    if not np.isfinite(period) or period <= 0:
        raise ValueError('snapshot period must be positive and finite')
    if origin_streams is None:
        origin_streams=sorted(set(origin_by_aircraft.values()))
    else:
        origin_streams=list(origin_streams)
    if not origin_streams or len(set(origin_streams)) != len(origin_streams):
        raise ValueError('origin streams must be a nonempty unique sequence')
    missing={row['aircraft_id'] for row in trace}-set(origin_by_aircraft)
    if missing:
        raise ValueError(f'missing origin identity for aircraft: {sorted(missing)}')

    grouped=defaultdict(lambda:defaultdict(list))
    for row in trace:
        timestamp=float(row['t_s'])
        if abs(timestamp/period-round(timestamp/period)) > 1e-8:
            continue
        identifier=row['aircraft_id']
        grouped[timestamp][origin_by_aircraft[identifier]].append(row)

    rows=[]
    for timestamp in sorted(grouped):
        by_origin=grouped[timestamp]
        if any(not by_origin.get(origin) for origin in origin_streams):
            continue
        row={'timestamp_s':timestamp}
        total_rate=0.
        total_observations=0
        total_counts={policy:0 for policy in 'CRF'}
        for origin in origin_streams:
            active=by_origin[origin]
            identifiers=[item['aircraft_id'] for item in active]
            if len(identifiers) != len(set(identifiers)):
                raise ValueError(
                    f'duplicate aircraft at capacity snapshot {timestamp} '
                    f'for origin {origin}')
            policies=[item['policy'] for item in active]
            spacing=np.asarray([cfg.spacing(policy) for policy in policies],float)
            rate=float(3600.*cfg.cruise_mps/spacing.mean())
            prefix=f'origin_{origin}'
            row[f'{prefix}_active_aircraft']=len(active)
            for policy in 'CRF':
                count=policies.count(policy)
                row[f'{prefix}_n_{policy}']=count
                total_counts[policy]+=count
            row[f'{prefix}_mean_target_spacing_m']=float(spacing.mean())
            row[f'{prefix}_capacity_uam_h']=rate
            total_rate+=rate
            total_observations+=len(active)
        row.update({
            'active_aircraft':total_observations,
            'n_C':total_counts['C'],'n_R':total_counts['R'],
            'n_F':total_counts['F'],
            'corridor_capacity_uam_h':total_rate,
        })
        rows.append(row)
    if not rows:
        raise ValueError('trace contains no complete origin-stream snapshots')

    total_observations=sum(row['active_aircraft'] for row in rows)
    counts={policy:sum(row[f'n_{policy}'] for row in rows) for policy in 'CRF'}
    per_origin={}
    for origin in origin_streams:
        values=np.asarray([row[f'origin_{origin}_capacity_uam_h'] for row in rows])
        per_origin[str(origin)]={
            'mean_uam_h':float(values.mean()),
            'median_uam_h':float(np.median(values)),
            'rho_uam_h':reliability_floor(values,rho),
            'minimum_uam_h':float(values.min()),
            'maximum_uam_h':float(values.max()),
        }
    total=np.asarray([row['corridor_capacity_uam_h'] for row in rows])
    summary={
        'definition':(
            'sum of TRB-style policy-conditioned planning rates across real '
            'origin streams; aircraft retain origin identity after lane changes'),
        'complete_origin_stream_snapshots_only':True,
        'origin_streams':[str(origin) for origin in origin_streams],
        'reference_speed_mps':float(cfg.cruise_mps),
        'snapshot_interval_s':period,
        'snapshot_count':len(rows),
        'aircraft_snapshot_observations':total_observations,
        'policy_counts':counts,
        'policy_shares':{policy:counts[policy]/total_observations for policy in 'CRF'},
        'target_spacing_m':{policy:float(cfg.spacing(policy)) for policy in 'CRF'},
        'reliability_rho':rho,
        'per_origin_stream':per_origin,
        'corridor_capacity_mean_uam_h':float(total.mean()),
        'corridor_capacity_median_uam_h':float(np.median(total)),
        'corridor_capacity_rho_uam_h':reliability_floor(total,rho),
        'corridor_capacity_minimum_uam_h':float(total.min()),
        'corridor_capacity_maximum_uam_h':float(total.max()),
    }
    return rows,summary


def observed_exit_rate(events):
    """Return the served exit rate, explicitly not a saturation capacity."""
    times=sorted(float(row['t_s']) for row in events if row.get('status')=='exit')
    if len(times)<2 or times[-1] <= times[0]:
        return None
    return {
        'completed_exits':len(times),
        'first_exit_s':times[0],
        'last_exit_s':times[-1],
        'rate_between_first_and_last_exit_uam_h':3600.*(len(times)-1)/(times[-1]-times[0]),
        'interpretation':'served throughput for this demand realization, not maximum sustainable capacity',
    }
