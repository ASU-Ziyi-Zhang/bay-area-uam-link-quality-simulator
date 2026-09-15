# Step 03 R0020 protocol — nominal-flow recovery addendum

**Approved direction:** retain the same 15-UAM, three-flow R0019 mechanism
case, restore the frozen Step-02 action rule, and add nominal-flow recovery as
the only new behavioral state. Capacity remains excluded.

## Frozen Step-02 controller

The 60 s warning and additional 60 s common evaluation horizon, policy loss
`l(C)=0`, `l(R)=1`, `l(F)=2`, quintic continuous motion, component limits and
curvature screen are unchanged. A policy-avoidance action is beneficial only
when

```math
J_P(a)<J_P(\mathrm{stay}).
```

For one aircraft, feasible beneficial candidates retain the confirmed order:

1. minimum integrated policy loss;
2. better common-horizon endpoint policy;
3. minimum transverse displacement;
4. minimum maneuver time;
5. deterministic coordinate tie-break.

The unapproved R0019 10-policy-cost-second gate is removed.

## New nominal-flow state

Each aircraft stores the flow assigned at corridor entry as immutable
`nominal_lane`. Its current lane may change.

When current and nominal lanes differ, the controller first evaluates one
step toward the nominal lane. The return candidate is eligible only if:

```math
T_{return}\le 120\ \mathrm{s},
```

```math
J_P(\mathrm{return})\le J_P(\mathrm{stay}),
```

and its common-horizon endpoint policy is no worse than staying. It must also
pass the unchanged target predecessor/follower gap, ACC forecast, curvature
and all-aircraft 3-D safety gates. If return is not eligible, the aircraft
stays unless a predicted degradation activates the frozen avoidance search.

This asymmetry is deliberate and user-approved: leaving the nominal flow must
strictly improve policy; safely returning may tie the stay policy because it
restores the assigned traffic organization.

## Multi-aircraft boundary retained for this pilot

The R0019 conservative implementation starts at most one globally selected
movement on each 5 s decision clock. R0020 does not claim this is an optimal
or confirmed arbitration law; it is retained unchanged so the nominal-flow
state is the isolated Step-03 change. Capacity, utilization, radio-load and
system-optimal lane balancing are outside R0020.
