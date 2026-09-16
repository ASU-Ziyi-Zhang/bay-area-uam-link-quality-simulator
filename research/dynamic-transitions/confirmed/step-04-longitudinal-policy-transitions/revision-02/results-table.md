### The six policy transitions

| Case | ΔS (m) | Controller | Request | T (s) | Final gap (m) | Target (m) | Gap − target (m) | Outcome | v min | v max | peak +a | peak −a |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C→R | 750 | baseline ACC | planned | 75.0 | 1958.3 | 2058.2 | -100.0 | 100 m short | 44.3 | 50.0 | 0.044 | -2.0 |
| C→R | 750 | KS2 reference | planned | 75.0 | 2117.5 | 2117.5 | 0.0 | on target | 30.0 | 50.0 | 0.7 | -0.933 |
| R→F | 1500 | baseline ACC | planned | 150.0 | 3351.4 | 3486.4 | -135.0 | 135 m short | 40.7 | 50.0 | 0.059 | -2.0 |
| R→F | 1500 | KS2 reference | planned | 150.0 | 3617.5 | 3617.5 | 0.0 | on target | 30.0 | 50.0 | 0.35 | -0.467 |
| C→F | 2250 | baseline ACC | planned | 225.0 | 3370.6 | 3495.9 | -125.2 | 125 m short | 36.0 | 50.0 | 0.089 | -2.0 |
| C→F | 2250 | KS2 reference | planned | 225.0 | 3617.5 | 3617.5 | 0.0 | on target | 30.0 | 50.0 | 0.233 | -0.311 |
| R→C | -750 | baseline ACC | planned | 51.2 | 2117.5 | 1367.5 | 750.0 | 750 m over | 50.0 | 50.0 | 0.0 | 0.0 |
| R→C | -750 | KS2 reference | planned | 51.2 | 1367.5 | 1367.5 | -0.0 | on target | 50.0 | 79.3 | 1.5 | -2.0 |
| F→R | -1500 | baseline ACC | planned | 100.0 | 3617.5 | 2117.5 | 1500.0 | 1500 m over | 50.0 | 50.0 | 0.0 | 0.0 |
| F→R | -1500 | KS2 reference | planned | 100.0 | 2117.5 | 2117.5 | 0.0 | on target | 50.0 | 80.0 | 0.787 | -1.05 |
| F→C | -2250 | baseline ACC | planned | 150.0 | 3617.5 | 1367.5 | 2250.0 | 2250 m over | 50.0 | 50.0 | 0.0 | 0.0 |
| F→C | -2250 | KS2 reference | planned | 150.0 | 1367.5 | 1367.5 | 0.0 | on target | 50.0 | 80.0 | 0.525 | -0.7 |

### Variant conditions

| Case | ΔS (m) | Controller | Request | T (s) | Final gap (m) | Target (m) | Gap − target (m) | Outcome | v min | v max | peak +a | peak −a |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gap already sufficient | 0 | baseline ACC | hold | 0.0 | 4500.0 | 3617.5 | 882.5 | held; gap already +882 m vs requirement | 50.0 | 50.0 | 0.0 | 0.0 |
| gap already sufficient | 0 | KS2 reference | hold | 0.0 | 4500.0 | 3617.5 | 882.5 | held; gap already +882 m vs requirement | 50.0 | 50.0 | 0.0 | 0.0 |
| closing with the old 50 m/s cap | -2250 | baseline ACC | infeasible | 0.0 | 3617.5 | 1367.5 | 2250.0 | refused before moving | 50.0 | 50.0 | 0.0 | 0.0 |
| closing with the old 50 m/s cap | -2250 | KS2 reference | infeasible | 0.0 | 3617.5 | 1367.5 | 2250.0 | refused before moving | 50.0 | 50.0 | 0.0 | 0.0 |
| leader brakes, gap sufficient | 0 | baseline ACC | hold | 0.0 | 3127.2 | 2614.1 | 513.1 | held; gap already +513 m vs requirement | 36.5 | 50.0 | 0.0 | -0.608 |
| leader brakes, gap sufficient | 0 | KS2 reference | hold | 0.0 | 3127.2 | 2614.1 | 513.1 | held; gap already +513 m vs requirement | 36.5 | 50.0 | 0.0 | -0.608 |
| leader brakes mid-transition | 2250 | baseline ACC | planned | 225.0 | 1733.9 | 2150.3 | -416.4 | 416 m short | 30.0 | 50.0 | 0.089 | -2.0 |
| leader brakes mid-transition | 2250 | KS2 reference | planned | 225.0 | 1498.0 | 2150.3 | -652.3 | 652 m short | 30.0 | 50.0 | 0.0 | -2.0 |
| opening, retired 0 m/s floor | 2250 | baseline ACC | planned | 90.0 | 3027.7 | 3327.6 | -300.0 | 300 m short | 36.0 | 50.0 | 0.089 | -2.0 |
| opening, retired 0 m/s floor | 2250 | KS2 reference | planned | 90.0 | 3617.5 | 3617.5 | 0.0 | on target | 0.0 | 50.0 | 1.458 | -1.944 |
| opening, jerk bound 0.05 | 2250 | baseline ACC | planned | 225.0 | 3370.6 | 3495.9 | -125.2 | 125 m short | 36.0 | 50.0 | 0.089 | -2.0 |
| opening, jerk bound 0.05 | 2250 | KS2 reference | planned | 225.0 | 3617.5 | 3617.5 | 0.0 | on target | 30.0 | 50.0 | 0.233 | -0.311 |

All values at the finest integration step, dt = 0.03125 s. The target is the policy spacing evaluated at the follower's final speed, and `Gap − target` is signed: negative means the follower ended closer to the leader than its policy allows, positive means it ended further away. For a `hold` or an `infeasible` request, not moving is the correct outcome, so those rows are not failures.

A request is `hold` when the existing gap already satisfies the new policy, and `infeasible` when the planner refuses because no admissible transition exists under the declared bounds.
