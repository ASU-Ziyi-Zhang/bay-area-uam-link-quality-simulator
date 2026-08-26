# Selected-aircraft 3D view: review packet

## Objective

Restore visual and interaction consistency with the accepted single-UAM 3D
view. The aircraft selected on the 2D traffic map must also drive the right-hand
metrics and the lower 3D focal view.

## Implemented behavior

- fixed 315-degree bearing, fixed pitch, and 3.3 km third-person range;
- camera translates with the selected aircraft only;
- selecting another aircraft transfers the focal camera immediately;
- selected aircraft reuses the Cesium drone model, altitude label, vertical
  altitude reference, and dashed serving-BS link from the single-UAM view;
- surrounding active aircraft reuse the same drone model and are outlined by
  their C/R/F policy color;
- physical tower/building/tree/water-tower station geometry and the ground and
  300 m route layers are restored;
- manual orbit/zoom enters free view and `Recenter` returns to focal follow;
- the non-Cesium fallback follows the selected aircraft on a real map.

## Scientific boundary

This is a presentation-layer revision. It does not change aircraft states,
RSRP/SINR, local groups, C/R/F assignment, or capacity results from the v3
one-second run.

## Checks required

- JavaScript syntax and Python tests pass;
- browser render confirms that selecting UAM006 on the upper map transfers the
  lower 3D focal label to `UAM006 · 300 m · F`;
- the selected-aircraft altitude reference and serving-site link are visible;
- aircraft render with the Cesium drone model, without black rectangles;
- Recenter is present and restores the fixed focal view.

Browser-QA capture: `multi_aircraft_focal_3d_v4.png`.

## Decision requested

Approve the synchronized focal-aircraft 3D view or request a final visual
revision before beginning multi-lane/multi-level implementation.
