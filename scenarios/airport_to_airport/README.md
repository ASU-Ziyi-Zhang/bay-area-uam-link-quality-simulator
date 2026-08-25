# Airport-access scenario

This scenario implements Toyota ITL's suggested endpoint revision while
retaining the same Caltrain-referenced route geometry:

- North endpoint: Millbrae Caltrain, used as the SFO access proxy
- South endpoint: Santa Clara Caltrain, used as the SJC access proxy
- Site rule: retain documented sites whose shortest planimetric distance to
  this subcorridor is no greater than 5 km
- Default flight: 300 m AGL, 50 m/s, zero lateral offset

The name `airport_to_airport` is a scenario label. The modeled endpoints are
rail-station access proxies, not coordinates on airport property. This avoids
claiming an airport vertiport location that Toyota has not specified.

Regenerate the derived geometry and site table with:

```powershell
python scripts\build_subcorridor_scenario.py
```

`build.json` records the endpoint coordinates and selection rule;
`build_report.json` records the projected endpoints, trims, length, and the
retained/excluded stable site IDs.
