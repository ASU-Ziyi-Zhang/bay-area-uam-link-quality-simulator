# Historical combined baselines (read-only)

Early runs computed the fixed streams and the continuous path search in one
execution. They are kept as one package with the original manifest rather than
split into runs that would look independent. New experiments go to their topic.

| ID | Original folder | Status |
|---|---|---|
| R0001 | `runs/itl-baselines-20260829-v1` | intermediate development version, superseded |
| R0002 | `runs/itl-baselines-20260829-v2` | intermediate development version, superseded |
| R0003 | `runs/itl-baselines-20260829-v3` | development version cited by the historical report |

In each run, `<scenario>/lanes/` holds the fixed-stream results and `offsets/`
the path results (local archive). Method: [protocol.md](protocol.md);
migration record: [../../migration-20260830.json](../../migration-20260830.json).

`configs/research_baselines.json` is the original configuration, byte for byte.
Its relative paths refer to the **former** `configs/` location, so it is kept as
a record and cannot be executed from here; use the relocated topic
configurations. None of the three runs recorded a Git SHA. Before migration, v1
had three source hashes differing from the code then present, v2 one, and v3
none. All output files pass their original manifest, which does not restore the
v1/v2 source or constitute scientific validation.
