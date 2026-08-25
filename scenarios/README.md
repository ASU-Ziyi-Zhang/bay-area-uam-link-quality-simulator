# Scenario packs

Each folder is a self-contained replaceable input package: route geometry,
macro-site table, scientific scenario configuration, simulator configuration,
and provenance notes. The radio, trajectory, policy, capacity, and dashboard
code under `src/` and `scripts/` is shared.

| Scenario | Endpoints | Purpose |
|---|---|---|
| `sf_sj_full` | SF 4th & King - San Jose Diridon | Frozen full-corridor baseline |
| `airport_to_airport` | Millbrae Caltrain - Santa Clara Caltrain | Toyota-recommended airport-access endpoint case |

Stable base-station IDs are not renumbered when a subcorridor is created. This
lets evidence for BS05, for example, refer to the same physical site in every
scenario.
