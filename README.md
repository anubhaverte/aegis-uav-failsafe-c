# Aegis — UAV Failsafe State Machine Simulator


Aegis is a C simulator and reference implementation of a five-state failsafe
finite state machine (FSM) for a UAV flight controller. It exists to make one
thing concrete: **failsafe decisions should be deterministic, debounced, and
testable — not a pile of `if` statements scattered through flight code that
nobody can verify.**

## Goals

- **Model a complete, unambiguous failsafe FSM.** Five states (`NORMAL` →
  `WARNING` → `RETURN_TO_HOME` → `AUTO_LAND` → `EMERGENCY_FAILSAFE`), seven
  independent fault conditions, and a strict severity ordering that resolves
  simultaneous faults without contradiction.
- **Treat the decision logic as a pure function.** `next_state()` takes a
  state and a set of debounced flags and returns the next state — no I/O, no
  side effects, so it can be unit tested exhaustively instead of only
  eyeballed from console output.
- **Reject noise, not just react to it.** Debouncing, timeout-based RC-loss
  detection, and separate handling for a memoryless sensor fault vs.
  persistent RC/GPS dropouts, so a single noisy sample can't trigger a
  failsafe escalation.
- **Be reproducible.** The simulation can be seeded and run non-interactively,
  so a specific scenario (e.g. "battery critical while under geofence stress")
  is a one-line, repeatable command instead of a manual stdin session.
- **Be honest about what it isn't.** This is a decision-logic simulator with
  synthetic sensor inputs, not flight software — see [Limitations](#known-limitations).

## State machine

```
NORMAL ──batt_warn──► WARNING ──alt_high──► WARNING
  │                      │
  │  rc_lost / gps_lost / fence_breach
  ▼                      ▼
RETURN_TO_HOME ──home_reached──► AUTO_LAND ──landed──► (loop ends)
  ▲
  │ batt_crit (from any state)
  │
  └── sensor fault (from ANY state, ANY time) ──► EMERGENCY_FAILSAFE (terminal)
```

Priority order when multiple conditions fire in the same cycle (highest
first): sensor fault → already latched (`EMERGENCY_FAILSAFE`/`AUTO_LAND`) →
critical battery → active RTH → RC/GPS/fence loss → high altitude → low
battery → normal. See `next_state()` in `src/fsm.c` for the exact logic and
the reasoning behind each rule.

## Layout

```
include/failsafe.h   Shared types (State, Reading, Sim, SimConfig) and tunables
src/fsm.c            State naming, transition logic, alert printing (pure logic)
src/sim.c            Synthetic sensor/telemetry generation
src/runner.c         The monitoring loop itself — callable by main() or by tests
src/main.c           CLI: argument parsing, interactive prompts, exit codes
tests/test_fsm.c     Unit tests for next_state()/debounce() + full-loop regression tests
Makefile             Build + test targets
.github/workflows/   CI: build, unit tests, and a scripted smoke-test on every push/PR
```

The split matters: `next_state()` and `debounce()` are pure and fully unit
tested, and `run_simulation()` is callable with a fixed seed from a test —
none of that was possible when everything lived in one `main()`.

## Build & run

```sh
make                # builds build/aegis_failsafe
./build/aegis_failsafe
```

**Interactive mode** (no flags) prompts for each value, same as the original
program: starting battery %, cruise altitude, altitude limit, geofence
radius, starting distance from home, and number of cycles.

**Scripted mode** — all six numeric parameters as flags, for reproducible
runs, scripting, or CI:

```sh
./build/aegis_failsafe --batt 15 --alt 50 --alt-limit 120 \
  --fence 100 --start-dist 90 --cycles 30 --seed 42
```

`--seed` makes the run reproducible (otherwise it's seeded from wall-clock
time, as before). `--quiet` suppresses per-cycle telemetry and just runs the
scenario. `--help` lists all options.

**Exit codes** (new — the original program had none): `0` = landed and
disarmed cleanly, `1` = escalated to `EMERGENCY_FAILSAFE`, `2` = cycle budget
ran out while still flying. Useful for scripting a batch of scenarios and
checking outcomes without parsing console output.

## Tests

```sh
make test
```

Covers:
- Transition priority ordering (sensor fault overrides everything,
  `AUTO_LAND`/`EMERGENCY_FAILSAFE` latch, critical battery overrides RTH,
  etc.) and the debounce counter's confirm/reset behavior.
- Full-loop regression tests via `run_simulation()` with a fixed seed: a
  critical-battery start reliably reaches `AUTO_LAND`, the same seed produces
  the same outcome twice, and a zero-cycle run ends `INCOMPLETE` at `NORMAL`.

CI runs the build, the unit tests, and a scripted smoke-test of the CLI on
every push and PR to `main`.

## What changed from the original single-file version

- Split `failsafe_sim.c` into `include/failsafe.h` + `src/fsm.c` (pure logic)
  + `src/sim.c` (telemetry generation) + `src/runner.c` (loop) + `src/main.c`
  (CLI) — same behavior, testable structure.
- Extracted the monitoring loop into `run_simulation()` so it's callable from
  tests with a fixed seed, instead of only runnable by typing numbers at a
  live `scanf` prompt.
- Added a scripted CLI mode (`--batt`, `--alt`, ... `--seed`, `--quiet`) on
  top of the original interactive prompts, which still work unchanged.
- Added exit codes (`SIM_EXIT_LANDED` / `SIM_EXIT_EMERGENCY` /
  `SIM_EXIT_INCOMPLETE`) so a run's outcome can be checked programmatically.
- Fixed `PI_F` from a hardcoded `3.14f` to a properly precise float constant
  — there was no reason for the original approximation; it only fed random
  angle generation and cost nothing to fix.
- Added unit tests (none existed before) and a CI workflow that builds, tests,
  and smoke-tests the CLI on every push/PR.

## Known limitations

Documented, not accidental:

- **No `EMERGENCY_FAILSAFE` recovery path.** By design: a real implementation
  would need an explicit re-arm after sensor data is revalidated.
- **Geofence breach is not debounced**, unlike every other fault condition —
  an intentional inconsistency worth flagging if this were hardened further.
- **Sensor fault model is memoryless** (re-rolled every cycle) rather than
  modeling intermittent/persistent/drifting fault classes.
- Single battery-drain curve with no temperature/payload/wind-load modeling.
- Position is a 2D random walk, not real vehicle dynamics.
- `fly_home()` assumes an unobstructed straight-line path — no obstacle
  avoidance or terrain-following.
- Fault probabilities (RC/GPS drop chance, sensor fault chance, drain spikes)
  are hardcoded, not configurable or derived from real telemetry statistics.
- This is a **console decision-logic simulator**, not embedded flight
  software — inputs are synthetic and generated in-process, not read from
  real ADC channels, a telemetry parser, or a GPS driver.

## License

MIT — see [LICENSE](LICENSE).
