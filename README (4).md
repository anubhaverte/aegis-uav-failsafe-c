# Aegis — UAV Failsafe State Machine Simulator

<p align="center">
  <img src="https://img.shields.io/badge/language-C-blue.svg" alt="C">
  <img src="https://img.shields.io/badge/build-make-orange.svg" alt="Make">
  <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="MIT License">
</p>

Aegis is a C simulator and reference implementation of a **five-state failsafe finite state machine (FSM)** for a UAV flight controller. It exists to make one thing concrete: **failsafe decisions should be deterministic, debounced, and testable** — not a pile of `if` statements scattered through flight code that nobody can verify.

---

## 🎯 Goals

| Goal | Description |
|------|-------------|
| **Deterministic FSM** | Five states (`NORMAL` → `WARNING` → `RETURN_TO_HOME` → `AUTO_LAND` → `EMERGENCY_FAILSAFE`), seven independent fault conditions, strict severity ordering |
| **Pure decision logic** | `next_state()` is a pure function — no I/O, no side effects, fully unit-testable |
| **Noise rejection** | Debouncing, timeout-based RC-loss detection, memoryless sensor fault handling |
| **Reproducible** | Seeded, non-interactive simulation — one-line repeatable commands |
| **Honest scope** | Decision-logic simulator with synthetic inputs, not flight software |

---

## 🔄 State Machine

```
                    ┌─────────────────────────────────────────┐
                    │                                         │
                    ▼                                         │
┌────────┐  batt_warn  ┌─────────┐  alt_high   ┌─────────┐    │
│ NORMAL │────────────►│ WARNING │────────────►│ WARNING │────┘
└───┬────┘             └────┬────┘             └─────────┘
    │                       │
    │ rc_lost / gps_lost    │ rc_lost / gps_lost / fence_breach
    │ / fence_breach        │
    ▼                       ▼
┌─────────────────┐  home_reached  ┌──────────┐  landed  ┌──────┐
│ RETURN_TO_HOME  │───────────────►│ AUTO_LAND│─────────►│ done │
└────────┬────────┘                └──────────┘          └──────┘
         │
         │ batt_crit (from any state)
         │
         ▼
┌────────────────────┐
│ EMERGENCY_FAILSAFE │ ◄── sensor fault (from ANY state, ANY time)
└────────────────────┘      (terminal — no recovery)
```

### Priority Order (highest first)

1. **Sensor fault** → `EMERGENCY_FAILSAFE`
2. **Already latched** → `EMERGENCY_FAILSAFE` / `AUTO_LAND`
3. **Critical battery** → `AUTO_LAND`
4. **Active RTH** → `RETURN_TO_HOME`
5. **RC/GPS/fence loss** → `RETURN_TO_HOME`
6. **High altitude** → `WARNING`
7. **Low battery** → `WARNING`
8. **Normal** → `NORMAL`

See `src/fsm.c` for the exact logic and reasoning behind each rule.

---

## 📁 Project Layout

```
aegis-uav-failsafe-c/
├── include/
│   └── failsafe.h       # Shared types (State, Reading, Sim, SimConfig) and tunables
├── src/
│   ├── fsm.c            # State naming, transition logic, alert printing (pure logic)
│   ├── sim.c            # Synthetic sensor/telemetry generation
│   ├── runner.c         # Monitoring loop — callable by main() or by tests
│   └── main.c           # CLI: argument parsing, interactive prompts, exit codes
├── tests/
│   └── test_fsm.c       # Unit tests for next_state()/debounce() + regression tests
├── Makefile             # Build + test targets
├── .github/
│   └── workflows/       # CI: build, unit tests, and smoke-test on every push/PR
└── README.md            # This file
```

The split matters: `next_state()` and `debounce()` are pure and fully unit tested, and `run_simulation()` is callable with a fixed seed from a test — none of that was possible when everything lived in one `main()`.

---

## 🚀 Build & Run

### Prerequisites
- GCC or Clang
- Make
- (Optional) Valgrind for memory checking

### Build
```sh
make
```
This produces `build/aegis_failsafe`.

### Interactive Mode
```sh
./build/aegis_failsafe
```
Prompts for: starting battery %, cruise altitude, altitude limit, geofence radius, starting distance from home, and number of cycles.

### Scripted Mode
All parameters as flags — for reproducible runs, scripting, or CI:
```sh
./build/aegis_failsafe --batt 15 --alt 50 --alt-limit 120 \
  --fence 100 --start-dist 90 --cycles 30 --seed 42
```

| Flag | Description | Default |
|------|-------------|---------|
| `--batt` | Starting battery % | Prompt |
| `--alt` | Cruise altitude (m) | Prompt |
| `--alt-limit` | Altitude limit (m) | Prompt |
| `--fence` | Geofence radius (m) | Prompt |
| `--start-dist` | Starting distance from home (m) | Prompt |
| `--cycles` | Number of simulation cycles | Prompt |
| `--seed` | Random seed (for reproducibility) | Wall-clock time |
| `--quiet` | Suppress per-cycle telemetry | Off |
| `--help` | Show usage | — |

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | `SIM_EXIT_LANDED` — Landed and disarmed cleanly |
| `1` | `SIM_EXIT_EMERGENCY` — Escalated to `EMERGENCY_FAILSAFE` |
| `2` | `SIM_EXIT_INCOMPLETE` — Cycle budget ran out while still flying |

---

## 🧪 Tests

```sh
make test
```

### What's Covered
- **Transition priority ordering** — sensor fault overrides everything, `AUTO_LAND`/`EMERGENCY_FAILSAFE` latch, critical battery overrides RTH, etc.
- **Debounce counter** — confirm/reset behavior
- **Full-loop regression tests** via `run_simulation()` with a fixed seed:
  - Critical-battery start reliably reaches `AUTO_LAND`
  - Same seed produces identical outcome twice
  - Zero-cycle run ends `INCOMPLETE` at `NORMAL`

CI runs the build, unit tests, and a scripted smoke-test of the CLI on every push and PR to `main`.

---

## 📋 Changelog (from original single-file version)

| Change | Impact |
|--------|--------|
| Modularized into `include/` + `src/` + `tests/` | Testable structure, same behavior |
| Extracted `run_simulation()` | Callable from tests with fixed seed |
| Added scripted CLI mode (`--batt`, `--alt`, `--seed`, `--quiet`) | Reproducible runs, CI-friendly |
| Added exit codes (`SIM_EXIT_*`) | Programmatic outcome checking |
| Fixed `PI_F` from `3.14f` to precise float constant | Correct random angle generation |
| Added unit tests + GitHub Actions CI | Automated verification on every push/PR |

---

## ⚠️ Known Limitations

> Documented, not accidental.

- **No `EMERGENCY_FAILSAFE` recovery path.** By design — a real implementation would need explicit re-arm after sensor revalidation.
- **Geofence breach is not debounced.** Unlike every other fault condition — an intentional inconsistency worth flagging if hardened further.
- **Sensor fault model is memoryless.** Re-rolled every cycle, not modeling intermittent/persistent/drifting fault classes.
- **Simplified physics.** Single battery-drain curve, no temperature/payload/wind-load modeling. Position is a 2D random walk, not real vehicle dynamics.
- **No obstacle avoidance.** `fly_home()` assumes unobstructed straight-line path.
- **Hardcoded fault probabilities.** Not configurable or derived from real telemetry statistics.
- **Simulator, not flight software.** Synthetic in-process inputs — not real ADC channels, telemetry parser, or GPS driver.

---

## 📄 License

MIT — see [LICENSE](LICENSE).

---

<p align="center">
  Built with precision. Tested with intent. <br>
  <em>Because flight code deserves better than "it looks right."</em>
</p>
