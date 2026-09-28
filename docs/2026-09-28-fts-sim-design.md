# Simulated drone flight termination system — design

Status: draft for owner review, 2026-09-28.

## Context

Demo project for Alpental Systems: a flight termination system (FTS) for a
small drone, simulated end to end without hardware, as proof of the "FTS"
work listed on the company site. It follows the pattern of the redundant
controller series: pure C logic tested on the host, a Zephyr firmware, a
written test log, and a Korean blog post with an English summary.

**Confidentiality.** The design uses only public references (ASTM F3322 for
small drone parachutes, FAA and EASA containment and operational-safety
requirements, RCC 319 for range-safety FTS) and generic engineering. Nothing
from any former employer's design appears in code, docs, or posts. The owner
reviews this spec for anything too close to former work.

## Goals

- An FTS that decides independently of the autopilot and terminates the
  flight on five triggers: geofence breach, altitude ceiling, loss of
  control, autopilot freeze, command link loss, plus a manual terminate.
- Termination: cut motor power, then deploy a parachute.
- Every trigger and a set of false-alarm cases tested in simulation, with
  measured times from fault to trigger, relay, and parachute.
- Screenshots and graphs good enough for the blog post.

## Non-goals

- The FTS's own sensor faults (GNSS or IMU failure inside the FTS); possible
  part 2.
- Redundant FTS channels or voting.
- A staged "land first" response through the autopilot.
- Real hardware; the design keeps it possible (see Hardware path).
- Certification artifacts.

## Environment

- UTM Ubuntu 22.04 VM on the owner's Mac (ARM64, 8 cores, 7.7 GB RAM, about
  12 GB free), ROS 2 Humble (desktop), Gazebo `gz sim` 8 with `ros_gz`.
- PX4 autopilot in software-in-the-loop (SITL) mode, flying PX4's x500
  quadcopter in Gazebo; ROS 2 talks to PX4 through the uXRCE-DDS agent and
  `px4_msgs`.
- Code lives in the `fts-sim` repo on the Mac and is synced to the VM for
  building and running; host tests also run on the Mac.
- Packages that need `sudo` on the VM are installed by the owner.

## Architecture

```
Gazebo (world, x500 + FTS sensors, gated motors, parachute)
   ▲ ▼ Gazebo transport
FTS I/O bridge (C++, no ROS 2) ── serial link 1 (PTY): sensors, heartbeat ──► FTS firmware
   ▲ PX4 MAVLink heartbeat (UDP)                                               (Zephyr, native_sim)
PX4 SITL ◄── uXRCE-DDS ──► ROS 2                                                 ▲
                             ├─ ground station node ── serial link 2 (PTY): radio ┘
                             ├─ test runner
                             └─ RViz, PlotJuggler, rosbag2
```

Independence rules:

- The FTS decides only from its own sensors (a second IMU, a GNSS receiver,
  and a barometer on the drone model) and two input lines: the autopilot
  heartbeat and the radio link. It never reads PX4's estimates or status.
- The FTS path (sensors, relay, parachute) does not use ROS 2. ROS 2 serves
  the ground station, the tests, and visualization.
- The motors listen to a gated topic. When the FTS opens the relay, the gate
  sends zero whatever PX4 commands.

## Components

| Component | Language | Tested on the host |
|---|---|---|
| FTS logic: state machine, triggers, fence geometry | C (pure) | yes |
| Frame protocol with CRC | C, shared by firmware, bridge, and ground station | yes |
| FTS firmware glue: UARTs, timing, Zephyr | C | runs on `native_sim` |
| FTS I/O bridge: sensor feed, relay gate, parachute, heartbeat, fault hooks | C++ with Gazebo transport | integration runs |
| Ground station node and test runner | Python ROS 2 packages | scenario runs |

## FTS logic

States:

- `PBIT` (power-on self-test) → `SAFE` on pass, `FAULT` on fail.
- `SAFE`: termination impossible (ground handling). `ARM` from the ground
  station moves to `ARMED` only with a GNSS fix and the drone inside the
  fence. `DISARM` returns to `SAFE`.
- `ARMED`: triggers are checked.
- `TERMINATED`: latched until restart.
- `FAULT`: cannot arm; reports the failed self-test item.

Triggers (checked only in `ARMED`; thresholds are settings):

| Trigger | Source | Condition |
|---|---|---|
| Geofence breach | FTS GNSS | outside the fence polygon for more than 0.5 s |
| Altitude ceiling | FTS barometer and GNSS | above the ceiling for more than 0.5 s |
| Loss of control | FTS IMU | tilt over 60° or body rate over 300°/s for more than 0.5 s |
| Autopilot freeze | heartbeat line, 10 Hz | no heartbeat for more than 1.0 s |
| Command link lost | radio link | no valid frame for more than 5 s |
| Manual terminate | radio link | `TERMINATE_ARM`, then `TERMINATE` within 3 s |

Termination sequence: open the motor relay immediately; fire the parachute
0.3 s later so the rotors spin down first; log the cause and the times of
trigger, relay, and parachute; report them to the ground station.

Timing: the FTS runs its checks at 100 Hz. Sensor feeds: IMU 100 Hz,
barometer 20 Hz, GNSS 10 Hz. Every sensor frame carries Gazebo simulation
time; the FTS's timers and event stamps use it, so all measurements share
one clock even if Gazebo runs slower than real time.

PBIT: sensor frames arrive at their expected rates; values are plausible
(gravity about 9.8 m/s² at rest); relay and parachute outputs read back
correctly; fence settings are valid (closed polygon, ceiling above ground).

## Simulation

- World: flat test field, geofence shown as posts and lines, steady wind of
  about 3 m/s.
- Drone: a copy of PX4's x500 with the FTS sensors added and the motor
  command topic renamed to the gated topic.
- Parachute: on command, the bridge applies drag `0.5 · ρ · Cd · A · v²`
  opposite to the velocity and shows a canopy model that follows the drone.
- Heartbeat: the bridge reads PX4's MAVLink heartbeat over UDP and forwards
  it on serial link 1; freezing the PX4 process stops it.
- Fault hooks in the bridge: cut one motor.

## Tests

Host tests (written first): state machine transitions, each trigger's
condition and confirmation time, fence geometry (inside, outside, edge,
concave polygon), manual terminate sequence and timeout, PBIT checks, frame
protocol.

Scenarios (ROS 2 test runner: arm the FTS, take off to 20 m, fly, inject,
record):

| Scenario | Cause | Expected |
|---|---|---|
| Geofence breach | route crosses the fence | terminate just past the fence |
| Altitude ceiling | climb above the ceiling | terminate just above it |
| Loss of control | bridge cuts one motor | terminate about 0.5 s after the tilt |
| Autopilot freeze | `SIGSTOP` the PX4 process | terminate about 1 s after the heartbeat stops |
| Command link lost | ground station stops sending | terminate about 5 s later |
| Manual terminate | two-step command | terminate at once |
| False alarms | flight near the fence; one noisy GNSS sample; one missed heartbeat | no termination |

Measured per scenario (simulation time): fault to trigger, relay, and
parachute; descent rate before and after the parachute; drift from the
termination point to landing; for fence breaches, landing distance beyond
the fence. Results go to `docs/test-log.md`.

## Hardware path

On hardware, the FTS logic and protocol stay the same; the firmware runs on
an MCU board with its own GNSS, IMU, and barometer, a relay in the motor
power line, and a parachute release output. The bridge and Gazebo drop out.

## Repository

```
fts-sim/
  docs/          design, plan, test log
  common/        frame protocol with CRC (C)
  fts/           Zephyr app (native_sim); FTS logic in pure C
  bridge/        FTS I/O bridge (C++, Gazebo transport)
  sim/           world, drone model with FTS sensors, parachute canopy
  ros2_ws/src/   ground station and test runner (Python)
  tests/host/    host tests
  tools/         launch script for the whole system
```

## Risks

| Risk | Handling |
|---|---|
| Gazebo slower than real time in the VM | measurements and FTS timers use simulation time |
| PX4 SITL rough edges on ARM64 Linux | plan step 0 is a feasibility check; ArduPilot SITL is the fallback (only the heartbeat source and flight commands change) |
| Small disk (about 12 GB free) | one workspace, caches cleaned, only needed PX4 targets built |
| Confidentiality | public references only; owner reviews the spec and post |
