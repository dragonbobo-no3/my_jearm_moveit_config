# MK2 guarded control bridge — 2026-11-09

## Current status: isolated SDK simulation only

The simulation-only build has now been deployed and exercised through the
installed MoveIt → JTC → ZMQ → native SDK route. Seven-axis small roundtrip
tracking and a latched command-loss watchdog passed. Physical control remains
blocked; the physical profile still uses its state-only plugin. Exact current
deployment, evidence and final watchdog-latched state are recorded in
[SIMULATION_CONTROL.md](SIMULATION_CONTROL.md). No ROS package rebuild was done.

The sections below retain the original offline-development evidence. Statements
about no deployment/restart refer to that earlier phase, not today's simulation
commissioning. Removed test fixtures have not been reintroduced.

**Later direction correction:** the user confirmed the original teach-pendant
URDF matches the physical arm. ROS now preserves its original axes/origins,
including negative-Z J4/J6, rather than forcing an all-positive-Z MDH chain.
The ROS control entrypoint still blocks physical/unknown backends, even with
`confirm_enable=true`; only an explicitly verified isolated simulator is allowed.
SDK command/model/physical coordinate mapping still requires physical validation. Earlier MDH agreement
and the offline tests below are not evidence that physical directions are right.
No encoder calibration, SDK angle sign or controller file was changed for this fix.

**Test cleanup:** at the user's request, the local test scripts and fake-SDK
fixtures were removed after acceptance. Test results below are historical,
not an executable test suite currently shipped in this repository. Production
sources, locked commissioning policy and runtime safety checks remain unchanged.

**Historical offline phase — not physical-control acceptance.** At that time the
live controller used the previously commissioned state-only plugin. That phase did
not modify controller files, restart RobotMain, enable/disable any drive or send
any real hold/trajectory command. SSH operations only read SDK declarations and
exported symbols; SDK headers were copied to a local temporary directory.

## Components

- [mk2_control_core.h](mk2_control_core.h) /
  [mk2_control_core.c](mk2_control_core.c): SDK-independent validator and state
  machine. All SDK mutations are four injected callbacks, replaced by fake
  functions during the historical offline tests.
- [mk2_control_plugin.c](mk2_control_plugin.c): standalone vendor C plugin. It is
  not a ROS executable and is intentionally not installed by the ROS package.
- [mk2_control_bridge.example.json](mk2_control_bridge.example.json): locked
  commissioning policy, `allow_commands: false`. It remains locked in source;
  only the isolated simulation profile has its own reviewed enabled policy.
- Historical offline validation covered 57 core cases and fake-SDK plugin
  lifecycle/loopback transport. It loaded **no real libRobotControl**, used
  temporary mock DeviceConfig/policy files and random **127.0.0.1** ports only.
  Its test sources and runner have now been removed.

The existing state-only source and its actual deployed library are unchanged.
The old hardware driver, generation-one launches and MK2 feedback default are
also unchanged by this control-development work.

## Verified SDK contract

The actual controller headers/exported C symbols were inspected, not inferred
from unverified legacy comments:

- `ServoStart(double q, double r, int robot_index)` returns int, **0 success**.
  `q` and `r` are process/measurement noise parameters of the SDK servo filter,
  **not control period or frequency**. Example uses the legacy `1, 0.0001` pair;
  its physical response still requires commissioning.
- `ServoJoint(robjoint *, double time, int robot_index)` returns int, 0 success.
  SDK calls `time` the data time. The existing ROS driver resets `global_time_`
  at activation and increments it by `command_dt` each write. Therefore the
  bridge preserves the **cumulative data time**, not a per-packet move duration.
- `RobotStop(0)` requests stopping current motion **without powering off**, and
  blocks subsequent motion until the controller's forced-stop state is recovered.
  This is a vendor software stop, **not a certified hardware emergency stop**.
- `RobotStopRun(0)` is separately documented by the actual SDK header to stop
  without powering off or blocking later motion. The source-only simulation
  clean-release extension uses this only for an explicitly healthy session,
  checks idle/stable feedback before ServoEnd, and never resets an existing fault.
- `ServoEnd(0)` closes the SDK servo function. It returns int, 0 success; its
  actual braking/holding behavior must be verified, not assumed from the name.
- `get_robot_move_state(0)`: 0 normal, negative abnormal, 7 forced stop, other
  positive values indicate running. Startup requires 0; external forced stop 7
  during ownership faults the bridge and is never automatically recovered.
- `GetRobotPowerState(0) == 1` means all robot axes enabled; 0 only means at
  least one disabled. `IsRobotDriverError` / `IsRobotMoveError` must both be zero.
  Before deployment/restart, independently verify **each** physical axis is
  disabled. The aggregate getter is not sufficient for that safety check.

The new plugin imports no `RobotPower`, `RobotPoweroff`, `ClearRobotError`,
`RobotStopRecover`, calibration, Cartesian, serial or gripper API. It checks
return values of all four mutating SDK calls. On SDK failure it attempts stop
and end, publishes the SDK error and remains latched; it does not clear faults.

## Command subset and state machine

The deployed baseline accepts **single-frame** `Switch ` and `Joint ` commands.
The newer source-only isolation-verified simulation extension additionally accepts
`Release {"Release":true}` when it advertises `clean_release_supported=true`.
It uses RobotStopRun, verifies idle/stability, ends servo and returns to ready;
its timeout/errors retain forced-stop behavior. It never resets stopped/fault,
and the physical build does not provide this capability. See
[deployment status and tests](SIMULATION_CONTROL.md). Exact baseline,
case-sensitive JSON shapes:

- `Switch {"Switch":true}`: arm a waiting-for-first-hold intent only. It does
  **not enable the arm or call ServoStart yet**. The operator must have manually
  enabled it; sample must be finite, in bounds, healthy and not already moving.
- `Joint {"Robot0":{"time":0.01,"joint":[q1,q2,q3,q4,q5,q6,q7]}}`:
  exactly seven finite SDK radians. No Robot1, Cartesian, EndEffector or extra
  fields. First target must be within 0.002 rad of the fresh measured position;
  only after full validation does the bridge call ServoStart then ServoJoint.
  This proximity condition does **not** prove zero physical displacement.
- Later Joint `time` must strictly increase, with increments in 0.005..0.05 s
  and cumulative progress consistent with local monotonic reception time to
  within 0.1 s. First time must be positive and no larger than 0.2 s.
- `Switch {"Switch":false}`: stop/end **only a servo acquired by this plugin**,
  keep drives powered, then latch stopped. If only waiting for first hold, it
  cancels that intent without any SDK mutation. The operator subsequently
  disables/supports the arm using the native safety procedure.

Phases: locked → ready → waiting_first_joint → active → stopped/fault.
`stopped` and `fault` cannot be reset by network Switch or fresh Joint frames.
Restart/recovery must be an explicit, supported, onsite operation; a controller
forced-stop flag may also require native operator recovery. No automatic recovery
API is used.

Bad JSON, duplicate/escaped command keys, nonfinite/nonnumeric values, missing/
extra axes, wrong robot, unsupported fields/topics and multipart frames are
rejected. During waiting/ownership they latch a fault. Invalid first targets
make **zero mutating SDK calls**, including ServoStart. Multipart closes the
receiver permanently instead of draining an unbounded stream or executing fragments.
Repeated Switch=true is rejected, not a heartbeat or restart operation.

## Policy and protections

The standalone plugin requires the exact new-left `--path`, a local profile
`mk2_control_bridge.json`, one seven-axis SDK robot, and a DeviceConfig SHA-256
matching the policy. Invalid/missing policy or mismatching DeviceConfig fails
before the worker/network/servo are started. Policy remains immutable until a
controlled program restart.

The example DeviceConfig hash and seven limits match the existing user-confirmed
ROS snapshot; offsets/directions/encoder scales are never applied a second time.
The network cannot change policy, ports, bounds, filter settings or recover faults.

- Default `allow_commands: false`: **no command SUB socket**, state PUB only.
- Configured command host `192.168.0.100`, PC PUB port 8001; controller connects
  as SUB. This is deliberately not the old `192.168.0.35` destination.
- State PUB port 8000, approximately 100 Hz. Only publish finite real SDK joint
  positions; optional velocity comes from SDK. Missing data is never zero-filled.
- Commissioning target speed cap 0.3 rad/s, acceleration cap 0.5 rad/s²;
  enforce differences between accepted command targets and cumulative times.
- Following-distance cap 0.05 rad, position limits from the confirmed snapshot.
- No valid first Joint within 2 s: latch fault without ever acquiring servo.
- No valid active Joint within 0.2 s: request stop/end, latch fault, keep power.
  Only accepted Joint messages renew this watchdog; Switch and invalid input do not.
- Power loss, SDK error, invalid/out-of-bounds sample, forced stop or clock
  reversal during ownership requests stop/end and latches.
- Unload joins the worker and stops/ends an owned servo; never power-offs or
  stops another plugin/native program's servo when no ownership was acquired.

State metadata includes `Bridge.mode = control_guarded`, `control_state`,
`commands_enabled`, `fault`, `sdk_error`, `accepted_joints`, `rejected_frames`,
`operator_enable_required = true`, `automatic_power_changes = false` and a
state sequence. `commands_enabled` is false when locked/not healthy/not manually
enabled/stopped/faulted. The existing ROS `--require-command-bridge` preflight
therefore remains conservative. Its true value is **not a safety certificate**.

## Historical offline validation

- 57 core cases passed with ordinary `-std=c11 -Wall -Wextra -Werror` compilation.
- Same 57 cases passed under AddressSanitizer and UndefinedBehaviorSanitizer.
- Loopback tests passed: locked zero-mutation mode, disabled arm refusal, valid
  100 Hz-style cumulative-time trajectory, explicit stop without power-off,
  command-loss fault, replayed time, wrong robot, bad numeric type, gripper/
  multipart refusal, invalid first hold, SDK start failure, unload cleanup,
  bad DeviceConfig hash and wrong profile.
- Mock plugin undefined-symbol audit checked exact API names (not a substring
  test that would mistake GetRobotPowerState for RobotPower); no power/recover/
  gripper API and no real vendor library dependency.
- The adapter passed `cc -fsyntax-only -Werror` against the **actual controller
  SDK headers**, and all referenced servo/stop/state symbols exist in the real
  library. This does **not** mean the new real SDK shared object was built,
  deployed, loaded or exercised on the robot.
- Existing ROS/MK2 static/protocol suite: **21 tests passed**, including the new
  locked-policy/snapshot consistency test. ROS and standalone controller builds
  remain separate.
- A final pure SUB observation still reported the live **feedback_only** bridge,
  `commands_enabled = false`, sequence 23111704, SDK PowerState 1. This is a
  historical observation that the arm was enabled at that time, not permission
  to restart or a substitute for fresh onsite per-axis checks.

The historical offline runner and fixtures have been removed at user request.
Current acceptance used temporary inline checks, not new repository test files.
Standalone controller builds remain separate from the ROS/CMake package.

## Known boundaries — required before acceptance

1. The legacy hardware driver lacks protocol ACK/error propagation and robust
   stale-state handling. It may keep returning OK and publishing after the
   controller bridge faults. The bridge then refuses SDK motion, but ROS action
   status is not proof the physical backend accepted a command. Do not declare
   full control integration accepted until driver-side fault/freshness behavior
   has also been addressed/verified.
2. Legacy frames have no authenticated session/sequence or sender timestamp.
   Monotonic cumulative time and clock-skew checks reject duplicates and some
   delayed streams, **not every replay, takeover or MITM attack**. There is no
   CURVE/TLS/authentication. Restrict commissioning to a trusted isolated robot
   network; stronger session/authentication is a separate requirement.
3. SDK state getters expose no independently verified fresh EtherCAT-cycle
   marker here. Finite state and a progressing bridge sequence do not prove
   the bus feedback is fresh. Validate controller/bus fault behavior separately.
4. Watchdog shares a worker with SDK calls. A blocking SDK call, worker/process
   crash or controller failure can prevent it from executing. Fake timing does
   **not** certify physical stop latency; vendor/hardware protections and an
   accessible emergency stop remain mandatory.
5. No physical proof yet of filter response, SDK first-target atomicity, braking/holding,
   whole-workspace trajectories, latency/jitter, numerical/noise tolerance or
  physical JTC execution. Small isolated-simulation JTC execution has passed;
  it does not resolve these physical requirements. Do not relax caps if commissioning
   trips them. Do not issue pendant teach/motion concurrently with ROS ownership.
6. Legacy driver still subscribes to gripper ROS commands. Any such command is
   unsupported and rejected/faulted by this bridge; do not use the old end-effector
   topics while commissioning the arm-only path.

## Future physical build/deployment gate

Simulation acceptance does not authorize physical deployment. Before modifying
the physical controller profile: obtain fresh confirmation of reliable support, all seven axes
disabled and permission for a backed-up controlled restart. Verify exact process,
SDK/ENI/DeviceConfig identities using the existing startup guards; no force kill,
calibration, automatic fault recovery or bypass.

Controller build prerequisites: cc, real vendor SDK headers/library, libzmq,
json-c and OpenSSL development files. **The controller currently lacks json-c
development headers/pkg-config metadata**. The simulation build used a staged
copy of matching-version development headers and linked the exact installed
runtime library; no system package was installed. Any system-package operation
for future builds still requires authorization.

Build a uniquely named real .so from core + plugin, linking libRobotControl,
json-c, zmq, crypto, pthread and libm, with undefined-symbol checks. Back up
GlobalConfig/DeviceConfig/BusConfig and stage root-owned artifacts. Keep the
example policy locked for initial deployment. Load only this new library in
the new-left profile using the original supervised/guarded restart flow, not
alongside the already bound state-only PUB and not by hot-loading an unsafe old
JEServer.

Verify real state and every disabled axis after restart before any manual enable.
Unlocking the immutable policy requires a separately controlled restart. Then,
only with renewed onsite approval, manually enable and test initial hold/stop
and fault handling before any small low-speed trajectory. Neither installing
the ROS package nor changing hardware_mode alone deploys/unlocks this bridge.