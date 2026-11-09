# Unified physical / SDK simulation feedback — 2026-11-09

**Subsequent model-direction correction:** the user verified the teach-pendant
model against the physical arm. The ROS runtime now preserves the original URDF
joint axes and origins (notably negative-Z J4/J6); SDK joint feedback is unchanged.
The MDH-generated geometry and FK results recorded below describe the earlier
acceptance, not proof of correct physical directions. Installed runtime model
preservation was rechecked in domain 223 with 1180 SDK samples and 35 passing
tests. Control is blocked pending coordinate revalidation even with confirmation.
This local fix did not modify/restart the controller or send any motion command.

**Subsequent cleanup:** the user requested removal of local development test
scripts and fake-SDK fixtures. They are no longer shipped in the repository;
the test results below record earlier acceptance. Controller deployment backups,
runtime sources and safety checks were not removed or changed by this cleanup.

## User intent and scope

The controller chooses its backend at startup. The upper computer consumes the
same `State {JSON}`, `Robot0.Joint[0..6]` SDK-radian feedback either way. SDK
simulation is **not** the PC CAD fake executor and missing feedback is never a
reason to fabricate joint states or switch to fake execution.

The user explicitly authorized controller changes and automatic simulation when
no arm is connected. This phase did not authorize physical enable, movement,
calibration, fault recovery or deployment of the guarded command bridge.

## Controller startup

The original controller entrypoint `/home/robot/Work/system/RobotMain_v2` now
defaults to `--auto`, so normally run the same existing entrypoint without a
mode argument. `--real`, `--check` and `--check-sim` remain available;
`--sim` is retained only as a compatibility alias, not required by normal startup.
Legacy explicit `--start` retains its previous real-start meaning.

[select_controller_backend.sh](select_controller_backend.sh) performs a bounded,
read-only EtherCAT slave query:

- Successful query, zero slaves: existing isolated SDK simulation launcher.
- Exactly seven expected axis slots/identities: original guarded real startup.
- Missing CLI, query failure/timeout, unexpected count, identity or layout:
  refuse; do not assume simulation.
- Selection occurs only at startup, not on feedback loss or hot-plugging.
- Optional `MK2_EXPECT_BACKEND=sim` is an additional commissioning restriction:
  detect normally but refuse if physical hardware appears. It never overrides
  detection. This restriction was used for this simulation-only acceptance start.

[prepare_robotmain_auto.py](prepare_robotmain_auto.py) prepares a separate copy
using exact reviewed substitutions; refusal occurs if the original entrypoint
has changed. Only dispatch/usage changed. Existing real SDK/ENI/identity checks,
disabled handoff, process ownership, locks and TERM-only cleanup remain intact.
The simulation launcher and timer selector were not edited. Its pinned global
configuration check remains mandatory.

The private simulation namespace still provides its own 16 MB shared memory and
binds `/dev/null` over `/dev/EtherCAT0`. No IgHEtherCATMaster is started. Real and
simulation sessions remain mutually exclusive; changing a running backend is
not automatic. Existing sessions must be stopped using their owned supervisor
and relevant safety procedure, not broad `pkill` or forced kill.

## Deployment and backups

Controller staging/deployment directory:
`/home/robot/Work/system/robot_config_new_left_sim/bridge_unified_20261109_NFVbjv`.

- Plugin: `libMK2SDKSimulationState.so`.
- Same [mk2_state_publisher.c](mk2_state_publisher.c) source, built against real
  vendor headers/library with exact simulation `PROFILE_ROOT` and
  `MK2_BACKEND=sdk_simulation`; no ROS/CMake build involved.
- Plugin SHA-256:
  `99b634f71eeecc055068050a12cb0df07ed59642450813efdd3c57c4ddf6e402`.
- Source SHA-256:
  `6898da2e7f30074484ad4ea0533d78b75917ae47efea6411bcff20603242d780`.
- New simulation GlobalConfig SHA-256:
  `e142a5a5139ae419b7e433d267fdb938a599d349085f27a8f160e24fe74643b9`.
- Only simulation `plugin_list` changed; `simulation: true` remains. Existing
  GlobalConfig hash pin was updated to the reviewed file, not removed/bypassed.
- `backup/` contains original simulation GlobalConfig and pin, DeviceConfig,
  BusConfig, original RobotMain_v2, unchanged simulation launcher, original
  hashes and the real profile configuration hash manifest.
- Deployment artifacts and selector are root-owned/non-group-writable. Original
  entrypoint and configuration file owners/modes were preserved on installation.
- Real GlobalConfig/DeviceConfig/BusConfig and simulation DeviceConfig/BusConfig
  remained byte-identical. No encoder offset, MDH, ENI or SDK library changed.

Before installation/start, the previous simulator had already exited: no
RobotMain/master process or state listener existed, and the slave query succeeded
with zero results. No existing controller process was killed. Original isolated
sim preflight passed and the **no-argument** entrypoint selected `AUTO BACKEND=sim`.
Historical acceptance supervisor PID 50024 and RobotMain PID 50036 are not safe
selectors for future process termination.

Startup log: deployment `auto-sim-start.log`; active SDK session at acceptance:
`robot_config_new_left_sim/log/sim_U1NB1lZL/controller.log`.

## Protocol and evidence

- PUB `tcp://192.168.0.99:8000`; `Bridge.mode=feedback_only`,
  `commands_enabled=false`, `backend=sdk_simulation`, monotonic sequence.
- No command SUB/socket or enable/calibration/gripper/motion SDK imports.
  Symbol audit found only robot counts/DOF and joint/power/device/velocity getters.
- Fake-SDK worker lifecycle/unload and wrong-profile rejection passed before
  loading the real compiled simulation plugin; mocks did not link the vendor SDK.
- Six-second controller sample: **599 frames, approximately 100 Hz, zero sequence
  gaps**, seven finite positions. Current simulation initial positions were all
  zero, as read from the SDK, not PC timeout fallback values.
- All seven simulation MDH parameter sets matched the real profile within
  `1e-12`; simulation-specific encoder offsets remain separate and unchanged.

The installed upper-computer default entrypoint was tested in isolated ROS
domain 222, without any physical/simulation flag and with RViz enabled:

- First valid frame then model/MoveIt/RViz start; single-argument Humble
  OnProcessIO callback and fragmented exact-marker gate were exercised.
- 1199 JointState samples; sole publisher `mk2_state_bridge`.
- No `controller_manager` or fake trajectory executor.
- MoveIt FK succeeded for `sdk_flange`; zero-SDK-joint flange position relative
  to `base_link`: approximately `[0.892440, 0.230000, 1.301002]` m.
- Acceptance log: `/tmp/mk2-unified-installed-pass-qzdc4329.log` (temporary).
- No-data installed entrypoint separately tested in domain 221, local unused
  port, startup timeout 0.5 s: receiver exited 2; no model/MoveIt/RViz was started
  and no secondary initialization crash occurred. Cancelled probe exited 2.
- **31** configuration/protocol/selector/readiness tests passed. The ROS package
  was not rebuilt by the agent: existing installed resources are source symlinks.

Known independent issue remains: normal shutdown reproduced system MoveIt 2.5.9
segfault (-11) and RViz abort (-6). Startup and feedback passed; this is not a
claim that all shutdown/destructor bugs have been fixed. Initial acceptance
also exposed an incorrect test-only log-string assertion and duplicate SIGINT
from group signaling; the corrected acceptance sent SIGINT only to launch.

## Boundaries / rollback

The real feedback plugin remains the previously commissioned state-only plugin.
The guarded control plugin remains local/offline, undeployed and locked. Neither
physical nor SDK-simulation trajectory execution was accepted here. `PowerState`
from simulation is not evidence of physical drive safety.

Rollback must first verify and normally stop only the owned simulation session;
if a real session has since started, renew support/disabled-axis confirmation and
use its original safe handoff procedure. Restore backed-up simulation GlobalConfig
**and its matching pin** together, restore the original native entrypoint, and
remove only the newly installed selector. Do not restore a legacy JEServer or
change simulation isolation. Configuration restoration alone does not unload a
running plugin. Recheck identities/hashes and use original guarded startup.