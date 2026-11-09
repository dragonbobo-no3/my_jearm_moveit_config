# MK2 state-only commissioning — 2026-11-06

## Scope and permissions

The user confirmed all axes disabled, reliable mechanical support, and explicitly
authorized backed-up bridge configuration changes and a controlled controller
restart. This phase did **not** authorize automatic enable or trajectory testing.
Only real feedback was commissioned. No startup calibration, power, servo, gripper,
serial, or motion SDK call was added or sent.

The controller's old JEServer also initializes legacy grippers and uses an old
command destination. It was intentionally **not** loaded. The original C plugin
[mk2_state_publisher.c](mk2_state_publisher.c) only publishes SDK state.

## Actual deployment

- Controller: `192.168.0.99`, new-left profile only.
- Profile: `/home/robot/Work/system/robot_config_new_left`.
- Deployment directory (root-owned after compilation/testing):
  `/home/robot/Work/system/robot_config_new_left/bridge_state_only_20261106_xRgeNZ`.
- Actual plugin: `libMK2StateOnly.so` inside that directory.
- Plugin SHA-256:
  `22d6f7f662d772201560b6b3ac506d5be466bee7ca1e67874b09c8647a94bfa1`.
- Existing SDK: `/lib/libRobotControl.so`; controller `cc` 11.4.0, libzmq 4.3.4.
- Standalone C compilation with `-std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared`
  and real profile SDK include directory, linking `RobotControl`, `zmq`, pthread,
  and libm with `-Wl,-z,defs`. This is not a ROS/CMake package build.
- Active endpoint: `tcp://192.168.0.99:8000`, topic prefix `State `.
- `Bridge.mode = feedback_only`, `Bridge.commands_enabled = false`;
  `Robot0.Joint` is seven SDK joint positions in radians. Optional
  `Robot0.JointVelocity` comes from the real SDK, not fabricated zeros.
- No command subscriber/port 8001 connection exists in this plugin.

Only the new-left GlobalConfig `plugin_list` was changed from an empty list to
the absolute plugin path above. Legacy profiles and legacy .so files were not
overwritten. The source/profile guard requires the exact `--path` above and one
seven-axis SDK robot. The worker joins on plugin unload.

## Backups and configuration identity

The deployment directory contains `backup/GlobalConfig.yaml`,
`backup/DeviceConfig.yaml`, `backup/BusConfig.yaml`, and `backup/original.sha256`.

Original GlobalConfig SHA-256:
`4e56afd4cdd59ecf1a356629c39085c250b61abd2446a554678f2953c6c30682`.

DeviceConfig SHA-256, unchanged after restart and identical to the ROS snapshot:
`a542dc92c51e2822e78dcfc385a90f925a909ebd70072fab0b5ab75d4e0c2bab`.

BusConfig SHA-256, unchanged:
`425db50795d4e2e9fc44e4005e7dcf797e677c7d4fa87a135236a449a5f6ab84`.

No saved encoder offset, axis direction, scale, limit, MDH parameter, ENI, startup
guard or SDK library was edited. DeviceConfig and BusConfig were compared
byte-for-byte against the backups after restart. Saved zero offsets were used.

## Restart and safety observations

The original `/home/robot/Work/system/RobotMain_v2` supervisor and existing
new-left startup/handoff scripts were retained with every safety check intact.

1. Immediately before restart: all seven slaves OP, each axis disabled, error
   register zero, mode display 8. Statuswords `0x0250`, except axis 3 `0x1250`;
   controlwords `0x0002`.
2. The first handoff terminated the old controller normally but reported that
   a controller still existed during its handoff check. It refused to start a
   new instance. No forced kill or bypass was used.
3. Independent inspection subsequently confirmed no RobotMain/master process,
   seven PREOP slaves, all axes still disabled and error-free. The original
   stopped-state preflight passed. The same supervisor was then started normally.
4. Supervisor reported seven OP slaves, WKC 21/21, shared-memory sizes 133/140;
   the original RobotMain loaded the new-left profile and the state-only plugin.
5. Post-restart direct EtherCAT uploads independently confirmed seven disabled
   axes, mode 8, error register zero. Controlwords were `0x0000`; statuswords
   remained `0x0250`, except axis 3 `0x1250`.

At commissioning time: supervisor PID 42057, master PID 42186, RobotMain PID 42550.
These are historical identifiers, **not** safe process selectors for future stops.
The supervisor remains running independently of the SSH terminal (`nohup`), owns
both child processes and retains its cleanup behavior. Do not start a second copy.

Logs:
- Deployment directory: `supervisor-start.log` (initial protective refusal) and
  `supervisor-start-2.log` (successful stopped-state startup).
- Active session: profile `log/v2_ZwTlTD6U/master.log` and `controller.log`.
- The real plugin mapping and TCP 8000 listener were confirmed in RobotMain.
- Startup prints `group init existing problem. Some fields are not found!` and
  missing optional bus-field diagnostics. Functional feedback checks passed;
  this does not certify every vendor initialization warning or motion readiness.

`PowerState = 0` alone does not prove seven disabled axes. The per-axis EtherCAT
readback above is separate evidence. This bridge is not a safety system.

## Tests and real feedback results

- Before loading: simulated-SDK wrong-profile rejection, correct-profile state
  publication and `dlclose`/worker shutdown passed. Simulation binaries were
  built without linking the real SDK and were run only while TCP 8000 was free.
- Actual shared-object undefined-symbol audit: SDK imports are only
  `robot_getNUM`, `robot_getDOF`, `GetCurrentJoint`, `GetRobotPowerState`,
  `get_deviceName`, `get_name_robot_device`, `GetGroupVelocity`.
- First real bounded probe passed with seven complete finite positions.
- Six-second real sample: **601 frames, 100.003 Hz, no sequence gaps**, all joint
  positions inside the user-confirmed DeviceConfig bounds, PowerState 0.
- Representative SDK joint coordinates (radians):
  `[0.0008386584, -1.5699575497, -0.0001675418, 0.0000719053,
    0.0002994870, 0.0001677791, 0.0002216488]`.
  These are measurements, not targets or replacement zero offsets.
- Source-directory feedback launch tested in isolated ROS domain **218**,
  without RViz, fake executor or controller_manager; existing C++ executables
  were reused and source mesh paths were resolved explicitly for the test.
  The new ROS resources have **not** been rebuilt/installed.
- ROS received 1176 real JointState samples during service readiness/testing;
  the sole `/joint_states` publisher was `mk2_state_bridge`.
- MoveIt `compute_fk` and TF resolved `base_link` to `sdk_flange`.
  Representative FK position: `[0.30063510, 0.23048974, 0.70866153]` m.
  Maximum position difference versus precise DeviceConfig MDH: `2.22e-16` m;
  maximum rotation-matrix element difference: `5.55e-16`.
  TF-versus-FK position difference: `3.54e-5` m with different sample times.
- MoveIt current-state validity returned **true**, no collision contacts.
  This validates only the sampled state, not the entire workspace or a motion.
- Local test processes were shut down; the controller state PUB remains active.
- Known issue reproduced: MoveIt 2.5.9/rclcpp destructor segfault at shutdown
  (move_group exit -11), despite LaunchService exit 0. Cause remains undiagnosed.
  Local acceptance log: `/tmp/mk2-live-feedback-15msvke_.log` (temporary artifact).
- Local static/protocol regression: **19 tests passed**, including 40 random
  URDF-versus-MDH FK samples and explicit bridge command-capability handling.
- The real read-only plugin is rejected with exit 4 when the pure SUB probe is
  passed `--require-command-bridge`. The control launch requires this probe
  before creating a write-capable controller_manager process.

## Pending / rollback

Not tested: command reception, enable/hold behavior, real JointTrajectoryController,
physical motion, workspace coverage, emergency-stop performance or runtime
stale-feedback handling of the legacy driver. A command-capability flag is a
conservative launch guard, not authenticated proof of control-path readiness.
Do not treat this feedback milestone as motion authorization.

The VS Code CMake entry still cannot configure the ROS package, so normal rebuild,
installation and installed-entry-point acceptance remain pending. The controller
plugin build and live source-directory test above do not resolve that limitation.

Rollback requires renewed confirmation of mechanical support and all seven axes
disabled. Restore only the backed-up GlobalConfig, then use the same original
supervisor/handoff procedure with intact identity and startup checks. Restoring
GlobalConfig without restarting does not unload the already-running plugin.
Never restore an old JEServer or startup-zero plugin as a shortcut.