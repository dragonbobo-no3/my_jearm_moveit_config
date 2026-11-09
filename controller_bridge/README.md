# MK2 controller-side state-only plugin

Guarded control development is separate and **not deployed**. See
[CONTROL_BRIDGE.md](CONTROL_BRIDGE.md) for the local-only implementation,
offline validation, manual-enable requirement and future deployment gates.
The state-only plugin described below is still the live commissioned plugin.

[mk2_state_publisher.c](mk2_state_publisher.c) is a standalone C plugin for the
vendor RobotMain SDK, not a ROS executable and not built by the ROS package.
Build it on the controller against the exact SDK headers/library used by RobotMain.

It publishes `State {"Bridge":...,"Robot0":...}` on TCP 8000 at approximately
100 Hz, using SDK joint coordinates. It checks the exact new-left profile path,
one robot and seven joints. Missing/nonfinite joint data is never replaced with
zeros. Optional velocity is omitted if unavailable.

There is **no command subscriber**, no TCP 8001 command connection, no serial
or gripper handling, no power/servo/calibration/motion calls. Enabling the PC's
control mode does NOT create a functioning command channel with this plugin.
It is for feedback commissioning only; motion control is a separate acceptance step.

Deployment must preserve DeviceConfig/BusConfig/ENI and existing saved zero offsets.
Before a controlled restart, verify every physical axis is disabled and the arm
is mechanically supported. Back up GlobalConfig and load this uniquely named
plugin by its absolute path only in the new-left profile. Use the existing
RobotMain_v2 supervisor and its identity/disabled/preflight checks; do not kill
unknown processes, force-start a second controller or bypass bus checks.

The source only references SDK getter symbols. Audit the built shared object's
undefined symbols before loading it. A simulated-SDK lifecycle/protocol test
must pass before the real controller restart. Rollback restores the backed-up
GlobalConfig and uses the same disabled/controlled restart procedure.

`PowerState == 0` only means **at least one** axis is disabled; it does not prove
all seven axes are disabled. Use independent per-axis EtherCAT status/controlword
readback before and after the restart. This publisher is not a safety system.