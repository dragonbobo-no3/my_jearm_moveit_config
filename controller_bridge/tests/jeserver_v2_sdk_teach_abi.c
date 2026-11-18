/* Compile-only check against the controller's actual vendor headers.
 * Never link or run this translation unit; it has no SDK calls. */
#include "HYYRobotInterface.h"
#define CHECK_API(name, type) _Static_assert(_Generic(&(name), type: 1, default: 0), #name " signature mismatch")
CHECK_API(set_robot_index, int (*)(int));
CHECK_API(set_robot_teach_coordinate, int (*)(int));
CHECK_API(robot_teach_enable, int (*)(void));
CHECK_API(GetSystemSimulationState, int (*)(void));
CHECK_API(GetRobotPowerState, int (*)(int));
CHECK_API(get_robot_move_state, int (*)(int));
CHECK_API(IsRobotDriverError, int (*)(int));
CHECK_API(IsRobotMoveError, int (*)(int));
CHECK_API(robot_teach_joint, int (*)(int, int));
CHECK_API(robot_teach_stop, int (*)(void));
CHECK_API(RobotStop, int (*)(int));
CHECK_API(set_robot_teach_velocity, void (*)(double));
CHECK_API(set_teach_move_type, void (*)(int));
CHECK_API(setMoveThread, void (*)(int));
CHECK_API(GetCurrentJoint, void (*)(double *, int));