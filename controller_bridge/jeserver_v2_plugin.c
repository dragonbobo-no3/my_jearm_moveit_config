/* Loadable experimental plugin for the PRIVATE SDK SIMULATION ONLY.
 * One SDK process, one mutation executor. This is not a physical build.
 * Native pendant compatibility, writer exit proof and PC re-claim transport
 * remain uncommissioned; unsupported handover is explicitly disabled.
 */
#if defined(MK2_PHYSICAL_CONTROL) || defined(MK2_PHYSICAL_HANDOVER) || defined(PROFILE_ROOT)
#error "JEServer_v2 experimental plugin has a fixed private simulation profile; no physical overrides"
#endif
#define PROFILE_ROOT "/tmp/mk2-pro/profile"
#define MK2_SIMULATION_ONLY 1
#define JESERVER_V2_PLUGIN 1
#ifndef JE2_SIM_WRITER_EXPERIMENT
#define JE2_SIM_WRITER_EXPERIMENT 0
#endif
#if JE2_SIM_WRITER_EXPERIMENT != 0 && JE2_SIM_WRITER_EXPERIMENT != 1
#error "Invalid explicit simulation writer experiment build"
#endif
#ifndef JE2_SIM_SESSION_EXPERIMENT
#define JE2_SIM_SESSION_EXPERIMENT 0
#endif
#if JE2_SIM_SESSION_EXPERIMENT != 0 && JE2_SIM_SESSION_EXPERIMENT != 1
#error "Invalid explicit simulation session experiment build"
#endif
#include "mk2_control_plugin.c"