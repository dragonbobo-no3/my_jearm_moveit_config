#ifndef JE2_LOOPBACK_SERVER_H
#define JE2_LOOPBACK_SERVER_H
#ifdef __cplusplus
extern "C" {
#endif
#include "jeserver_v2.h"
#include "jeserver_v2_session.h"

/* Test-only memory-backed motion callbacks, not an SDK emulator or a plugin.
 * No SDK/vendor header dependency. Never installed or used by a production
 * ROS node; the isolated action test uses it as its fake controller. */
struct je2_loopback_server {
    struct jeserver_v2 engine;
    struct je2_session session;
    struct mk2_sample sample;
    double now;
    uint64_t observation;
    unsigned reads, starts, joints, stops, ends, releases, jogs;
    int joint_result, release_result;
    enum je2_writer writer;
    double target[MK2_AXES], data_time;
};
int je2_loopback_init(struct je2_loopback_server *);
int je2_loopback_receive(struct je2_loopback_server *, const char *, size_t, bool more, double now);
int je2_loopback_snapshot(struct je2_loopback_server *, double now, char *, size_t);
int je2_loopback_button(struct je2_loopback_server *, const char *, size_t, double now);
void je2_loopback_tick(struct je2_loopback_server *, double now);
#ifdef __cplusplus
}
#endif
#endif