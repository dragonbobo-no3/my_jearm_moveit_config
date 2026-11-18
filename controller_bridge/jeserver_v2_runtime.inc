/* Included only by the explicit JESERVER_V2_PLUGIN build. Reuses the exact
 * verified policy loader, SDK read adapter, guards and state serializer. */
#include "jeserver_v2.h"
#include "jeserver_v2_transport.h"
#include "jeserver_v2_session.h"
#include <net/if.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <limits.h>
#include <stdlib.h>

static atomic_int je2_plugin_status = ATOMIC_VAR_INIT(0);
static atomic_uint je2_button_port = ATOMIC_VAR_INIT(0);
/* No pointers to worker-owned engine state escape the executor. */
int JEServerV2Status(void) { return atomic_load(&je2_plugin_status); }
unsigned JEServerV2ButtonPort(void) { return atomic_load(&je2_button_port); }

static bool je2_simulation_guard(void)
{
    static const char *names[] = {"mnt", "ipc", "net", "pid", "user"};
    static const char *env[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET", "MK2_HOST_PID", "MK2_HOST_USER"};
    char path[80], actual[128];
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
        const char *host = getenv(env[i]);
        if (!host || !*host) return false;
        snprintf(path, sizeof(path), "/proc/self/ns/%s", names[i]);
        ssize_t n = readlink(path, actual, sizeof(actual) - 1);
        if (n <= 0) return false;
        actual[n] = '\0';
        if (!strcmp(host, actual)) return false;
    }
    struct statvfs root;
    struct statfs shm;
    if (statvfs("/", &root) || !(root.f_flag & ST_RDONLY) || statfs("/dev/shm", &shm) ||
        shm.f_type != 0x01021994 || access("/dev/EtherCAT0", F_OK) == 0 ||
        access("/dev/bus/usb", F_OK) == 0 ||
        access("/home/robot/Work/system/robot_config_new_left/DeviceConfig.yaml", F_OK) == 0) return false;
    struct if_nameindex *interfaces = if_nameindex();
    if (!interfaces) return false;
    bool network_ok = true;
    for (struct if_nameindex *entry = interfaces; entry->if_index; ++entry)
        if (strcmp(entry->if_name, "lo")) network_ok = false;
    if_freenameindex(interfaces);
    if (!network_ok) return false;
    FILE *status = fopen("/proc/self/status", "r");
    if (!status) return false;
    unsigned long long capabilities = ULLONG_MAX;
    char line[256];
    while (fgets(line, sizeof(line), status))
        if (sscanf(line, "CapEff:\t%llx", &capabilities) == 1) break;
    fclose(status);
    /* Only after OS isolation has passed call the SDK simulation/model getters. */
    return capabilities == 0 && GetSystemSimulationState() == 1 &&
        robot_getNUM() == 1 && robot_getDOF(0) == MK2_AXES;
}
static enum je2_writer je2_sdk_writer(void *unused)
{ (void)unused; return JE2_WRITER_UNKNOWN; }
#if JE2_SIM_WRITER_EXPERIMENT
#include "jeserver_v2_writer_adapter.inc"
#endif
static int je2_sdk_jog(void *unused, int axis, int direction)
{
    (void)unused;
    if (set_robot_index(0) || set_robot_teach_coordinate(0)) return -1;
    setMoveThread(1);
    set_teach_move_type(0);
    set_robot_teach_velocity(.01); /* bounded simulation research speed */
    return robot_teach_joint(axis, direction);
}
static int je2_sdk_jog_stop(void *unused)
{ (void)unused; return set_robot_index(0) ? -1 : robot_teach_stop(); }
static struct je2_feedback je2_read_feedback(void)
{
    /* Stamp the beginning, not the end of sequential SDK reads. */
    struct je2_feedback f = {.acquired_at = monotonic_time()};
    f.sample = read_sample();
    return f;
}
#if JE2_SIM_SESSION_EXPERIMENT
#include "jeserver_v2_session.inc"
#endif
#include "jeserver_v2_owner.inc"
static void je2_publish(void *publisher, const struct jeserver_v2 *v,
                        const struct mk2_sample *sample, uint64_t sequence,
                        unsigned port, const struct je2_session *session)
{
    struct mk2_core reported = v->core;
    /* Experimental release callback is not the commissioned legacy/manual
     * handover contract. Do not advertise that capability to old clients. */
    reported.io.stop_run = NULL;
    reported.policy.allow_commands = !session && reported.policy.allow_commands &&
        (v->owner == JE2_AVAILABLE || v->owner == JE2_PC) && v->legacy_open;
    publish_state(publisher, &reported, sample, sequence);
    /* Separate topic: do not advertise old clean-release capability as verified
     * automatic takeover or alter the existing Robot0 feedback schema. */
    char frame[MK2_FRAME_MAX];
    bool instance_exited = false;
    double exit_time = 0;
#if JE2_SIM_WRITER_EXPERIMENT
    const struct je2_writer_adapter *writer = v->raw_io.context;
    instance_exited = writer && writer->task.phase == 3;
    if (writer) exit_time = writer->exit_time;
#endif
    int n = je2_format_owner(frame, sizeof(frame), v, sample, sequence, port, session,
        JE2_SIM_WRITER_EXPERIMENT != 0, instance_exited, exit_time);
    if (n > 0)
        zmq_send(publisher, frame, (size_t)n, ZMQ_DONTWAIT);
}
static void je2_apply_button(struct jeserver_v2 *v, const struct je2_event *event)
{
    double now;
    switch (event->type) {
    case JE2_EVENT_JOG: {
        struct je2_feedback f = je2_read_feedback();
        now = monotonic_time();
        (void)je2_pendant_jog(v, event->axis, event->direction, now, now, &f);
        break;
    }
    case JE2_EVENT_STOP: case JE2_EVENT_DISCONNECT: case JE2_EVENT_REJECT:
        (void)je2_pendant_stop(v, monotonic_time()); break;
    default: break;
    }
}
static void je2_runtime_shutdown(struct jeserver_v2 *v)
{
    /* Actual plugin shutdown is a trusted event, unlike retired PC cleanup. */
    je2_abort(v, MK2_UNLOAD);
}
struct je2_jog_bound {
    bool active;
    double since, joint[MK2_AXES];
};
static void je2_bound_jog(struct jeserver_v2 *v, struct je2_jog_bound *bound,
                         double now, const struct je2_feedback *f)
{
    if (!v->jog_live || v->owner == JE2_FAULT) { bound->active = false; return; }
    if (!isfinite(now) || !f->sample.valid) { je2_abort(v, MK2_STATE_ERROR); return; }
    if (!bound->active) {
        bound->active = true;
        bound->since = now;
        memcpy(bound->joint, f->sample.joint, sizeof(bound->joint));
    }
    for (int i = 0; i < MK2_AXES; ++i)
        if (!isfinite(f->sample.joint[i]) || fabs(f->sample.joint[i] - bound->joint[i]) >= .02) {
            je2_abort(v, MK2_LIMIT); return;
        }
    /* This prototype is NOT a continuous-hold UI: a held connection cannot
     * renew the deadline. Restart/commissioning required after this fault. */
    if (now < bound->since || now - bound->since >= .5) je2_abort(v, MK2_WATCHDOG);
}
static int je2_drain_buttons(struct jeserver_v2 *v, struct je2_transport *button)
{
    for (int count = 0; count < 16; ++count) {
        struct je2_event event;
        int received = je2_transport_next(button, &event);
        if (received < 0) return -1;
        if (!received) return 0;
        je2_apply_button(v, &event);
    }
    /* Budget exhausted: cancellation may still be queued. Never advance the
     * post-exit barrier into jog execution with an undrained event budget. */
    (void)je2_pendant_stop(v, monotonic_time());
    return 0;
}
static void *run_bridge(void *unused)
{
    (void)unused;
    atomic_store(&je2_plugin_status, -1);
    /* Default refuses PC->pendant release: UNKNOWN writer + NULL StopRun.
     * Only the explicit private-simulation build below installs the observer;
     * there is no physical or environment-force opt-in. */
    struct mk2_io io = {.start = sdk_start, .joint = sdk_joint, .stop = sdk_stop, .end = sdk_end};
    struct je2_io teach = {.writer = je2_sdk_writer, .jog = je2_sdk_jog, .jog_stop = je2_sdk_jog_stop};
#if JE2_SIM_WRITER_EXPERIMENT
    struct je2_writer_adapter writer;
    if (je2_writer_prepare(&writer)) return NULL;
    io.context = &writer;
    io.start = je2_writer_start;
    io.stop_run = je2_writer_stop_run;
    teach.context = &writer;
    teach.writer = je2_writer_observe;
#endif
    struct jeserver_v2 engine;
    if (je2_init(&engine, &policy, &io, &teach)) return NULL;
    const struct je2_session *session_info = NULL;
#if JE2_SIM_SESSION_EXPERIMENT
    struct je2_session session;
    if (je2_session_init(&session)) return NULL;
    session_info = &session;
#endif
    struct je2_transport button;
    if (je2_transport_open(&button, 16666)) return NULL;
    void *context = zmq_ctx_new(), *publisher = NULL, *subscriber = NULL;
    if (!context) goto close_button;
    publisher = zmq_socket(context, ZMQ_PUB);
    subscriber = zmq_socket(context, ZMQ_SUB);
    if (!publisher || !subscriber) goto close_sockets;
    const int linger = 0, send_hwm = 2, receive_hwm = 16;
    const int64_t maximum_frame = MK2_FRAME_MAX;
    char endpoint[128];
    snprintf(endpoint, sizeof(endpoint), "tcp://127.0.0.1:%d", policy.state_port);
    if (zmq_setsockopt(publisher, ZMQ_LINGER, &linger, sizeof(linger)) ||
        zmq_setsockopt(subscriber, ZMQ_LINGER, &linger, sizeof(linger)) ||
        zmq_setsockopt(publisher, ZMQ_SNDHWM, &send_hwm, sizeof(send_hwm)) ||
        zmq_setsockopt(subscriber, ZMQ_RCVHWM, &receive_hwm, sizeof(receive_hwm)) ||
        zmq_setsockopt(subscriber, ZMQ_MAXMSGSIZE, &maximum_frame, sizeof(maximum_frame)) ||
        zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0) || zmq_bind(publisher, endpoint)) goto close_sockets;
    if (strcmp(policy.command_host, "127.0.0.1") || policy.state_port == 16666 || policy.command_port == 16666)
        goto close_sockets;
    snprintf(endpoint, sizeof(endpoint), "tcp://127.0.0.1:%d", policy.command_port);
    if (zmq_connect(subscriber, endpoint)) goto close_sockets;
    atomic_store(&je2_button_port, button.bound_port);
    atomic_store(&je2_plugin_status, 1);
    fprintf(stderr, "[JEServer_v2] private-simulation worker ready; button=127.0.0.1:%u; "
        "writer_experiment=%d; production priority NOT commissioned; no power/clear/direct recovery API\n",
        button.bound_port, JE2_SIM_WRITER_EXPERIMENT);
    uint64_t sequence = 0;
    struct je2_jog_bound jog_bound = {0};
    double next_publish = 0;
    bool failure_logged = false;
    while (!atomic_load(&stopping)) {
        if (!backend_verified()) break;
        /* Process cancellation/pendant intent BEFORE accepting any PC frame. */
        if (je2_drain_buttons(&engine, &button)) break;
        struct je2_feedback f = je2_read_feedback();
        double now = monotonic_time();
        je2_bound_jog(&engine, &jog_bound, now, &f);
        je2_tick(&engine, now, &f);
        je2_bound_jog(&engine, &jog_bound, now, &f);
        if (!isfinite(now)) break;
        if (now >= next_publish) {
            je2_publish(publisher, &engine, &f.sample, ++sequence, button.bound_port, session_info);
            next_publish = now + .01;
        }
        zmq_pollitem_t item = {.socket = subscriber, .events = ZMQ_POLLIN};
        if (zmq_poll(&item, 1, 2) < 0) break;
        if (item.revents & ZMQ_POLLIN) {
            zmq_msg_t message;
            zmq_msg_init(&message);
            int size = zmq_msg_recv(&message, subscriber, ZMQ_DONTWAIT);
            bool multipart = size >= 0 && zmq_msg_more(&message);
            if (size >= 0) {
#if JE2_SIM_SESSION_EXPERIMENT
                (void)je2_session_receive(&engine, &session, zmq_msg_data(&message),
                    zmq_msg_size(&message), multipart);
#else
                f = je2_read_feedback(); now = monotonic_time();
                (void)je2_pc_frame(&engine, 0, multipart ? NULL : zmq_msg_data(&message),
                    multipart ? 0 : zmq_msg_size(&message), now, &f);
#endif
            }
            zmq_msg_close(&message);
#if !JE2_SIM_SESSION_EXPERIMENT
            if (multipart) break; /* No unbounded drain or partial execution. */
#endif
        }
        log_state_failure(&engine.core, &failure_logged);
    }
    je2_runtime_shutdown(&engine);
    log_state_failure(&engine.core, &failure_logged);
    atomic_store(&je2_plugin_status, -1);
    atomic_store(&je2_button_port, 0);
close_sockets:
    if (subscriber) zmq_close(subscriber);
    if (publisher) zmq_close(publisher);
    zmq_ctx_term(context);
close_button:
    je2_transport_close(&button);
#if JE2_SIM_WRITER_EXPERIMENT
    je2_task_watch_close(&writer.task);
#endif
    return NULL;
}