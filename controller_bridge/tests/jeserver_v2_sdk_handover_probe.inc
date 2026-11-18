/* Dedicated real-SDK VIRTUAL experiment. Never included in the default probe.
 * All traffic is inside the existing five-namespace/hash/RO/hardware guard.
 * The probe enables the virtual robot; only PluginMain performs Servo/jog.
 * Text buttons are a prototype, NOT the complete native pendant protocol.
 */
#include "../jeserver_v2.h"
#include <arpa/inet.h>
#include <sys/socket.h>
struct handover_feedback {
    bool state_seen, owner_seen, exited, stop_sent;
    int owner, move_state;
    uint64_t accepted, sequence, generation, dropped;
    double exit_time;
    char phase[32];
};
static bool handover_receive(void *subscriber, struct handover_feedback *f)
{
    zmq_pollitem_t item = {.socket = subscriber, .events = ZMQ_POLLIN};
    int rc = zmq_poll(&item, 1, 5);
    if (rc < 0) return errno == EINTR;
    if (!rc) return true;
    char text[MK2_FRAME_MAX + 1];
    int n = zmq_recv(subscriber, text, sizeof(text) - 1, ZMQ_DONTWAIT);
    if (n <= 6 || n >= (int)sizeof(text) - 1) return false;
    text[n] = '\0';
    struct json_object *root = json_tokener_parse(text + 6);
    if (!root) return false;
    bool ok = false;
    if (!strncmp(text, "State ", 6)) {
        struct json_object *b = field(root, "Bridge"), *r = field(root, "Robot0");
        ok = b && r && boolean(b, "simulation_isolated", true) &&
            boolean(b, "configuration_verified", true) && boolean(b, "automatic_power_changes", false) &&
            boolean(b, "ownership_handover_supported", false) && string_is(b, "profile", PROFILE) &&
            field(b, "control_state") && field(b, "accepted_joints") && field(b, "sequence") &&
            field(r, "PowerState") && json_object_get_int(field(r, "PowerState")) == 1 &&
            field(r, "DriverError") && json_object_get_int(field(r, "DriverError")) == 0 &&
            field(r, "MoveError") && json_object_get_int(field(r, "MoveError")) == 0 && field(r, "MoveState");
        if (ok) {
            snprintf(f->phase, sizeof(f->phase), "%s", json_object_get_string(field(b, "control_state")));
            f->accepted = json_object_get_uint64(field(b, "accepted_joints"));
            f->sequence = json_object_get_uint64(field(b, "sequence"));
            f->state_seen = true;
            f->move_state = json_object_get_int(field(r, "MoveState"));
            if (f->stop_sent && f->move_state == 7)
                puts("JE2_HANDOVER_STOP_NOT_IDLE sdk_state=7 ACCEPT_AS_IDLE=0 NO_RECOVERY_AFTER_STOP=1");
            ok = strcmp(f->phase, "fault") != 0 && f->move_state >= 0 && f->move_state != 7;
        }
    } else if (!strncmp(text, "Owner ", 6)) {
        ok = boolean(root, "experimental", true) && boolean(root, "loopback_only", true) &&
            boolean(root, "automatic_priority_ready", false) && boolean(root, "writer_exit_verified", false) &&
            boolean(root, "pc_reclaim_supported", false) && boolean(root, "writer_observer_enabled", true) &&
            field(root, "owner") && field(root, "generation") && field(root, "dropped_pc") &&
            field(root, "writer_instance_exited") && field(root, "writer_exit_time");
        if (ok) {
            f->owner = json_object_get_int(field(root, "owner"));
            f->generation = json_object_get_uint64(field(root, "generation"));
            f->dropped = json_object_get_uint64(field(root, "dropped_pc"));
            f->exited = json_object_get_boolean(field(root, "writer_instance_exited"));
            f->exit_time = json_object_get_double(field(root, "writer_exit_time"));
            f->owner_seen = true;
            ok = f->owner != JE2_FAULT;
        }
    }
    if (!ok) fprintf(stderr, "JE2_HANDOVER_BAD_FEEDBACK %s\n", text);
    json_object_put(root);
    return ok;
}
static bool handover_send(void *publisher, const char *text)
{ return zmq_send(publisher, text, strlen(text), ZMQ_DONTWAIT) == (int)strlen(text); }
static int handover_probe(void *sdk)
{
    int (*select_robot)(int), (*enable)(void), (*power)(int), (*state)(int), (*driver)(int), (*error)(int);
    void (*actual)(double *, int), (*move_thread)(int);
    SYMBOL(sdk, select_robot, "set_robot_index"); SYMBOL(sdk, enable, "robot_teach_enable");
    SYMBOL(sdk, power, "GetRobotPowerState"); SYMBOL(sdk, state, "get_robot_move_state");
    SYMBOL(sdk, driver, "IsRobotDriverError"); SYMBOL(sdk, error, "IsRobotMoveError");
    SYMBOL(sdk, actual, "GetCurrentJoint"); SYMBOL(sdk, move_thread, "setMoveThread");
    puts("JE2_HANDOVER_EXPERIMENT REAL_SDK=1 VIRTUAL_ONLY=1 PLUGIN_EXECUTOR=1 PROTOTYPE_BUTTONS=1 PRODUCTION_READY=0");
    if (select_robot(0)) return 79;
    move_thread(1);
    int enabled = enable();
    printf("JE2_HANDOVER_VIRTUAL_ENABLE result=%d\n", enabled);
    if (enabled) return 79;
    for (unsigned i = 0; i < 50 && power(0) != 1; ++i) writer_pause();
    if (power(0) != 1 || state(0) != 0 || driver(0) || error(0)) return 79;
    double hold[MK2_AXES]; actual(hold, 0);
    for (int i = 0; i < MK2_AXES; ++i) if (!finite_value(hold[i])) return 79;
    void *context = zmq_ctx_new(), *pub = NULL, *sub = NULL, *plugin = NULL;
    int button = -1;
    bool ok = false, unloaded = false;
    bool pc_verified = false, pendant_verified = false, motion_verified = false, stop_idle_verified = false;
    struct handover_feedback feedback = {0};
    if (!context) goto cleanup;
    pub = zmq_socket(context, ZMQ_PUB); sub = zmq_socket(context, ZMQ_SUB);
    const int zero = 0, hwm = 32;
    if (!pub || !sub || zmq_setsockopt(pub, ZMQ_LINGER, &zero, sizeof(zero)) ||
        zmq_setsockopt(sub, ZMQ_LINGER, &zero, sizeof(zero)) ||
        zmq_setsockopt(sub, ZMQ_RCVHWM, &hwm, sizeof(hwm)) ||
        zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0) ||
        zmq_bind(pub, "tcp://127.0.0.1:18001") || zmq_connect(sub, "tcp://127.0.0.1:18000")) goto cleanup;
    plugin = dlopen(PLUGIN, RTLD_NOW | RTLD_LOCAL);
    if (!plugin) { fprintf(stderr, "JE2_HANDOVER_PLUGIN_LOAD_FAILED %s\n", dlerror()); goto cleanup; }
    void (*entry)(void); int (*status)(void);
    SYMBOL(plugin, entry, "PluginMain"); SYMBOL(plugin, status, "JEServerV2Status");
    entry();
    double deadline = now() + 3;
    /* Status=-1 also covers the worker's pre-ready initialization interval.
     * It is NOT a terminal startup result. Require positive status plus actual
     * State/Owner ACKs within the same bounded deadline; never assume ready. */
    while (now() < deadline && !(status() == 1 && feedback.state_seen && feedback.owner_seen && feedback.sequence >= 10)) {
        if (!handover_receive(sub, &feedback)) goto cleanup;
    }
    if (status() != 1 || !feedback.state_seen || !feedback.owner_seen || feedback.sequence < 10 || feedback.owner != JE2_AVAILABLE) goto cleanup;
    printf("JE2_HANDOVER_PLUGIN_READY sequence=%llu owner=%d\n", (unsigned long long)feedback.sequence, feedback.owner);
    deadline = now() + 1;
    double next_switch = 0;
    while (now() < deadline && strcmp(feedback.phase, "waiting_first_joint")) {
        if (now() >= next_switch) {
            if (!handover_send(pub, "Switch {\"Switch\":true}")) goto cleanup;
            next_switch = now() + .03;
        }
        if (!handover_receive(sub, &feedback)) goto cleanup;
    }
    if (strcmp(feedback.phase, "waiting_first_joint")) goto cleanup;
    char joint[1024];
    int n = snprintf(joint, sizeof(joint),
        "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]}}",
        hold[0], hold[1], hold[2], hold[3], hold[4], hold[5], hold[6]);
    if (n < 0 || n >= (int)sizeof(joint) || !handover_send(pub, joint)) goto cleanup;
    deadline = now() + .15;
    while (now() < deadline && !(feedback.owner == JE2_PC && feedback.accepted == 1))
        if (!handover_receive(sub, &feedback)) goto cleanup;
    if (feedback.owner != JE2_PC || feedback.accepted != 1 || state(0) != 13) goto cleanup;
    pc_verified = true;
    actual(hold, 0);
    printf("JE2_HANDOVER_PC_HOLD owner=%d accepted=1 sdk_state=13\n", feedback.owner);
    button = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(16666)};
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (button < 0 || connect(button, (struct sockaddr *)&address, sizeof(address)) ||
        send(button, "tv49!", 5, MSG_NOSIGNAL) != 5) goto cleanup;
    deadline = now() + 1;
    while (now() < deadline && feedback.owner != JE2_PENDANT)
        if (!handover_receive(sub, &feedback)) goto cleanup;
    if (feedback.owner != JE2_PENDANT || feedback.generation != 2 || !feedback.exited ||
        !finite_value(feedback.exit_time) || feedback.exit_time <= 0 || now() - feedback.exit_time < .1) goto cleanup;
    pendant_verified = true;
    printf("JE2_HANDOVER_PENDANT owner=%d generation=2 KERNEL_INSTANCE_EXIT=1 post_exit_elapsed=%.9f\n",
        feedback.owner, now() - feedback.exit_time);
    if (!handover_send(pub, "Switch {\"Switch\":false}") ||
        !handover_send(pub, "Release {\"Release\":true}") ||
        !handover_send(pub, "Joint malformed")) goto cleanup;
    bool moved = false;
    deadline = now() + .2;
    while (now() < deadline && !(moved && feedback.dropped >= 3)) {
        if (!handover_receive(sub, &feedback)) goto cleanup;
        double q[MK2_AXES]; actual(q, 0);
        for (int i = 0; i < MK2_AXES; ++i) {
            double delta = q[i] - hold[i];
            if (!finite_value(q[i]) || delta >= .02 || delta <= -.02 ||
                (i && (delta > 1e-5 || delta < -1e-5))) goto cleanup;
            if (!i && (delta > 1e-6 || delta < -1e-6)) moved = true;
        }
    }
    if (!moved || feedback.dropped < 3 || feedback.owner != JE2_PENDANT ||
        send(button, "tv33!", 5, MSG_NOSIGNAL) != 5) goto cleanup;
    motion_verified = true;
    feedback.stop_sent = true;
    printf("JE2_HANDOVER_JOG real_joint_motion=1 stale_pc_fenced=%llu STOP_SENT=1\n", (unsigned long long)feedback.dropped);
    deadline = now() + .3;
    while (now() < deadline && state(0) != 0)
        if (!handover_receive(sub, &feedback)) goto cleanup;
    if (state(0) != 0) goto cleanup;
    double stationary[MK2_AXES]; actual(stationary, 0);
    for (unsigned tick = 0; tick < 12; ++tick) {
        double q[MK2_AXES]; actual(q, 0);
        if (!handover_receive(sub, &feedback) || power(0) != 1 || state(0) != 0 || driver(0) || error(0)) goto cleanup;
        for (int i = 0; i < MK2_AXES; ++i)
            if (!finite_value(q[i]) || q[i] - stationary[i] > 1e-5 || q[i] - stationary[i] < -1e-5) goto cleanup;
        writer_pause();
    }
    stop_idle_verified = true;
    ok = true;
cleanup:
    if (!ok) printf("JE2_HANDOVER_FAILURE owner=%d phase=%s accepted=%llu exited=%d dropped=%llu\n",
        feedback.owner, feedback.phase, (unsigned long long)feedback.accepted, feedback.exited, (unsigned long long)feedback.dropped);
    if (button >= 0) close(button);
    if (plugin) {
        unloaded = dlclose(plugin) == 0;
        void *still = dlopen(PLUGIN, RTLD_NOW | RTLD_NOLOAD);
        if (still) { unloaded = false; dlclose(still); }
    }
    if (pub) zmq_close(pub);
    if (sub) zmq_close(sub);
    if (context) zmq_ctx_term(context);
    ok = ok && unloaded && power(0) == 1 && state(0) == 0 && !driver(0) && !error(0);
    printf("JE2_HANDOVER_STAGES pc_hold=%d pendant_after_exit=%d actual_jog_and_fencing=%d stop_idle=%d overall=%s\n",
        pc_verified, pendant_verified, motion_verified, stop_idle_verified, ok ? "PASS" : "FAIL");
    printf("JE2_HANDOVER_FINISHED result=%s plugin_unloaded=%d SDK_RETAINED_UNTIL_PRIVATE_PROCESS_EXIT=1 PRODUCTION_READY=0\n",
        ok ? "PASS" : "FAIL", unloaded);
    return ok ? 0 : 79;
}