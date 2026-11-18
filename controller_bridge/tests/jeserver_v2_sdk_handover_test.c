/* Dedicated experiment policy/CLI negative tests; NEVER loads an SDK. */
#define JE2_SDK_HANDOVER_PROBE 1
#define main probe_main_under_test
#include "jeserver_v2_sdk_probe.c"
#undef main
#include <assert.h>
static void check_stop_feedback(void)
{
    /* Constructed JSON is ONLY a parser fixture, never SDK/task exit evidence.
     * Real inproc sockets, no TCP or vendor library loaded. */
    void *ctx = zmq_ctx_new(); assert(ctx);
    void *tx = zmq_socket(ctx, ZMQ_PUSH), *rx = zmq_socket(ctx, ZMQ_PULL);
    assert(tx && rx && !zmq_bind(rx, "inproc://stop-state-parser") && !zmq_connect(tx, "inproc://stop-state-parser"));
    const int states[] = {0, 7, 7};
    const char *phases[] = {"ready", "ready", "fault"};
    for (unsigned i = 0; i < 3; ++i) {
        char text[1024];
        int n = snprintf(text, sizeof(text),
            "State {\"Bridge\":{\"simulation_isolated\":true,\"configuration_verified\":true,"
            "\"automatic_power_changes\":false,\"ownership_handover_supported\":false,"
            "\"profile\":\"%s\",\"control_state\":\"%s\",\"accepted_joints\":1,\"sequence\":43},"
            "\"Robot0\":{\"PowerState\":1,\"DriverError\":0,\"MoveError\":0,\"MoveState\":%d}}",
            PROFILE, phases[i], states[i]);
        assert(n > 0 && n < (int)sizeof(text) && zmq_send(tx, text, n, 0) == n);
        struct handover_feedback f = {.stop_sent = true};
        assert(handover_receive(rx, &f) == (i == 0));
        assert(f.state_seen && f.move_state == states[i]);
    }
    assert(!zmq_close(tx) && !zmq_close(rx) && !zmq_ctx_term(ctx));
    puts("PASS: STOP state7 never accepted as idle, even with misleading ready phase; constructed inproc parser fixtures ONLY");
}
static void check_policy(const char *directory, struct json_object *source, bool commands)
{
    char path[PATH_MAX], text[MK2_FRAME_MAX + 1]; struct mk2_policy p;
    assert(snprintf(path, sizeof(path), "%s/mk2_control_bridge.json", directory) < (int)sizeof(path));
    assert(policy_read(path, &p, text) && p.allow_commands == commands);
    assert(!strcmp(p.command_host, "127.0.0.1") && p.command_port == 18001 && p.state_port == 18000);
    struct json_object *result = json_tokener_parse(text);
    assert(result && json_object_object_length(result) == json_object_object_length(source));
    json_object_object_foreach(source, key, value) {
        if (!strcmp(key, "allow_commands") || !strcmp(key, "command_host") || !strcmp(key, "command_port") ||
            !strcmp(key, "state_port") || !strcmp(key, "device_config_sha256")) continue;
        assert(json_object_equal(value, field(result, key))); /* ALL limits and guards. */
    }
    char sha[65]; assert(hash_file(path, sha));
    assert(prepare_policy_mode("/does/not/exist", directory, true) == 77);
    assert(matches(path, sha));
    json_object_put(result);
}
int main(int argc, char **argv)
{
    assert(argc == 3);
    check_stop_feedback();
    const char *names[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET", "MK2_HOST_PID", "MK2_HOST_USER"};
    for (unsigned i = 0; i < 5; ++i) assert(!unsetenv(names[i]));
    struct json_object *source = json_object_from_file(argv[1]); assert(source);
    char source_sha[65]; assert(hash_file(argv[1], source_sha));
    for (unsigned commands = 0; commands <= 1; ++commands) {
        char parent[PATH_MAX], directory[PATH_MAX], path[PATH_MAX];
        assert(snprintf(parent, sizeof(parent), "%s/handover-%u", argv[2], commands) < (int)sizeof(parent));
        assert(!mkdir(parent, 0700));
        assert(snprintf(directory, sizeof(directory), "%s/profile", parent) < (int)sizeof(directory));
        assert(!mkdir(directory, 0700));
        assert(snprintf(path, sizeof(path), "%s/DeviceConfig.yaml", directory) < (int)sizeof(path));
        FILE *file = fopen(path, "wx"); assert(file);
        assert(fputs("abc", file) >= 0 && !fclose(file));
        assert(!prepare_policy_mode(argv[1], directory, commands));
        check_policy(directory, source, commands);
        char output[PATH_MAX], sha[65];
        assert(snprintf(output, sizeof(output), "%s/mk2_control_bridge.json", directory) < (int)sizeof(output));
        assert(hash_file(output, sha));
        assert(prepare_policy_mode(argv[1], directory, commands) == 77 && matches(output, sha));
    }
    char *modes[] = {"--check-handover-virtual", "--handover-virtual", "--load", "--writer-exit-virtual", "--check"};
    for (unsigned i = 0; i < 5; ++i) {
        char *arguments[] = {argv[0], modes[i], "--path", PROFILE, NULL};
        assert(probe_main_under_test(4, arguments) == 77);
    }
    /* Outside the dedicated staging family cannot create a command policy. */
    assert(prepare_policy_mode(argv[1], "/tmp", true) == 77);
    assert(matches(argv[1], source_sha));
    json_object_put(source);
    puts("PASS: dedicated handover compile; strict policy/ALL limits preserved, exclusive outputs, host and legacy modes refused; NO SDK loaded");
    return 0;
}