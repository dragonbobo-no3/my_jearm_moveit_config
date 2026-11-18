#define _POSIX_C_SOURCE 200809L
/* FULL PluginMain/worker/destructor integration, fake SDK definitions ONLY.
 * No production includes, guard overrides, SDK library, or OS interposition. */
#include "jeserver_v2_fake_sdk.h"
#include "../jeserver_v2.h"
#ifdef JE2_FAKE_SESSION
#include "../je2_session_client.h"
#endif
#include <arpa/inet.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <json-c/json.h>
#include <limits.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <time.h>
#include <unistd.h>
#include <zmq.h>

static double now(void)
{
    struct timespec t;
    assert(clock_gettime(CLOCK_MONOTONIC, &t) == 0);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static void pause_tick(void)
{
    struct timespec t = {.tv_nsec = 1000000};
    while (nanosleep(&t, &t)) assert(errno == EINTR);
}
/* Independent pre-dlopen refusal: never create the fixed profile on the host.
 * Markers are real host readlinks supplied by the script, not fabricated IDs.
 * The unchanged production guard must ALSO pass inside PluginMain. */
static bool isolated(void)
{
    const char *names[] = {"mnt", "ipc", "net", "pid", "user"};
    const char *markers[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET", "MK2_HOST_PID", "MK2_HOST_USER"};
    if (getenv("LD_PRELOAD") || getenv("LD_AUDIT") || getenv("LD_LIBRARY_PATH")) return false;
    for (size_t i = 0; i < 5; ++i) {
        char path[80], actual[128];
        const char *host = getenv(markers[i]);
        if (!host || !*host) return false;
        snprintf(path, sizeof(path), "/proc/self/ns/%s", names[i]);
        ssize_t n = readlink(path, actual, sizeof(actual) - 1);
        if (n <= 0) return false;
        actual[n] = '\0';
        if (!strcmp(host, actual)) return false;
    }
    struct statvfs root;
    struct statfs shm;
    if (statvfs("/", &root) || !(root.f_flag & ST_RDONLY) ||
        statfs("/dev/shm", &shm) || shm.f_type != 0x01021994 ||
        access("/dev/EtherCAT0", F_OK) == 0 || access("/dev/bus/usb", F_OK) == 0 ||
        access("/home/robot/Work/system/robot_config_new_left/DeviceConfig.yaml", F_OK) == 0) return false;
    struct if_nameindex *interfaces = if_nameindex();
    if (!interfaces) return false;
    bool ok = true;
    for (struct if_nameindex *p = interfaces; p->if_index; ++p)
        if (strcmp(p->if_name, "lo")) ok = false;
    if_freenameindex(interfaces);
    FILE *f = fopen("/proc/self/status", "r");
    if (!f) return false;
    char line[256];
    unsigned long long caps = ULLONG_MAX;
    while (fgets(line, sizeof(line), f))
        if (sscanf(line, "CapEff:\t%llx", &caps) == 1) break;
    assert(fclose(f) == 0);
    return ok && caps == 0;
}
static void fixture(void)
{
    /* Exclusive creation only AFTER isolation: source/profile never on host. */
    assert(mkdir("/tmp/mk2-pro", 0700) == 0);
    assert(mkdir("/tmp/mk2-pro/profile", 0700) == 0);
    FILE *f = fopen("/tmp/mk2-pro/profile/DeviceConfig.yaml", "wx");
    assert(f && fclose(f) == 0);
    f = fopen("/tmp/mk2-pro/profile/mk2_control_bridge.json", "wx");
    assert(f);
    assert(fputs("{\"version\":1,\"allow_commands\":true,"
        "\"command_host\":\"127.0.0.1\",\"command_port\":18001,\"state_port\":18000,"
        "\"device_config_sha256\":\"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855\","
        "\"watchdog_seconds\":0.2,\"first_command_timeout\":1.0,"
        "\"min_dt\":0.005,\"max_dt\":0.05,\"clock_skew\":0.1,"
        "\"first_hold_tolerance\":0.001,\"following_tolerance\":0.02,"
        "\"servo_q\":1.0,\"servo_r\":0.0001,\"limits\":{"
        "\"lower\":[-1,-1,-1,-1,-1,-1,-1],\"upper\":[1,1,1,1,1,1,1],"
        "\"velocity\":[0.2,0.2,0.2,0.2,0.2,0.2,0.2],"
        "\"acceleration\":[0.4,0.4,0.4,0.4,0.4,0.4,0.4]}}", f) >= 0);
    assert(fclose(f) == 0);
}
static void symbol(void *handle, const char *name, void *target, size_t size)
{
    dlerror(); void *address = dlsym(handle, name);
    const char *error = dlerror();
    if (error) fprintf(stderr, "%s: %s\n", name, error);
    assert(!error && address && size == sizeof(address));
    memcpy(target, &address, size);
}
static unsigned tasks(void)
{
    DIR *d = opendir("/proc/self/task"); assert(d);
    unsigned n = 0;
    struct dirent *entry;
    while ((entry = readdir(d))) if (entry->d_name[0] != '.') ++n;
    assert(closedir(d) == 0); return n;
}
struct session {
    void *handle, *context, *sub, *pub;
    int (*status)(void);
    unsigned (*port)(void);
    unsigned baseline;
    int owner;
    uint64_t generation, sequence, accepted;
#ifdef JE2_FAKE_SESSION
    char boot[33], client[33];
    uint64_t session_generation, last_sequence, dropped;
    struct je2_client *pc_client;
    int snapshot_fault;
#endif
    char phase[32];
    bool state_seen, owner_seen;
};
static struct json_object *field(struct json_object *o, const char *name)
{
    struct json_object *v = NULL;
    assert(json_object_object_get_ex(o, name, &v) && v); return v;
}
static void receive(struct session *s)
{
    zmq_pollitem_t item = {.socket = s->sub, .events = ZMQ_POLLIN};
    int rc = zmq_poll(&item, 1, 5);
    assert(rc >= 0 || errno == EINTR);
    if (rc <= 0 || !(item.revents & ZMQ_POLLIN)) return;
    char text[MK2_FRAME_MAX + 1];
    int n = zmq_recv(s->sub, text, sizeof(text) - 1, ZMQ_DONTWAIT);
    assert(n > 6 && n < (int)sizeof(text)); text[n] = '\0';
    struct json_object *o = json_tokener_parse(text + 6); assert(o);
    if (!strncmp(text, "State ", 6)) {
        struct json_object *b = field(o, "Bridge");
        assert(json_object_get_boolean(field(b, "simulation_isolated")));
        assert(json_object_get_boolean(field(b, "configuration_verified")));
        assert(!json_object_get_boolean(field(b, "automatic_power_changes")));
        assert(!json_object_get_boolean(field(b, "ownership_handover_supported")));
        assert(!strcmp(json_object_get_string(field(b, "profile")), "/tmp/mk2-pro/profile"));
        s->sequence = json_object_get_uint64(field(b, "sequence"));
        s->accepted = json_object_get_uint64(field(b, "accepted_joints"));
        snprintf(s->phase, sizeof(s->phase), "%s", json_object_get_string(field(b, "control_state")));
        s->state_seen = true;
    } else {
        assert(!strncmp(text, "Owner ", 6));
        const char *flags[] = {"automatic_priority_ready", "writer_exit_verified", "pc_reclaim_supported"};
        for (size_t i = 0; i < 3; ++i) assert(!json_object_get_boolean(field(o, flags[i])));
        assert(json_object_get_boolean(field(o, "loopback_only")));
        assert(json_object_get_int(field(o, "button_port")) == 16666);
        s->owner = json_object_get_int(field(o, "owner"));
        s->generation = json_object_get_uint64(field(o, "generation"));
    #ifdef JE2_FAKE_SESSION
        assert(json_object_get_boolean(field(o, "session_transport_enabled")));
        const char *boot = json_object_get_string(field(o, "session_boot"));
        const char *client = json_object_get_string(field(o, "session_client"));
        assert(strlen(boot) == 32 && (strlen(client) == 0 || strlen(client) == 32));
        snprintf(s->boot, sizeof(s->boot), "%s", boot);
        snprintf(s->client, sizeof(s->client), "%s", client);
        s->session_generation = json_object_get_uint64(field(o, "session_generation"));
        s->last_sequence = json_object_get_uint64(field(o, "session_last_sequence"));
        s->dropped = json_object_get_uint64(field(o, "dropped_pc"));
        s->snapshot_fault = json_object_get_int(field(o, "snapshot_fault"));
        if (s->pc_client) (void)je2_client_observe(s->pc_client, text, (size_t)n, now());
    #endif
        s->owner_seen = true;
    }
    json_object_put(o);
}
static void reset_fake(void)
{
    /* Only between fully joined/unmapped workers; no memset of live atomics. */
    atomic_store(&fake.getters, 0); atomic_store(&fake.mutations, 0);
    atomic_store(&fake.starts, 0); atomic_store(&fake.joints, 0);
    atomic_store(&fake.stops, 0); atomic_store(&fake.ends, 0);
    atomic_store(&fake.jogs, 0); atomic_store(&fake.jog_stops, 0);
    atomic_store(&fake.state, 0);
#ifdef JE2_FAKE_WRITER_TASKS
    assert(!fake_writer.count && !fake_writer.alive);
    fake_writer.stuck = fake_writer.duplicate = fake_writer.missing = false;
    fake_writer.log_initializations = fake_writer.stop_runs = 0;
    fake_writer.jog_stop_state = 0;
    fake.start_result = fake.end_result = fake.stop_result = fake.driver_error = 0;
#endif
}
static void open_session(struct session *s, const char *library)
{
    memset(s, 0, sizeof(*s)); reset_fake(); s->owner = -1;
    s->context = zmq_ctx_new(); assert(s->context);
    s->sub = zmq_socket(s->context, ZMQ_SUB);
    s->pub = zmq_socket(s->context, ZMQ_PUB); assert(s->sub && s->pub);
    int zero = 0;
    assert(zmq_setsockopt(s->sub, ZMQ_LINGER, &zero, sizeof(zero)) == 0);
    assert(zmq_setsockopt(s->pub, ZMQ_LINGER, &zero, sizeof(zero)) == 0);
    assert(zmq_setsockopt(s->sub, ZMQ_SUBSCRIBE, "", 0) == 0);
    assert(zmq_bind(s->pub, "tcp://127.0.0.1:18001") == 0);
    assert(zmq_connect(s->sub, "tcp://127.0.0.1:18000") == 0);
    s->baseline = tasks();
    s->handle = dlopen(library, RTLD_NOW | RTLD_LOCAL);
    if (!s->handle) fprintf(stderr, "dlopen: %s\n", dlerror());
    assert(s->handle);
    void (*entry)(void) = NULL;
    symbol(s->handle, "PluginMain", &entry, sizeof(entry));
    symbol(s->handle, "JEServerV2Status", &s->status, sizeof(s->status));
    symbol(s->handle, "JEServerV2ButtonPort", &s->port, sizeof(s->port));
    assert(s->status() == 0 && s->port() == 0);
    const char *forbidden[] = {"RobotPower", "RobotPoweroff", "ClearRobotError", "RobotStopRecover"};
    for (size_t i = 0; i < 4; ++i) assert(!dlsym(s->handle, forbidden[i]));
    entry(); entry(); /* Actual duplicate guard; exactly one worker. */
    double deadline = now() + 2;
    while (now() < deadline && !(s->status() == 1 && s->port() == 16666 && s->state_seen && s->owner_seen)) receive(s);
    assert(s->status() == 1 && s->port() == 16666 && s->state_seen && s->owner_seen);
    assert(s->owner == JE2_AVAILABLE && s->generation == 1 && fake.mutations == 0);
}
static int button(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0); assert(fd >= 0);
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(16666), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    assert(connect(fd, (struct sockaddr *)&address, sizeof(address)) == 0); return fd;
}
static void send_button(int fd, const char *text)
{ assert(send(fd, text, strlen(text), MSG_NOSIGNAL) == (ssize_t)strlen(text)); }
static void close_after_unload(int fd)
{
    /* Production intentionally has no SO_REUSEADDR. Reset the test client's
     * remaining half-connection AFTER unload so the previous server endpoint
     * does not linger in TCP TIME_WAIT across fresh sequential DSO loads.
     * Disconnect test cases below still use ordinary graceful close/EOF. */
    struct linger reset = {.l_onoff = 1, .l_linger = 0};
    assert(setsockopt(fd, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset)) == 0);
    assert(close(fd) == 0);
}
static void await_owner(struct session *s, int owner)
{
    double deadline = now() + 1;
    do { receive(s); } while (now() < deadline && s->owner != owner);
    assert(s->owner == owner);
}
static void await_counter(struct session *s, atomic_uint *value, unsigned expected)
{
    double deadline = now() + 1;
    while (now() < deadline && atomic_load(value) < expected) receive(s);
    assert(atomic_load(value) == expected);
}
static void close_session(struct session *s, const char *library)
{
    double began = now();
    assert(dlclose(s->handle) == 0); /* Production destructor joins worker. */
    assert(now() - began < 1);
    assert(!dlopen(library, RTLD_NOW | RTLD_NOLOAD));
#ifdef JE2_FAKE_WRITER_TASKS
    fake_writer_cleanup(); /* Fixture cleanup only AFTER actual DSO unload. */
#endif
    /* All worker and worker-owned ZMQ threads gone, not merely SDK quiescence. */
    double deadline = now() + .2;
    while (tasks() != s->baseline && now() < deadline) pause_tick();
    assert(tasks() == s->baseline);
    unsigned getters = fake.getters, mutations = fake.mutations;
    deadline = now() + .05;
    while (now() < deadline) {
        assert(fake.getters == getters && fake.mutations == mutations); pause_tick();
    }
    assert(zmq_close(s->sub) == 0 && zmq_close(s->pub) == 0 && zmq_ctx_term(s->context) == 0);
    /* Listener closed as well; sequential fresh DSO loads rebind same ports. */
    int fd = socket(AF_INET, SOCK_STREAM, 0); assert(fd >= 0);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(16666), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    assert(connect(fd, (struct sockaddr *)&a, sizeof(a)) == -1 && errno == ECONNREFUSED);
    assert(close(fd) == 0);
}
static void pendant_case(const char *library, bool pending, int cancellation)
{
    struct session s; open_session(&s, library);
    int fd = button(); double sent = now(); send_button(fd, "tv49!");
    await_owner(&s, JE2_EXIT_BARRIER);
    assert(s.generation == 2 && fake.jogs == 0 && fake.mutations == 0);
    if (!pending) {
        await_counter(&s, &fake.jogs, 1);
        assert(now() - sent >= .1); /* Actual fresh 100ms worker barrier. */
        await_owner(&s, JE2_PENDANT);
    }
    if (cancellation == 1) send_button(fd, "tv33!");
    if (cancellation == 2) { assert(close(fd) == 0); fd = -1; }
    if (cancellation) {
        if (!pending) await_counter(&s, &fake.jog_stops, 1);
        /* Process real State/Owner traffic beyond the barrier, not a guessed
         * shell delay: cancelled pending intent must never turn into a jog. */
        double deadline = now() + .15;
        uint64_t sequence = s.sequence;
        while (now() < deadline) receive(&s);
        assert(s.sequence > sequence);
        assert(fake.jogs == (pending ? 0u : 1u));
        assert(fake.jog_stops == (pending ? 0u : 1u));
        assert(fake.stops == 0 && fake.ends == 0);
    }
    close_session(&s, library);
    assert(fake.jogs == (pending ? 0u : 1u));
    assert(fake.jog_stops == (pending ? 0u : 1u));
    /* Explicit STOP + qualified idle retires jog; active unload alone stops. */
    assert(fake.stops == (!pending && !cancellation ? 1u : 0u));
    assert(fake.ends == 0 && fake.starts == 0 && fake.joints == 0);
    if (fd >= 0) close_after_unload(fd);
    printf("PASS: full worker %s jog, %s; once-only cleanup, joined/unmapped, no post-unload SDK calls\n",
        pending ? "pending" : "live", cancellation == 1 ? "TCP STOP" : cancellation == 2 ? "TCP disconnect" : "active dlclose");
}
#ifndef JE2_FAKE_SESSION
static void pc_case(const char *library, int scenario)
{
    struct session s; open_session(&s, library);
#ifdef JE2_FAKE_WRITER_TASKS
    assert(fake_writer.log_initializations == 1);
    fake_writer.stuck = scenario == 3;
    fake.start_result = scenario == 4 ? -9 : 0;
    fake_writer.missing = scenario == 5;
    fake_writer.duplicate = scenario == 6;
    fake_writer.jog_stop_state = scenario == 7 ? 7 : 0;
#else
    (void)scenario;
#endif
    const char *sw = "Switch {\"Switch\":true}";
    double deadline = now() + 1, next = 0;
    /* PUB slow-joiner handled by retransmission UNTIL actual State ACK, never
     * by assuming a subscription is ready after an arbitrary delay. */
    while (now() < deadline && strcmp(s.phase, "waiting_first_joint")) {
        if (now() >= next) { assert(zmq_send(s.pub, sw, strlen(sw), 0) >= 0); next = now() + .02; }
        receive(&s);
    }
    assert(!strcmp(s.phase, "waiting_first_joint"));
    assert(!fake.starts);
    const char *joint = "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}";
    assert(zmq_send(s.pub, joint, strlen(joint), 0) >= 0);
#ifdef JE2_FAKE_WRITER_TASKS
    if (scenario >= 4 && scenario <= 6) {
        await_owner(&s, JE2_FAULT);
        assert(fake.starts == 1 && fake.joints == 0 && fake.ends == 1 && fake.stops == 1 && !fake.jogs);
        close_session(&s, library);
        assert(fake.ends == 1 && fake.stops == 1);
        printf("PASS: writer experiment partial/missing/duplicate Start scenario=%d fails closed, cleanup once\n", scenario);
        return;
    }
#endif
    deadline = now() + .15;
    while (now() < deadline && s.accepted != 1) receive(&s);
    assert(s.accepted == 1 && fake.starts == 1 && fake.joints == 1);
    await_owner(&s, JE2_PC);
    int fd = button(); send_button(fd, "tv49!");
#ifdef JE2_FAKE_WRITER_TASKS
    await_owner(&s, JE2_YIELDING);
    assert(s.generation == 2 && fake_writer.stop_runs == 1 && !fake.jogs);
    if (scenario == 1) send_button(fd, "tv33!");
    if (scenario == 2) { assert(close(fd) == 0); fd = -1; }
    if (scenario == 3) {
        deadline = now() + 1.5;
        while (now() < deadline && s.owner != JE2_FAULT) receive(&s);
        assert(s.owner == JE2_FAULT && !fake.jogs && fake.ends == 1 && fake.stops == 1);
        assert(fake_writer.alive == 1); /* SDK End=0 while real task is alive. */
    } else {
        await_owner(&s, JE2_PENDANT);
        assert(fake.ends == 1 && !fake.stops && fake_writer.stop_runs == 1);
        if (scenario == 0 || scenario == 7) {
            await_counter(&s, &fake.jogs, 1);
            assert(fake_writer.alive == 0);
            /* Retired generation zero traffic cannot stop the new owner. */
            const char *retired[] = {"Switch {\"Switch\":false}", "Release {\"Release\":true}", "Joint malformed"};
            for (unsigned i = 0; i < 3; ++i)
                assert(zmq_send(s.pub, retired[i], strlen(retired[i]), 0) >= 0);
            deadline = now() + .04;
            while (now() < deadline) receive(&s);
            assert(!fake.stops && fake.ends == 1 && fake.joints == 1);
            send_button(fd, "tv33!");
            await_counter(&s, &fake.jog_stops, 1);
            if (scenario == 7) {
                /* Actual fixed SDK produced this state after native STOP.
                 * Do NOT treat it as idle, clear it, or use StopRun again. */
                await_owner(&s, JE2_FAULT);
                assert(fake.state == 7 && fake.stops == 1 && fake_writer.stop_runs == 1);
                send_button(fd, "tv49!"); /* A new jog cannot silently recover. */
            }
        }
        deadline = now() + .15;
        while (now() < deadline) receive(&s);
        assert(fake.jogs == (scenario == 0 || scenario == 7 ? 1u : 0u));
    }
    close_session(&s, library);
    if (fd >= 0) close_after_unload(fd);
    assert(fake.ends == 1 && fake.joints == 1);
    if (scenario == 7) {
        assert(fake.state == 7 && fake.stops == 1 && fake.jog_stops == 1 && fake_writer.stop_runs == 1);
        puts("PASS: native STOP -> state7 stays FAULT; no second jog, recovery/StopRun or duplicate cleanup");
    }
    printf("PASS: full writer experiment scenario=%d, kernel tasks + ZMQ/TCP + real worker, no premature jog\n", scenario);
#else
    /* Remain inside watchdog while observing publications past a jog barrier.
     * Unknown writer capability must deny pendant without releasing PC servo. */
    deadline = now() + .12;
    uint64_t sequence = s.sequence;
    while (now() < deadline) {
        receive(&s);
        assert(s.owner == JE2_PC && s.generation == 1);
        assert(!fake.jogs && !fake.jog_stops && !fake.stops && !fake.ends);
    }
    assert(s.sequence > sequence && fake.starts == 1 && fake.joints == 1);
    close_session(&s, library);
    assert(fake.stops == 1 && fake.ends == 1 && !fake.jogs);
    close_after_unload(fd);
    puts("PASS: real ZMQ PUB Switch/Joint + State ACK; PC-owned unknown-writer pendant denied; dlclose stop=1/end=1");
#endif
}
#else
#include "jeserver_v2_session_sandbox_test.inc"
#endif
int main(int argc, char **argv)
{
    assert(setvbuf(stdout, NULL, _IOLBF, 0) == 0);
    if (!isolated()) {
        fputs("REFUSE: sandbox markers/OS isolation absent; BEFORE dlopen/profile/SDK calls\n", stderr);
        return 77;
    }
    assert(argc == 4 && !strcmp(argv[2], "--path") && !strcmp(argv[3], "/tmp/mk2-pro/profile"));
    assert(fake.getters == 0 && fake.mutations == 0);
    fixture();
    for (int cancellation = 0; cancellation < 3; ++cancellation) {
        pendant_case(argv[1], true, cancellation);
        pendant_case(argv[1], false, cancellation);
    }
#ifdef JE2_FAKE_SESSION
    session_cases(argv[1]);
#elif defined(JE2_FAKE_WRITER_TASKS)
    for (int scenario = 0; scenario < 8; ++scenario) pc_case(argv[1], scenario);
#else
    pc_case(argv[1], 0);
#endif
    assert(unlink("/tmp/mk2-pro/profile/mk2_control_bridge.json") == 0);
    assert(unlink("/tmp/mk2-pro/profile/DeviceConfig.yaml") == 0);
    assert(rmdir("/tmp/mk2-pro/profile") == 0 && rmdir("/tmp/mk2-pro") == 0);
    puts("PASS: actual PluginMain lifecycle in five REAL private namespaces; FAKE SDK ONLY, no enable exports");
    return 0;
}