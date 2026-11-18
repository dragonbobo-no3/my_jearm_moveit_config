/* Standalone REAL SDK, private simulation. --load is READ-ONLY lifecycle.
 * No vendor library dependency at link time. Never load a plugin or SDK before
 * OS/config/hash validation. Separate --writer-exit-virtual explicitly enables
 * virtual holding Servo only; never weakens the locked plugin policy.
 * SDK has no verified teardown API: retain SDK until _Exit of private PID ns.
 */
#define _GNU_SOURCE
#include "Base/RobotStruct.h"
#include "../mk2_control_core.h"
#include <dlfcn.h>
#include <errno.h>
#include <float.h>
#include <json-c/json.h>
#include <limits.h>
#include <net/if.h>
#include <openssl/evp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <time.h>
#include <unistd.h>
#include <zmq.h>

#define PROFILE "/tmp/mk2-pro/profile"
#define PLUGIN "/tmp/mk2-pro/libJEServer_v2_sim.so"
#ifndef JE2_SDK_HANDOVER_PROBE
#define JE2_SDK_HANDOVER_PROBE 0
#endif
#if JE2_SDK_HANDOVER_PROBE != 0 && JE2_SDK_HANDOVER_PROBE != 1
#error "Invalid explicit handover experiment build"
#endif
#ifndef JE2_SDK_TEACH_PROBE
#define JE2_SDK_TEACH_PROBE 0
#endif
#if (JE2_SDK_TEACH_PROBE != 0 && JE2_SDK_TEACH_PROBE != 1) || (JE2_SDK_TEACH_PROBE && JE2_SDK_HANDOVER_PROBE)
#error "Invalid exclusive native teach experiment build"
#endif
static const char sdk_sha[] = "40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1";
static const char technology_sha[] = "cb854d244ca3835cce6e992792d229f414bb821edd7f01a4342b21826c451227";
static const char expected_global[] =
    "---\nversion:\n  major: 12\n  minor: 2\n  build: 1\nserial: JE\n"
    "description: Isolated JEServer_v2 SDK lifecycle probe\n"
    "app_path: /tmp/mk2-pro/profile/app\nsimulation: true\nplugin_list: []\n";
static int refuse(const char *reason)
{
    fprintf(stderr, "JE2_SDK_REFUSED: %s; BEFORE SDK/plugin loading\n", reason);
    return 77;
}
static bool read_text(const char *path, char *text, size_t capacity, size_t *length)
{
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    size_t n = fread(text, 1, capacity, f);
    bool ok = !ferror(f) && n < capacity;
    fclose(f);
    if (ok) { text[n] = '\0'; *length = n; }
    return ok;
}
static bool hash_file(const char *path, char output[65])
{
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    bool ok = ctx && EVP_DigestInit_ex(ctx, EVP_sha256(), NULL) == 1;
    unsigned char buffer[8192], digest[EVP_MAX_MD_SIZE];
    unsigned length = 0;
    size_t n;
    while (ok && (n = fread(buffer, 1, sizeof(buffer), f)))
        ok = EVP_DigestUpdate(ctx, buffer, n) == 1;
    ok = ok && !ferror(f) && EVP_DigestFinal_ex(ctx, digest, &length) == 1 && length == 32;
    fclose(f);
    EVP_MD_CTX_free(ctx);
    if (ok) for (unsigned i = 0; i < length; ++i) snprintf(output + 2 * i, 3, "%02x", digest[i]);
    return ok;
}
static bool matches(const char *path, const char *sha)
{
    char actual[65];
    return sha && strlen(sha) == 64 && hash_file(path, actual) && !strcmp(sha, actual);
}
static bool policy_read(const char *path, struct mk2_policy *policy, char text[MK2_FRAME_MAX + 1])
{
    char error[128]; size_t length;
    return read_text(path, text, MK2_FRAME_MAX + 1, &length) &&
        !mk2_policy_parse(text, length, policy, error, sizeof(error));
}
static bool staging_directory(const char *directory)
{
    char canonical[PATH_MAX]; struct stat st;
    const char prefix[] = "/tmp/jeserver-v2-sdk.";
    size_t length = strlen(directory);
    return realpath(directory, canonical) && !strcmp(directory, canonical) &&
        !strncmp(canonical, prefix, sizeof(prefix) - 1) && length >= 8 &&
        !strcmp(directory + length - 8, "/profile") && !stat(directory, &st) &&
        S_ISDIR(st.st_mode) && st.st_uid == getuid();
}
static const char old_eni_line[] =
    "    device_description: \"/home/robot/Work/system/robot_config_new_left_sim/eni7_sim.xml\"";
static const char private_eni_line[] =
    "    device_description: \"" PROFILE "/eni7_sim.xml\"";
static bool exact_line_once(const char *text, const char *line)
{
    const char *at = strstr(text, line);
    return at && (at == text || at[-1] == '\n') &&
        (at[strlen(line)] == '\n' || at[strlen(line)] == '\0') &&
        !strstr(at + strlen(line), line);
}
static bool private_bus_valid(const char *text)
{
    return exact_line_once(text, private_eni_line) && !strstr(text, "/home/robot/");
}
static int prepare_bus(const char *input, const char *directory)
{
    /* A narrow, byte-preserving relocation of ONE known fixture reference,
     * not a generic YAML rewrite and never an edit to installed BusConfig. */
    char text[16384], output[16384], path[PATH_MAX]; size_t length;
    struct stat eni;
    if (!staging_directory(directory) || !read_text(input, text, sizeof(text), &length) ||
        strlen(text) != length || !exact_line_once(text, old_eni_line))
        return refuse("invalid staging directory/expected unique ENI source line");
    if (snprintf(path, sizeof(path), "%s/eni7_sim.xml", directory) >= (int)sizeof(path) ||
        lstat(path, &eni) || !S_ISREG(eni.st_mode)) return refuse("missing regular private ENI copy");
    char *at = strstr(text, old_eni_line);
    size_t prefix = (size_t)(at - text);
    size_t result_length = prefix + strlen(private_eni_line) + length - prefix - strlen(old_eni_line);
    if (result_length >= sizeof(output)) return refuse("relocated bus configuration too large");
    memcpy(output, text, prefix);
    memcpy(output + prefix, private_eni_line, strlen(private_eni_line));
    memcpy(output + prefix + strlen(private_eni_line), at + strlen(old_eni_line),
        length - prefix - strlen(old_eni_line) + 1);
    if (!private_bus_valid(output)) return refuse("bus retains an installed-profile reference");
    if (snprintf(path, sizeof(path), "%s/BusConfig.yaml", directory) >= (int)sizeof(path))
        return refuse("bus output path too long");
    FILE *f = fopen(path, "wx");
    if (!f) return refuse("private BusConfig already exists or cannot be created");
    bool ok = fwrite(output, 1, result_length, f) == result_length;
    if (fclose(f)) ok = false;
    if (!ok) return refuse("private BusConfig write failed");
    puts("JE2_BUS_PREPARED private_eni=1 unique_relocation=1 NO_SDK_LOADED=1");
    return 0;
}
static int prepare_policy_mode(const char *input, const char *directory, bool commands)
{
    /* Host-only JSON utility: fixed temporary staging family, never overwrite
     * an existing policy, and never reach SDK/dlopen from this branch. */
    char path[PATH_MAX], text[MK2_FRAME_MAX + 1], sha[65];
    struct mk2_policy parsed;
    if (commands && !JE2_SDK_HANDOVER_PROBE) return refuse("handover policy requires dedicated experiment binary");
    if (!staging_directory(directory) ||
        !policy_read(input, &parsed, text)) return refuse("invalid staging directory/source strict policy");
    if (snprintf(path, sizeof(path), "%s/DeviceConfig.yaml", directory) >= (int)sizeof(path) ||
        !hash_file(path, sha)) return refuse("cannot hash staged DeviceConfig");
    struct json_object *root = json_tokener_parse(text);
    if (!root) return refuse("source JSON parse failed");
    json_object_object_add(root, "allow_commands", json_object_new_boolean(commands));
    json_object_object_add(root, "command_host", json_object_new_string("127.0.0.1"));
    json_object_object_add(root, "command_port", json_object_new_int(18001));
    json_object_object_add(root, "state_port", json_object_new_int(18000));
    json_object_object_add(root, "device_config_sha256", json_object_new_string(sha));
    const char *encoded = json_object_to_json_string_ext(root, JSON_C_TO_STRING_PLAIN);
    char error[128];
    bool ok = encoded && strlen(encoded) <= MK2_FRAME_MAX &&
        !mk2_policy_parse(encoded, strlen(encoded), &parsed, error, sizeof(error)) && parsed.allow_commands == commands;
    if (ok && snprintf(path, sizeof(path), "%s/mk2_control_bridge.json", directory) < (int)sizeof(path)) {
        FILE *out = fopen(path, "wx");
        if (!out) ok = false;
        else { ok = fputs(encoded, out) >= 0; if (fclose(out)) ok = false; }
    } else ok = false;
    json_object_put(root);
    if (!ok) return refuse("locked policy creation failed (never overwrites)");
    printf("JE2_POLICY_PREPARED allow_commands=%s loopback_only=1 NO_SDK_LOADED=1 HANDOVER_EXPERIMENT=%d\n",
        commands ? "true" : "false", commands);
    return 0;
}
static int prepare_policy(const char *input, const char *directory)
{ return prepare_policy_mode(input, directory, false); }
static int isolation(void)
{
    static const char *names[] = {"mnt", "ipc", "net", "pid", "user"};
    static const char *env[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET", "MK2_HOST_PID", "MK2_HOST_USER"};
    if (getenv("LD_PRELOAD") || getenv("LD_AUDIT") || getenv("LD_LIBRARY_PATH")) return refuse("library injection environment");
    for (size_t i = 0; i < 5; ++i) {
        char path[80], actual[128]; const char *host = getenv(env[i]);
        if (!host || !*host) return refuse("missing host namespace marker");
        snprintf(path, sizeof(path), "/proc/self/ns/%s", names[i]);
        ssize_t n = readlink(path, actual, sizeof(actual) - 1);
        if (n <= 0) return refuse("namespace unreadable");
        actual[n] = '\0';
        if (!strcmp(actual, host)) return refuse("namespace not private");
        printf("JE2_NAMESPACE %s host=%s private=%s\n", names[i], host, actual);
    }
    struct statvfs root; struct statfs shm;
    if (statvfs("/", &root) || !(root.f_flag & ST_RDONLY) ||
        statfs("/dev/shm", &shm) || shm.f_type != 0x01021994 ||
        access("/dev/EtherCAT0", F_OK) == 0 || access("/dev/bus/usb", F_OK) == 0 ||
        access("/home/robot/Work/system/robot_config_new_left/DeviceConfig.yaml", F_OK) == 0)
        return refuse("root/shared-memory/hardware isolation");
    struct if_nameindex *interfaces = if_nameindex();
    if (!interfaces) return refuse("network cannot be inspected");
    bool ok = true;
    for (struct if_nameindex *it = interfaces; it->if_index; ++it) if (strcmp(it->if_name, "lo")) ok = false;
    if_freenameindex(interfaces);
    if (!ok) return refuse("non-loopback interface");
    FILE *f = fopen("/proc/self/status", "r");
    if (!f) return refuse("capabilities unreadable");
    char line[256]; unsigned long long caps = ULLONG_MAX;
    while (fgets(line, sizeof(line), f)) if (sscanf(line, "CapEff:\t%llx", &caps) == 1) break;
    fclose(f);
    if (caps) return refuse("effective capabilities not zero");
    char text[MK2_FRAME_MAX + 1]; size_t length; struct mk2_policy policy;
    if (!read_text(PROFILE "/GlobalConfig.yaml", text, sizeof(text), &length) ||
        (length != sizeof(expected_global) - 1 && length != sizeof(expected_global) - 2) ||
        memcmp(text, expected_global, length)) return refuse("GlobalConfig must be exact simulation and NO plugins");
    if (!policy_read(PROFILE "/mk2_control_bridge.json", &policy, text) || policy.allow_commands != !!JE2_SDK_HANDOVER_PROBE ||
        strcmp(policy.command_host, "127.0.0.1") || policy.command_port != 18001 || policy.state_port != 18000 ||
        !matches(PROFILE "/DeviceConfig.yaml", policy.device_config_sha256)) return refuse("policy must be locked and match staged device");
    char bus[16384]; size_t bus_length;
    if (!read_text(PROFILE "/BusConfig.yaml", bus, sizeof(bus), &bus_length) ||
        strlen(bus) != bus_length || !private_bus_valid(bus) || access(PROFILE "/eni7_sim.xml", R_OK))
        return refuse("bus must reference only the private ENI copy");
    static const char *readonly[] = {PLUGIN, PROFILE "/GlobalConfig.yaml", PROFILE "/DeviceConfig.yaml",
        PROFILE "/BusConfig.yaml", PROFILE "/eni7_sim.xml", PROFILE "/mk2_control_bridge.json"};
    for (size_t i = 0; i < sizeof(readonly) / sizeof(readonly[0]); ++i)
        if (statvfs(readonly[i], &root) || !(root.f_flag & ST_RDONLY)) return refuse("guard input not read-only mounted");
    if (!matches("/lib/libRobotControl.so", sdk_sha) || !matches("/lib/libHYYRobotTechnology.so", technology_sha) ||
        !matches(PLUGIN, getenv("JE2_EXPECTED_PLUGIN_SHA256"))) return refuse("pinned SDK/technology/plugin hash mismatch");
    printf("JE2_ISOLATION_OK root_readonly=1 private_shm=1 hardware_hidden=1 plugins=0 capabilities=0 policy_locked=%d HANDOVER_EXPERIMENT=%d\n",
        !JE2_SDK_HANDOVER_PROBE, JE2_SDK_HANDOVER_PROBE);
    return 0;
}
static double now(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t)) return -1;
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static void *load_symbol(void *library, const char *name)
{
    void *p = dlsym(library, name);
    if (!p) { fprintf(stderr, "JE2_SYMBOL_MISSING %s\n", name); fflush(NULL); _Exit(78); }
    return p;
}
#define SYMBOL(handle, target, name) do { \
    void *symbol = load_symbol(handle, name); \
    _Static_assert(sizeof(target) == sizeof(symbol), "function pointer size"); \
    memcpy(&(target), &symbol, sizeof(target)); \
} while (0)
static bool finite_value(double v) { return v >= -DBL_MAX && v <= DBL_MAX; }
static struct json_object *field(struct json_object *object, const char *name)
{
    struct json_object *value = NULL;
    return json_object_object_get_ex(object, name, &value) ? value : NULL;
}
static bool boolean(struct json_object *o, const char *name, bool expected)
{
    struct json_object *v = field(o, name);
    return v && json_object_is_type(v, json_type_boolean) && !!json_object_get_boolean(v) == expected;
}
static bool string_is(struct json_object *o, const char *name, const char *expected)
{
    struct json_object *v = field(o, name);
    return v && json_object_is_type(v, json_type_string) && !strcmp(json_object_get_string(v), expected);
}
static bool receive_feedback(void *subscriber, bool *state, bool *owner, uint64_t *sequence)
{
    zmq_pollitem_t item = {.socket = subscriber, .events = ZMQ_POLLIN};
    int result = zmq_poll(&item, 1, 10);
    if (result < 0) return errno == EINTR;
    if (!result || !(item.revents & ZMQ_POLLIN)) return true;
    char text[MK2_FRAME_MAX + 1];
    int n = zmq_recv(subscriber, text, sizeof(text) - 1, ZMQ_DONTWAIT);
    if (n <= 6 || n >= (int)sizeof(text)) return false;
    text[n] = '\0';
    struct json_object *root = json_tokener_parse(text + 6);
    if (!root) return false;
    bool ok = false;
    if (!strncmp(text, "State ", 6)) {
        struct json_object *b = field(root, "Bridge"), *robot = field(root, "Robot0");
        ok = b && robot && boolean(b, "simulation_isolated", true) && boolean(b, "configuration_verified", true) &&
            boolean(b, "commands_enabled", false) && boolean(b, "automatic_power_changes", false) &&
            boolean(b, "ownership_handover_supported", false) && string_is(b, "control_state", "locked") &&
            string_is(b, "profile", PROFILE) && field(b, "accepted_joints") &&
            json_object_get_uint64(field(b, "accepted_joints")) == 0 && field(b, "sequence");
        if (ok) { *state = true; *sequence = json_object_get_uint64(field(b, "sequence")); }
    } else if (!strncmp(text, "Owner ", 6)) {
        ok = boolean(root, "experimental", true) && boolean(root, "loopback_only", true) &&
            boolean(root, "automatic_priority_ready", false) && boolean(root, "writer_exit_verified", false) &&
            boolean(root, "pc_reclaim_supported", false) && field(root, "owner") &&
            json_object_get_int(field(root, "owner")) == 0;
        if (ok) *owner = true;
    }
    if (!ok) fprintf(stderr, "JE2_BAD_FEEDBACK %s\n", text);
    json_object_put(root);
    return ok;
}
#include "jeserver_v2_sdk_writer_probe.inc"
#if JE2_SDK_HANDOVER_PROBE
#include "jeserver_v2_sdk_handover_probe.inc"
#endif
#if JE2_SDK_TEACH_PROBE
#include "jeserver_v2_sdk_teach_probe.inc"
#endif
int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc == 4 && !strcmp(argv[1], "--prepare-bus")) return prepare_bus(argv[2], argv[3]);
    if (argc == 4 && !strcmp(argv[1], "--prepare-policy")) return prepare_policy(argv[2], argv[3]);
#if JE2_SDK_TEACH_PROBE
    if (argc != 4 || (strcmp(argv[1], "--check-teach-virtual") && strcmp(argv[1], "--teach-virtual") &&
        strcmp(argv[1], "--teach-external-stop-virtual") && strcmp(argv[1], "--teach-overlap-stop-virtual")) ||
        strcmp(argv[2], "--path") || strcmp(argv[3], PROFILE)) return refuse("dedicated teach mode/profile required");
#elif JE2_SDK_HANDOVER_PROBE
    if (argc == 4 && !strcmp(argv[1], "--prepare-handover-policy")) return prepare_policy_mode(argv[2], argv[3], true);
    if (argc != 4 || (strcmp(argv[1], "--check-handover-virtual") && strcmp(argv[1], "--handover-virtual")) ||
        strcmp(argv[2], "--path") || strcmp(argv[3], PROFILE)) return refuse("dedicated handover mode/profile required");
#else
    if (argc != 4 || (strcmp(argv[1], "--check") && strcmp(argv[1], "--load") &&
        strcmp(argv[1], "--writer-exit-virtual")) ||
        strcmp(argv[2], "--path") || strcmp(argv[3], PROFILE)) return refuse("unknown mode/profile");
#endif
    int result = isolation();
    if (result || !strcmp(argv[1], "--check") || !strcmp(argv[1], "--check-handover-virtual") ||
        !strcmp(argv[1], "--check-teach-virtual")) return result;
    struct rlimit rt;
    if (getrlimit(RLIMIT_RTPRIO, &rt) || rt.rlim_cur < 38) return refuse("realtime ceiling below 38");
    const struct rlimit budget = {.rlim_cur = 100000, .rlim_max = 500000};
    if (setrlimit(RLIMIT_RTTIME, &budget)) return refuse("cannot bound realtime CPU");
    puts("JE2_LOADING_REAL_SDK_AFTER_GUARD");
    void *sdk = dlopen("/lib/libRobotControl.so", RTLD_NOW | RTLD_GLOBAL);
    if (!sdk) { fprintf(stderr, "JE2_SDK_LOAD_FAILED %s\n", dlerror()); fflush(NULL); _Exit(78); }
    int (*parse)(const char *, command_arg *), (*initialize)(command_arg *), (*simulation)(void), (*count)(void);
    int (*dof)(int), (*power)(int), (*move_state)(int), (*driver_error)(int), (*move_error)(int);
    void (*joint)(double *, int);
    SYMBOL(sdk, parse, "commandLineParser1"); SYMBOL(sdk, initialize, "system_initialize");
    SYMBOL(sdk, simulation, "GetSystemSimulationState"); SYMBOL(sdk, count, "robot_getNUM");
    SYMBOL(sdk, dof, "robot_getDOF"); SYMBOL(sdk, power, "GetRobotPowerState");
    SYMBOL(sdk, move_state, "get_robot_move_state"); SYMBOL(sdk, joint, "GetCurrentJoint");
    SYMBOL(sdk, driver_error, "IsRobotDriverError"); SYMBOL(sdk, move_error, "IsRobotMoveError");
    command_arg argument = {0};
    if (parse("--path " PROFILE, &argument) || !argument.path || strcmp(argument.path, PROFILE)) { fflush(NULL); _Exit(78); }
    result = initialize(&argument);
    printf("JE2_SDK_INITIALIZE result=%d\n", result);
    if (result || simulation() != 1 || count() != 1 || dof(0) != MK2_AXES) { fflush(NULL); _Exit(79); }
    int initial_power = power(0), initial_state = move_state(0);
    double initial[MK2_AXES], final[MK2_AXES]; joint(initial, 0);
    if ((initial_power != 0 && initial_power != 1) || initial_state != 0 || driver_error(0) || move_error(0)) { fflush(NULL); _Exit(79); }
    for (int i = 0; i < MK2_AXES; ++i) if (!finite_value(initial[i])) { fflush(NULL); _Exit(79); }
#if JE2_SDK_TEACH_PROBE
    enum teach_stop_source source = !strcmp(argv[1], "--teach-external-stop-virtual") ? TEACH_EXTERNAL_STOP :
        !strcmp(argv[1], "--teach-overlap-stop-virtual") ? TEACH_OVERLAP_STOP : TEACH_NATIVE_STOP;
    result = teach_probe(sdk, source);
    fflush(NULL); _Exit(result);
#elif JE2_SDK_HANDOVER_PROBE
    result = handover_probe(sdk);
    fflush(NULL); _Exit(result);
#endif
    if (!strcmp(argv[1], "--writer-exit-virtual")) {
        result = writer_probe(sdk);
        /* SDK stays mapped until the complete private PID namespace exits. */
        fflush(NULL); _Exit(result);
    }
    printf("JE2_SDK_INITIALIZED simulation=1 axes=7 power=%d state=%d NO_ENABLE_OR_MOTION=1\n", initial_power, initial_state);
    void *context = zmq_ctx_new(), *subscriber = context ? zmq_socket(context, ZMQ_SUB) : NULL;
    const int zero = 0;
    if (!subscriber || zmq_setsockopt(subscriber, ZMQ_LINGER, &zero, sizeof(zero)) ||
        zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0) || zmq_connect(subscriber, "tcp://127.0.0.1:18000")) { fflush(NULL); _Exit(78); }
    void *plugin = dlopen(PLUGIN, RTLD_NOW | RTLD_LOCAL);
    if (!plugin) { fprintf(stderr, "JE2_PLUGIN_LOAD_FAILED %s\n", dlerror()); fflush(NULL); _Exit(78); }
    void (*entry)(void); int (*status)(void); unsigned (*port)(void);
    SYMBOL(plugin, entry, "PluginMain"); SYMBOL(plugin, status, "JEServerV2Status"); SYMBOL(plugin, port, "JEServerV2ButtonPort");
    entry();
    bool state = false, owner = false, ok = true; uint64_t sequence = 0;
    double deadline = now() + 3;
    while (ok && now() < deadline && !(status() == 1 && port() == 16666 && state && owner && sequence >= 5)) {
        /* -1 also means pre-ready initialization; bounded positive ACK below
         * distinguishes ready from failure without racing the new worker. */
        ok = receive_feedback(subscriber, &state, &owner, &sequence);
    }
    ok = ok && status() == 1 && port() == 16666 && state && owner && sequence >= 5;
    printf("JE2_PLUGIN_READY verified=%d state=%d owner=%d sequence=%llu commands_enabled=0\n", ok, state, owner, (unsigned long long)sequence);
    /* No ownership was ever acquired: destructor must only join its worker.
     * Never dlclose the SDK with its internal threads still running. */
    bool unloaded = dlclose(plugin) == 0;
    if (dlopen(PLUGIN, RTLD_NOW | RTLD_NOLOAD)) unloaded = false;
    if (!unloaded) ok = false;
    zmq_close(subscriber); zmq_ctx_term(context);
    joint(final, 0);
    for (int i = 0; i < MK2_AXES; ++i) {
        double delta = final[i] - initial[i];
        if (!finite_value(final[i]) || delta > 1e-8 || delta < -1e-8) ok = false;
    }
    if (power(0) != initial_power || move_state(0) != initial_state || driver_error(0) || move_error(0)) ok = false;
    printf("JE2_SDK_LIFECYCLE_FINISHED result=%s plugin_unloaded=%d SDK_RETAINED_UNTIL_PRIVATE_PROCESS_EXIT=1 NO_ENABLE_OR_MOTION=1\n", ok ? "PASS" : "FAIL", unloaded);
    fflush(NULL); _Exit(ok ? 0 : 79);
}