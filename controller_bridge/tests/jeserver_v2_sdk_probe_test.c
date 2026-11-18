/* Offline only: real JSON/core and real negative C guard, no SDK substitutes.
 * Rename only the entry point to expose static helpers, never OS/SDK symbols.
 * CLI children execute the separately compiled, unmodified probe as well.
 */
#define main je2_probe_main_under_test
#include "jeserver_v2_sdk_probe.c"
#undef main
#include <sys/wait.h>

#define REQUIRE(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s (errno=%d)\n", __FILE__, __LINE__, #condition, errno); \
        exit(1); \
    } \
} while (0)

static const char *binary, *artifacts, *staging;
static unsigned cases;
static const char abc_sha[] =
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

static void path_join(char output[PATH_MAX], const char *parent, const char *name)
{
    REQUIRE(snprintf(output, PATH_MAX, "%s/%s", parent, name) < PATH_MAX);
}
static void write_fixture(const char *path, const char *text)
{
    FILE *f = fopen(path, "wx");
    REQUIRE(f != NULL);
    REQUIRE(fputs(text, f) >= 0);
    REQUIRE(fclose(f) == 0);
}
static void write_json(const char *path, struct json_object *root)
{
    const char *text = json_object_to_json_string_ext(root, JSON_C_TO_STRING_PLAIN);
    REQUIRE(text != NULL);
    write_fixture(path, text);
}
static void profile(char output[PATH_MAX], const char *name)
{
    char parent[PATH_MAX], device[PATH_MAX];
    path_join(parent, staging, name);
    REQUIRE(mkdir(parent, 0700) == 0);
    path_join(output, parent, "profile");
    REQUIRE(mkdir(output, 0700) == 0);
    path_join(device, output, "DeviceConfig.yaml");
    /* A generated harmless fixture, with an independent known SHA256 vector. */
    write_fixture(device, "abc");
}
static void absent_policy(const char *directory)
{
    char output[PATH_MAX]; struct stat st;
    path_join(output, directory, "mk2_control_bridge.json");
    errno = 0;
    REQUIRE(lstat(output, &st) == -1 && errno == ENOENT);
}
static void unchanged(const char *path, const char *before)
{
    char after[65];
    REQUIRE(hash_file(path, after));
    REQUIRE(strcmp(before, after) == 0);
}

enum invocation { CLI, HELPER, GUARD, BUS_HELPER };
static void run_case(const char *name, enum invocation kind, char *const arguments[],
                     int expected, const char *message)
{
    char log[PATH_MAX], text[16384]; size_t length;
    path_join(log, artifacts, name);
    REQUIRE(fflush(NULL) == 0);
    pid_t pid = fork();
    REQUIRE(pid >= 0);
    if (pid == 0) {
        FILE *f = fopen(log, "wx");
        if (!f || dup2(fileno(f), STDOUT_FILENO) < 0 ||
            dup2(fileno(f), STDERR_FILENO) < 0 || fclose(f)) _exit(120);
        /* Remove markers only in the child. LD environment must already be
         * absent on entry; it is never changed by this test executable. */
        const char *markers[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET",
            "MK2_HOST_PID", "MK2_HOST_USER", "JE2_EXPECTED_PLUGIN_SHA256"};
        for (size_t i = 0; i < sizeof(markers) / sizeof(markers[0]); ++i)
            if (unsetenv(markers[i])) _exit(121);
        alarm(10);
        if (kind == CLI) {
            execv(binary, arguments);
            _exit(122);
        }
        int result = kind == GUARD ? isolation() : kind == BUS_HELPER ?
            prepare_bus(arguments[2], arguments[3]) : prepare_policy(arguments[2], arguments[3]);
        if (fflush(NULL)) _exit(123);
        _exit(result);
    }
    int status;
    REQUIRE(waitpid(pid, &status, 0) == pid);
    REQUIRE(read_text(log, text, sizeof(text), &length));
    if (!WIFEXITED(status) || WEXITSTATUS(status) != expected || !strstr(text, message)) {
        fprintf(stderr, "%s: status=%d expected=%d\n%s\n", name, status, expected, text);
        REQUIRE(false);
    }
    static const char *forbidden[] = {"JE2_LOADING_REAL_SDK", "JE2_SDK_LOAD_FAILED",
        "JE2_SYMBOL_MISSING", "JE2_SDK_INITIAL", "JE2_PLUGIN_", "JE2_SDK_LIFECYCLE",
        "JE2_ISOLATION_OK", "JE2_NAMESPACE", "AddressSanitizer", "runtime error:",
        "LeakSanitizer"};
    for (size_t i = 0; i < sizeof(forbidden) / sizeof(forbidden[0]); ++i)
        REQUIRE(strstr(text, forbidden[i]) == NULL);
    if (expected != 0) {
        REQUIRE(strstr(text, "JE2_POLICY_PREPARED") == NULL);
        REQUIRE(strstr(text, "JE2_BUS_PREPARED") == NULL);
    }
    printf("PASS %s (exit=%d, no SDK/guard-success markers)\n", name, expected);
    ++cases;
}
static void prepare_case(const char *name, enum invocation kind, const char *input,
                         const char *directory, bool success)
{
    char *args[] = {(char *)binary, "--prepare-policy", (char *)input, (char *)directory, NULL};
    run_case(name, kind, args, success ? 0 : 77,
             success ? "JE2_POLICY_PREPARED allow_commands=false loopback_only=1 NO_SDK_LOADED=1" :
                       "JE2_SDK_REFUSED:");
}
static bool replaced_field(const char *key)
{
    return !strcmp(key, "allow_commands") || !strcmp(key, "command_host") ||
        !strcmp(key, "command_port") || !strcmp(key, "state_port") ||
        !strcmp(key, "device_config_sha256");
}
static void verify_policy(const char *directory, struct json_object *source)
{
    char output[PATH_MAX], device[PATH_MAX], text[MK2_FRAME_MAX + 1];
    struct mk2_policy parsed;
    path_join(output, directory, "mk2_control_bridge.json");
    path_join(device, directory, "DeviceConfig.yaml");
    REQUIRE(policy_read(output, &parsed, text));
    REQUIRE(!parsed.allow_commands);
    REQUIRE(!strcmp(parsed.command_host, "127.0.0.1"));
    REQUIRE(parsed.command_port == 18001 && parsed.state_port == 18000);
    REQUIRE(!strcmp(parsed.device_config_sha256, abc_sha));
    REQUIRE(matches(device, abc_sha));
    struct json_object *generated = json_tokener_parse(text);
    REQUIRE(generated != NULL);
    REQUIRE(json_object_object_length(source) == json_object_object_length(generated));
    /* Compare EVERY other field, including the complete nested limit arrays,
     * rather than checking only selected defaults or structure padding. */
    json_object_object_foreach(source, key, value) {
        if (!replaced_field(key)) {
            struct json_object *actual = NULL;
            REQUIRE(json_object_object_get_ex(generated, key, &actual));
            REQUIRE(json_object_equal(value, actual));
        }
    }
    json_object_put(generated);
}
static struct json_object *clone_json(struct json_object *source)
{
    struct json_object *copy = json_tokener_parse(json_object_to_json_string(source));
    REQUIRE(copy != NULL);
    return copy;
}
static void invalid_json_case(const char *name, const char *text)
{
    char input[PATH_MAX], directory[PATH_MAX], before[65];
    path_join(input, artifacts, name);
    write_fixture(input, text);
    REQUIRE(hash_file(input, before));
    profile(directory, name);
    char log_name[128];
    REQUIRE(snprintf(log_name, sizeof(log_name), "%s.log", name) < (int)sizeof(log_name));
    prepare_case(log_name, CLI, input, directory, false);
    absent_policy(directory);
    unchanged(input, before);
}

/* Independent golden bytes: do not derive expected output from production's
 * relocation constants or its validator. Include mode/type/PDO and whitespace
 * sentinels so a YAML reserialization or broader edit cannot pass unnoticed. */
#define BUS_OLD "    device_description: \"/home/robot/Work/system/robot_config_new_left_sim/eni7_sim.xml\""
#define BUS_PRIVATE "    device_description: \"/tmp/mk2-pro/profile/eni7_sim.xml\""
#define BUS_PREFIX "---\n# retain whitespace and ordering\nbuses:\n  - mode: 0  # simulation\n    type: EtherCAT\n"
#define BUS_SUFFIX "\n    PDO:\n      rx: [0x6040, 0x607a]\n      tx: [0x6041, 0x6064]\n    period: 0.001000\n\n# trailing spaces  \n"
static const char bus_source[] = BUS_PREFIX BUS_OLD BUS_SUFFIX;
static const char bus_expected[] = BUS_PREFIX BUS_PRIVATE BUS_SUFFIX;

static void absent_bus(const char *directory)
{
    char path[PATH_MAX]; struct stat st;
    path_join(path, directory, "BusConfig.yaml");
    errno = 0;
    REQUIRE(lstat(path, &st) == -1 && errno == ENOENT);
}
static void bus_profile(char directory[PATH_MAX], const char *name)
{
    char eni[PATH_MAX];
    profile(directory, name);
    path_join(eni, directory, "eni7_sim.xml");
    write_fixture(eni, "<?xml version=\"1.0\"?><EtherCATConfig><!-- copied fixture --></EtherCATConfig>\n");
}
static void bus_case(const char *name, enum invocation kind, const char *input,
                     const char *directory, bool success)
{
    char *args[] = {(char *)binary, "--prepare-bus", (char *)input, (char *)directory, NULL};
    run_case(name, kind, args, success ? 0 : 77,
             success ? "JE2_BUS_PREPARED private_eni=1 unique_relocation=1 NO_SDK_LOADED=1" :
                       "JE2_SDK_REFUSED:");
}
static void verify_bus(const char *directory, const char *expected)
{
    char path[PATH_MAX], text[16384]; size_t length; struct stat st;
    path_join(path, directory, "BusConfig.yaml");
    REQUIRE(lstat(path, &st) == 0 && S_ISREG(st.st_mode) && st.st_uid == getuid());
    REQUIRE(read_text(path, text, sizeof(text), &length));
    REQUIRE(length == strlen(expected) && !memcmp(text, expected, length));
}
static void bus_tests(void)
{
    char input[PATH_MAX], directory[PATH_MAX], output[PATH_MAX], eni[PATH_MAX];
    char source_hash[65], eni_hash[65], before[65], invalid[PATH_MAX], target[PATH_MAX];
    struct stat source_stat, st;
    path_join(input, artifacts, "source-BusConfig.yaml");
    write_fixture(input, bus_source);
    REQUIRE(chmod(input, 0440) == 0);
    REQUIRE(lstat(input, &source_stat) == 0 && S_ISREG(source_stat.st_mode));
    REQUIRE(hash_file(input, source_hash));
    for (unsigned i = 0; i < 2; ++i) {
        bus_profile(directory, i ? "bus-cli" : "bus-helper");
        path_join(eni, directory, "eni7_sim.xml");
        REQUIRE(hash_file(eni, eni_hash));
        bus_case(i ? "bus-cli.log" : "bus-helper.log", i ? CLI : BUS_HELPER,
                 input, directory, true);
        verify_bus(directory, bus_expected);
        unchanged(input, source_hash);
        unchanged(eni, eni_hash);
        path_join(output, directory, "BusConfig.yaml");
        REQUIRE(hash_file(output, before));
        REQUIRE(lstat(output, &st) == 0);
        ino_t inode = st.st_ino;
        bus_case(i ? "bus-existing-cli.log" : "bus-existing-helper.log",
                 i ? CLI : BUS_HELPER, input, directory, false);
        unchanged(output, before);
        REQUIRE(chmod(output, 0400) == 0);
        bus_case(i ? "bus-readonly-cli.log" : "bus-readonly-helper.log",
                 i ? CLI : BUS_HELPER, input, directory, false);
        unchanged(output, before);
        REQUIRE(lstat(output, &st) == 0 && S_ISREG(st.st_mode) &&
                st.st_ino == inode && (st.st_mode & 0777) == 0400);
    }
    /* Reject ambiguous/unsupported source bytes before creating any output. */
    static const struct { const char *name, *text; } bad[] = {
        {"bus-duplicate", BUS_PREFIX BUS_OLD "\n" BUS_OLD BUS_SUFFIX},
        {"bus-missing-line", BUS_PREFIX "    period: 0.001\n"},
        {"bus-unknown-line", BUS_PREFIX "    device_description: \"unknown.xml\"\n"},
        {"bus-other-installed", BUS_PREFIX "    device_description: \"/home/robot/Work/system/robot_config_new_left/eni7_sim.xml\"\n"},
        {"bus-residual-installed", BUS_PREFIX BUS_OLD BUS_SUFFIX "extra: /home/robot/Work/system/other.xml\n"},
        {"bus-commented-line", BUS_PREFIX "#" BUS_OLD BUS_SUFFIX},
        {"bus-wrong-indent", BUS_PREFIX " " BUS_OLD BUS_SUFFIX},
        {"bus-trailing-comment", BUS_PREFIX BUS_OLD " # not exact\n"},
        {"bus-crlf-line", BUS_PREFIX BUS_OLD "\r\n"},
        {"bus-ambiguous-private", BUS_PREFIX BUS_OLD "\n" BUS_PRIVATE BUS_SUFFIX},
        {"bus-already-private", BUS_PREFIX BUS_PRIVATE BUS_SUFFIX},
        {"bus-empty", ""}
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        char path[PATH_MAX], hash[65], log[128];
        path_join(path, artifacts, bad[i].name);
        write_fixture(path, bad[i].text);
        REQUIRE(hash_file(path, hash));
        bus_profile(directory, bad[i].name);
        REQUIRE(snprintf(log, sizeof(log), "%s.log", bad[i].name) < (int)sizeof(log));
        bus_case(log, i % 2 ? CLI : BUS_HELPER, path, directory, false);
        absent_bus(directory);
        unchanged(path, hash);
    }
    bus_profile(directory, "bus-no-eni");
    path_join(eni, directory, "eni7_sim.xml");
    REQUIRE(unlink(eni) == 0);
    bus_case("bus-no-eni.log", CLI, input, directory, false);
    absent_bus(directory);
    REQUIRE(mkdir(eni, 0700) == 0);
    bus_case("bus-directory-eni.log", BUS_HELPER, input, directory, false);
    absent_bus(directory);
    bus_profile(invalid, "bus-real-eni");
    path_join(target, invalid, "eni7_sim.xml");
    REQUIRE(hash_file(target, eni_hash));
    bus_profile(directory, "bus-symlink-eni");
    path_join(eni, directory, "eni7_sim.xml");
    REQUIRE(unlink(eni) == 0 && symlink(target, eni) == 0);
    bus_case("bus-symlink-eni.log", CLI, input, directory, false);
    absent_bus(directory);
    unchanged(target, eni_hash);
    REQUIRE(lstat(eni, &st) == 0 && S_ISLNK(st.st_mode));

    for (unsigned i = 0; i < 2; ++i) {
        bus_profile(directory, i ? "bus-dangling-output" : "bus-symlink-output");
        path_join(output, directory, "BusConfig.yaml");
        path_join(target, artifacts, i ? "bus-absent-target" : "bus-sentinel");
        if (!i) { write_fixture(target, "sentinel unchanged\n"); REQUIRE(hash_file(target, before)); }
        REQUIRE(symlink(target, output) == 0);
        bus_case(i ? "bus-dangling-output.log" : "bus-symlink-output.log",
                 i ? BUS_HELPER : CLI, input, directory, false);
        REQUIRE(lstat(output, &st) == 0 && S_ISLNK(st.st_mode));
        if (i) REQUIRE(lstat(target, &st) == -1 && errno == ENOENT);
        else unchanged(target, before);
    }
    /* Same-owner directories still must be in the canonical staging family. */
    path_join(directory, artifacts, "bus-outside");
    REQUIRE(mkdir(directory, 0700) == 0);
    path_join(target, directory, "profile");
    REQUIRE(mkdir(target, 0700) == 0);
    path_join(eni, target, "eni7_sim.xml");
    write_fixture(eni, "regular copied ENI\n");
    bus_case("bus-outside-family.log", CLI, input, target, false);
    absent_bus(target);
    bus_profile(directory, "bus-canonical");
    path_join(invalid, directory, ".");
    bus_case("bus-dot-path.log", BUS_HELPER, input, invalid, false);
    path_join(invalid, directory, "");
    bus_case("bus-trailing-slash.log", CLI, input, invalid, false);
    path_join(invalid, staging, "bus-linked-profile");
    REQUIRE(symlink(directory, invalid) == 0);
    bus_case("bus-linked-profile.log", CLI, input, invalid, false);
    path_join(target, staging, "bus-canonical");
    path_join(invalid, staging, "bus-linked-parent");
    REQUIRE(symlink(target, invalid) == 0);
    path_join(target, invalid, "profile");
    bus_case("bus-linked-parent.log", BUS_HELPER, input, target, false);
    bus_case("bus-not-profile.log", CLI, input, staging, false);
    absent_bus(staging);
    absent_bus(directory);
    path_join(invalid, artifacts, "bus-missing-input");
    bus_case("bus-missing-input.log", CLI, invalid, directory, false);
    absent_bus(directory);
    char *short_args[] = {(char *)binary, "--prepare-bus", input, NULL};
    char *extra_args[] = {(char *)binary, "--prepare-bus", input, directory, "extra", NULL};
    run_case("bus-short-args.log", CLI, short_args, 77, "unknown mode/profile");
    run_case("bus-extra-args.log", CLI, extra_args, 77, "unknown mode/profile");
    unchanged(input, source_hash);
    REQUIRE(lstat(input, &st) == 0 && st.st_mode == source_stat.st_mode &&
            st.st_ino == source_stat.st_ino && st.st_uid == source_stat.st_uid);

    /* Exercise the pre-dlopen content predicate without manufacturing OS
     * namespace/mount evidence or reaching the actual SDK loading branch. */
    REQUIRE(private_bus_valid(bus_expected));
    REQUIRE(private_bus_valid(BUS_PRIVATE));
    REQUIRE(!private_bus_valid(bus_source));
    REQUIRE(!private_bus_valid(""));
    REQUIRE(!private_bus_valid(BUS_PRIVATE "\n" BUS_PRIVATE "\n"));
    REQUIRE(!private_bus_valid("#" BUS_PRIVATE "\n"));
    REQUIRE(!private_bus_valid(BUS_PRIVATE " # not exact\n"));
    REQUIRE(!private_bus_valid(BUS_PRIVATE "\nother: /home/robot/installed.xml\n"));
    puts("PASS bus-private-validator (8 SDK-free predicate assertions)");
    ++cases;
}

int main(int argc, char **argv)
{
    REQUIRE(argc == 5);
    binary = argv[1]; artifacts = argv[3]; staging = argv[4];
    REQUIRE(binary[0] == '/' && artifacts[0] == '/' && staging[0] == '/');
    REQUIRE(!getenv("LD_PRELOAD") && !getenv("LD_AUDIT") && !getenv("LD_LIBRARY_PATH"));
    bus_tests();
    struct json_object *source = json_object_from_file(argv[2]);
    REQUIRE(source != NULL);
    /* Non-default valid guard values make accidental resetting visible. */
    json_object_object_add(source, "allow_commands", json_object_new_boolean(true));
    json_object_object_add(source, "watchdog_seconds", json_object_new_double(0.17));
    json_object_object_add(source, "first_command_timeout", json_object_new_double(1.7));
    json_object_object_add(source, "min_dt", json_object_new_double(0.007));
    json_object_object_add(source, "max_dt", json_object_new_double(0.04));
    json_object_object_add(source, "clock_skew", json_object_new_double(0.08));
    json_object_object_add(source, "first_hold_tolerance", json_object_new_double(0.001));
    json_object_object_add(source, "following_tolerance", json_object_new_double(0.03));
    json_object_object_add(source, "servo_q", json_object_new_double(0.7));
    json_object_object_add(source, "servo_r", json_object_new_double(0.0002));
    char input[PATH_MAX], source_hash[65], text[MK2_FRAME_MAX + 1];
    struct mk2_policy parsed;
    path_join(input, artifacts, "source.json");
    write_json(input, source);
    REQUIRE(policy_read(input, &parsed, text) && parsed.allow_commands);
    REQUIRE(hash_file(input, source_hash));

    char direct[PATH_MAX], cli[PATH_MAX], output[PATH_MAX], before[65];
    profile(direct, "direct-helper");
    prepare_case("direct-helper.log", HELPER, input, direct, true);
    verify_policy(direct, source);
    profile(cli, "cli-helper");
    prepare_case("cli-helper.log", CLI, input, cli, true);
    verify_policy(cli, source);
    path_join(output, cli, "mk2_control_bridge.json");
    REQUIRE(hash_file(output, before));
    prepare_case("existing-policy.log", CLI, input, cli, false);
    unchanged(output, before);
    REQUIRE(chmod(output, 0400) == 0);
    prepare_case("readonly-existing-policy.log", HELPER, input, cli, false);
    unchanged(output, before);

    invalid_json_case("malformed", "{broken");
    invalid_json_case("trailing-garbage", "{} garbage");
    invalid_json_case("non-object", "[]");
    struct json_object *bad = clone_json(source);
    json_object_object_add(bad, "unknown_guard", json_object_new_boolean(true));
    invalid_json_case("unknown-field", json_object_to_json_string(bad));
    json_object_put(bad);
    bad = clone_json(source);
    json_object_object_add(bad, "version", json_object_new_int(2));
    invalid_json_case("unknown-version", json_object_to_json_string(bad));
    json_object_put(bad);
    bad = clone_json(source);
    json_object_object_del(bad, "watchdog_seconds");
    invalid_json_case("missing-guard", json_object_to_json_string(bad));
    json_object_put(bad);
    bad = clone_json(source);
    json_object_object_add(field(bad, "limits"), "unknown", json_object_new_int(1));
    invalid_json_case("unknown-limit", json_object_to_json_string(bad));
    json_object_put(bad);
    bad = clone_json(source);
    json_object_object_add(bad, "allow_commands", json_object_new_string("true"));
    invalid_json_case("wrong-boolean-type", json_object_to_json_string(bad));
    json_object_put(bad);

    char outside[PATH_MAX], device[PATH_MAX], invalid[PATH_MAX], target[PATH_MAX];
    path_join(outside, artifacts, "profile");
    REQUIRE(mkdir(outside, 0700) == 0);
    path_join(device, outside, "DeviceConfig.yaml");
    write_fixture(device, "abc");
    prepare_case("outside-family.log", CLI, input, outside, false);
    absent_policy(outside);
    prepare_case("not-profile.log", CLI, input, staging, false);
    absent_policy(staging);
    profile(target, "canonical-target");
    path_join(invalid, target, ".");
    prepare_case("dot-path.log", CLI, input, invalid, false);
    path_join(invalid, target, "");
    prepare_case("trailing-slash.log", CLI, input, invalid, false);
    path_join(invalid, staging, "linked-profile");
    REQUIRE(symlink(target, invalid) == 0);
    prepare_case("symlink-directory.log", CLI, input, invalid, false);
    char parent[PATH_MAX], alias[PATH_MAX];
    path_join(parent, staging, "canonical-target");
    path_join(alias, staging, "linked-parent");
    REQUIRE(symlink(parent, alias) == 0);
    path_join(invalid, alias, "profile");
    prepare_case("symlink-parent.log", CLI, input, invalid, false);
    absent_policy(target);

    profile(target, "output-symlink");
    path_join(invalid, artifacts, "sentinel");
    write_fixture(invalid, "must remain unchanged\n");
    REQUIRE(hash_file(invalid, before));
    path_join(output, target, "mk2_control_bridge.json");
    REQUIRE(symlink(invalid, output) == 0);
    prepare_case("output-symlink.log", CLI, input, target, false);
    unchanged(invalid, before);
    struct stat st;
    REQUIRE(lstat(output, &st) == 0 && S_ISLNK(st.st_mode));
    profile(target, "dangling-output-symlink");
    path_join(output, target, "mk2_control_bridge.json");
    path_join(invalid, artifacts, "must-not-be-created");
    REQUIRE(symlink(invalid, output) == 0);
    prepare_case("dangling-output-symlink.log", CLI, input, target, false);
    REQUIRE(lstat(invalid, &st) == -1 && errno == ENOENT);
    REQUIRE(lstat(output, &st) == 0 && S_ISLNK(st.st_mode));
    profile(target, "missing-device");
    path_join(device, target, "DeviceConfig.yaml");
    REQUIRE(unlink(device) == 0);
    prepare_case("missing-device.log", CLI, input, target, false);
    absent_policy(target);

    char *check[] = {(char *)binary, "--check", "--path", PROFILE, NULL};
    char *load[] = {(char *)binary, "--load", "--path", PROFILE, NULL};
    char *writer[] = {(char *)binary, "--writer-exit-virtual", "--path", PROFILE, NULL};
    run_case("actual-c-guard.log", GUARD, NULL, 77, "missing host namespace marker");
    run_case("host-check.log", CLI, check, 77, "missing host namespace marker");
    run_case("host-load.log", CLI, load, 77, "missing host namespace marker");
    run_case("host-writer.log", CLI, writer, 77, "missing host namespace marker");
    char *none[] = {(char *)binary, NULL};
    char *short_check[] = {(char *)binary, "--check", NULL};
    char *short_prepare[] = {(char *)binary, "--prepare-policy", (char *)input, NULL};
    char *unknown[] = {(char *)binary, "--init", "--path", PROFILE, NULL};
    char *wrong_flag[] = {(char *)binary, "--check", "--profile", PROFILE, NULL};
    char *wrong_path[] = {(char *)binary, "--load", "--path", "/tmp/not-the-profile", NULL};
    char *extra[] = {(char *)binary, "--check", "--path", PROFILE, "extra", NULL};
    char **bad_args[] = {none, short_check, short_prepare, unknown, wrong_flag, wrong_path, extra};
    const char *names[] = {"no-args.log", "short-check.log", "short-prepare.log", "unknown-mode.log",
                          "wrong-flag.log", "wrong-profile.log", "extra-arg.log"};
    for (size_t i = 0; i < sizeof(bad_args) / sizeof(bad_args[0]); ++i)
        run_case(names[i], CLI, bad_args[i], 77, "unknown mode/profile");
    unchanged(input, source_hash);
    verify_policy(cli, source);
    verify_policy(direct, source);
    json_object_put(source);
    printf("PASS: %u offline cases; source/output immutable, limits/guards preserved; no SDK loaded.\n", cases);
    return 0;
}