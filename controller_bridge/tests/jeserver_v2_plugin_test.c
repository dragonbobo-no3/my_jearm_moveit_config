#define _POSIX_C_SOURCE 200809L
#include "jeserver_v2_fake_sdk.h"
#include "../mk2_control_core.h"
#include <dlfcn.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

/* Fixed-profile fixture is necessary to reach the REAL isolation guard through
 * PluginMain. Exclusively create it; never overwrite an existing profile. No
 * namespace/env spoofing, OS interposition, SDK library or worker startup. */
static bool profile_created, parent_created;
static void cleanup(void)
{
    if (profile_created) {
        assert(unlink("/tmp/mk2-pro/profile/mk2_control_bridge.json") == 0);
        assert(unlink("/tmp/mk2-pro/profile/DeviceConfig.yaml") == 0);
        assert(rmdir("/tmp/mk2-pro/profile") == 0);
    }
    if (parent_created) assert(rmdir("/tmp/mk2-pro") == 0);
}
static void fixture(void)
{
    if (mkdir("/tmp/mk2-pro", 0700) == 0) parent_created = true;
    else {
        struct stat st;
        assert(errno == EEXIST && lstat("/tmp/mk2-pro", &st) == 0);
        assert(S_ISDIR(st.st_mode) && st.st_uid == getuid());
    }
    if (mkdir("/tmp/mk2-pro/profile", 0700)) {
        perror("exclusive test profile (will not overwrite)"); exit(1);
    }
    profile_created = true;
    assert(atexit(cleanup) == 0);
    FILE *f = fopen("/tmp/mk2-pro/profile/DeviceConfig.yaml", "wx");
    assert(f && fclose(f) == 0); /* SHA256 of empty test data, not a robot model. */
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
    /* A policy parse failure must not masquerade as an isolation refusal. */
    f = fopen("/tmp/mk2-pro/profile/mk2_control_bridge.json", "rb");
    assert(f);
    char text[MK2_FRAME_MAX], error[128];
    size_t length = fread(text, 1, sizeof(text), f);
    assert(!ferror(f) && length < sizeof(text) && fclose(f) == 0);
    struct mk2_policy parsed;
    assert(mk2_policy_parse(text, length, &parsed, error, sizeof(error)) == 0);
    assert(parsed.allow_commands && !strcmp(parsed.command_host, "127.0.0.1"));
}
static void symbol(void *handle, const char *name, void *target, size_t size)
{
    dlerror(); void *address = dlsym(handle, name);
    const char *error = dlerror();
    if (error) fprintf(stderr, "%s: %s\n", name, error);
    assert(!error && address && size == sizeof(address));
    /* POSIX dlsym conversion without ISO C function-pointer cast warnings. */
    memcpy(target, &address, size);
}
int main(int argc, char **argv)
{
    assert(argc == 5 && !strcmp(argv[3], "--path"));
    bool unisolated = !strcmp(argv[2], "unisolated");
    assert(!strcmp(argv[2], "wrongprofile") || unisolated);
    assert(!getenv("LD_PRELOAD") && !getenv("LD_AUDIT"));
    static const char *env[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET", "MK2_HOST_PID", "MK2_HOST_USER"};
    for (size_t i = 0; i < sizeof(env) / sizeof(env[0]); ++i) assert(!getenv(env[i]));
    if (unisolated) { assert(!strcmp(argv[4], "/tmp/mk2-pro/profile")); fixture(); }
    else assert(strcmp(argv[4], "/tmp/mk2-pro/profile"));
    for (int i = 0; i < 32; ++i) {
        void *handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
        if (!handle) fprintf(stderr, "dlopen: %s\n", dlerror());
        assert(handle);
        void (*entry)(void) = NULL;
        int (*status)(void) = NULL;
        unsigned (*port)(void) = NULL;
        symbol(handle, "PluginMain", &entry, sizeof(entry));
        symbol(handle, "JEServerV2Status", &status, sizeof(status));
        symbol(handle, "JEServerV2ButtonPort", &port, sizeof(port));
        assert(status() == 0 && port() == 0); /* Fresh DSO static state. */
        entry(); entry();
        assert(status() == -1 && port() == 0);
        assert(fake.getters == 0 && fake.mutations == 0);
        assert(dlclose(handle) == 0);
        assert(fake.getters == 0 && fake.mutations == 0);
        handle = dlopen(argv[1], RTLD_NOW | RTLD_NOLOAD);
        assert(!handle); /* Destructor/unmapping, not just another reference. */
    }
    printf("PASS: actual DSO %s refusal, 32 load/entry/entry/unload cycles; ZERO SDK calls\n", argv[2]);
    return 0;
}