/* SDK-free execution of the dedicated native semantics probe's control flow.
 * These callbacks are fixtures only; never label their results SDK evidence. */
#define JE2_SDK_TEACH_PROBE 1
#define main probe_main
#include "jeserver_v2_sdk_probe.c"
#undef main
#include <assert.h>

static struct {
    int scenario, power, state, enable, jog, stop, force_stop;
    double q;
} fake;
static int select_robot(int robot) { assert(robot == 0); return 0; }
static int enable(void) { ++fake.enable; fake.power = 1; return 0; }
static int simulation(void) { return fake.scenario == 8 ? 0 : 1; }
static int power(int robot) { (void)robot; return fake.power; }
static int state(int robot) { (void)robot; return fake.state; }
static int driver(int robot) { (void)robot; return fake.scenario == 3 && (fake.stop || fake.force_stop) ? 1 : 0; }
static int error(int robot) { (void)robot; return 0; }
static int jog(int axis, int direction)
{
    assert(axis == 0 && direction == 1);
    ++fake.jog;
    assert(fake.state == (fake.jog == 1 ? 0 : 7));
    fake.state = fake.scenario == 9 ? 7 : 1; /* model internal recovery */
    return fake.scenario == 5 && fake.jog == 2 ? -9 : 0;
}
static int stop(void)
{
    ++fake.stop;
    fake.state = fake.scenario == 1 ? 0 : 7;
    if (fake.scenario == 4) fake.power = 0;
    return 0;
}
static int force_stop(int robot)
{
    assert(robot == 0);
    ++fake.force_stop;
    fake.state = fake.scenario == 1 || fake.scenario == 11 ? 0 : 7;
    if (fake.scenario == 4) fake.power = 0;
    return fake.scenario == 10 ? -11 : 0;
}
static void velocity(double value) { assert(value == .01); }
static void move_type(int value) { assert(value == 0); }
static void move_thread(int value) { assert(value == 1); }
static void actual(double *q, int robot)
{
    assert(robot == 0);
    for (int i = 0; i < MK2_AXES; ++i) q[i] = 0;
    if (fake.state == 1 && fake.scenario != 2) fake.q += .000002;
    q[0] = fake.q;
    if (fake.jog && fake.scenario == 6) q[1] = .001;
    if (fake.jog && fake.scenario == 7) q[0] = strtod("nan", NULL);
}
int main(void)
{
    struct teach_api api = {.select_robot = select_robot, .coordinate = select_robot,
        .enable = enable, .simulation = simulation, .power = power, .state = state,
        .driver = driver, .error = error, .jog = jog, .stop = stop, .force_stop = force_stop,
        .velocity = velocity, .move_type = move_type, .move_thread = move_thread, .actual = actual};
    for (int scenario = 0; scenario <= 9; ++scenario) {
        memset(&fake, 0, sizeof(fake)); fake.scenario = scenario;
        int result = teach_experiment(&api, TEACH_NATIVE_STOP);
        assert(result == (scenario == 0 ? 0 : 79));
        assert(fake.jog == (scenario == 8 ? 0 : (scenario == 0 || scenario == 5) ? 2 : 1));
        assert(fake.stop == fake.jog && fake.enable == (scenario == 8 ? 0 : 1));
        assert(fake.force_stop == 0);
    }
    for (int source = TEACH_EXTERNAL_STOP; source <= TEACH_OVERLAP_STOP; ++source) {
        for (int scenario = 0; scenario <= 11; ++scenario) {
            memset(&fake, 0, sizeof(fake)); fake.scenario = scenario;
            int result = teach_experiment(&api, (enum teach_stop_source)source);
            bool success = scenario == 0 || scenario == 5; /* no second jog to fail */
            assert(result == (success ? 0 : 79));
            assert(fake.jog == (scenario == 8 ? 0 : 1));
            assert(fake.stop <= 1 && fake.force_stop <= 1);
            if (success) {
                assert(fake.force_stop == 1 && fake.state == 7);
                assert(fake.stop == (source == TEACH_OVERLAP_STOP ? 1 : 0));
            }
            /* Every failure is terminal; no second native jog can implicitly
             * recover an external stop, even after our own teach STOP. */
            assert(fake.enable == (scenario == 8 ? 0 : 1));
        }
    }
    const char *rejected[] = {"--load", "--writer-exit-virtual", "--handover-virtual", "--check", "--prepare-handover-policy"};
    for (size_t i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
        char *argv[] = {"probe", (char *)rejected[i], "--path", PROFILE, NULL};
        assert(probe_main(4, argv) == 77);
    }
    puts("PASS: 34 native-teach/source control-flow fixtures + exclusive CLI refusals; fake SDK ONLY, no real library loaded");
    return 0;
}