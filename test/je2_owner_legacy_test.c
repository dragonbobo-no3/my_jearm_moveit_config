#define JE2_SIM_SESSION_EXPERIMENT 0
#include "jeserver_v2.h"
#include "jeserver_v2_session.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "jeserver_v2_owner.inc"

int main(void)
{
    const struct jeserver_v2 engine = {.owner = JE2_AVAILABLE, .generation = 1};
    char wire[MK2_FRAME_MAX];
    const char expected[] =
        "Owner {\"implementation\":\"JEServer_v2\",\"experimental\":true,"
        "\"owner\":0,\"generation\":1,\"dropped_pc\":0,\"button_port\":16666,\"loopback_only\":true,"
        "\"automatic_priority_ready\":false,\"writer_exit_verified\":false,\"pc_reclaim_supported\":false,"
        "\"writer_observer_enabled\":false,\"writer_instance_exited\":false,\"writer_exit_time\":0.000000000,"
        "\"session_transport_enabled\":false,\"session_boot\":\"\",\"session_client\":\"\","
        "\"session_generation\":0,\"session_last_sequence\":0}";
    assert(je2_format_owner(wire, sizeof(wire), &engine, NULL, 999, 16666, NULL, false, false, 0) ==
        (int)strlen(expected));
    assert(!strcmp(wire, expected));
    assert(je2_format_owner(wire, 4, &engine, NULL, 999, 16666, NULL, false, false, 0) == -1);
    assert(!wire[0]);
    puts("PASS: non-session Owner byte compatibility, capabilities remain false, bounded output");
    return 0;
}