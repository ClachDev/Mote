// Non-interactive servo ping for startup self-checks.
//
// Pings a fixed set of servo IDs on the drive bus and exits with a status
// code: 0 if every requested ID responded, 1 if any did not (or the bus could
// not be opened). Prints one line per ID plus a summary, so a self-check
// script can both gate on the exit code and log which servo is missing.
//
// The interactive `servo_debug` tool has a `ping` command for exploring the
// bus by hand; this is the scriptable counterpart that a service can run
// before bringup opens the bus.
//
// Build: added as `servo_ping` executable in mote_hardware/CMakeLists.txt.
// Run:   ros2 run mote_hardware servo_ping [port] [baud] [id ...]
//        defaults: /dev/mote_servos 1000000, IDs 7 and 9 (robot.yaml drive IDs)

#include "SMS_STS.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
    const char *port = (argc > 1) ? argv[1] : "/dev/mote_servos";
    int baud = (argc > 2) ? std::atoi(argv[2]) : 1000000;

    std::vector<int> ids;
    for (int i = 3; i < argc; ++i) ids.push_back(std::atoi(argv[i]));
    if (ids.empty()) ids = {7, 9};

    SMS_STS sms;
    if (!sms.begin(baud, port)) {
        std::printf("FAIL: could not open %s @ %d\n", port, baud);
        return 1;
    }

    int missing = 0;
    for (int id : ids) {
        // ReadPos returns -1 when the servo does not answer within the SDK's
        // read timeout, which is how servo_debug's ping detects presence.
        int pos = sms.ReadPos(id);
        if (pos >= 0) {
            std::printf("id=%d OK (pos=%d)\n", id, pos);
        } else {
            std::printf("id=%d MISSING (no response)\n", id);
            ++missing;
        }
    }
    sms.end();

    if (missing) {
        std::printf("FAIL: %d/%zu servos did not respond\n", missing, ids.size());
        return 1;
    }
    std::printf("OK: all %zu servos responded\n", ids.size());
    return 0;
}
