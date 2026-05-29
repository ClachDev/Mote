// Guided servo ID setup tool.
//
// Assigns the wheel servo IDs Mote expects: left = 7, right = 9 (matching
// LeKiwi). Fresh Feetech servos all ship as ID 1, so two of them on the same
// bus are indistinguishable. This tool sidesteps that by configuring one servo
// at a time: connect a single servo, it detects whatever ID is currently on the
// bus and reassigns it to the target.
//
// Build: added as `setup_ids` executable in mote_hardware/CMakeLists.txt.
// Run:   ros2 run mote_hardware setup_ids [serial_port] [baud]

#include "SMS_STS.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

constexpr int LEFT_ID  = 7;
constexpr int RIGHT_ID = 9;

// Range scanned to find the connected servo. Covers the factory default (1)
// and Mote's wheel IDs (7, 9) so re-runs work too.
constexpr int SCAN_LO = 0;
constexpr int SCAN_HI = 20;

std::vector<int> scan(SMS_STS &sms)
{
    std::vector<int> found;
    for (int id = SCAN_LO; id <= SCAN_HI; ++id) {
        if (sms.ReadPos(id) != -1) found.push_back(id);
    }
    return found;
}

bool set_id(SMS_STS &sms, u8 current_id, u8 new_id)
{
    if (current_id == new_id) return true;

    // The servo responds to new_id immediately after the write, so LockEeprom
    // must use new_id — otherwise the lock packet is ignored.
    sms.unLockEeprom(current_id);
    usleep(10000);

    u8 val = new_id;
    int ret = sms.genWrite(current_id, SMS_STS_ID, &val, 1);
    usleep(10000);
    if (!ret) return false;

    sms.LockEeprom(new_id);
    usleep(10000);
    return true;
}

// Prompts the user to connect a single servo and assigns it `target`.
// Returns false if the user quits.
bool assign(SMS_STS &sms, const char *label, int target)
{
    while (true) {
        std::printf(
            "\nConnect ONLY the %s wheel servo (disconnect the other), "
            "then press Enter ('q' to quit): ", label);
        std::fflush(stdout);

        std::string line;
        if (!std::getline(std::cin, line)) return false;
        if (line == "q" || line == "quit") return false;

        std::vector<int> found = scan(sms);

        if (found.empty()) {
            std::printf(
                "  No servo detected on IDs %d..%d. Check power and wiring, "
                "then try again.\n", SCAN_LO, SCAN_HI);
            continue;
        }
        if (found.size() > 1) {
            std::printf("  Multiple servos detected (");
            for (size_t i = 0; i < found.size(); ++i) {
                std::printf("%s%d", i ? ", " : "", found[i]);
            }
            std::printf("). Connect only ONE servo at a time.\n");
            continue;
        }

        int current = found.front();
        std::printf("  Found servo at ID %d -> setting to %d... ", current, target);
        std::fflush(stdout);
        if (set_id(sms, static_cast<u8>(current), static_cast<u8>(target))) {
            std::printf("OK\n");
            return true;
        }
        std::printf("FAILED (no response). Try again.\n");
    }
}

}  // namespace

int main(int argc, char *argv[])
{
    const char *port = (argc > 1) ? argv[1] : "/dev/mote_servos";
    int baud = (argc > 2) ? std::atoi(argv[2]) : 1000000;

    SMS_STS sms;
    if (!sms.begin(baud, port)) {
        std::printf("Failed to open %s @ %d\n", port, baud);
        return 1;
    }

    std::printf("Opened %s @ %d.\n", port, baud);
    std::printf("This will assign wheel IDs: left=%d, right=%d.\n", LEFT_ID, RIGHT_ID);

    if (!assign(sms, "LEFT", LEFT_ID))   { sms.end(); return 1; }
    if (!assign(sms, "RIGHT", RIGHT_ID)) { sms.end(); return 1; }

    std::printf(
        "\nDone. Reconnect both servos, then verify with:\n"
        "  ros2 run mote_hardware servo_debug\n"
        "  > ping %d %d\n", LEFT_ID, RIGHT_ID);

    sms.end();
    return 0;
}
