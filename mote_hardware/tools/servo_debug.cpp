// Interactive servo debug tool.
//
// Lets you drive individual servos and inspect raw position/speed feedback
// without ros2_control in the loop. Useful for figuring out which physical
// wheel a servo ID drives, the relationship between WriteSpe sign and
// physical rotation, and the relationship between WriteSpe sign and the
// sign of ReadPos / ReadSpeed feedback.
//
// Build: added as `servo_debug` executable in mote_hardware/CMakeLists.txt.
// Run:   ros2 run mote_hardware servo_debug [serial_port] [baud]
//
// Commands (type `help` to print this):
//   mv  <id> <speed>     drive servo <id> at <speed> (wheel mode, -32767..32767)
//   stop <id>            stop servo <id>
//   stopall              stop all servos seen this session
//   r   <id>             read pos + speed once
//   m   <id> [hz]        monitor pos + speed (default 5 Hz, ctrl-C to stop)
//   ping <lo> <hi>       ping every ID in [lo, hi] and report which respond
//   q | quit | exit      exit

#include "SMS_STS.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_interrupt{false};

void on_sigint(int) { g_interrupt = true; }

void print_help()
{
    std::printf(
        "Commands:\n"
        "  mv   <id> <speed>     drive servo at speed (wheel mode)\n"
        "  stop <id>             stop servo <id>\n"
        "  stopall               stop all servos seen this session\n"
        "  r    <id>             read pos + speed once\n"
        "  m    <id> [hz]        monitor pos + speed (ctrl-C to stop)\n"
        "  ping <lo> <hi>        ping IDs in range, report responders\n"
        "  help                  print this help\n"
        "  q | quit | exit       exit\n");
}

void cmd_mv(SMS_STS &sms, std::set<int> &touched, int id, int speed)
{
    sms.Mode(static_cast<u8>(id), SMS_STS_MODE_WHEEL_CLOSED);
    int ret = sms.WriteSpe(static_cast<u8>(id), static_cast<s16>(speed), 0);
    if (ret) {
        touched.insert(id);
        std::printf("WriteSpe(%d, %d) -> OK\n", id, speed);
    } else {
        std::printf("WriteSpe(%d, %d) -> FAILED (no response)\n", id, speed);
    }
}

void cmd_stop(SMS_STS &sms, int id)
{
    int ret = sms.WriteSpe(static_cast<u8>(id), 0, 0);
    std::printf("Stop %d -> %s\n", id, ret ? "OK" : "FAILED");
}

void cmd_read(SMS_STS &sms, int id)
{
    int pos   = sms.ReadPos(id);
    int speed = sms.ReadSpeed(id);
    std::printf("id=%d  pos=%d  speed=%d\n", id, pos, speed);
}

void cmd_monitor(SMS_STS &sms, int id, double hz)
{
    if (hz <= 0) hz = 5.0;
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
    g_interrupt = false;
    std::printf("Monitoring id=%d at %.1f Hz (ctrl-C to stop)\n", id, hz);
    while (!g_interrupt) {
        int pos   = sms.ReadPos(id);
        int speed = sms.ReadSpeed(id);
        std::printf("  pos=%6d  speed=%6d\n", pos, speed);
        std::fflush(stdout);
        std::this_thread::sleep_for(period);
    }
    std::printf("monitor stopped\n");
}

void cmd_ping(SMS_STS &sms, int lo, int hi)
{
    std::printf("Pinging IDs %d..%d\n", lo, hi);
    for (int id = lo; id <= hi; ++id) {
        int pos = sms.ReadPos(id);
        if (pos != -1) std::printf("  id=%d responded (pos=%d)\n", id, pos);
    }
    std::printf("done.\n");
}

bool dispatch(SMS_STS &sms, std::set<int> &touched, const std::string &line)
{
    std::istringstream iss(line);
    std::string cmd;
    if (!(iss >> cmd)) return true;

    if (cmd == "q" || cmd == "quit" || cmd == "exit") return false;
    if (cmd == "help" || cmd == "h" || cmd == "?") { print_help(); return true; }

    if (cmd == "mv") {
        int id, speed;
        if (iss >> id >> speed) cmd_mv(sms, touched, id, speed);
        else std::printf("usage: mv <id> <speed>\n");
    } else if (cmd == "stop") {
        int id;
        if (iss >> id) cmd_stop(sms, id);
        else std::printf("usage: stop <id>\n");
    } else if (cmd == "stopall") {
        for (int id : touched) cmd_stop(sms, id);
    } else if (cmd == "r") {
        int id;
        if (iss >> id) cmd_read(sms, id);
        else std::printf("usage: r <id>\n");
    } else if (cmd == "m") {
        int id;
        double hz = 5.0;
        if (iss >> id) {
            iss >> hz;
            cmd_monitor(sms, id, hz);
        } else {
            std::printf("usage: m <id> [hz]\n");
        }
    } else if (cmd == "ping") {
        int lo, hi;
        if (iss >> lo >> hi) cmd_ping(sms, lo, hi);
        else std::printf("usage: ping <lo> <hi>\n");
    } else {
        std::printf("unknown command: %s (type 'help')\n", cmd.c_str());
    }
    return true;
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
    std::printf("Opened %s @ %d. Type 'help' for commands.\n", port, baud);

    std::signal(SIGINT, on_sigint);

    std::set<int> touched;
    std::string line;
    while (true) {
        std::printf("> ");
        std::fflush(stdout);
        if (!std::getline(std::cin, line)) break;
        if (!dispatch(sms, touched, line)) break;
    }

    for (int id : touched) sms.WriteSpe(static_cast<u8>(id), 0, 0);
    sms.end();
    return 0;
}
