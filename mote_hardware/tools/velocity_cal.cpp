// Measures velocity_scale for mote_hardware.
//
// Commands a series of raw speeds to a servo and computes actual angular
// velocity by sampling the position encoder at 20 Hz and accumulating ticks
// with rollover handling. More accurate than a single start/end read because
// it handles multi-revolution spans correctly.
//
// Build: added as `velocity_cal` in mote_hardware/CMakeLists.txt.
// Run:   ros2 run mote_hardware velocity_cal [servo_id] [serial_port] [baud]
//        e.g. ros2 run mote_hardware velocity_cal 7
//
// IMPORTANT: lift the robot off the ground so the wheels spin freely.

#include "SMS_STS.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>
#include <vector>

static constexpr double TICKS_PER_REV = 4096.0;
static constexpr double TWO_PI        = 2.0 * M_PI;
static constexpr int    SETTLE_MS     = 600;   // wait for speed to stabilise
static constexpr int    MEASURE_S     = 8;     // measurement window
static constexpr int    SAMPLE_HZ     = 20;

std::atomic<bool> g_interrupt{false};
void on_sigint(int) { g_interrupt = true; }

// Returns measured rad/s, or -1.0 on failure.
double measure(SMS_STS& sms, u8 id, int speed)
{
    sms.WriteSpe(id, static_cast<s16>(speed), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_MS));

    int last_pos = sms.ReadPos(id);
    if (last_pos == -1) {
        std::printf("  ReadPos failed\n");
        sms.WriteSpe(id, 0, 0);
        return -1.0;
    }

    auto t_start  = std::chrono::steady_clock::now();
    auto t_end    = t_start + std::chrono::seconds(MEASURE_S);
    auto interval = std::chrono::milliseconds(1000 / SAMPLE_HZ);

    long long total_ticks = 0;
    int       samples     = 0;

    while (std::chrono::steady_clock::now() < t_end && !g_interrupt) {
        std::this_thread::sleep_for(interval);
        int pos = sms.ReadPos(id);
        if (pos == -1) continue;

        int delta = pos - last_pos;
        if (delta >  2048) delta -= 4096;
        if (delta < -2048) delta += 4096;

        total_ticks += delta;
        last_pos = pos;
        ++samples;
    }

    sms.WriteSpe(id, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    double rad_per_s = (total_ticks / TICKS_PER_REV) * TWO_PI / elapsed;

    std::printf("  ticks=%-8lld  elapsed=%.2fs  samples=%d  rad/s=%.4f\n",
        total_ticks, elapsed, samples, rad_per_s);

    return rad_per_s;
}

int main(int argc, char* argv[])
{
    int         id   = (argc > 1) ? std::atoi(argv[1]) : 7;
    const char* port = (argc > 2) ? argv[2] : "/dev/mote_servos";
    int         baud = (argc > 3) ? std::atoi(argv[3]) : 1000000;

    SMS_STS sms;
    if (!sms.begin(baud, port)) {
        std::printf("Failed to open %s @ %d\n", port, baud);
        return 1;
    }
    std::printf("Opened %s @ %d, servo ID %d\n\n", port, baud, id);
    std::printf("IMPORTANT: lift the robot so wheels spin freely, then press Enter.\n");
    std::getchar();

    std::signal(SIGINT, on_sigint);
    sms.Mode(static_cast<u8>(id), SMS_STS_MODE_WHEEL_CLOSED);

    const std::vector<int> test_speeds = {500, 1000, 1500, 2000, 2500};
    std::vector<std::pair<int, double>> results;

    for (int speed : test_speeds) {
        if (g_interrupt) break;
        std::printf("speed=%d\n", speed);

        double rad_per_s = measure(sms, static_cast<u8>(id), speed);
        if (rad_per_s <= 0.0) {
            std::printf("  skipped (invalid reading)\n\n");
            continue;
        }

        double scale = speed / rad_per_s;
        results.emplace_back(speed, scale);
        std::printf("  -> velocity_scale = %.1f\n\n", scale);
    }

    sms.end();

    if (results.empty()) {
        std::printf("No valid measurements.\n");
        return 1;
    }

    double sum = 0.0;
    for (auto& [spd, scale] : results) sum += scale;
    double mean = sum / static_cast<double>(results.size());

    std::printf("=== Summary ===\n");
    for (auto& [spd, scale] : results)
        std::printf("  speed=%4d  velocity_scale=%.1f\n", spd, scale);
    std::printf("\nRecommended velocity_scale: %.1f\n\n", mean);
    std::printf("Update in:\n");
    std::printf("  mote_description/urdf/mote.urdf.xacro  (velocity_scale default)\n");
    std::printf("  mote_bringup/launch/mote_launch.py      (velocity_scale value)\n");

    return 0;
}
