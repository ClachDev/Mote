#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mote_health/health_monitor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mote_health::HealthMonitor>());
  } catch (const std::exception & exc) {
    // A malformed health.yaml is the realistic case, and the message names the
    // offending entry. Reported before the context is torn down so systemd's
    // journal carries it rather than an exit code alone.
    RCLCPP_FATAL(rclcpp::get_logger("health_monitor"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
