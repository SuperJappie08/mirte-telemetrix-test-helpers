#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mirte_msgs/srv/set_oled_image.hpp>

#include <mirte_telemetrix_test_helpers/visibility_control.h>

namespace mirte_telemetrix_test_helpers
{

class ImageServiceTester : public rclcpp::Node
{
public:
  MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC
  explicit ImageServiceTester(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::Client<mirte_msgs::srv::SetOLEDImage>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mirte_telemetrix_test_helpers
