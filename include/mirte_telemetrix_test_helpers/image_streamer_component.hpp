#pragma once
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <mirte_msgs/srv/set_oled_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <mirte_telemetrix_test_helpers/visibility_control.h>

namespace mirte_telemetrix_test_helpers
{

class ImageStreamer
{
public:
  MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC
  explicit ImageStreamer(const rclcpp::NodeOptions & options);

  MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  rclcpp::Client<mirte_msgs::srv::SetOLEDImage>::SharedPtr client_;
  image_transport::Subscriber sub_;
};

}  // namespace mirte_telemetrix_test_helpers
