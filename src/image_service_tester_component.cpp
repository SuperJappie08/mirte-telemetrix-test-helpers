#include <chrono>
#include <memory>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

// Pre & Post IRON compatability.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <std_msgs/msg/header.hpp>

#include <mirte_telemetrix_test_helpers/image_service_tester_component.hpp>

using namespace std::chrono_literals;

namespace mirte_telemetrix_test_helpers
{

ImageServiceTester::ImageServiceTester(const rclcpp::NodeOptions & options)
: Node("image_service_tester", options)
{
  declare_parameter(
    "image_path", "/usr/local/src/mirte/mirte-oled-images/images/mirte_logo_inv.png");

  client_ = create_client<mirte_msgs::srv::SetOLEDImage>("io/set_oled_image");

  // Note(dhood): The timer period must be greater than the duration of the timer callback.
  // Otherwise, the timer can starve a single-threaded executor.
  // See https://github.com/ros2/rclcpp/issues/392 for updates.
  timer_ = create_wall_timer(2s, [this]() { return this->on_timer(); });
}

void ImageServiceTester::on_timer()
{
  if (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  auto image =
    cv::imread(this->get_parameter("image_path").as_string(), cv::ImreadModes::IMREAD_GRAYSCALE);

  auto request = std::make_shared<mirte_msgs::srv::SetOLEDImage::Request>();
  cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg(request->image);

  // In order to wait for a response to arrive, we need to spin().
  // However, this function is already being called from within another spin().
  // Unfortunately, the current version of spin() is not recursive and so we
  // cannot call spin() from within another spin().
  // Therefore, we cannot wait for a response to the request we just made here
  // within this callback, because it was executed by some other spin function.
  // The workaround for this is to give the async_send_request() method another
  // argument which is a callback that gets executed once the future is ready.
  // We then return from this callback so that the existing spin() function can
  // continue and our callback will get called once the response is received.
  using ServiceResponseFuture = rclcpp::Client<mirte_msgs::srv::SetOLEDImage>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    RCLCPP_INFO(this->get_logger(), "Got result: [%s]", (future.get()->status) ? "Ok" : "Failed");
  };
  auto future_result = client_->async_send_request(request, response_received_callback);
}
}  // namespace mirte_telemetrix_test_helpers

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mirte_telemetrix_test_helpers::ImageServiceTester)