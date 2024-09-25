#include <chrono>
#include <functional>
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

#include <mirte_telemetrix_test_helpers/image_streamer_component.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace mirte_telemetrix_test_helpers
{

ImageStreamer::ImageStreamer(const rclcpp::NodeOptions & options)
: nh_(std::make_shared<rclcpp::Node>("image_streamer", options))
{
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);
  client_ = nh_->create_client<mirte_msgs::srv::SetOLEDImage>("io/set_oled_image");

  sub_ = it_->subscribe("/image", 1, std::bind(&ImageStreamer::image_callback, this, _1));
}

void ImageStreamer::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(nh_->get_logger(), "Service not available after waiting");
    return;
  }

  auto cvb_img = cv_bridge::toCvCopy(msg, "mono8");

  /* Set Region of Interest */

  int smallest =
    (cvb_img->image.cols > cvb_img->image.rows) ? cvb_img->image.rows : cvb_img->image.cols;

  cv::Rect roi;
  roi.x = (cvb_img->image.cols - smallest) / 2;
  roi.y = (cvb_img->image.rows - smallest / 2) / 2;
  roi.width = smallest;
  roi.height = smallest / 2;
  cvb_img->image = cvb_img->image(roi);

  cv::resize(cvb_img->image, cvb_img->image, cv::Size(128, 64));

  auto request = std::make_shared<mirte_msgs::srv::SetOLEDImage::Request>();
  cvb_img->toImageMsg(request->image);

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
    RCLCPP_INFO(
      this->nh_->get_logger(), "Got result: [%s]", (future.get()->status) ? "Ok" : "Failed");
  };
  auto future_result = client_->async_send_request(request, response_received_callback);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ImageStreamer::get_node_base_interface() const
{
  return nh_->get_node_base_interface();
}

}  // namespace mirte_telemetrix_test_helpers

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mirte_telemetrix_test_helpers::ImageStreamer)