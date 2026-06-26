// C++ header
#include <memory>

// ROS header
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

// Local header
#include "scnn_lane_detection/scnn_lane_detection.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<scnn_lane_detection::SCNNLaneDetection>();

  // EventsCBGExecutor: uses 10-15% less CPU than MultiThreadedExecutor,
  // supports multiple ROS time sources, and manages threading internally.
  rclcpp::executors::EventsCBGExecutor executor;

  // Add node to executor
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting SCNN Lane Detection with EventCBGExecutor");

  // Spin with multiple threads
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
