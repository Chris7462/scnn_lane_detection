// C++ header
#include <algorithm>
#include <memory>
#include <thread>

// ROS header
#include <rclcpp/executors/multi_threaded_executor.hpp>

// Local header
#include "scnn_lane_detection/scnn_lane_detection.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<scnn_lane_detection::SCNNLaneDetection>();

  // Create multi-threaded executor with optimal thread count
  // Use 2 threads minimum: one for callbacks, one for processing
  size_t num_threads = std::max(2u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);

  // Add node to executor
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting SCNN Lane Detection with %zu threads", num_threads);

  // Spin with multiple threads
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
