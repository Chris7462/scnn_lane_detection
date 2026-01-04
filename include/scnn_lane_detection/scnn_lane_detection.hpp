#pragma once

// C++ header
#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV header
#include <opencv2/core.hpp>

// Local header
#include "scnn_trt_backend/scnn_trt_backend.hpp"


namespace scnn_lane_detection
{

namespace fs = std::filesystem;

class SCNNLaneDetection : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for SCNNLaneDetection node
   */
  SCNNLaneDetection();

  /**
   * @brief Destructor for SCNNLaneDetection node
   */
  ~SCNNLaneDetection();

private:
  /**
   * @brief Initialize node parameters with validation
   * @return true if initialization successful, false otherwise
   */
  bool initialize_parameters();

  /**
   * @brief Initialize TensorRT inferencer
   * @return true if initialization successful, false otherwise
   */
  bool initialize_inferencer();

  /**
   * @brief Initialize ROS2 publishers, subscribers, and timers
   */
  void initialize_ros_components();

  /**
   * @brief Callback function for incoming images
   * @param msg Incoming image message
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Timer callback for processing images at regular intervals
   */
  void timer_callback();

  /**
   * @brief Process input image through SCNN lane detection
   * @param input_image Input OpenCV image
   * @return SCNNResult containing segmentation mask and lane existence probabilities
   */
  scnn_trt_backend::SCNNResult process_image(const cv::Mat & input_image);

  /**
   * @brief Publish lane segmentation result
   * @param segmentation Segmentation result as OpenCV Mat
   * @param header Original message header for timestamp consistency
   */
  void publish_segmentation_result(
    const cv::Mat & segmentation,
    const std_msgs::msg::Header & header);

  /**
   * @brief Publish overlay result
   * @param overlay Overlay image as OpenCV Mat
   * @param header Original message header for timestamp consistency
   */
  void publish_overlay_result(
    const cv::Mat & overlay,
    const std_msgs::msg::Header & header);

  /**
   * @brief Publish lane existence probabilities
   * @param exist_pred Lane existence probabilities
   * @param header Original message header for timestamp consistency
   */
  void publish_lane_existence(
    const std::array<float, 4> & exist_pred,
    const std_msgs::msg::Header & header);

private:
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lane_overlay_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr exist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback groups for parallel execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // TensorRT inferencer
  std::shared_ptr<scnn_trt_backend::SCNNTrtBackend> detector_;

  // ROS2 parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string output_overlay_topic_;
  std::string output_exist_topic_;
  int queue_size_;
  double processing_frequency_;
  int max_processing_queue_size_;

  scnn_trt_backend::SCNNTrtBackend::Config config_;
  fs::path engine_path_;
  std::string engine_filename_;

  // Simplified image buffer
  std::queue<sensor_msgs::msg::Image::SharedPtr> img_buff_;
  std::mutex mtx_;
  std::atomic<bool> processing_in_progress_;
};

}  // namespace scnn_lane_detection
