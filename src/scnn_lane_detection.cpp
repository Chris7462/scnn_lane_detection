// C++ header
#include <string>
#include <chrono>
#include <functional>
#include <exception>

// OpenCV header
#include <opencv2/highgui.hpp>

// ROS header
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>

// Local header
#include "scnn_lane_detection/scnn_lane_detection.hpp"
#include "scnn_trt_backend/lane_utils.hpp"


namespace scnn_lane_detection
{

SCNNLaneDetection::SCNNLaneDetection()
: Node("scnn_lane_detection_node"),
  processing_in_progress_(false)
{
  // Initialize ROS2 parameters with validation
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // Initialize TensorRT inferencer
  if (!initialize_inferencer()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize TensorRT inferencer");
    rclcpp::shutdown();
    return;
  }

  // Initialize ROS2 components
  initialize_ros_components();

  RCLCPP_INFO(get_logger(),
    "SCNN Lane Detection node initialized successfully with bounded queue (max: %d)",
    max_processing_queue_size_);
}

SCNNLaneDetection::~SCNNLaneDetection()
{
  RCLCPP_INFO(get_logger(), "SCNN Lane Detection node shutting down");
}

bool SCNNLaneDetection::initialize_parameters()
{
  try {
    // ROS2 parameters
    input_topic_ = declare_parameter("input_topic",
      std::string("kitti/camera/color/left/image_raw"));
    output_topic_ = declare_parameter("output_topic", std::string("scnn_lane_detection"));
    output_overlay_topic_ = declare_parameter("output_topic_overlay",
      std::string("scnn_lane_detection_overlay"));
    output_exist_topic_ = declare_parameter("output_exist_topic",
      std::string("scnn_lane_existence"));
    queue_size_ = declare_parameter<int>("queue_size", 10);
    processing_frequency_ = declare_parameter<double>("processing_frequency", 40.0);

    // Processing queue parameter - small bounded queue for burst handling
    max_processing_queue_size_ = declare_parameter<int>("max_processing_queue_size", 3);

    // Declare and get parameters with validation
    std::string engine_package = declare_parameter("engine_package",
      std::string("scnn_trt_backend"));
    std::string engine_filename = declare_parameter("engine_filename",
      std::string("scnn_vgg16_288x952.engine"));
    config_.height = declare_parameter<int>("height", 288);
    config_.width = declare_parameter<int>("width", 952);
    config_.num_classes = declare_parameter<int>("num_classes", 5);
    config_.num_lanes = declare_parameter<int>("num_lanes", 4);
    config_.exist_threshold = declare_parameter<double>("exist_threshold", 0.5);
    config_.warmup_iterations = declare_parameter<int>("warmup_iterations", 2);
    config_.log_level = static_cast<scnn_trt_backend::Logger::Severity>(
      declare_parameter<int>("log_level", 3));

    // Validation
    if (engine_filename.empty()) {
      RCLCPP_ERROR(get_logger(), "Engine filename cannot be empty");
      return false;
    }

    if (config_.width <= 0 || config_.height <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid image dimensions: %dx%d", config_.width, config_.height);
      return false;
    }

    if (config_.num_classes <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid number of classes: %d", config_.num_classes);
      return false;
    }

    if (config_.num_lanes <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid number of lanes: %d", config_.num_lanes);
      return false;
    }

    if (processing_frequency_ <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid processing frequency: %.2f Hz", processing_frequency_);
      return false;
    }

    if (max_processing_queue_size_ <= 0 || max_processing_queue_size_ > 10) {
      RCLCPP_ERROR(get_logger(), "Invalid max processing queue size: %d (should be 1-10)",
        max_processing_queue_size_);
      return false;
    }

    // Construct engine file path
    fs::path package_path = ament_index_cpp::get_package_share_directory(engine_package);
    engine_path_ = package_path / "engines" / engine_filename;

    RCLCPP_INFO(get_logger(),
      "Parameters initialized - Engine: %s, Classes: %d, Lanes: %d, Height: %d, Width: %d",
      engine_path_.c_str(), config_.num_classes, config_.num_lanes, config_.height, config_.width);

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

bool SCNNLaneDetection::initialize_inferencer()
{
  // Check if engine file exists
  if (!fs::exists(engine_path_)) {
    RCLCPP_ERROR(get_logger(), "Engine file does not exist: %s", engine_path_.c_str());
    return false;
  }

  try {
    detector_ = std::make_shared<scnn_trt_backend::SCNNTrtBackend>(engine_path_, config_);

    if (!detector_) {
      RCLCPP_ERROR(get_logger(), "Failed to create SCNNTrtBackend instance");
      return false;
    }

    RCLCPP_INFO(get_logger(), "TensorRT inferencer initialized successfully");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception creating SCNNTrtBackend: %s", e.what());
    return false;
  }
}

void SCNNLaneDetection::initialize_ros_components()
{
  // Configure QoS profile for reliable image transport
  rclcpp::QoS image_qos(queue_size_);
  image_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  image_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  image_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // Create a single REENTRANT callback group for all callbacks
  // Since they're thread-safe, they can all run in parallel
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Create subscription options with dedicated callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  // Create subscriber with proper callback binding
  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_, image_qos,
    std::bind(&SCNNLaneDetection::image_callback, this, std::placeholders::_1),
    sub_options
  );

  // Create publishers
  lane_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, image_qos);
  lane_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(output_overlay_topic_, image_qos);
  exist_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_exist_topic_, image_qos);

  // Create timer for processing at specified frequency
  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&SCNNLaneDetection::timer_callback, this),
    callback_group_
  );

  RCLCPP_INFO(get_logger(), "ROS components initialized with separate callback groups");
  RCLCPP_INFO(get_logger(), "Input: %s, Output: %s, Frequency: %.1f Hz",
    input_topic_.c_str(), output_topic_.c_str(), processing_frequency_);
}

void SCNNLaneDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Thread-safe queue management
    std::lock_guard<std::mutex> lock(mtx_);

    // Check if queue is full
    if (img_buff_.size() >= static_cast<size_t>(max_processing_queue_size_)) {
      // Remove oldest image to make room for new one
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest image (queue size: %ld)", img_buff_.size());
      img_buff_.pop();
    }

    // Add new image to queue
    img_buff_.push(msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in image callback: %s", e.what());
  }
}

void SCNNLaneDetection::timer_callback()
{
  // Skip if already processing or no subscribers
  if (processing_in_progress_.load()) {
    return;
  }

  // Get next image from queue
  sensor_msgs::msg::Image::SharedPtr msg;
  bool has_image = false;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!img_buff_.empty()) {
      msg = img_buff_.front();
      img_buff_.pop();
      has_image = true;
    }
  }

  if (!has_image) {
    return;  // No image to process
  }

  // Set processing flag
  processing_in_progress_.store(true);

  try {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty or invalid image");
      processing_in_progress_.store(false);
      return;
    }

    // Process the image
    scnn_trt_backend::SCNNResult result = process_image(cv_ptr->image);

    if (!result.seg_pred.empty()) {
      // Create overlay
      cv::Mat overlay = scnn_trt_backend::utils::create_overlay(
        cv_ptr->image, result.seg_pred, 0.5f);

      // Publish results
      if (lane_pub_->get_subscription_count() > 0) {
        publish_segmentation_result(result.seg_pred, msg->header);
      }

      if (lane_overlay_pub_->get_subscription_count() > 0) {
        publish_overlay_result(overlay, msg->header);
      }

      if (exist_pub_->get_subscription_count() > 0) {
        publish_lane_existence(result.exist_pred, msg->header);
      }
    } else {
      RCLCPP_WARN(get_logger(), "Lane detection processing returned empty result");
    }

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during image processing: %s", e.what());
  }

  // Clear processing flag
  processing_in_progress_.store(false);
}

scnn_trt_backend::SCNNResult SCNNLaneDetection::process_image(const cv::Mat & input_image)
{
  if (input_image.empty()) {
    RCLCPP_WARN(get_logger(), "Input image is empty");
    return scnn_trt_backend::SCNNResult();
  }

  try {
    // Resize image to model input size if necessary
    cv::Mat processed_image;
    if (input_image.cols != config_.width || input_image.rows != config_.height) {
      cv::resize(input_image, processed_image, cv::Size(config_.width, config_.height), 0, 0,
        cv::INTER_LINEAR);
    } else {
      processed_image = input_image;
    }

    // Run inference
    scnn_trt_backend::SCNNResult result = detector_->infer(processed_image);

    if (result.seg_pred.empty()) {
      RCLCPP_WARN(get_logger(), "Inference returned empty result");
      return scnn_trt_backend::SCNNResult();
    }

    // Resize segmentation back to original size if necessary
    if (result.seg_pred.cols != input_image.cols || result.seg_pred.rows != input_image.rows) {
      cv::Mat resized_mask;
      cv::resize(result.seg_pred, resized_mask,
        cv::Size(input_image.cols, input_image.rows), 0, 0, cv::INTER_NEAREST);
      result.seg_pred = resized_mask;
    }

    return result;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during image processing: %s", e.what());
    return scnn_trt_backend::SCNNResult();
  }
}

void SCNNLaneDetection::publish_segmentation_result(
  const cv::Mat & segmentation,
  const std_msgs::msg::Header & header)
{
  try {
    // Convert OpenCV image back to ROS message
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = segmentation;

    // Publish the result
    auto output_msg = cv_image.toImageMsg();
    lane_pub_->publish(*output_msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during result publishing: %s", e.what());
  }
}

void SCNNLaneDetection::publish_overlay_result(
  const cv::Mat & overlay,
  const std_msgs::msg::Header & header)
{
  try {
    // Convert OpenCV image back to ROS message
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = overlay;

    // Publish the result
    auto output_msg = cv_image.toImageMsg();
    lane_overlay_pub_->publish(*output_msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during result publishing: %s", e.what());
  }
}

void SCNNLaneDetection::publish_lane_existence(
  const std::array<float, 4> & exist_pred,
  const std_msgs::msg::Header & header)
{
  try {
    std_msgs::msg::Float32MultiArray msg;

    // Set up layout (optional but good practice)
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].label = "lanes";
    msg.layout.dim[0].size = 4;
    msg.layout.dim[0].stride = 4;
    msg.layout.data_offset = 0;

    // Copy data
    msg.data.assign(exist_pred.begin(), exist_pred.end());

    // Publish
    exist_pub_->publish(msg);

    // Note: Float32MultiArray doesn't have a header field
    // If timestamp synchronization is needed, consider using a custom message
    (void)header;  // Suppress unused parameter warning

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during lane existence publishing: %s", e.what());
  }
}

}  // namespace scnn_lane_detection
