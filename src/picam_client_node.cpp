#include "picam_client_ros/picam_client_node.hpp"
#include "base64.h"
#include <cv_bridge/cv_bridge.h>

PicamClientNode::PicamClientNode() : Node("picam_client")
{
  // Declare parameters
  this->declare_parameter("server_ip", "127.0.0.1");
  this->declare_parameter("server_port", 8080);
  this->declare_parameter("camera_width", 640);
  this->declare_parameter("camera_height", 480);

  // Get parameters
  server_ip_ = this->get_parameter("server_ip").as_string();
  server_port_ = this->get_parameter("server_port").as_int();
  camera_width_ = this->get_parameter("camera_width").as_int();
  camera_height_ = this->get_parameter("camera_height").as_int();

  // Create publishers
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
  image_marked_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_marked", 10);
  bbox_pub_ = this->create_publisher<picam_client_ros::msg::BoundingBox>("detections", 10);

  // Create services
  set_confidence_srv_ = this->create_service<picam_client_ros::srv::SetConfidence>(
    "set_confidence", 
    std::bind(&PicamClientNode::handle_set_confidence, this, std::placeholders::_1, std::placeholders::_2));

  set_iou_srv_ = this->create_service<picam_client_ros::srv::SetIOU>(
    "set_iou",
    std::bind(&PicamClientNode::handle_set_iou, this, std::placeholders::_1, std::placeholders::_2));

  stream_control_srv_ = this->create_service<picam_client_ros::srv::StreamControl>(
    "stream_control",
    std::bind(&PicamClientNode::handle_stream_control, this, std::placeholders::_1, std::placeholders::_2));

  // Create timer for main loop
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PicamClientNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "PiCam Client Node initialized");
  RCLCPP_INFO(this->get_logger(), "OpenCV version: %s", CV_VERSION);
}

void PicamClientNode::timer_callback()
{
  if (connected_) {
    if (disconnect_counter_ >= DISCONNECT_THRESHOLD) {
      RCLCPP_ERROR(this->get_logger(), "Connection lost");
      connected_ = false;
      close(sockfd_);
      return;
    }

    data_.resize(HEADER_SIZE_1);
    if (!receive_data(sockfd_, data_.data(), 1)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read message type");
      std::this_thread::sleep_for(std::chrono::seconds(SLEEP_INTERVAL));
      disconnect_counter_++;
      return;
    }

    uint8_t message_type = data_[0];

    switch(message_type) {
      case 1:
      case 2:
        handle_image_message(data_, message_type == 2);
        break;
      case 3:
        handle_detection_message(data_);
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown message type: %d", message_type);
    }
  } else {
    if (connect_to_server() < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_INTERVAL));
    } else {
      RCLCPP_INFO(this->get_logger(), "Connected to server");
      connected_ = true;
    }
  }
}

// Additional method implementations would follow...