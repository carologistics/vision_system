// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "picam_client/picam_client_node.hpp"
#include "picam_client/base64.h"
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <thread>

namespace picam_client {

PicamClientNode::PicamClientNode() : Node("picam_client") {
  declare_parameter("server_ip", "127.0.0.1");
  declare_parameter("server_port", 8080);
  declare_parameter("camera_width", 640);
  declare_parameter("camera_height", 480);

  // Declare detection parameters
  declare_parameter("detection.confidence", 0.5);
  declare_parameter("detection.iou", 0.45);

  server_ip_ = get_parameter("server_ip").as_string();
  server_port_ = get_parameter("server_port").as_int();
  camera_width_ = get_parameter("camera_width").as_int();
  camera_height_ = get_parameter("camera_height").as_int();

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
  image_marked_pub_ =
      create_publisher<sensor_msgs::msg::Image>("camera/image_marked", 10);
  detections_pub_ =
      create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);

  set_confidence_srv_ = create_service<picam_client::srv::SetConfidence>(
      "/picam_client/set_confidence", // Add full path
      std::bind(&PicamClientNode::handle_set_confidence, this,
                std::placeholders::_1, std::placeholders::_2));

  set_iou_srv_ = create_service<picam_client::srv::SetIOU>(
      "/picam_client/set_iou", // Add full path
      std::bind(&PicamClientNode::handle_set_iou, this, std::placeholders::_1,
                std::placeholders::_2));

  stream_control_srv_ = create_service<picam_client::srv::StreamControl>(
      "/picam_client/stream_control", // Add full path
      std::bind(&PicamClientNode::handle_stream_control, this,
                std::placeholders::_1, std::placeholders::_2));

  // After creating services, add debug logs
  RCLCPP_INFO(get_logger(), "Service '/set_confidence' created at: %s",
              set_confidence_srv_->get_service_name());
  RCLCPP_INFO(get_logger(), "Service '/set_iou' created at: %s",
              set_iou_srv_->get_service_name());
  RCLCPP_INFO(get_logger(), "Service '/stream_control' created at: %s",
              stream_control_srv_->get_service_name());

  // Add a small delay to allow service discovery
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Remove timer creation
  // Instead, start the read thread:
  read_thread_ = std::thread(&PicamClientNode::read_loop, this);

  RCLCPP_INFO(get_logger(), "PiCam Client Node initialized");
  RCLCPP_INFO(get_logger(), "OpenCV version: %s", CV_VERSION);
}

PicamClientNode::~PicamClientNode() {
  should_exit_ = true;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (sockfd_ >= 0) {
    close(sockfd_);
  }
}

void PicamClientNode::read_loop() {
  while (!should_exit_) {
    if (!connected_) {
      if (connect_to_server() < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to connect, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_INTERVAL));
        continue;
      }
      RCLCPP_INFO(get_logger(), "Connected to server");
      connected_ = true;
    }

    // Read incoming messages
    data_.resize(1);
    if (!receive_data(sockfd_, data_.data(), 1)) {
      RCLCPP_ERROR(get_logger(), "Connection lost");
      connected_ = false;
      close(sockfd_);
      continue;
    }

    uint8_t message_type = data_[0];
    switch (message_type) {
    case 1:
      handle_image_message(data_, false);
      break;
    case 2:
      handle_image_message(data_, true);
      break;
    case 3:
      handle_detection_message(data_);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown message type: %d", message_type);
    }
  }
}

int PicamClientNode::connect_to_server() {
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Socket creation error");
    return -1;
  }

  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(server_port_);
  if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
    RCLCPP_ERROR(get_logger(), "Invalid address");
    close(sockfd_);
    return -1;
  }

  if (connect(sockfd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) <
      0) {
    RCLCPP_ERROR(get_logger(), "Connection failed");
    close(sockfd_);
    return -1;
  }

  send_configure_message();
  send_control_message(13, get_parameter("detection.iou").as_double());
  send_control_message(12, get_parameter("detection.confidence").as_double());

  return 0;
}

void PicamClientNode::send_configure_message() {
  char header[CONFIGURE_MESSAGE_SIZE];

  auto declare_and_get = [this](const std::string &name, float default_val) {
    declare_parameter(name, default_val);
    return get_parameter(name).as_double();
  };

  float old_f_x = declare_and_get("camera_matrix.old_f_x", 0.0);
  float old_f_y = declare_and_get("camera_matrix.old_f_y", 0.0);
  float old_ppx = declare_and_get("camera_matrix.old_ppx", 0.0);
  float old_ppy = declare_and_get("camera_matrix.old_ppy", 0.0);
  float new_f_x = declare_and_get("camera_matrix.new_f_x", 0.0);
  float new_f_y = declare_and_get("camera_matrix.new_f_y", 0.0);
  float new_ppx = declare_and_get("camera_matrix.new_ppx", 0.0);
  float new_ppy = declare_and_get("camera_matrix.new_ppy", 0.0);
  float k1 = declare_and_get("camera_matrix.k1", 0.0);
  float k2 = declare_and_get("camera_matrix.k2", 0.0);
  float k3 = declare_and_get("camera_matrix.k3", 0.0);
  float k4 = declare_and_get("camera_matrix.k4", 0.0);
  float k5 = declare_and_get("camera_matrix.k5", 0.0);

  int offset = 0;
  auto pack = [this, &header, &offset](float val) {
    uint32_t network_val = this->htonf(val);
    std::memcpy(&header[offset], &network_val, 4);
    offset += 4;
  };

  pack(old_f_x);
  pack(old_f_y);
  pack(old_ppx);
  pack(old_ppy);
  pack(new_f_x);
  pack(new_f_y);
  pack(new_ppx);
  pack(new_ppy);
  pack(k1);
  pack(k2);
  pack(k3);
  pack(k4);
  pack(k5);

  if (send(sockfd_, header, CONFIGURE_MESSAGE_SIZE, 0) !=
      CONFIGURE_MESSAGE_SIZE) {
    RCLCPP_ERROR(get_logger(), "Failed to send configuration");
  }
}

void PicamClientNode::send_control_message(uint8_t message_type) {
  uint8_t header[CONTROL_HEADER_SIZE] = {message_type};
  send(sockfd_, header, CONTROL_HEADER_SIZE, 0);
}

void PicamClientNode::send_control_message(uint8_t message_type,
                                           float payload) {
  char header[CONTROL_HEADER_SIZE_PAYLOAD];
  header[0] = message_type;
  uint32_t network_payload = htonf(payload);
  std::memcpy(&header[1], &network_payload, 4);
  send(sockfd_, header, CONTROL_HEADER_SIZE_PAYLOAD, 0);
}

bool PicamClientNode::receive_data(int sockfd, char *buffer, size_t size) {
  size_t total_bytes = 0;
  while (total_bytes < size) {
    ssize_t bytes = read(sockfd, buffer + total_bytes, size - total_bytes);
    if (bytes <= 0)
      return false;
    total_bytes += bytes;
  }
  return true;
}

void PicamClientNode::handle_image_message(const std::vector<char> & /*data*/,
                                           bool marked) {
  data_.resize(HEADER_SIZE_1);
  if (!receive_data(sockfd_, data_.data(), HEADER_SIZE_1)) {
    RCLCPP_ERROR(get_logger(), "Failed to read image header");
    return;
  }

  uint64_t timestamp;
  uint32_t width, height, length;
  std::memcpy(&timestamp, &data_[0], 8);
  std::memcpy(&height, &data_[8], 4);
  std::memcpy(&width, &data_[12], 4);
  std::memcpy(&length, &data_[16], 4);

  timestamp = ntohll(timestamp);
  height = ntohl(height);
  width = ntohl(width);
  length = ntohl(length);

  std::vector<char> image_data(length);
  if (!receive_data(sockfd_, image_data.data(), length)) {
    RCLCPP_ERROR(get_logger(), "Failed to read image data");
    return;
  }

  std::string base64_image(image_data.begin(), image_data.end());
  std::string decoded_image = base64_decode(base64_image);
  std::vector<uchar> img_data(decoded_image.begin(), decoded_image.end());
  cv::Mat img = cv::imdecode(img_data, cv::IMREAD_COLOR);

  if (img.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to decode image");
    return;
  }

  auto msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  msg->header.stamp = rclcpp::Time(timestamp);

  if (marked) {
    image_marked_pub_->publish(*msg);
  } else {
    image_pub_->publish(*msg);
  }
}

void PicamClientNode::handle_detection_message(
    const std::vector<char> & /*data*/) {
  data_.resize(HEADER_SIZE_3);
  if (!receive_data(sockfd_, data_.data(), HEADER_SIZE_3)) {
    RCLCPP_ERROR(get_logger(), "Failed to read detection header");
    return;
  }

  uint64_t timestamp;
  float x, y, h, w, acc;
  uint32_t cls;

  std::memcpy(&timestamp, &data_[0], 8);
  std::memcpy(&x, &data_[8], 4);
  std::memcpy(&y, &data_[12], 4);
  std::memcpy(&h, &data_[16], 4);
  std::memcpy(&w, &data_[20], 4);
  std::memcpy(&acc, &data_[24], 4);
  std::memcpy(&cls, &data_[28], 4);

  timestamp = ntohll(timestamp);
  x = ntohlf(x);
  y = ntohlf(y);
  h = ntohlf(h);
  w = ntohlf(w);
  acc = ntohlf(acc);
  cls = ntohl(cls);

  if (timestamp > last_msg_time_) {
    last_msg_time_ = timestamp;
    msg_counter_ = 0;
  }

  auto detection = vision_msgs::msg::Detection2D();
  detection.bbox.center.position.x = x;
  detection.bbox.center.position.y = y;
  detection.bbox.size_x = w;
  detection.bbox.size_y = h;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.score = acc;
  hypothesis.hypothesis.class_id = std::to_string(cls);
  detection.results.push_back(hypothesis);

  auto array_msg = vision_msgs::msg::Detection2DArray();
  array_msg.header.stamp = rclcpp::Time(timestamp);
  array_msg.detections.push_back(detection);
  detections_pub_->publish(array_msg);

  msg_counter_++;
}

void PicamClientNode::handle_set_confidence(
    const std::shared_ptr<picam_client::srv::SetConfidence::Request> request,
    std::shared_ptr<picam_client::srv::SetConfidence::Response> response) {
  send_control_message(12, request->confidence);
  response->success = true;
  response->message = "Confidence set";
}

void PicamClientNode::handle_set_iou(
    const std::shared_ptr<picam_client::srv::SetIOU::Request> request,
    std::shared_ptr<picam_client::srv::SetIOU::Response> response) {
  send_control_message(13, request->iou);
  response->success = true;
  response->message = "IOU set";
}

void PicamClientNode::handle_stream_control(
    const std::shared_ptr<picam_client::srv::StreamControl::Request> request,
    std::shared_ptr<picam_client::srv::StreamControl::Response> response) {
  switch (request->command) {
  case picam_client::srv::StreamControl::Request::ACTIVATE:
    send_control_message(4);
    response->success = true;
    response->message = "Stream activated";
    break;
  case picam_client::srv::StreamControl::Request::ACTIVATE_MARKED:
    send_control_message(5);
    response->success = true;
    response->message = "Marked stream activated";
    break;
  case picam_client::srv::StreamControl::Request::DEACTIVATE:
    send_control_message(6);
    response->success = true;
    response->message = "Stream deactivated";
    break;
  case picam_client::srv::StreamControl::Request::DEACTIVATE_MARKED:
    send_control_message(7);
    response->success = true;
    response->message = "Marked stream deactivated";
    break;
  case picam_client::srv::StreamControl::Request::SWITCH_TO_WORKPIECE:
    send_control_message(8);
    response->success = true;
    response->message = "Switched to workpiece detection";
    break;
  case picam_client::srv::StreamControl::Request::SWITCH_TO_CONVEYOR:
    send_control_message(9);
    response->success = true;
    response->message = "Switched to conveyor detection";
    break;
  case picam_client::srv::StreamControl::Request::SWITCH_TO_SLIDE:
    send_control_message(10);
    response->success = true;
    response->message = "Switched to slide detection";
    break;
  case picam_client::srv::StreamControl::Request::SWITCH_OFF_DETECTION:
    send_control_message(11);
    response->success = true;
    response->message = "Detection switched off";
    break;
  default:
    response->success = false;
    response->message = "Unknown command";
    break;
  }
}

uint64_t PicamClientNode::ntohll(uint64_t val) {
  return ((uint64_t)ntohl(val & 0xFFFFFFFF) << 32) | ntohl(val >> 32);
}

float PicamClientNode::ntohlf(float val) {
  uint32_t temp;
  std::memcpy(&temp, &val, 4);
  temp = ntohl(temp);
  std::memcpy(&val, &temp, 4);
  return val;
}

uint32_t PicamClientNode::htonf(float val) {
  uint32_t temp;
  std::memcpy(&temp, &val, 4);
  return htonl(temp);
}

} // namespace picam_client

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<picam_client::PicamClientNode>());
  rclcpp::shutdown();
  return 0;
}
