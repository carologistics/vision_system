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

#ifndef picam_client__PICAM_CLIENT_NODE_HPP_
#define picam_client__PICAM_CLIENT_NODE_HPP_

#include <picam_client/srv/set_confidence.hpp>
#include <picam_client/srv/set_iou.hpp>
#include <picam_client/srv/stream_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <arpa/inet.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

namespace picam_client {

class PicamClientNode : public rclcpp::Node {
public:
  explicit PicamClientNode();
  virtual ~PicamClientNode();

private:
  // ROS parameters
  std::string server_ip_;
  int server_port_;
  int camera_width_;
  int camera_height_;

  // Connection state
  bool connected_{false};
  int disconnect_counter_{0};
  uint64_t last_msg_time_{0};
  int msg_counter_{0};

  // Socket vars
  int sockfd_;
  struct sockaddr_in server_addr_;
  std::vector<char> data_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr
      detections_pub_;

  // Services
  rclcpp::Service<picam_client::srv::SetConfidence>::SharedPtr
      set_confidence_srv_;
  rclcpp::Service<picam_client::srv::SetIOU>::SharedPtr set_iou_srv_;
  rclcpp::Service<picam_client::srv::StreamControl>::SharedPtr
      stream_control_srv_;

  // Remove timer_ member
  // Add new members:
  std::thread read_thread_;
  bool should_exit_{false};
  void read_loop();

  // Methods
  void timer_callback();
  int connect_to_server();
  void send_configure_message();
  void send_control_message(uint8_t message_type);
  void send_control_message(uint8_t message_type, float payload);
  bool receive_data(int sockfd, char *buffer, size_t size);
  void handle_image_message(const std::vector<char> &data, bool marked);
  void handle_detection_message(const std::vector<char> &data);

  // Service callbacks
  void handle_set_confidence(
      const std::shared_ptr<picam_client::srv::SetConfidence::Request> request,
      std::shared_ptr<picam_client::srv::SetConfidence::Response> response);
  void handle_set_iou(
      const std::shared_ptr<picam_client::srv::SetIOU::Request> request,
      std::shared_ptr<picam_client::srv::SetIOU::Response> response);
  void handle_stream_control(
      const std::shared_ptr<picam_client::srv::StreamControl::Request> request,
      std::shared_ptr<picam_client::srv::StreamControl::Response> response);

  // Utility functions
  uint64_t ntohll(uint64_t val);
  float ntohlf(float val);
  uint32_t htonf(float val);

  static constexpr int HEADER_SIZE_1 = 20;
  static constexpr int HEADER_SIZE_2 = 20;
  static constexpr int HEADER_SIZE_3 = 32;
  static constexpr int CONTROL_HEADER_SIZE = 1;
  static constexpr int CONTROL_HEADER_SIZE_PAYLOAD = 5;
  static constexpr int CONFIGURE_MESSAGE_SIZE = 57;
  static constexpr int SLEEP_INTERVAL = 1;
  static constexpr int DISCONNECT_THRESHOLD = 10;
  static constexpr int RECONNECT_INTERVAL = 5;
};

} // namespace picam_client

#endif // picam_client__PICAM_CLIENT_NODE_HPP_
