#ifndef PICAM_CLIENT_ROS__PICAM_CLIENT_NODE_HPP_
#define PICAM_CLIENT_ROS__PICAM_CLIENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <picam_client_ros/srv/set_confidence.hpp>
#include <picam_client_ros/srv/set_iou.hpp>
#include <picam_client_ros/srv/stream_control.hpp>

#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <chrono>
#include <string>
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
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;

  // Services
  rclcpp::Service<picam_client_ros::srv::SetConfidence>::SharedPtr set_confidence_srv_;
  rclcpp::Service<picam_client_ros::srv::SetIOU>::SharedPtr set_iou_srv_;
  rclcpp::Service<picam_client_ros::srv::StreamControl>::SharedPtr stream_control_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Methods
  void timer_callback();
  int connect_to_server();
  void send_configure_message();
  void send_control_message(uint8_t message_type);
  void send_control_message(uint8_t message_type, float payload);
  bool receive_data(int sockfd, char* buffer, size_t size);
  void handle_image_message(const std::vector<char>& data, bool marked);
  void handle_detection_message(const std::vector<char>& data);
  
  // Service callbacks
  void handle_set_confidence(
    const std::shared_ptr<picam_client_ros::srv::SetConfidence::Request> request,
    std::shared_ptr<picam_client_ros::srv::SetConfidence::Response> response);
  void handle_set_iou(
    const std::shared_ptr<picam_client_ros::srv::SetIOU::Request> request,
    std::shared_ptr<picam_client_ros::srv::SetIOU::Response> response);
  void handle_stream_control(
    const std::shared_ptr<picam_client_ros::srv::StreamControl::Request> request,
    std::shared_ptr<picam_client_ros::srv::StreamControl::Response> response);

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

}  // namespace picam_client

#endif  // PICAM_CLIENT_ROS__PICAM_CLIENT_NODE_HPP_