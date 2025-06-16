#ifndef OBJECT_TRACKING__OBJECT_TRACKING_HPP_
#define OBJECT_TRACKING__OBJECT_TRACKING_HPP_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <picam_client/srv/stream_control.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>

using ObjectTrackingService = robotino_vision_msgs::srv::ToggleObjectTracking;
using ObjectTrackingRequest = ObjectTrackingService::Request;
using ObjectTrackingResponse = ObjectTrackingService::Response;

class ObjectTrackingServer : public rclcpp::Node {
    public:
    ObjectTrackingServer();
    void handle_msgs(const std::shared_ptr<ObjectTrackingRequest>,
                        std::shared_ptr<ObjectTrackingResponse>);

    private:
    void init();
    void update_pose();
    bool closest_position(vision_msgs::msg::Detection2DArray  bounding_boxes,
                                        float                 mps_angle,
                                        std::string           reference_frame,
                                        float                 distance_threshold,
                                        float                 closest_pos[3]);
    void project_3d_point(vision_msgs::msg::BoundingBox2D bounding_box, float mps_angle, float point[3]);

    int   camera_width_;
    int   camera_height_;
    float camera_ppx_;
    float camera_ppy_;
    float camera_fx_;
    float camera_fy_;
    std::vector<float> object_widths_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_subscription_;
    rclcpp::Publisher<picam_client::srv::StreamControl>::SharedPtr yolo_publisher_;
    vision_msgs::msg::Detection2DArray yolo_detections_;
    double filter_weights_[5];
    int filter_size_;
    std::deque<geometry_msgs::msg::TransformStamped> past_responses_;
    bool tracking_active_;
    std::string current_reference_frame_;
    double current_distance_threshold_;
    std::string current_object_tf_name_;
    std::string current_object_type_;
    float puck_height_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<ObjectTrackingService>::SharedPtr object_tracking_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char **argv);


#endif // OBJECT_TRACKING__OBJECT_TRACKING_HPP_