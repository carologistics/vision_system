#ifndef OBJECT_TRACKING__OBJECT_TRACKING_HPP_
#define OBJECT_TRACKING__OBJECT_TRACKING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <memory>

using ObjectTrackingService = robotino_vision_msgs::srv::ToggleObjectTracking;

//public:
using ObjectTrackingRequest = ObjectTrackingService::Request;
using ObjectTrackingResponse = ObjectTrackingService::Response;

//protected:
void init();
void compute_pose(const std::shared_ptr<ObjectTrackingRequest>,
                       std::shared_ptr<ObjectTrackingResponse>);
bool closest_position(std::vector<std::array<float, 4>>      bounding_boxes,
                                       float                 mps_angle,
                                       std::string           reference_frame,
                                       float                 distance_threshold,
                                       float                 closest_pos[3]);
void project_3d_point(std::array<float, 4> bounding_box, float mps_angle, float point[3]);

int   camera_width_;
int   camera_height_;
float camera_ppx_;
float camera_ppy_;
float camera_fx_;
float camera_fy_;
std::vector<float> object_widths_;
double filter_weights_[5];
int filter_size_;
std::deque<geometry_msgs::msg::TransformStamped> past_responses_;
std::string current_object_type_;
float puck_height_;




int main(int argc, char **argv);


#endif // OBJECT_TRACKING__OBJECT_TRACKING_HPP_