#ifndef OBJECT_TRACKING__OBJECT_TRACKING_HPP_
#define OBJECT_TRACKING__OBJECT_TRACKING_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using ObjectTrackingService = robotino_vision_msgs::srv::ToggleObjectTracking;
using ObjectTrackingRequest = ObjectTrackingService::Request;
using ObjectTrackingResponse = ObjectTrackingService::Response;

class ObjectTrackingServer : public rclcpp::Node {
    public:
    ObjectTrackingServer();
    ~ObjectTrackingServer() override;
    void handle_msgs(const std::shared_ptr<ObjectTrackingRequest>,
                        std::shared_ptr<ObjectTrackingResponse>);

    private:
    void init();
    void save_debug_image(const sensor_msgs::msg::Image & image);
    void update_pose();
    bool init_segmentation_model();
    bool create_segmentation_mask(const sensor_msgs::msg::Image & image,
                                        sensor_msgs::msg::Image & segmentation_mask);
    bool closest_position(const sensor_msgs::msg::Image & segmentation_mask,
                                        const std::string & reference_frame,
                                        double              distance_threshold,
                                        double              closest_pos[3],
                                        std::string &       point_frame,
                                        rclcpp::Time &      point_stamp);
    bool project_3d_point(const sensor_msgs::msg::Image & segmentation_mask,
                                        uint8_t             mask_id,
                                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
                                        double              point[3],
                                        std::string &       point_frame,
                                        rclcpp::Time &      point_stamp);

    std::string image_topic_;
    std::string debug_image_path_;
    std::string pointcloud_topic_;
    std::string camera_frame_;
    std::string segmentation_model_path_;
    int segmentation_target_size_;
    double segmentation_confidence_;
    double segmentation_iou_;
    int min_mask_points_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_pointcloud_;
    std::mutex latest_image_mutex_;
    std::mutex latest_pointcloud_mutex_;
    struct SegmentationOnnxState;
    std::unique_ptr<SegmentationOnnxState> segmentation_onnx_;
    bool segmentation_model_ready_;
    double filter_weights_[5];
    int filter_size_;
    std::deque<geometry_msgs::msg::TransformStamped> past_responses_;
    bool tracking_active_;
    std::string current_reference_frame_;
    double current_distance_threshold_;
    std::string current_object_tf_name_;
    std::string current_object_prompt_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<ObjectTrackingService>::SharedPtr object_tracking_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};


int main(int argc, char **argv);


#endif // OBJECT_TRACKING__OBJECT_TRACKING_HPP_
