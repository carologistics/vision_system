#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "object_tracking/object_tracking.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/quaternion.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>

ObjectTrackingServer::ObjectTrackingServer() : Node("object_tracking_server")
{
	RCLCPP_INFO(this->get_logger(), "Object tracking node starting...");
	this->init();

	// create service
	object_tracking_service_ = this->create_service<ObjectTrackingService>(
    "object_tracking",
    [this](const std::shared_ptr<ObjectTrackingRequest> request,
           std::shared_ptr<ObjectTrackingResponse> response) {
        this->handle_msgs(request, response);
    });

	// create buffer for pose publisher
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());

	// create yolo starting publisher
    yolo_publisher_ = this->create_publisher<picam_client::srv::StreamControl>("/picam_client/stream_control", 10);

	// create tf broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	// call update_pose for every time YOLO outputs something
    auto yolo_callback =
      [this](vision_msgs::msg::Detection2DArray yolo_output) {
        if (!tracking_active_) return;
        yolo_detections_ = yolo_output;
		this->update_pose();
    };
    yolo_subscription_ =
      this->create_subscription<vision_msgs::msg::Detection2DArray>("detections", rclcpp::SensorDataQoS(), yolo_callback);
}

void ObjectTrackingServer::init()
{
	// Helper to declare and get parameters strictly
	auto get_param_strict = [this](const std::string & name, auto & variable) {
		this->declare_parameter<typename std::remove_reference<decltype(variable)>::type>(name);
		if (!this->get_parameter(name, variable)) {
			RCLCPP_FATAL(this->get_logger(), "Parameter '%s' not set! Crashing node.", name.c_str());
			throw std::runtime_error("Required parameter missing: " + name);
		}
	};

	get_param_strict("camera_width", camera_width_);
	get_param_strict("camera_height", camera_height_);
	get_param_strict("camera_ppx", camera_ppx_);
	get_param_strict("camera_ppy", camera_ppy_);
	get_param_strict("camera_fy", camera_fy_);
	get_param_strict("camera_fx", camera_fx_);
	get_param_strict("object_widths", object_widths_);
	get_param_strict("puck_height", puck_height_);

	std::vector<double> weights;
	get_param_strict("filter_weights", weights);
	
	filter_size_ = std::min(static_cast<int>(weights.size()), 5); 
	for (int i = 0; i < filter_size_; ++i) {
		filter_weights_[i] = weights[i];
	}

	past_responses_.clear();
	tracking_active_ = false;
}

void ObjectTrackingServer::handle_msgs(const std::shared_ptr<ObjectTrackingRequest> request,
          std::shared_ptr<ObjectTrackingResponse> response)
{
	RCLCPP_INFO(this->get_logger(), "Handle control request");
	if(request->enable){
		// sanity checks
		if(request->object_type != "WORKPIECE" &&
		   request->object_type != "CONVEYOR" &&
		   request->object_type != "SLIDE"){
			RCLCPP_ERROR(this->get_logger(), "Invalid Request Parameter! Object type: %s unknown!", request->object_type.c_str());
			tracking_active_ = false;
			response->error = "Invalid Request Parameter";
			response->success = false;
			return;
		} else if(request->distance_threshold < 0){
			RCLCPP_ERROR(this->get_logger(), "Invalid Request Parameter! Negative distance threshold: %f", request->distance_threshold);
			tracking_active_ = false;
			response->error = "Invalid Request Parameter";
			response->success = false;
			return;
		}
	// create yolo message
    auto start_yolo_message = picam_client::srv::StreamControl::Request();

	if(request->object_type == "WORKPIECE"){
		start_yolo_message.command = picam_client::srv::StreamControl::Request::SWITCH_TO_WORKPIECE;
	} else if(request->object_type == "CONVEYOR"){
		start_yolo_message.command = picam_client::srv::StreamControl::Request::SWITCH_TO_CONVEYOR;
	} else if(request->object_type == "SLIDE"){
		start_yolo_message.command = picam_client::srv::StreamControl::Request::SWITCH_TO_SLIDE;
	}
	// toggle yolo
	this->yolo_publisher_->publish(start_yolo_message);

	//todo: toggle laser-line / expected pose estimation on

	response->success = true;
	} else {
		// create yolo message
		auto end_yolo_message = picam_client::srv::StreamControl::Request();
		end_yolo_message.command = picam_client::srv::StreamControl::Request::SWITCH_OFF_DETECTION;

		//todo: toggle laser-line / expected pose estimation off
		response->success = true;
		return;
	}

	RCLCPP_INFO(this->get_logger(), "Incoming request: \nobject_type: %s \nreference_frame: %s \ndistance_threshold: %f \nobject_tf_name: %s",
                request->object_type.c_str(), request->reference_frame.c_str(), request->distance_threshold, request->object_tf_name.c_str());

	tracking_active_ = true;
	current_object_type_ = request->object_type;
	current_reference_frame_ = request->reference_frame;
	current_distance_threshold_ = request->distance_threshold;
	current_object_tf_name_ = request->object_tf_name;
}

void ObjectTrackingServer::update_pose()
{
	if (!tracking_active_) return;

	RCLCPP_INFO(this->get_logger(), "update");

	// get angle of target MPS (though 6D tf pose)
	geometry_msgs::msg::TransformStamped t_mps;
	try {
		t_mps = tf_buffer_->lookupTransform(
		current_reference_frame_,
		"base_link",
		yolo_detections_.header.stamp,
		std::chrono::milliseconds(20));
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s", current_reference_frame_.c_str(), ex.what());
		return;
	}
	double mps_angle = tf2::getYaw(t_mps.transform.rotation);

	float cur_object_pos_target[3];
	bool detected = closest_position(yolo_detections_, mps_angle, current_reference_frame_, current_distance_threshold_, cur_object_pos_target);

	if(!detected) return;

	// create transform from cam to odom
	geometry_msgs::msg::TransformStamped t_odom;
	try {
		t_odom = tf_buffer_->lookupTransform(
		"odom",
		"cam_frame",
		yolo_detections_.header.stamp,
		std::chrono::milliseconds(20));
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN(this->get_logger(), "Could not transform cam_frame to odom: %s", ex.what());
		return;
	}

	// transform cur_object_pos_target from cam_frame to odom
	geometry_msgs::msg::TransformStamped t_cam;

	t_cam.header.stamp = yolo_detections_.header.stamp;
	t_cam.header.frame_id = "cam_frame";
	t_cam.child_frame_id = "cur_target_object";

	t_cam.transform.translation.x = cur_object_pos_target[0];
	t_cam.transform.translation.y = cur_object_pos_target[1];
	t_cam.transform.translation.z = cur_object_pos_target[2];
	tf2::Quaternion q;
	q.setRPY(
	0,
	0, 
	mps_angle);
	t_cam.transform.rotation.x = q.x();
	t_cam.transform.rotation.y = q.y();
	t_cam.transform.rotation.z = q.z();
	t_cam.transform.rotation.w = q.w();

	geometry_msgs::msg::TransformStamped t_current_detection;
	tf2::doTransform(t_cam, t_current_detection, t_odom);

	//late todo: only show used bounding box in image and publish

	// compute weighted average to improve robustness of object position
	double weighted_object_pos[3];
	double sum_weights = 0;

	weighted_object_pos[0] = filter_weights_[0] * t_current_detection.transform.translation.x;
	weighted_object_pos[1] = filter_weights_[0] * t_current_detection.transform.translation.y;
	weighted_object_pos[2] = filter_weights_[0] * t_current_detection.transform.translation.z;
	sum_weights = filter_weights_[0];
	

	for (size_t i = 0; i < past_responses_.size(); i++) {
		//late todo: transform each past_response[i] to current time before adding (shouldn't matter, since odom is static)
		weighted_object_pos[0] += filter_weights_[1 + i] * past_responses_[i].transform.translation.x;
		weighted_object_pos[1] += filter_weights_[1 + i] * past_responses_[i].transform.translation.y;
		weighted_object_pos[2] += filter_weights_[1 + i] * past_responses_[i].transform.translation.z;
		sum_weights += filter_weights_[1 + i];
	}

	weighted_object_pos[0] /= sum_weights;
	weighted_object_pos[1] /= sum_weights;
	weighted_object_pos[2] /= sum_weights;

	past_responses_.push_front(t_current_detection);
	if (static_cast<int>(past_responses_.size()) == filter_size_) {
		past_responses_.pop_back();
	}

	// update tf
	geometry_msgs::msg::TransformStamped t_pub;

	t_pub.header.stamp = yolo_detections_.header.stamp;
	t_pub.header.frame_id = "odom";
	t_pub.child_frame_id = current_object_tf_name_;

	t_pub.transform.translation.x = weighted_object_pos[0];
	t_pub.transform.translation.y = weighted_object_pos[1];
	t_pub.transform.translation.z = weighted_object_pos[2];
	tf2::Quaternion q_pub;
	q_pub.setRPY(
	0,
	0, 
	mps_angle);
	t_pub.transform.rotation.x = q_pub.x();
	t_pub.transform.rotation.y = q_pub.y();
	t_pub.transform.rotation.z = q_pub.z();
	t_pub.transform.rotation.w = q_pub.w();
	
    // Send the transformation
    tf_broadcaster_->sendTransform(t_pub);
	RCLCPP_INFO(this->get_logger(), "yes yes");
}

bool ObjectTrackingServer::closest_position(vision_msgs::msg::Detection2DArray      yolo_detections,
                                       float                 mps_angle,
                                       std::string           reference_frame,
                                       float                 distance_threshold,
                                       float                 closest_pos[3])
{
	float min_dist = distance_threshold;

	// create transform from cam to expected pose
	geometry_msgs::msg::TransformStamped t_ref;
	try {
		t_ref = tf_buffer_->lookupTransform(
		reference_frame,
		"cam_frame",
		yolo_detections_.header.stamp,
		std::chrono::milliseconds(20));
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN(this->get_logger(), "Lookup failed for tracking reference: %s", ex.what());
		return false;
	}

	// compute all poses of bounding boxes
	for (size_t i = 0; i < yolo_detections.detections.size(); ++i) {
		float pos[3];
		project_3d_point(yolo_detections.detections[i].bbox, mps_angle, pos);

		// compare with reference frame
		geometry_msgs::msg::TransformStamped t_pos;

		t_pos.header.stamp = yolo_detections.header.stamp;
		t_pos.header.frame_id = "cam_frame";
		t_pos.child_frame_id = "potential_object_pos";

		t_pos.transform.translation.x = pos[0];
		t_pos.transform.translation.y = pos[1];
		t_pos.transform.translation.z = pos[2];
		tf2::Quaternion q_pos;
		q_pos.setRPY(
		0,
		0, 
		mps_angle);
		t_pos.transform.rotation.x = q_pos.x();
		t_pos.transform.rotation.y = q_pos.y();
		t_pos.transform.rotation.z = q_pos.z();
		t_pos.transform.rotation.w = q_pos.w();

		geometry_msgs::msg::TransformStamped t_diff;
		tf2::doTransform(t_pos, t_diff, t_ref);

		// check distance to expected pose
		float dist = sqrt(t_diff.transform.translation.x * t_diff.transform.translation.x +
		                  t_diff.transform.translation.y * t_diff.transform.translation.y +
						  t_diff.transform.translation.z * t_diff.transform.translation.z);
		RCLCPP_INFO(this->get_logger(), "dist: %f", dist);
		if (dist < min_dist) {
			min_dist          = dist;
			closest_pos[0]    = pos[0];
			closest_pos[1]    = pos[1];
			closest_pos[2]    = pos[2];
		}
	}

	return min_dist < distance_threshold;
}

void ObjectTrackingServer::project_3d_point(vision_msgs::msg::BoundingBox2D bounding_box, float mps_angle, float point[3])
{
	// compute bounding box values
	float bb_left    = bounding_box.center.position.x - bounding_box.size_x / 2;
	float bb_right   = bounding_box.center.position.x + bounding_box.size_x / 2;
	float bb_bottom  = bounding_box.center.position.y - bounding_box.size_y / 2;
	//float bb_top     = bounding_box.center.position.y + bounding_box.size_y / 2;
	float bb_centerY = bounding_box.center.position.y;

	//delta values:
	float dx_left   = (bb_left * camera_width_ - camera_ppx_) / camera_fx_;
	float dx_right  = (bb_right * camera_width_ - camera_ppx_) / camera_fx_;
	float dy_bottom = (bb_bottom * camera_height_ - camera_ppy_) / camera_fy_;
	//float dy_top    = (bb_top * camera_height_ - camera_ppy_) / camera_fy_;
	float dy_center = (bb_centerY * camera_height_ - camera_ppy_) / camera_fy_;

	if (dx_left >= dx_right) {
    RCLCPP_INFO(this->get_logger(), "Width of 0: Cannot project into 3D space!");
		point[0] = 0;
		point[1] = 0;
		point[2] = 0;
		return;
	}

	float object_width;
	if(current_object_type_ == "CONVEYOR") object_width = object_widths_[0];
	else if(current_object_type_ == "SLIDE") object_width = object_widths_[1];
	else if(current_object_type_ == "WORKPIECE") object_width = object_widths_[2];
	else RCLCPP_INFO(this->get_logger(), "Unknown object_type!");

	float angle = mps_angle;

	// workpiece angles depend only on the camera view and not the mps
	if (current_object_type_ == "WORKPIECE") {
		angle = atan((dx_right + dx_left) / 2);
	}

	// distance towards object center point
	float dist = ((cos(angle) + sin(angle) * dx_left) * object_width) / (dx_right - dx_left)
	             + sin(angle) * object_width / 2;

	// compute middle point with deltas and distance
	point[0] = dist;
	point[1] = -(dx_left + dx_right) * dist / 2;

	if (current_object_type_ == "WORKPIECE") {
		// compute base middle point using the bottom point + wp_height/2
		point[2] = dy_bottom * dist + puck_height_ / 2;
	} else {
		point[2] = dy_center * dist;
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectTrackingServer>());
	rclcpp::shutdown();
	return 0;
}
