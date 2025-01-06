#include "rclcpp/rclcpp.hpp"
#include "object_tracking/object_tracking.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <memory>

void init()
{
	//get camera params (later todo: make it parameterizable)
	camera_width_     = 480;
	camera_height_    = 640;
	camera_ppx_       = 251.00801023972;
	camera_ppy_       = 301.32794812152997;
	camera_fy_        = 634.6348457656962;
	camera_fx_        = 642.6147379302428;

	//set object params
	//               {Conveyor, Slide, Workpiece}
	object_widths_ = {0.03, 0.0585, 0.04};

	//set up weighted average filter
	//-------------------------------------------------------------------------
	filter_weights_[0] = 0.2; // current response
	filter_weights_[1] = 0.2; // last response
	filter_weights_[2] = 0.2; // 2. last response
	filter_weights_[3] = 0.2; // 3. last response
	filter_weights_[4] = 0.2; // 4. last response
	//-------------------------------------------------------------------------
	filter_size_ = sizeof(filter_weights_) / sizeof(filter_weights_[0]);

	past_responses_.clear();

	puck_height_ = 0.025;
}

void compute_pose(const std::shared_ptr<ObjectTrackingRequest> request,
          std::shared_ptr<ObjectTrackingResponse> response)
{
	if(request->enable){
		//sanity checks
		if(request->object_type != "WORKPIECE" &&
		   request->object_type != "CONVEYOR" &&
		   request->object_type != "SLIDE"){
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid Request Parameter! Object type: %s unknown!", request->object_type.c_str());
			response->error = "Invalid Request Parameter";
			response->success = false;
			return;
		} else if(request->distance_threshold < 0){
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid Request Parameter! Negative distance threshold: %f", request->distance_threshold);
			response->error = "Invalid Request Parameter";
			response->success = false;
			return;
		}
	//todo: check if laserline and yolo is updating

	response->success = true;
	} else {
		//todo: stop spinning node
		response->success = true;
		return;
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: \nobject_type: %s \nreference_frame: %s \ndistance_threshold: %f \nobject_tf_name: %s",
                request->object_type.c_str(), request->reference_frame.c_str(), request->distance_threshold, request->object_tf_name.c_str());

	current_object_type_ = request->object_type;

	//todo: get angle of target MPS (though 6D tf pose)
	float mps_angle = 0;

	//todo: get image bounding boxes
	std::vector<std::array<float, 4>> bounding_boxes = {{0.5, 0.2, 0.3, 0.1}};

	float cur_object_pos_target[3];
	bool detected = closest_position(bounding_boxes, mps_angle, request->reference_frame, request->distance_threshold, cur_object_pos_target);

	if(!detected) return;

	//todo: transform cur_object_pos_target from cam_frame to odom

	//late todo: only show used bounding box in image and publish

	//compute weighted average to improve robustness of object position
	double weighted_object_pos[3];
	double sum_weights = 0;

	weighted_object_pos[0] = filter_weights_[0] * cur_object_pos_target[0];
	weighted_object_pos[1] = filter_weights_[0] * cur_object_pos_target[1];
	weighted_object_pos[2] = filter_weights_[0] * cur_object_pos_target[2];
	sum_weights = filter_weights_[0];
	

	for (size_t i = 0; i < past_responses_.size(); i++) {
		//todo: transform each past_response[i] to current time before adding
		weighted_object_pos[0] += filter_weights_[1 + i] * past_responses_[i].transform.translation.x;
		weighted_object_pos[1] += filter_weights_[1 + i] * past_responses_[i].transform.translation.y;
		weighted_object_pos[2] += filter_weights_[1 + i] * past_responses_[i].transform.translation.z;
		sum_weights += filter_weights_[1 + i];
	}

	weighted_object_pos[0] /= sum_weights;
	weighted_object_pos[1] /= sum_weights;
	weighted_object_pos[2] /= sum_weights;

	geometry_msgs::msg::TransformStamped t;

	//t.header.stamp = std::chrono::system_clock::now(); //this->get_clock()->now(); todo: needs to be a node before
	t.header.frame_id = "/odom";
	t.child_frame_id = "target_object";

	t.transform.translation.x = cur_object_pos_target[0];
	t.transform.translation.y = cur_object_pos_target[1];
	t.transform.translation.z = cur_object_pos_target[2];
	tf2::Quaternion q;
	q.setRPY(
	0,
	0, 
	0); //todo: add rotation depending on the mps yaw
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();

	past_responses_.push_front(t);
	if (past_responses_.size() == filter_size_) {
		past_responses_.pop_back();
	}

	//todo: update tf
	geometry_msgs::msg::TransformStamped t_pub;

	//t_pub.header.stamp = std::chrono::system_clock::now(); //this->get_clock()->now(); todo: needs to be a node before
	t_pub.header.frame_id = "/odom";
	t_pub.child_frame_id = "target_object";

	t_pub.transform.translation.x = weighted_object_pos[0];
	t_pub.transform.translation.y = weighted_object_pos[1];
	t_pub.transform.translation.z = weighted_object_pos[2];
	tf2::Quaternion q_pub;
	q_pub.setRPY(
	0,
	0, 
	0); //todo: add rotation depending on the mps yaw
	t_pub.transform.rotation.x = q_pub.x();
	t_pub.transform.rotation.y = q_pub.y();
	t_pub.transform.rotation.z = q_pub.z();
	t_pub.transform.rotation.w = q_pub.w();
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "yes yes");
}

bool closest_position(std::vector<std::array<float, 4>>      bounding_boxes,
                                       float                 mps_angle,
                                       std::string           reference_frame,
                                       float                 distance_threshold,
                                       float                 closest_pos[3])
{
	float min_dist = distance_threshold;

	//compute all poses of bounding boxes
	for (size_t i = 0; i < bounding_boxes.size(); ++i) {
		float pos[3];
		project_3d_point(bounding_boxes[i], mps_angle, pos);

		//compare with reference frame
		//todo: transform to reference_frame

		//check distance to expected pose
		float dist = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dist: %f", dist);
		if (dist < min_dist) {
			min_dist          = dist;
			closest_pos[0]    = pos[0];
			closest_pos[1]    = pos[1];
			closest_pos[2]    = pos[2];
		}
	}

	return min_dist < distance_threshold;
}

void project_3d_point(std::array<float, 4> bounding_box, float mps_angle, float point[3])
{
	//compute bounding box values
	float bb_left    = bounding_box[0] - bounding_box[2] / 2;
	float bb_right   = bounding_box[0] + bounding_box[2] / 2;
	float bb_bottom  = bounding_box[1] - bounding_box[3] / 2;
	//float bb_top     = bounding_box[1] + bounding_box[3] / 2;
	float bb_centerY = bounding_box[1];

	//delta values:
	float dx_left   = (bb_left * camera_width_ - camera_ppx_) / camera_fx_;
	float dx_right  = (bb_right * camera_width_ - camera_ppx_) / camera_fx_;
	float dy_bottom = (bb_bottom * camera_height_ - camera_ppy_) / camera_fy_;
	//float dy_top    = (bb_top * camera_height_ - camera_ppy_) / camera_fy_;
	float dy_center = (bb_centerY * camera_height_ - camera_ppy_) / camera_fy_;

	if (dx_left >= dx_right) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Width of 0: Cannot project into 3D space!");
		point[0] = 0;
		point[1] = 0;
		point[2] = 0;
		return;
	}

	float object_width;
	if(current_object_type_ == "CONVEYOR") object_width = object_widths_[0];
	else if(current_object_type_ == "SLIDE") object_width = object_widths_[1];
	else if(current_object_type_ == "WORKPIECE") object_width = object_widths_[2];
	else RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Unknown object_type!");

	float angle = mps_angle;

	//workpiece angles depend only on the camera view and not the mps
	if (current_object_type_ == "WORKPIECE") {
		angle = atan((dx_right + dx_left) / 2);
	}

	//distance towards object center point
	float dist = ((cos(angle) + sin(angle) * dx_left) * object_width) / (dx_right - dx_left)
	             + sin(angle) * object_width / 2;

	//compute middle point with deltas and distance
	point[0] = dist;
	point[1] = -(dx_left + dx_right) * dist / 2;

	if (current_object_type_ == "WORKPIECE") {
		//compute base middle point using the bottom point + wp_height/2
		point[2] = dy_bottom * dist + puck_height_ / 2;
	} else {
		point[2] = dy_center * dist;
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	init();

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("object_tracking_server");

	rclcpp::Service<ObjectTrackingService>::SharedPtr service =
    node->create_service<ObjectTrackingService>("object_tracking", &compute_pose);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to update object poses.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}
