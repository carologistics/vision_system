#include "object_tracking/object_tracking.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "onnxruntime_cxx_api.h"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
const sensor_msgs::msg::PointField * find_point_field(
	const sensor_msgs::msg::PointCloud2 & cloud,
	const std::string & name)
{
	for (const auto & field : cloud.fields) {
		if (field.name == name) {
			return &field;
		}
	}
	return nullptr;
}

bool read_float32_point_field(
	const sensor_msgs::msg::PointCloud2 & cloud,
	size_t base_offset,
	const sensor_msgs::msg::PointField & field,
	double & value)
{
	const size_t offset = base_offset + field.offset;
	if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
	    offset + sizeof(float) > cloud.data.size()) {
		return false;
	}

	float raw = 0.0F;
	std::memcpy(&raw, cloud.data.data() + offset, sizeof(raw));
	value = raw;
	return true;
}

size_t scaled_index(size_t index, uint32_t source_size, uint32_t target_size)
{
	if (source_size == 0 || target_size == 0) {
		return 0;
	}

	const double scaled =
		(static_cast<double>(index) + 0.5) * static_cast<double>(target_size) /
		static_cast<double>(source_size) - 0.5;
	const long rounded = std::lround(scaled);
	const long max_index = static_cast<long>(target_size) - 1;
	return static_cast<size_t>(std::clamp(rounded, 0L, max_index));
}

bool is_valid_point(double x, double y, double z)
{
	return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
	       (std::abs(x) + std::abs(y) + std::abs(z)) > std::numeric_limits<double>::epsilon();
}

std::string to_lower_ascii(const std::string & text)
{
	std::string lower = text;
	std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return lower;
}

bool is_prompt_input_name(const std::string & name)
{
	const std::string lower = to_lower_ascii(name);
	return lower.find("prompt") != std::string::npos ||
	       lower.find("text") != std::string::npos ||
	       lower.find("class") != std::string::npos;
}

bool is_blank(const std::string & text)
{
	return std::all_of(text.begin(), text.end(), [](unsigned char c) {
		return std::isspace(c) != 0;
	});
}

std::vector<int64_t> concrete_string_tensor_shape(const std::vector<int64_t> & shape)
{
	std::vector<int64_t> concrete_shape;
	concrete_shape.reserve(shape.size());
	for (const int64_t dim : shape) {
		concrete_shape.push_back(dim > 0 ? dim : 1);
	}
	return concrete_shape;
}

size_t tensor_element_count(const std::vector<int64_t> & shape)
{
	size_t count = 1;
	for (const int64_t dim : shape) {
		count *= static_cast<size_t>(dim > 0 ? dim : 1);
	}
	return count;
}
}  // namespace

struct ObjectTrackingServer::SegmentationOnnxState
{
	Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "object_tracking_segmentation"};
	Ort::SessionOptions session_options;
	std::unique_ptr<Ort::Session> session;
	std::vector<std::string> input_names;
	std::vector<std::string> output_names;
	std::vector<const char *> input_name_ptrs;
	std::vector<const char *> output_name_ptrs;
	std::vector<ONNXTensorElementDataType> input_types;
	std::vector<std::vector<int64_t>> input_shapes;
	size_t image_input_index{0};
	int prompt_input_index{-1};
};

ObjectTrackingServer::ObjectTrackingServer()
: Node("object_tracking_server"),
  segmentation_model_ready_(false)
{
	RCLCPP_INFO(this->get_logger(), "Object tracking node starting...");

	image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/frame_rgb");
	debug_image_path_ = this->declare_parameter<std::string>("debug_image_path", "/tmp/object_tracking_latest_image.ppm");
	pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/camera/frame_pc");
	camera_frame_ = this->declare_parameter<std::string>("camera_frame", "cam_frame");
	min_mask_points_ = std::max(1, static_cast<int>(this->declare_parameter<int>("min_mask_points", 20)));
	segmentation_model_path_ = this->declare_parameter<std::string>("segmentation_model_path", "yoloe-26n-seg.onnx");
	segmentation_target_size_ = std::max(1, static_cast<int>(this->declare_parameter<int>("segmentation_target_size", 640)));
	segmentation_confidence_ = this->declare_parameter<double>("segmentation_confidence", 0.25);
	segmentation_iou_ = this->declare_parameter<double>("segmentation_iou", 0.5);

	image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
		image_topic_,
		rclcpp::SensorDataQoS(),
		[this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
			{
				std::lock_guard<std::mutex> lock(latest_image_mutex_);
				latest_image_ = msg;
			}
			if (tracking_active_) {
				this->update_pose();
			}
		});
	RCLCPP_INFO(this->get_logger(), "Subscribing to camera images on %s", image_topic_.c_str());
	if (!debug_image_path_.empty()) {
		RCLCPP_INFO(this->get_logger(), "Will save latest debug image to %s", debug_image_path_.c_str());
	}

	if (!pointcloud_topic_.empty()) {
		pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			pointcloud_topic_,
			rclcpp::SensorDataQoS(),
			[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
				std::lock_guard<std::mutex> lock(latest_pointcloud_mutex_);
				latest_pointcloud_ = msg;
			});
		RCLCPP_INFO(this->get_logger(), "Subscribing to point cloud on %s", pointcloud_topic_.c_str());
	}

	this->init();

	object_tracking_service_ = this->create_service<ObjectTrackingService>(
		"object_tracking",
		[this](const std::shared_ptr<ObjectTrackingRequest> request,
		       std::shared_ptr<ObjectTrackingResponse> response) {
			this->handle_msgs(request, response);
		});

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void ObjectTrackingServer::init()
{
	auto get_param_strict = [this](const std::string & name, auto & variable) {
		this->declare_parameter<typename std::remove_reference<decltype(variable)>::type>(name);
		if (!this->get_parameter(name, variable)) {
			RCLCPP_FATAL(this->get_logger(), "Parameter '%s' not set! Crashing node.", name.c_str());
			throw std::runtime_error("Required parameter missing: " + name);
		}
	};

	std::vector<double> weights;
	get_param_strict("filter_weights", weights);
	if (weights.empty()) {
		RCLCPP_FATAL(this->get_logger(), "Parameter 'filter_weights' must contain at least one value");
		throw std::runtime_error("Invalid filter_weights parameter");
	}

	filter_size_ = std::min(static_cast<int>(weights.size()), 5);
	for (int i = 0; i < 5; ++i) {
		filter_weights_[i] = 0.0;
	}
	for (int i = 0; i < filter_size_; ++i) {
		filter_weights_[i] = weights[i];
	}

	past_responses_.clear();
	tracking_active_ = false;
}

ObjectTrackingServer::~ObjectTrackingServer()
{
	segmentation_onnx_.reset();
}

bool ObjectTrackingServer::init_segmentation_model()
{
	if (segmentation_model_ready_) {
		return true;
	}

	auto state = std::make_unique<SegmentationOnnxState>();
	state->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

	try {
		state->session = std::make_unique<Ort::Session>(
			state->env,
			segmentation_model_path_.c_str(),
			state->session_options);

		Ort::AllocatorWithDefaultOptions allocator;
		const size_t input_count = state->session->GetInputCount();
		const size_t output_count = state->session->GetOutputCount();
		for (size_t i = 0; i < input_count; ++i) {
			auto name = state->session->GetInputNameAllocated(i, allocator);
			state->input_names.emplace_back(name.get());

			auto type_info = state->session->GetInputTypeInfo(i);
			if (type_info.GetONNXType() != ONNX_TYPE_TENSOR) {
				state->input_types.push_back(ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED);
				state->input_shapes.emplace_back();
				continue;
			}

			auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
			state->input_types.push_back(tensor_info.GetElementType());
			state->input_shapes.push_back(tensor_info.GetShape());
		}
		for (size_t i = 0; i < output_count; ++i) {
			auto name = state->session->GetOutputNameAllocated(i, allocator);
			state->output_names.emplace_back(name.get());
		}
	} catch (const Ort::Exception & ex) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Could not load ONNX segmentation model '%s': %s",
			segmentation_model_path_.c_str(),
			ex.what());
		return false;
	}

	if (state->input_names.empty() || state->output_names.size() < 2) {
		RCLCPP_ERROR(
			this->get_logger(),
			"ONNX segmentation model must have at least one input and at least two outputs: detections and mask prototypes");
		return false;
	}

	bool found_image_input = false;
	for (size_t i = 0; i < state->input_names.size(); ++i) {
		if (!found_image_input &&
		    state->input_types[i] == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
		    state->input_shapes[i].size() == 4) {
			state->image_input_index = i;
			found_image_input = true;
			continue;
		}

		if (state->input_types[i] == ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING &&
		    (state->prompt_input_index < 0 || is_prompt_input_name(state->input_names[i]))) {
			state->prompt_input_index = static_cast<int>(i);
		}
	}

	if (!found_image_input) {
		RCLCPP_ERROR(this->get_logger(), "ONNX segmentation model does not expose a float image tensor input");
		return false;
	}

	for (size_t i = 0; i < state->input_names.size(); ++i) {
		if (i == state->image_input_index || static_cast<int>(i) == state->prompt_input_index) {
			continue;
		}

		RCLCPP_ERROR(
			this->get_logger(),
			"ONNX segmentation model input '%s' is not supported by this node. Runtime prompts require a string tensor input.",
			state->input_names[i].c_str());
		return false;
	}

	for (const auto & name : state->input_names) {
		state->input_name_ptrs.push_back(name.c_str());
	}
	for (const auto & name : state->output_names) {
		state->output_name_ptrs.push_back(name.c_str());
	}

	segmentation_onnx_ = std::move(state);
	segmentation_model_ready_ = true;

	RCLCPP_INFO(
		this->get_logger(),
		"Loaded ONNX segmentation model %s",
		segmentation_model_path_.c_str());
	if (segmentation_onnx_->prompt_input_index >= 0) {
		RCLCPP_INFO(
			this->get_logger(),
			"Using ONNX input '%s' for runtime object_prompt text",
			segmentation_onnx_->input_names[static_cast<size_t>(segmentation_onnx_->prompt_input_index)].c_str());
	} else {
		RCLCPP_WARN(
			this->get_logger(),
			"ONNX model has no string prompt input; for Ultralytics YOLOE exports, prompts are usually baked into the exported weights");
	}
	return true;
}

bool ObjectTrackingServer::create_segmentation_mask(
	const sensor_msgs::msg::Image & image,
	sensor_msgs::msg::Image & segmentation_mask)
{
	if (image.width == 0 || image.height == 0 ||
	    image.step == 0 ||
	    image.data.size() < static_cast<size_t>(image.step) * image.height) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Cannot segment invalid image: %ux%u, step=%u, data=%zu",
			image.width,
			image.height,
			image.step,
			image.data.size());
		return false;
	}

	if (!init_segmentation_model()) {
		return false;
	}

	if (image.encoding != "bgr8") {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Cannot segment image with encoding '%s'; expected bgr8",
			image.encoding.c_str());
		return false;
	}

	const int input_size = segmentation_target_size_;
	const int image_width = static_cast<int>(image.width);
	const int image_height = static_cast<int>(image.height);

	cv::Mat bgr(image_height, image_width, CV_8UC3);
	for (int y = 0; y < image_height; ++y) {
		const auto * row = image.data.data() + static_cast<size_t>(y) * image.step;
		std::memcpy(bgr.ptr(y), row, static_cast<size_t>(image_width) * 3);
	}
	cv::Mat rgb;
	cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

	const double scale = static_cast<double>(input_size) / std::max(image_width, image_height);
	const int resized_width = std::max(1, static_cast<int>(std::round(image_width * scale)));
	const int resized_height = std::max(1, static_cast<int>(std::round(image_height * scale)));
	const int pad_x = (input_size - resized_width) / 2;
	const int pad_y = (input_size - resized_height) / 2;

	cv::Mat resized;
	cv::resize(rgb, resized, cv::Size(resized_width, resized_height));
	cv::Mat padded(input_size, input_size, CV_8UC3, cv::Scalar(114, 114, 114));
	resized.copyTo(padded(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

	std::vector<float> input_data(static_cast<size_t>(3) * input_size * input_size);
	for (int y = 0; y < input_size; ++y) {
		const auto * row = padded.ptr<cv::Vec3b>(y);
		for (int x = 0; x < input_size; ++x) {
			const size_t pixel_index = static_cast<size_t>(y) * input_size + x;
			input_data[pixel_index] = static_cast<float>(row[x][0]) / 255.0F;
			input_data[static_cast<size_t>(input_size) * input_size + pixel_index] = static_cast<float>(row[x][1]) / 255.0F;
			input_data[static_cast<size_t>(2) * input_size * input_size + pixel_index] = static_cast<float>(row[x][2]) / 255.0F;
		}
	}

	std::array<int64_t, 4> input_shape{
		1,
		3,
		input_size,
		input_size
	};
	Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
	Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
		memory_info,
		input_data.data(),
		input_data.size(),
		input_shape.data(),
		input_shape.size());

	std::vector<Ort::Value> input_tensors;
	std::vector<const char *> run_input_names;
	input_tensors.push_back(std::move(input_tensor));
	run_input_names.push_back(segmentation_onnx_->input_name_ptrs[segmentation_onnx_->image_input_index]);

	std::vector<Ort::Value> outputs;
	try {
		Ort::AllocatorWithDefaultOptions string_allocator;
		if (segmentation_onnx_->prompt_input_index >= 0) {
			const auto prompt_input_index = static_cast<size_t>(segmentation_onnx_->prompt_input_index);
			const std::vector<int64_t> prompt_shape =
				concrete_string_tensor_shape(segmentation_onnx_->input_shapes[prompt_input_index]);
			const size_t prompt_value_count = tensor_element_count(prompt_shape);
			const std::string object_prompt = current_object_prompt_;
			std::vector<const char *> prompt_values(prompt_value_count, object_prompt.c_str());
			Ort::Value prompt_tensor = Ort::Value::CreateTensor(
				string_allocator,
				prompt_shape.empty() ? nullptr : prompt_shape.data(),
				prompt_shape.size(),
				ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING);
			prompt_tensor.FillStringTensor(prompt_values.data(), prompt_values.size());

			input_tensors.push_back(std::move(prompt_tensor));
			run_input_names.push_back(segmentation_onnx_->input_name_ptrs[prompt_input_index]);
		}

		outputs = segmentation_onnx_->session->Run(
			Ort::RunOptions{nullptr},
			run_input_names.data(),
			input_tensors.data(),
			input_tensors.size(),
			segmentation_onnx_->output_name_ptrs.data(),
			segmentation_onnx_->output_name_ptrs.size());
	} catch (const Ort::Exception & ex) {
		RCLCPP_ERROR(this->get_logger(), "ONNX segmentation inference failed: %s", ex.what());
		return false;
	}

	if (outputs.size() < 2 || !outputs[0].IsTensor() || !outputs[1].IsTensor()) {
		RCLCPP_ERROR(this->get_logger(), "ONNX segmentation model returned invalid outputs");
		return false;
	}

	auto det_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
	auto proto_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
	if (det_shape.size() != 3 || proto_shape.size() != 4) {
		RCLCPP_ERROR(this->get_logger(), "Unsupported ONNX segmentation output shapes");
		return false;
	}

	float * det_data = outputs[0].GetTensorMutableData<float>();
	float * proto_data = outputs[1].GetTensorMutableData<float>();
	const int proto_channels = static_cast<int>(proto_shape[1]);
	const int proto_height = static_cast<int>(proto_shape[2]);
	const int proto_width = static_cast<int>(proto_shape[3]);

	int detection_count = 0;
	int attribute_count = 0;
	bool transposed = false;
	if (det_shape[1] < det_shape[2]) {
		detection_count = static_cast<int>(det_shape[2]);
		attribute_count = static_cast<int>(det_shape[1]);
		transposed = true;
	} else {
		detection_count = static_cast<int>(det_shape[1]);
		attribute_count = static_cast<int>(det_shape[2]);
	}

	const int class_count = attribute_count - 4 - proto_channels;
	if (detection_count <= 0 || class_count <= 0 || proto_channels <= 0) {
		RCLCPP_ERROR(this->get_logger(), "Unsupported ONNX segmentation tensor dimensions");
		return false;
	}

	auto detection_value = [det_data, detection_count, attribute_count, transposed](int detection, int attribute) {
		if (transposed) {
			return det_data[static_cast<size_t>(attribute) * detection_count + detection];
		}
		return det_data[static_cast<size_t>(detection) * attribute_count + attribute];
	};

	std::vector<cv::Rect> boxes;
	std::vector<float> scores;
	std::vector<std::vector<float>> mask_coefficients;

	for (int i = 0; i < detection_count; ++i) {
		int best_class = 0;
		float best_score = detection_value(i, 4);
		for (int cls = 1; cls < class_count; ++cls) {
			const float score = detection_value(i, 4 + cls);
			if (score > best_score) {
				best_score = score;
				best_class = cls;
			}
		}

		(void)best_class;
		if (best_score < segmentation_confidence_) {
			continue;
		}

		const float cx = detection_value(i, 0);
		const float cy = detection_value(i, 1);
		const float w = detection_value(i, 2);
		const float h = detection_value(i, 3);

		const int left = static_cast<int>(std::round(cx - w / 2.0F));
		const int top = static_cast<int>(std::round(cy - h / 2.0F));
		const int width = static_cast<int>(std::round(w));
		const int height = static_cast<int>(std::round(h));
		if (width <= 0 || height <= 0) {
			continue;
		}

		boxes.emplace_back(left, top, width, height);
		scores.push_back(best_score);

		std::vector<float> coeffs(proto_channels);
		for (int coeff = 0; coeff < proto_channels; ++coeff) {
			coeffs[coeff] = detection_value(i, 4 + class_count + coeff);
		}
		mask_coefficients.push_back(std::move(coeffs));
	}

	std::vector<int> kept_indices;
	cv::dnn::NMSBoxes(boxes, scores, segmentation_confidence_, segmentation_iou_, kept_indices);

	cv::Mat label_mask(input_size, input_size, CV_8UC1, cv::Scalar(0));
	int label = 1;
	for (const int detection_index : kept_indices) {
		if (label > 255) {
			break;
		}

		cv::Mat mask_logits(proto_height, proto_width, CV_32FC1, cv::Scalar(0.0F));
		for (int coeff = 0; coeff < proto_channels; ++coeff) {
			const float weight = mask_coefficients[detection_index][coeff];
			const float * proto = proto_data + static_cast<size_t>(coeff) * proto_height * proto_width;
			for (int y = 0; y < proto_height; ++y) {
				float * out_row = mask_logits.ptr<float>(y);
				const float * proto_row = proto + static_cast<size_t>(y) * proto_width;
				for (int x = 0; x < proto_width; ++x) {
					out_row[x] += weight * proto_row[x];
				}
			}
		}

		cv::Mat mask_prob;
		cv::exp(-mask_logits, mask_prob);
		mask_prob = 1.0 / (1.0 + mask_prob);
		cv::resize(mask_prob, mask_prob, cv::Size(input_size, input_size), 0.0, 0.0, cv::INTER_LINEAR);

		cv::Mat instance_mask = mask_prob > 0.5;
		cv::Rect clipped_box = boxes[detection_index] & cv::Rect(0, 0, input_size, input_size);
		if (clipped_box.empty()) {
			continue;
		}

		cv::Mat box_mask(input_size, input_size, CV_8UC1, cv::Scalar(0));
		instance_mask(clipped_box).copyTo(box_mask(clipped_box));
		label_mask.setTo(static_cast<uint8_t>(label), box_mask);
		++label;
	}

	const cv::Rect image_area(pad_x, pad_y, resized_width, resized_height);
	cv::Mat unpadded = label_mask(image_area);
	cv::Mat original_size_mask;
	cv::resize(unpadded, original_size_mask, cv::Size(image_width, image_height), 0.0, 0.0, cv::INTER_NEAREST);

	segmentation_mask = sensor_msgs::msg::Image();
	segmentation_mask.header = image.header;
	segmentation_mask.height = image.height;
	segmentation_mask.width = image.width;
	segmentation_mask.encoding = "mono8";
	segmentation_mask.is_bigendian = 0;
	segmentation_mask.step = image.width;
	const size_t mask_bytes = static_cast<size_t>(image.width) * image.height;
	segmentation_mask.data.assign(original_size_mask.data, original_size_mask.data + mask_bytes);
	return true;
}

void ObjectTrackingServer::save_debug_image(const sensor_msgs::msg::Image & image)
{
	if (debug_image_path_.empty()) {
		return;
	}

	if (image.encoding != "bgr8") {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Cannot save debug image with encoding '%s'; expected bgr8",
			image.encoding.c_str());
		return;
	}

	const size_t width = image.width;
	const size_t height = image.height;
	const size_t min_step = width * 3;
	const size_t expected_size = static_cast<size_t>(image.step) * height;
	if (width == 0 || height == 0 || image.step < min_step || image.data.size() < expected_size) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Cannot save invalid debug image: %ux%u, step=%u, data=%zu",
			image.width,
			image.height,
			image.step,
			image.data.size());
		return;
	}

	std::ofstream file(debug_image_path_, std::ios::binary);
	if (!file) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Could not open debug image file '%s' for writing",
			debug_image_path_.c_str());
		return;
	}

	file << "P6\n" << width << " " << height << "\n255\n";
	for (size_t y = 0; y < height; ++y) {
		const auto * row = image.data.data() + y * image.step;
		for (size_t x = 0; x < width; ++x) {
			const auto * pixel = row + x * 3;
			file.put(static_cast<char>(pixel[2]));
			file.put(static_cast<char>(pixel[1]));
			file.put(static_cast<char>(pixel[0]));
		}
	}

	if (!file) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Failed while writing debug image file '%s'",
			debug_image_path_.c_str());
		return;
	}

	RCLCPP_INFO_THROTTLE(
		this->get_logger(),
		*this->get_clock(),
		2000,
		"Saved latest image to %s (%ux%u, %s)",
		debug_image_path_.c_str(),
		image.width,
		image.height,
		image.encoding.c_str());
}

void ObjectTrackingServer::handle_msgs(
	const std::shared_ptr<ObjectTrackingRequest> request,
	std::shared_ptr<ObjectTrackingResponse> response)
{
	RCLCPP_INFO(this->get_logger(), "Handle control request");
	response->error.clear();

	if (request->enable) {
		if (is_blank(request->object_prompt)) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid Request Parameter! object_prompt must not be empty");
			tracking_active_ = false;
			response->error = "Invalid Request Parameter";
			response->success = false;
			return;
		}

		if (request->distance_threshold < 0.0) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid Request Parameter! Negative distance threshold: %f",
				request->distance_threshold);
			tracking_active_ = false;
			response->error = "Invalid Request Parameter";
			response->success = false;
			return;
		}

		if (!init_segmentation_model()) {
			tracking_active_ = false;
			response->error = "Could not load segmentation model";
			response->success = false;
			return;
		}

		response->success = true;
	} else {
		tracking_active_ = false;
		past_responses_.clear();
		response->success = true;
		return;
	}

	RCLCPP_INFO(
		this->get_logger(),
		"Incoming request: \nobject_prompt: %s \nreference_frame: %s \ndistance_threshold: %f \nobject_tf_name: %s",
		request->object_prompt.c_str(),
		request->reference_frame.c_str(),
		request->distance_threshold,
		request->object_tf_name.c_str());

	past_responses_.clear();
	tracking_active_ = true;
	current_object_prompt_ = request->object_prompt;
	current_reference_frame_ = request->reference_frame;
	current_distance_threshold_ = request->distance_threshold;
	current_object_tf_name_ = request->object_tf_name;
}

void ObjectTrackingServer::update_pose()
{
	if (!tracking_active_) {
		return;
	}

	sensor_msgs::msg::Image::ConstSharedPtr image;
	{
		std::lock_guard<std::mutex> lock(latest_image_mutex_);
		image = latest_image_;
	}
	if (!image) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"No image received yet on %s; skipping pose update",
			image_topic_.c_str());
		return;
	}

	save_debug_image(*image);

	sensor_msgs::msg::Image segmentation_mask;
	if (!create_segmentation_mask(*image, segmentation_mask)) {
		return;
	}

	double cur_object_pos_target[3] = {0.0, 0.0, 0.0};
	std::string point_frame;
	rclcpp::Time point_stamp(0, 0, RCL_ROS_TIME);
	const bool detected = closest_position(
		segmentation_mask,
		current_reference_frame_,
		current_distance_threshold_,
		cur_object_pos_target,
		point_frame,
		point_stamp);

	if (!detected) {
		return;
	}

	geometry_msgs::msg::TransformStamped t_mps;
	try {
		t_mps = tf_buffer_->lookupTransform(
			current_reference_frame_,
			"base_link",
			point_stamp,
			std::chrono::milliseconds(20));
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN(
			this->get_logger(),
			"Could not transform %s to base_link: %s",
			current_reference_frame_.c_str(),
			ex.what());
		return;
	}
	const double mps_angle = tf2::getYaw(t_mps.transform.rotation);

	geometry_msgs::msg::TransformStamped t_odom;
	try {
		t_odom = tf_buffer_->lookupTransform(
			"odom",
			point_frame,
			point_stamp,
			std::chrono::milliseconds(20));
	} catch (const tf2::TransformException & ex) {
		RCLCPP_WARN(
			this->get_logger(),
			"Could not transform %s to odom: %s",
			point_frame.c_str(),
			ex.what());
		return;
	}

	geometry_msgs::msg::TransformStamped t_camera;
	t_camera.header.stamp = point_stamp;
	t_camera.header.frame_id = point_frame;
	t_camera.child_frame_id = "cur_target_object";
	t_camera.transform.translation.x = cur_object_pos_target[0];
	t_camera.transform.translation.y = cur_object_pos_target[1];
	t_camera.transform.translation.z = cur_object_pos_target[2];

	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, mps_angle);
	t_camera.transform.rotation.x = q.x();
	t_camera.transform.rotation.y = q.y();
	t_camera.transform.rotation.z = q.z();
	t_camera.transform.rotation.w = q.w();

	geometry_msgs::msg::TransformStamped t_current_detection;
	tf2::doTransform(t_camera, t_current_detection, t_odom);

	double weighted_object_pos[3];
	double sum_weights = 0.0;
	weighted_object_pos[0] = filter_weights_[0] * t_current_detection.transform.translation.x;
	weighted_object_pos[1] = filter_weights_[0] * t_current_detection.transform.translation.y;
	weighted_object_pos[2] = filter_weights_[0] * t_current_detection.transform.translation.z;
	sum_weights = filter_weights_[0];

	for (size_t i = 0; i < past_responses_.size() && i + 1 < static_cast<size_t>(filter_size_); ++i) {
		weighted_object_pos[0] += filter_weights_[1 + i] * past_responses_[i].transform.translation.x;
		weighted_object_pos[1] += filter_weights_[1 + i] * past_responses_[i].transform.translation.y;
		weighted_object_pos[2] += filter_weights_[1 + i] * past_responses_[i].transform.translation.z;
		sum_weights += filter_weights_[1 + i];
	}

	if (sum_weights <= std::numeric_limits<double>::epsilon()) {
		RCLCPP_WARN(this->get_logger(), "Skipping pose update because temporal filter weights sum to zero");
		return;
	}

	weighted_object_pos[0] /= sum_weights;
	weighted_object_pos[1] /= sum_weights;
	weighted_object_pos[2] /= sum_weights;

	past_responses_.push_front(t_current_detection);
	while (static_cast<int>(past_responses_.size()) >= filter_size_) {
		past_responses_.pop_back();
	}

	geometry_msgs::msg::TransformStamped t_pub;
	t_pub.header.stamp = point_stamp;
	t_pub.header.frame_id = "odom";
	t_pub.child_frame_id = current_object_tf_name_;
	t_pub.transform.translation.x = weighted_object_pos[0];
	t_pub.transform.translation.y = weighted_object_pos[1];
	t_pub.transform.translation.z = weighted_object_pos[2];

	tf2::Quaternion q_pub;
	q_pub.setRPY(0.0, 0.0, mps_angle);
	t_pub.transform.rotation.x = q_pub.x();
	t_pub.transform.rotation.y = q_pub.y();
	t_pub.transform.rotation.z = q_pub.z();
	t_pub.transform.rotation.w = q_pub.w();

	tf_broadcaster_->sendTransform(t_pub);
	RCLCPP_INFO(
		this->get_logger(),
		"Published tracked %s as %s at %.3f %.3f %.3f in odom",
		current_object_prompt_.c_str(),
		current_object_tf_name_.c_str(),
		weighted_object_pos[0],
		weighted_object_pos[1],
		weighted_object_pos[2]);
}

bool ObjectTrackingServer::closest_position(
	const sensor_msgs::msg::Image & segmentation_mask,
	const std::string & reference_frame,
	double distance_threshold,
	double closest_pos[3],
	std::string & point_frame,
	rclcpp::Time & point_stamp)
{
	if (segmentation_mask.encoding != "mono8") {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Segmentation mask encoding '%s' is not supported; expected mono8",
			segmentation_mask.encoding.c_str());
		return false;
	}

	if (segmentation_mask.width == 0 || segmentation_mask.height == 0 ||
	    segmentation_mask.step < segmentation_mask.width ||
	    segmentation_mask.data.size() < static_cast<size_t>(segmentation_mask.step) * segmentation_mask.height) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"Invalid segmentation mask image: %ux%u, step=%u, data=%zu",
			segmentation_mask.width,
			segmentation_mask.height,
			segmentation_mask.step,
			segmentation_mask.data.size());
		return false;
	}

	std::array<bool, 256> labels{};
	for (size_t y = 0; y < segmentation_mask.height; ++y) {
		const auto * row = segmentation_mask.data.data() + y * segmentation_mask.step;
		for (size_t x = 0; x < segmentation_mask.width; ++x) {
			labels[row[x]] = true;
		}
	}

	sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
	{
		std::lock_guard<std::mutex> lock(latest_pointcloud_mutex_);
		pointcloud = latest_pointcloud_;
	}

	if (!pointcloud) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"No point cloud received yet on %s; skipping pose update",
			pointcloud_topic_.c_str());
		return false;
	}

	double min_dist = distance_threshold;
	bool found = false;

	for (size_t label_id = 1; label_id < labels.size(); ++label_id) {
		if (!labels[label_id]) {
			continue;
		}

		double pos[3] = {0.0, 0.0, 0.0};
		std::string candidate_frame;
		rclcpp::Time candidate_stamp(0, 0, RCL_ROS_TIME);
		if (!project_3d_point(
			    segmentation_mask,
			    static_cast<uint8_t>(label_id),
			    pointcloud,
			    pos,
			    candidate_frame,
			    candidate_stamp)) {
			continue;
		}

		geometry_msgs::msg::TransformStamped t_ref;
		try {
			t_ref = tf_buffer_->lookupTransform(
				reference_frame,
				candidate_frame,
				candidate_stamp,
				std::chrono::milliseconds(20));
		} catch (const tf2::TransformException & ex) {
			RCLCPP_WARN(this->get_logger(), "Lookup failed for tracking reference: %s", ex.what());
			continue;
		}

		geometry_msgs::msg::PointStamped point_camera;
		point_camera.header.stamp = candidate_stamp;
		point_camera.header.frame_id = candidate_frame;
		point_camera.point.x = pos[0];
		point_camera.point.y = pos[1];
		point_camera.point.z = pos[2];

		geometry_msgs::msg::PointStamped point_reference;
		tf2::doTransform(point_camera, point_reference, t_ref);

		const double dist =
			std::sqrt(point_reference.point.x * point_reference.point.x +
			          point_reference.point.y * point_reference.point.y +
			          point_reference.point.z * point_reference.point.z);
		RCLCPP_DEBUG(this->get_logger(), "Mask %zu distance to reference: %.3f", label_id, dist);

		if (dist < min_dist) {
			min_dist = dist;
			closest_pos[0] = pos[0];
			closest_pos[1] = pos[1];
			closest_pos[2] = pos[2];
			point_frame = candidate_frame;
			point_stamp = candidate_stamp;
			found = true;
		}
	}

	return found;
}

bool ObjectTrackingServer::project_3d_point(
	const sensor_msgs::msg::Image & segmentation_mask,
	uint8_t mask_id,
	const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
	double point[3],
	std::string & point_frame,
	rclcpp::Time & point_stamp)
{
	if (pointcloud && pointcloud->width > 0 && pointcloud->height > 1 && !pointcloud->data.empty()) {
		const auto * x_field = find_point_field(*pointcloud, "x");
		const auto * y_field = find_point_field(*pointcloud, "y");
		const auto * z_field = find_point_field(*pointcloud, "z");
		if (x_field && y_field && z_field &&
		    x_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
		    y_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
		    z_field->datatype == sensor_msgs::msg::PointField::FLOAT32) {
			double sum_x = 0.0;
			double sum_y = 0.0;
			double sum_z = 0.0;
			int valid_points = 0;

			for (size_t cloud_y = 0; cloud_y < pointcloud->height; ++cloud_y) {
				// RGB segmentation is 640x480 while the organized point cloud is 240x180.
				// Map by relative image position so each cloud point is checked against the mask once.
				const size_t mask_y = scaled_index(cloud_y, pointcloud->height, segmentation_mask.height);
				const auto * mask_row = segmentation_mask.data.data() + mask_y * segmentation_mask.step;

				for (size_t cloud_x = 0; cloud_x < pointcloud->width; ++cloud_x) {
					const size_t mask_x = scaled_index(cloud_x, pointcloud->width, segmentation_mask.width);
					if (mask_row[mask_x] != mask_id) {
						continue;
					}

					const size_t point_offset =
						static_cast<size_t>(pointcloud->row_step) * cloud_y +
						static_cast<size_t>(pointcloud->point_step) * cloud_x;

					double x = 0.0;
					double y = 0.0;
					double z = 0.0;
					if (!read_float32_point_field(*pointcloud, point_offset, *x_field, x) ||
					    !read_float32_point_field(*pointcloud, point_offset, *y_field, y) ||
					    !read_float32_point_field(*pointcloud, point_offset, *z_field, z) ||
					    !is_valid_point(x, y, z)) {
						continue;
					}

					sum_x += x;
					sum_y += y;
					sum_z += z;
					++valid_points;
				}
			}

			if (valid_points >= min_mask_points_) {
				point[0] = sum_x / valid_points;
				point[1] = sum_y / valid_points;
				point[2] = sum_z / valid_points;
				point_frame = pointcloud->header.frame_id.empty() ? camera_frame_ : pointcloud->header.frame_id;
				point_stamp = rclcpp::Time(pointcloud->header.stamp);
				return true;
			}

			RCLCPP_DEBUG(
				this->get_logger(),
				"Mask %u only had %d valid point-cloud points",
				mask_id,
				valid_points);
		} else {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				2000,
				"Point cloud on %s does not contain FLOAT32 x/y/z fields",
				pointcloud_topic_.c_str());
		}
	}
	return false;
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectTrackingServer>());
	rclcpp::shutdown();
	return 0;
}
