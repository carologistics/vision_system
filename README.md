# Object Tracking
The `object_tracking` node runs the ONNX segmentation model configured in `params.yaml` (`yoloe-26n-seg.onnx` by default) on `/camera/frame_rgb` as `bgr8`. It does not use depth images. For each candidate mask, it maps the organized 240x180 `/camera/frame_pc` points into the 640x480 segmentation mask by relative pixel position, averages the matching 3D points, and publishes the closest candidate to the requested reference frame.

Install ONNX Runtime C++ and set `ONNXRUNTIME_ROOT` if CMake cannot find it automatically.

---
Start object tracking plugin with:
```
ros2 launch object_tracking object_tracking.launch.py namespace:=robotinobase1
```

If you want to capture images:
```
ros2 launch object_tracking object_tracking.launch.py namespace:=robotinobase1 capture:=true
```

If you want to visualize the segmented image in rviz:
```
ros2 launch object_tracking object_tracking.launch.py namespace:=robotinobase1 debug:=true
```
---

Send service request over terminal with:
```
ros2 service call object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: bool, object_prompt: 'OBJECT PROMPT', reference_frame: 'frame_name', distance_threshold: x.y, segmentation_confidence: 0.2, target_color: '', object_tf_name: 'name'}"
```

Example:
```
ros2 service call /robotinobase1/object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: true, object_prompt: 'grey eurobox EG 3212', reference_frame: 'base_link', distance_threshold: 10.0, segmentation_confidence: 0.2, target_color: '', object_tf_name: 'tracked_object'}"
```

Track a yellow lego brick with color filtering:
```
ros2 service call /robotinobase1/object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: true, object_prompt: 'toy brick', reference_frame: 'base_link', distance_threshold: 10.0, segmentation_confidence: 0.1, target_color: 'yellow', object_tf_name: 'tracked_object'}"
```

enable:             turns the tool on (if all other parameters are set) or off (enough if enable == false)

object_prompt:      free-text object prompt for YOLOE, for example 'red workpiece'

reference_frame:    name for the expected pose of the object

distance_threshold: acceptable distance between expected pose and detected object pose in meter

segmentation_confidence: YOLO segmentation confidence threshold for this tracking request. Use 0.0 to keep the configured default.

target_color:      optional mask color filter. Empty disables filtering; supported values are green, blue, red, and yellow.

object_tf_name:     name the resulting object by any string
