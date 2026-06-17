# Object Tracking
The `object_tracking` node runs the ONNX segmentation model configured in `params.yaml` (`yoloe-26n-seg.onnx` by default) on `/camera/frame_rgb` as `bgr8`. It does not use depth images. For each candidate mask, it maps the organized 240x180 `/camera/frame_pc` points into the 640x480 segmentation mask by relative pixel position, averages the matching 3D points, and publishes the closest candidate to the requested reference frame.

Install ONNX Runtime C++ and set `ONNXRUNTIME_ROOT` if CMake cannot find it automatically.

Start object tracking plugin with:
```
ros2 launch object_tracking object_tracking.launch.py
```

Send service request over terminal with:
```
ros2 service call object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: bool, object_prompt: 'OBJECT PROMPT', reference_frame: 'frame_name', distance_threshold: x.y, object_tf_name: 'name'}"
```

Example:
```
ros2 service call object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: true, object_prompt: 'red workpiece', reference_frame: '/cam', distance_threshold: 0.2, object_tf_name: 'workpiece'}"
```

enable:             turns the tool on (if all other parameters are set) or off (enough if enable == false)

object_prompt:      free-text object prompt for YOLOE, for example 'red workpiece'

reference_frame:    name for the expected pose of the object

distance_threshold: aceptable distance between expected pose and detected object pose in meter

object_tf_name:     name the resulting object by any string
