# Object Tracking
Start object tracking plugin with:
```
ros2 run object_tracking object_tracking
```

Send service request over terminal with:
```
ros2 service call object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: bool, object_type: 'OBJECT', reference_frame: 'frame_name', distance_threshold: x.y, object_tf_name: 'name'}"
```

Example:
```
ros2 service call object_tracking robotino_vision_msgs/srv/ToggleObjectTracking "{enable: true, object_type: 'WORKPIECE', reference_frame: '/cam', distance_threshold: 0.2, object_tf_name: 'workpiece'}"
```

enable:             turns the tool on (if all other parameters are set) or off (enough if enable == false)

object_type:        either 'WORKPIECE', 'CONVEYOR', or 'SLIDE'

reference_frame:    name for the expected pose of the object

distance_threshold: aceptable distance between expected pose and detected object pose in meter

object_tf_name:     name the resulting object by any string
