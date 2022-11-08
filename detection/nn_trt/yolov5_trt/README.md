Artifact Reporting using TRT
====================
### change the fan speed to max
```
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'
cat /sys/devices/pwm-fan/cur_pwm
```

## Installation
```
catkin build torch_trt_ros
```

### how to run
```
source devel/setup.bash
roslaunch trt_ros trt_detection_rs.launch robot_namespace:=husky4` # rgb only
roslaunch trt_ros trt_detection_thermal.launch robot_namespace:=husky4 #thermal only
roslaunch trt_ros trt_detection_ros.launch robot_namespace:=husky4 #both rgb and thermal + color filter
```

### TO:
1. image selection
2. multi threading
3. [done] artifact-specific conf threshold
