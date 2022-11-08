ROS package to run artifact detection using a tflite model. An EdgeTPU USB accelerator will be used if found, otherwise inference will be done on CPU. Dooway detection now doesn't have CPU version of the model.

It is necessary to install the Edge TPU runtime:
```
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list

curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

sudo apt-get update

sudo apt-get install libedgetpu1-std
```
### Potensial Opencv path issue:
### 1. /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake 
add Opencv path, for example
```
set(_include_dirs "include;/usr/include;/usr/include/opencv4")
```
### 2. in artifact_detection_nn_tflite/CMakelist.txt
comment out 
```
find_package(OpenCV 3.0 QUIET)
```
add Opencv path, for example:
```
find_package(OpenCV REQUIRED PATHS "/usr/include/opencv4")
```

### HOW to CLONE

### 0. Set up git LFS
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs
```

### 1. Clone this repository into ROS workspace
```
git clone https://gitlab.robotics.caltech.edu/rollocopter/artifact/artifact_detection_nn_tflite.git
cd artifact_detection_nn_tflite && git lfs install
git checkout feature/doorway_detection
```

### 2. Build
```
catkin build
```

### HOW TO RUN
```
roslaunch artifact_detection_nn_tflite  edgetpu_detection_doorway.launch robot_namespace:=spot2 
```

More details about the requirements and installations can be found at [https://coral.ai/docs/accelerator/get-started/](https://coral.ai/docs/accelerator/get-started/).
