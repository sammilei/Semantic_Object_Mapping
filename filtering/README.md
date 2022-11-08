Artifact Detection Filters
==========================

This is a python node. 

It subscribes to topics from darknet_ros node to get a bounding box and image, 
and calculates the color probability. It then combines (currently addition) 
the color and yolo probability into a new probability, which overwrites the 
previous yolo probability in the detected Object message. The updated 
detected Object message is then published and will be used in the 
artifact_localization node.

There is a debug feature which allows to save the image with overlaid bounding box
and color score.

The color filter has its launchfile, which is called inside the darknet_ros launch file.
Therefore the system should be launched automatically when detection node is launched.


**! IMPORTANT !** In darknet_ros config files 
(ros_front.yaml, ros_left.yaml, ros_right.yaml, thermal.yaml) change 
*publishers/detected_object/topic* from ***detected_object*** 
to ***detected_object_raw***. IF YOU USE OTHER CONFIG FILES, MAKE SURE
THE TOPIC FOR PUBLISHERS ARE CORRECT

Note: thermal detection also go through this node, 
but for the moment no filters are implemented. 
The node just subscribes and republish.


## Important tasks still to be done:
- ~~**Launch file** that can be called either when we run darknet_ros.launch or 
  artifact_localization.launch~~
- ~~**parametrs** currently defined at the top of python code, should so either a 
config file or argument to the launch file~~
- ~~can I get the robot name directly in the code (not as a parameter)~~
- ~~Need to do a script for thermal detection, which does not do anything 
  but transfer the message further 
  (because I changed the types of message published)~~
- the same detections are sent multiple times (10x-20x) from darknet ros. 
I don't know why this happens ... I did a small fix to avoid processing some of these 
  useless messages, but there are still some (when multiple detections at same time)
- tune the blue color filter
