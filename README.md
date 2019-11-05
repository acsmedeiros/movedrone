This package requires ROS, Openpose and YOLO!

A sample result:


![Image of sample result](https://github.com/acsmedeiros/movedrone/blob/master/3.jpg)



Getting Started with the "movedrone" package:

1. Create a folder for a new catkin workspace:
    mkdir catkin_ws
    
2. Create a "src" folder inside the catkin workspace and copy the files from these repository inside it.
    cd catkin_ws
    mkdir src
    cd src
    git clone https://github.com/acsmedeiros/movedrone.git
    
3. Cut and past all 3 folders inside "src" folder, make sure to get the following path for the file folders (delete any remaining empty folders):
   catkin_ws/src/darknet_ros_msgs	
   catkin_ws/src/movedrone	
   catkin_ws/src/openpose_ros_msgs
   
4. get to catkin_ws/ and run "catkin build"
