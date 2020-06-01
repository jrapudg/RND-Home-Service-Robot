# Home Service Robot
This project aims to develop the code in ROS for a home service robot for:
* Localization
* Mapping
* Path planning 
* Navigation

## Install rospackages
When installing tutlrbot_gazebo you must install dependencies:
 
	rosdep -i install turtlebot_gazebo

After you should use command:
	
	catkin_make

To create a map you should install 
	sudo apt-get install libignition-math2-dev protobuf-compiler

Then, download pgm_map_creator to src folder:
	cd /home/workspace/catkin_ws/src/
	git clone https://github.com/udacity/pgm_map_creator.git

Copy the Gazebo world you created to the world folder

	cp catkin_ws/src/worlds/u.world 

Insert the map creator to map fil. Open map file using editor. Add the following tag towards the end of the file, just before </world> tag
	<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>

