# ros2_robot
Set-up or a UGV with Solidworks and ROS2

# Left tasks
	1. Fix 'mujoco_connect.py' (ugv_pkg) - it is the main node, in charge
	   of opening the simulator and publishing the data in topics
	   (data must be: camera images, LIDAR values and robot main position)
	2. Fix 'camera.py' (ugv_pkg) - node in charge of showing the image
	   (OpenCV)
	3. Fix 'lidar.py' (ugv_pkg) - node in charge of receiving the LIDAR
	   values and together with the main position of the robot, changing
           the way of navigation (sending joint values through another topic)
	4. Change RViz configuration so as to include the robot updated data
	   (location and laser visualization)
