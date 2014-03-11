fydp_demo
=======================
Instructions to run this demo code:
- Set up a ROS workspace as described here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Source your ROS workspace
- Clone this repo into the src folder of your ROS workspace
- Build it (i.e. navigate to the root of your ROS workspace and catkin_make)

- In order to run the Gazebo simulation, you also need to clone the following repo into your home folder: https://github.com/velveteenrobot/.gazebo
- If you already have a .gazebo folder in your home folder you can just copy the models/igvc_heightmap folder into your home .gazebo/models folder

- You'll need 6 terminals open because I haven't chained any of the roslaunch files... These instructions assume you're in the root of your ROS workspace at all times and have sourced your workspace in all 6 terminals. Here's what to do in each of your terminals (in this order):
- 
1) roslaunch fydp_demo turtlebot_igvc.launch

2) rosrun map_server map_server src/fydp_demo/config/igvc_square.yaml

3) rosrun fydp_demo sim_pose_publisher

4) rosrun rviz rviz -d src/fydp_demo/config/fydp.rviz

5) rosrun fydp_demo turtlebot_example

6) cd src/fydp_demo/src/fydp_demo; python planner_test.py

Here's a breakdown of what's happening:
- This command starts up the Gazebo simulator with an environment pre-loaded and the robot in the start position:
1) roslaunch fydp_demo turtlebot_igvc.launch
- This loads a copy of the map of the environment to the map_server
2) rosrun map_server map_server src/fydp_demo/config/igvc_square.yaml
- This publishes position data (in place of GPS data)
3) rosrun fydp_demo sim_pose_publisher
- This open Rviz with the right config file
4) rosrun rviz rviz -d src/fydp_demo/config/fydp.rviz
- This runs the 'local planner'. It waits for the global planner to run before starting.
5) rosrun fydp_demo turtlebot_example
- This starts the global planner. It publishes intermediate waypoints for the local planner. 
6) cd src/fydp_demo/src/fydp_demo; python planner_test.py

If you have any questions, open an issue or email sksellio@gmail.com
