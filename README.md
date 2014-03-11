fydp_demo
=======================
Instructions to run this demo code:
- Set up a ROS workspace as described here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Source your ROS workspace
- Clone this repo into the src folder of your ROS workspace
- Build it (i.e. navigate to the root of your ROS workspace and catkin_make)

- In order to run the Gazebo simulation, you also need to clone the following repo into your home folder: https://github.com/velveteenrobot/.gazebo
- If you already have a .gazebo folder in your home folder you can just copy the models/igvc_heightmap folder into your home .gazebo/models folder

- You'll need 7 terminals open because I haven't chained any of the roslaunch files... These instructions assume you're in the root of your ROS workspace at all times and have sourced your workspace in all 6 terminals. Here's what to do in each of your terminals (in this order):

1) roslaunch turtlebot_gazebo turtlebot_empty_world.launch

-You will want to add some random blocks and stuff in the empty Gazebo environment for the robot to navigate around.

2) rosrun fydp_demo sim_pose_publisher

3) rosrun topic_tools throttle messages scan 5

4) rosrun fydp_demo make_a_map

5) rosrun rviz rviz -d src/fydp_demo/config/fydp.rviz

6) rosrun fydp_demo turtlebot_example

7) cd src/fydp_demo/src/fydp_demo; python planner_test.py

If you have any questions, open an issue or email sksellio@gmail.com
