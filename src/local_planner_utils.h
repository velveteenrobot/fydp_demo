#pragma once

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>

#include "Map.h"
#include "marker.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <fydp_demo/ips_msg.h>
#include <math.h>

#include <vector>
#define PI 3.14159265

using namespace std;

float calc_norm(float x1, float x2);
float calc_norm(Pose p1, Pose p2);
Twist getError(Pose curPose, Pose nextPose);
Pose propogateDynamics(Pose start, float speed, float turnRate, float prop_time);
Pose get_random_pos(Pose start, Map* roomMap);
std::vector< std::vector<int> > bresenham(int x0,int y0,int x1,int y1);
int get_closest_waypoint(Pose current_pose, vector<Pose> waypoint_list);
bool has_los(int x1,int y1,int x2,int y2, Map* roomMap);