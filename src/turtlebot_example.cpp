//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include "turtlebot_example.h"
#include "Map.h"
#include "marker.h"
#include "RRT.h"
#include "tracking.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <turtlebot_example/ips_msg.h>

#include <vector>

using namespace std;

//Callback function for the Position topic (LIVE)

static Map* roomMap = NULL;
static bool poseReady = false;
static Pose pose;
vector<Pose> waypoints;
bool waypointsDone = false;


void pose_callback(const turtlebot_example::ips_msg& msg)
{
  //This function is called when a new position message is received
  if(msg.tag_id != TAGID) {
    return;
  }

  pose.position.x = msg.X;
  pose.position.y = msg.Y;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(0, 0, msg.Yaw),
      pose.orientation);
  poseReady = true;
}


//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
  //This function is called when a new map is received
  //you probably want to save the map into a form which is easy to work with

  roomMap = new Map(msg);
  // TODO: calculate the path
}

void waypoints_callback(const geometry_msgs::Pose msg)
{
  Pose shifted_point;
  shifted_point.position.y = (roomMap->getHeight() - msg.position.y)*roomMap->getRes();
  shifted_point.position.x = msg.position.x*roomMap->getRes();

  waypoints.push_back(shifted_point);

  vector<Point> points;

  // plot /indoor_pos 
  points.push_back(shifted_point.position);
  Pose closePose = shifted_point;

  closePose.position.x = shifted_point.position.x + 0.01;
  closePose.position.z = shifted_point.position.z + 0.5;
  points.push_back(closePose.position);

  cout<<"drawing line: "<<shifted_point.position.x<<", "<<shifted_point.position.y<<endl;
  
  drawLine(CARROT, points);
}

void waypoints_done_callback(std_msgs::Bool msg)
{
  waypointsDone = msg.data;
}

void spinOnce(ros::Rate& loopRate) {
  loopRate.sleep(); //Maintain the loop rate
  ros::spinOnce();   //Check for new messages
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;

  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber waypoints_sub = n.subscribe("/waypoints", 1, waypoints_callback);
  ros::Subscriber done_waypoints_sub = n.subscribe("/waypoints_done", 1, waypoints_done_callback);

  //Setup topics to Publish from this node
  markerInit(n);

  //Set the loop rate
  ros::Rate loopRate(1/CYCLE_TIME);    //20Hz update rate

  cout<<"wait for the position"<<endl;
  while (!poseReady) {
    spinOnce(loopRate);
  }
  cout<<"wait for the map"<<endl;
  while (roomMap == NULL) {
    spinOnce(loopRate);
  }
  cout<<"Map width: "<<roomMap->getWidth()<<endl;
  cout<<"Map heigh: "<<roomMap->getHeight()<<endl;

  cout<<"Wait for all waypoints"<<endl;
  while (waypointsDone==false){
    spinOnce(loopRate);
  }
  // plan a path
  cout<<"Running RRT"<<endl;


  list<Milestone*> path = doRRTWaypoints(pose, waypoints, *roomMap);
  /*
  list<Milestone*> path;
  Pose unusedPose;
  path.push_back(new Milestone(NULL, unusedPose, 0.2, 0, 100));
  */
  // track the path
  Tracking tracking(path, pose, n);
  while (ros::ok()) {
    //Velocity control variable
    if (!tracking.doCycle(pose)) {
      break;
    }
    spinOnce(loopRate);
  }
  // TODO: free memory
  cout<<"Done"<<endl;
  return 0;
}
