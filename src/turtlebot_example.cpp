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

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>

#include "turtlebot_example.h"
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

//Callback function for the Position topic (LIVE)

static Map* roomMap = NULL;
static bool poseReady = false;
static Pose pose;
vector<Pose> waypoints;
bool waypointsDone = false;


void pose_callback(const fydp_demo::ips_msg& msg)
{
  //This function is called when a new position message is received
  /*if(msg.tag_id != TAGID) {
    return;
  }*/

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

void waypoints_callback(const geometry_msgs::PoseArray msg)
{
  waypoints.clear();
  Pose shifted_point;
  quaternionTFToMsg(
      tf::createQuaternionFromRPY(0, 0, 0),
      shifted_point.orientation);

  cout << msg.poses.size() << endl;
  cout << msg.header.stamp << endl;
  
  std::vector<Pose> points;
  for (int i = 0; i < msg.poses.size(); i++)
  {
    
    //cout<<"balls: "<<msg.poses[i].position.x<<", "<<msg.poses[i].position.y<<endl;
    shifted_point.position.y = (float(roomMap->getHeight()) - float(msg.poses[i].position.y))*float(roomMap->getRes());
    shifted_point.position.x = float(msg.poses[i].position.x)*float(roomMap->getRes());
    //cout<<"drawing marker: "<<shifted_point.position.x<<", "<<shifted_point.position.y<<endl;
    waypoints.push_back(shifted_point);
    points.push_back(shifted_point);
  }

  drawLine(points);
  points.clear();
  cout << "Balls: " << waypoints.size() <<endl;


  

  //vector<Point> points;
  


  //drawLine(shifted_point);
  //points.push_back(shifted_point.position);
  //Pose closePose = shifted_point;

  //closePose.position.x = shifted_point.position.x + 0.01;
  //closePose.position.z = shifted_point.position.z + 0.5;
  //points.push_back(closePose.position);

  


  
  //drawLine(CARROT, points);
}

void waypoints_done_callback(std_msgs::Bool msg)
{
  waypointsDone = msg.data;
}

std::vector< std::vector<int> > bresenham(int x0,int y0,int x1,int y1)
{
  
  bool steep = abs(y1 - y0) > abs(x1 - x0);

  if (steep)
  {
    x0 = (x0+y0) - (y0=x0);
    x1 = (x1+y1) - (y1=x1);
  }

  int dx=abs(x1-x0);
  int dy=abs(y1-y0);
  int error = dx / 2;
  int ystep;
  int y = y0;

  int inc;

  std::vector< std::vector<int> > q(dx, std::vector<int>(2,0));

  
  if (x0 < x1)
  {
    inc = 1; 
  }
  else 
  {
    inc = -1;
  }
  if (y0 < y1)
  {
    ystep = 1;
  }
  else 
  {
    ystep = -1;
  }

  int i= 0;

  for (int x = x0; x < x1; x+=inc)
  {
    if (steep)
    {
      q[i][0] = y;
      q[i][1] = x;
    }
    else 
    {
      q[i][0] = x;
      q[i][1] = y;
    }

    error = error - dy;
    if (error < 0)
     {
      y = y + ystep;
      error = error + dx;
     } 
     i++;        
  }

  for (int x = x0; x > x1; x+=inc)
  {
    if (steep)
    {
      q[i][0] = y;
      q[i][1] = x;
    }
    else 
    {
      q[i][0] = x;
      q[i][1] = y;
    }

    error = error - dy;
    if (error < 0)
     {
      y = y + ystep;
      error = error + dx;
     } 
     i++; 
  }
  return q;
}

Twist getError(Pose curPose, Pose nextPose) {
  float xError = nextPose.position.x - curPose.position.x;
  float yError = nextPose.position.y - curPose.position.y;

  tf::Quaternion q;
  double unusedRoll, unusedPitch;
  double curYaw, expectedYaw;

  quaternionMsgToTF(curPose.orientation, q);
  tf::Matrix3x3(q).getRPY(unusedRoll, unusedPitch, curYaw);
  quaternionMsgToTF(nextPose.orientation, q);
  tf::Matrix3x3(q).getRPY(unusedRoll, unusedPitch, expectedYaw);

  Twist error;

  error.angular.z = fmod(expectedYaw - curYaw, 2*PI);
  if (error.angular.z > PI) {
    error.angular.z -= PI;
  }

  // put x/y error in terms of the robot's orientation
  error.linear.x = xError * cos(curYaw) + yError * sin(curYaw);
  error.linear.y = xError * (-sin(curYaw)) + yError * cos(curYaw);

  return error;
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
  markerInit(n);

  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber waypoints_sub = n.subscribe("/waypoints", 1, waypoints_callback);
  ros::Subscriber done_waypoints_sub = n.subscribe("/waypoints_done", 1, waypoints_done_callback);
  ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>(
      "/cmd_vel_mux/input/navi",1);

  //Setup topics to Publish from this node
  
  /*if (waypoints.empty())
  {
    waypoints.push_back(pose);
  }
  */
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

  /*cout<<"Wait for all waypoints"<<endl;
  while (waypointsDone==false){
    spinOnce(loopRate);
  }*/
  // plan a path
  cout<<"Running local planner"<<endl;

  


  //typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);
  //move_base_msgs::MoveBaseGoal goal;

  Pose currentWaypoint;
  //Pose nextWaypoint;
  //Pose closestPos;
  //double closestDist = 1000;

  //wait for the action server to come up
  /*while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }*/

  while (true)
  {
    //cout<< "Num waypoints: "<< waypoints.size()<<endl;

    if (!waypoints.empty())
    {
      currentWaypoint = waypoints[1];
      //nextWaypoint = waypoints[1];
      //closestPos = nextWaypoint;
      //closestDist = sqrt(pow(pose.position.x - nextWaypoint.position.x, 2) + pow(pose.position.y - nextWaypoint.position.y, 2));

      //int x1 = currentWaypoint.position.x;
      //int x2 = nextWaypoint.position.x;
      //int y1 = currentWaypoint.position.y;
      //int y2 = nextWaypoint.position.y;

      //std::vector< std::vector<int> > bres = bresenham(x1, y1, x2, y2);

      //iterate through check which point is closest to current position
      /*for (int i=0; i < bres.size(); i++)
      {
        double dist = sqrt(pow(pose.position.x - bres[i][0], 2) + pow(pose.position.y - bres[i][1], 2));
        if (dist < closestDist)
        {
          closestPos.position.x = bres[i][0];
          closestPos.position.y = bres[i][1];
          closestDist = dist;
        }
          
      }
      */
      Twist vel;
      Twist error = getError(pose, currentWaypoint);
      cout<<"Error x: "<<error.linear.x
          <<", y: "<<error.linear.y
          <<", yaw: "<<error.angular.z<<endl;
      vel.linear.x += 0.8 * error.linear.x;
      vel.angular.z += 1.0 * error.linear.y;
      // vel.angular.z -= 0.1 * error.angular.z;

      velocityPublisher.publish(vel); // Publish the command velocity
      ros::Duration(0.5).sleep();

      drawPose(currentWaypoint);
      ros::spinOnce();
      waypoints.erase(waypoints.begin());
      

    }
    else
      spinOnce(loopRate);
      
  }


  
  
  /*while (ros::ok()) {
 
    spinOnce(loopRate);
  }*/
  // TODO: free memory
  cout<<"Done"<<endl;
  return 0;
}
