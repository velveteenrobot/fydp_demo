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
const double PROP_TIME = 0.2;

float calc_norm(float x1, float x2){
  return sqrt(pow(x1,2)+pow(x2,2));
}



Pose propogateDynamics(Pose start, float speed, float turnRate) {
  Pose result = start;
  double roll, pitch, yaw;

  tf::Quaternion bt_q;
  quaternionMsgToTF(start.orientation, bt_q);
  tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);

  result.position.x += PROP_TIME * speed * cos(yaw);
  result.position.y += PROP_TIME * speed * sin(yaw);
  yaw += PROP_TIME * turnRate;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(roll, pitch, yaw),
      result.orientation);
  return result;
}

Pose get_random_pos(Pose start) {
  Pose result;
  bool running = true;
  while (running) {
    int r = rand() % 360;
    int d = rand() % 50 + 50;
    float dist = d / 100.0;
    float MAX_DIST = 1.0;
    float rad = float(r) / 360.0 * 2 * PI;
    result.position.x = start.position.x + cos(rad)*MAX_DIST * dist;
    result.position.y = start.position.y + sin(rad)*MAX_DIST * dist;

    if (!roomMap->isOccupied(result.position.x,result.position.y)){
      return result;
    }
  }

}


void pose_callback(const fydp_demo::ips_msg& msg)
{
  //This function is called when a new position message is received
  /*if(msg.tag_id != TAGID) {
    return;
  }*/

  if (roomMap != NULL)
  {
    
    pose.position.x = msg.X; //+ float(roomMap->getWidth())/2.0*float(roomMap->getRes());
    pose.position.y = msg.Y; //+ float(roomMap->getHeight())/2.0*float(roomMap->getRes());

    //shifted_point.position.y = (float(roomMap->getHeight()) - float(msg.poses[i].position.y))*float(roomMap->getRes());
    //shifted_point.position.x = float(msg.poses[i].position.x)*float(roomMap->getRes());

    quaternionTFToMsg(
        tf::createQuaternionFromRPY(0, 0, msg.Yaw),
        pose.orientation);    poseReady = true;
  }
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
    // converting from planner frame to gazebo/rviz frame
    shifted_point.position.x = -(-float(msg.poses[i].position.x) + float(roomMap->getWidth())/2.0) * float(roomMap->getRes());
    shifted_point.position.y = (-float(msg.poses[i].position.y) + float(roomMap->getHeight())/2.0) * float(roomMap->getRes());

    // truncates paths if they intersect with obstacle
    if (!roomMap->isOccupied(shifted_point.position.x,shifted_point.position.y))
    {
      Pose offsetWaypoint;
      offsetWaypoint = shifted_point;

      waypoints.push_back(shifted_point);
      points.push_back(offsetWaypoint);
    }
    else
    {
      if (waypoints.size() > 3)
      {
        waypoints.pop_back(); waypoints.pop_back(); waypoints.pop_back();
      } else 
      {
        waypoints.clear();
      }
      break;
    }

    
  }

  drawLine(points);
  points.clear();

}

void waypoints_done_callback(std_msgs::Bool msg)
{
  cout<<"Waypoints done callback"<<endl;
  waypointsDone = msg.data;
  cout<<"Finished waypoints done callback"<<endl;
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


  cout<<"wait for the map"<<endl;
  while (roomMap == NULL) {
    spinOnce(loopRate);
  }

  cout<<"wait for the position"<<endl;
  while (!poseReady) {
    cout<<"Balls"<<endl;
    loopRate.sleep(); //Maintain the loop rate
    cout<<"Balls2"<<endl;
    ros::spinOnce();
    cout<<"Balls3"<<endl;
  }
  
  cout<<"Map width: "<<roomMap->getWidth()<<endl;
  cout<<"Map heigh: "<<roomMap->getHeight()<<endl;

  /*cout<<"Wait for all waypoints"<<endl;
  while (waypointsDone==false){spinOnce
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
      /*
      Going to try to avoid prior problems being 'ahead' of the fist waypoint by finding the closest waypoint to
      current position, and selecting that at the target.
      */
      float rolling_error = 10000000.0;
      std::vector<Pose>::iterator start_iterator;
      for (std::vector<Pose>::iterator it = waypoints.begin(); it != waypoints.end(); it++){
        Twist error = getError(pose, *it);
        float mag_error = sqrt(pow(error.linear.x, 2.0) + pow(error.linear.y, 2.0));
        if (mag_error < rolling_error)
        {
          rolling_error = mag_error;
          start_iterator = it;
        }   
      }

      waypoints.erase(waypoints.begin(), start_iterator);

      currentWaypoint = waypoints[1];

      
      // calculating error and generating a velocity command
      Twist vel;
      Twist error = getError(pose, currentWaypoint);
      cout<<"Error x: "<<error.linear.x
          <<", y: "<<error.linear.y
          <<", yaw: "<<error.angular.z<<endl;
      vel.linear.x += 0.6 * error.linear.x;
      vel.angular.z += 1.0 * error.linear.y;
      // vel.angular.z -= 0.1 * error.angular.z;

      // checking if the command is valid
      Pose nextPose = propogateDynamics(pose, calc_norm(vel.linear.x, vel.linear.y), vel.angular.z);
      if (roomMap->isOccupied(nextPose.position.x, nextPose.position.y))
      {
        waypoints.clear();
        Pose random_goal = get_random_pos(pose);
        waypoints.push_back(random_goal);
        ros::spinOnce();
      }
      else
      {
        velocityPublisher.publish(vel); // Publish the command velocity
        ros::Duration(0.1).sleep();

        Pose offsetWaypoint;
        offsetWaypoint = currentWaypoint;
        double norm = calc_norm(error.linear.x, error.linear.y);
        drawPose(offsetWaypoint);
        ros::spinOnce();
        if(norm < 0.25)
        {
          waypoints.erase(waypoints.begin());
        }

      }

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
