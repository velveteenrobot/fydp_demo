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

#include "make_a_map.h"
#include "marker.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fydp_demo/ips_msg.h>

#include <vector>
#include <math.h>       // sqrt, pow 
#include <algorithm>    // max

#define PI 3.14159265
#define MAP_IDX(width, i, j) ((width) * (j) + (i))

using namespace std;

//Callback function for the Position topic (LIVE)

static bool poseReady = false;
static geometry_msgs::Pose pose;
bool updateMap = true;

int cnt = 0;

double mapRes = 0.10;
double mapWidth = 20;
double mapHeight = 20;
double robotMotion [3] = {0.5, 0, -0.5};
double yaw = 0.0;
nav_msgs::OccupancyGrid knownMapMsg;
std::vector< std::vector<double> > knownMap(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));

std::vector< std::vector<double> > LO(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));
std::vector< std::vector<double> > L(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));
ros::Publisher map_pub; 

int round_int( double r ) 
{
  if (r > 0.0)
  {
    r = r + 0.5;
  }
  else
  {
    r = r - 0.5;
  }

  return int (r);
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

std::vector< std::vector<double> > get_inverse_m_m(int M, int N, double theta, double r, double rmax)
{


  //Range finder inverse measurement model
  int x1 = max(1,min(M,round_int(pose.position.x/mapRes)));
  int y1 = max(1,min(N,round_int(pose.position.y/mapRes)));

  double endpt_x, endpt_y;

  

  if ( -2*PI <= theta && theta <= 2*PI)
  {  
    endpt_x = double (pose.position.x/mapRes) + r*cos(theta);
    endpt_y = double (pose.position.y/mapRes) + r*sin(theta);
  }
  else 
  {
    endpt_x = double (pose.position.x/mapRes);
    endpt_y = double (pose.position.y/mapRes);
  }

  

  int x2 = max(1,min(M,round_int(endpt_x)));
  int y2 = max(1,min(N,round_int(endpt_y)));

  //[list(:,1) list(:,2)] = bresenham(x1,y1,x2,y2);
  std::vector< std::vector<int> > bres = bresenham(x1, y1, x2, y2);

  //cout<<"Got bresenham"<<endl;

  std::vector< std::vector<double> > invMod(bres.size(), std::vector<double>(3,0.1));
  

  for(int i = 0; i < invMod.size(); i++) 
  {

    invMod[i][0] = double (bres[i][0]);
    invMod[i][1] = double (bres[i][1]);
  }

  if (r < rmax)
  {
    invMod[bres.size() - 1][2] = 0.9;
    //invMod[bres.size() - 2][2] = 0.6;
    //invMod[bres.size() - 3][2] = 0.6;
    //cout<<invMod[bres.size() - 1][0]<< " " << invMod[bres.size() - 1][1]<< " "<< invMod[bres.size() - 1][2] <<endl;
    //cout<<"r less than rmax"<<endl;
  }


    
  return invMod;
}

void pose_callback(const fydp_demo::ips_msg& msg)
{
  //This function is called when a new position message is received
  /*if(msg.tag_id != TAGID) {
    return;
  }*/

  if ((msg.X - pose.position.x) < 0.02 && (msg.Y - pose.position.y) < 0.02 && (msg.Yaw - yaw) < 0.02)
    updateMap = true;
  else
    updateMap = true;

  pose.position.x = msg.X - knownMapMsg.info.origin.position.x;
  pose.position.y = msg.Y - knownMapMsg.info.origin.position.y;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(0, 0, msg.Yaw),
      pose.orientation);

  yaw = msg.Yaw;
  poseReady = true;

  vector<Point> points;

  // plot /indoor_pos 
  /*Pose cur_pose;
  cur_pose.position.x = msg.X;
  cur_pose.position.y = msg.Y;


  points.push_back(cur_pose.position);
  Pose closePose = cur_pose;

  closePose.position.x = cur_pose.position.x + 0.01;
  closePose.position.y = cur_pose.position.y + 0.01;
  points.push_back(closePose.position);

  drawLine(CARROT, points);*/



}

//Callback function for the map
void scan_callback(const sensor_msgs::LaserScan& msg)
{
  //cout<<"Scan received"<<endl;

  int M = int (mapHeight/mapRes);
  int N = int (mapWidth/mapRes);

  double theta, ix, iy, il;
  std::vector< std::vector<double> > invMod;
  if (updateMap)
  {
    for (int i = 0; i < msg.ranges.size(); i++)
    {
      theta = yaw + (msg.angle_min  + msg.angle_increment*i);
      if (!isnan(msg.ranges[i]/mapRes))
      {
        //cout<<"theta: "<< theta<<endl;
        invMod = get_inverse_m_m(M, N, theta, msg.ranges[i]/mapRes, msg.range_max/mapRes);

        if(i==3)
          cout<<"invmod:"<<endl;

        

        //cout <<"Got invmod"<<endl;
        for (int j = 0; j < invMod.size(); j++)
        {
          ix = invMod[j][0];
          iy = invMod[j][1];
          il = invMod[j][2];
          if (i==3)
          {
            cout<<invMod[j][0]<< " " << invMod[j][1]<< " " << invMod[j][2]<<endl;
          }

          //Calculate updated log odds
          L[int(ix)][int(iy)] = L[int(ix)][int(iy)] + log(il/(1.0-il)) - LO[int(ix)][int(iy)];

          if (L[int(ix)][int(iy)] > 5)
            L[int(ix)][int(iy)] = 5;

          if (L[int(ix)][int(iy)] < -5)
            L[int(ix)][int(iy)] = -5;

          //if (cnt%100 == 0)
          //  cout<<"Log odds: "<<L[int(ix)][int(iy)]<<endl;

          //cnt++;
        }
        if (i==3)
          cout<<endl;
        //cout<<endl;
      }
    }
  }
  //Calculate probabilties
   //m = exp(L)./(1+exp(L));

  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      knownMap[i][j] = exp(L[i][j])/(1+exp(L[i][j]));
      if (knownMap[i][j] > 1.0)
        cout<<"Greater than 1"<<endl;
    }
  }

  //cout<<"Got probabilities"<<endl;
  
  //Put it in message
  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      if (knownMap[i][j] == 0.5)
        knownMapMsg.data[MAP_IDX(knownMapMsg.info.width, i, j)] = -1;
      /*else if (knownMap[i][j] >= 0.5)
        knownMapMsg.data[MAP_IDX(knownMapMsg.info.width, i, j)] = 100;*/
      else 
      {

        knownMapMsg.data[MAP_IDX(knownMapMsg.info.width, i, j)] = min(100.0,knownMap[i][j]*100);
        
      }
        
    }
  }

  map_pub.publish(knownMapMsg);
  cout<<"publishing map"<<endl;
  
}

void spinOnce(ros::Rate& loopRate) {
  loopRate.sleep(); //Maintain the loop rate
  ros::spinOnce();   //Check for new messages
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"mapping");
  ros::NodeHandle n;


  markerInit(n);

  //inialise map
  knownMapMsg.info.resolution = mapRes;
  knownMapMsg.info.height = int(mapHeight/mapRes);
  knownMapMsg.info.width = int(mapWidth/mapRes);
  knownMapMsg.info.origin.position.x = -int(mapWidth/2);
  knownMapMsg.info.origin.position.y = -int(mapHeight/2);
  /*knownMapMsg.info.origin.orientation.z = 0.707107;
  knownMapMsg.info.origin.orientation.w = 0.707107;*/
  knownMapMsg.data.resize(int (mapWidth/mapRes * mapHeight/mapRes));

  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

  //Set the loop rate
  ros::Rate loopRate(1/CYCLE_TIME);    //20Hz update rate

  cout<<"wait for the position"<<endl;
  while (!poseReady) {
    spinOnce(loopRate);
  }

  cout<<"Got position"<<endl;

  ros::Subscriber scan_pub = n.subscribe("/scan_throttle", 1, scan_callback);

  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      knownMap[i][j] = 0.5;
    }
  }

  //L0 = log(m./(1-m));

  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      LO[i][j] = log(knownMap[i][j]/(1.0-knownMap[i][j]));
    }
  }

  L = LO;
  
  // Bilal test commit
  while (ros::ok()) {
    //cout<<"OK"<<endl;
  
    spinOnce(loopRate);
  }
  // TODO: free memory
  return 0;
}
