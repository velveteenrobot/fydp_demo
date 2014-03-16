#include "local_planner_utils.h"

float calc_norm(float x1, float x2){
  return sqrt(pow(x1,2)+pow(x2,2));
}

/*
	Calculates the 2-norm between two poses
*/
float calc_norm(Pose p1, Pose p2)
{
  return calc_norm(p1.position.x - p2.position.y, p1.position.y - p2.position.y);
}

/*
	Returns the error in position and orientation between two poses
*/
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

/*
	Euler integrates out one time step to predict where the robot will be in prop_time in the 
	future.
*/
Pose propogateDynamics(Pose start, float speed, float turnRate, float prop_time) {
  Pose result = start;
  double roll, pitch, yaw;

  tf::Quaternion bt_q;
  quaternionMsgToTF(start.orientation, bt_q);
  tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);

  result.position.x += prop_time * speed * cos(yaw);
  result.position.y += prop_time * speed * sin(yaw);
  yaw += prop_time * turnRate;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(roll, pitch, yaw),
      result.orientation);
  return result;
}

/*
	Generates a random point around the start pose, tries to makes sure taht it is a valid point
*/
Pose get_random_pos(Pose start, Map* roomMap) {
  Pose result = start;
  bool running = true;
  while (running) {
    int r = rand() % 360;
    int d = rand() % 50 + 50;
    float dist = d / 100.0;
    float MAX_DIST = 1.0;
    float rad = float(r) / 360.0 * 2 * PI;
    result.position.x += + cos(rad)*MAX_DIST * dist;
    result.position.y += + sin(rad)*MAX_DIST * dist;

    if (!roomMap->isOccupied(result.position.x,result.position.y)){
      return result;
    }
  }
}

int get_closest_waypoint(Pose current_pose, vector<Pose> waypoint_list)
{
  float max_dist = 1000000.0;
  int idx = 0;
  for (int i=0;i<waypoint_list.size();i++)
  {
    float d = calc_norm(current_pose, waypoint_list[i]);
    if (d < max_dist)
    {
      max_dist = d;
      idx = i;
    }
  }
}
bool has_los(int x1,int y1,int x2,int y2, Map* roomMap)
{
  std::vector< std::vector<int> > bres = bresenham(x1, y1, x2, y2);
  for (int i = 0; i < bres.size(); i++)
  {
    if (roomMap->isOccupied(bres[i][0], bres[i][1]))
    {
      return false;
    }
  }
  return true;
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
