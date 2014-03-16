#include "tracking.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "marker.h"

#define PI 3.14159265

bool Tracking::doCycle(Pose& curPose) {
  float speed = (*mCurMilestone)->getSpeed();
  float turnRate = (*mCurMilestone)->getTurnRate();

  mExpectedPose = propogateDynamics(mExpectedPose, speed, turnRate);

  Twist vel;
  
  vector<Point> points;

  // plot /indoor_pos 
  points.push_back(curPose.position);
  Pose closePose = curPose;

  closePose.position.x = curPose.position.x + 0.01;
  closePose.position.y = curPose.position.y + 0.01;
  points.push_back(closePose.position);
  
  drawLine(CARROT, points);

  // feed forward control
  // vel.linear.x = speed;
  // vel.angular.z = turnRate;

  // feed back control
  Twist error = getError(curPose);
  cout<<"Error x: "<<error.linear.x
      <<", y: "<<error.linear.y
      <<", yaw: "<<error.angular.z<<endl;
  vel.linear.x += 1.0 * error.linear.x;
  vel.angular.z += 4.0 * error.linear.y;
  // vel.angular.z -= 0.1 * error.angular.z;

  mVelocityPublisher.publish(vel); // Publish the command velocity

  // cout<<"finished cycle "<<mCurCycle<<endl;
  mCurCycle++;
  if (mCurCycle > (*mCurMilestone)->getNumCycles()) {
    mCurCycle = 0;
    mExpectedPose = (*mCurMilestone)->getEndPose();
    mCurMilestone++;
    if (mCurMilestone == mMilestones.end()) {
      return false;
    }
  }
  return true;
}

/**
 * Find the current positonal error of the robot
 * @param curPose The current position of the robot
 * @returns the error of the position, in the robot's current frame of reference
 */
Twist Tracking::getError(Pose& curPose) {
  float xError = mExpectedPose.position.x - curPose.position.x;
  float yError = mExpectedPose.position.y - curPose.position.y;

  tf::Quaternion q;
  double unusedRoll, unusedPitch;
  double curYaw, expectedYaw;

  quaternionMsgToTF(curPose.orientation, q);
  tf::Matrix3x3(q).getRPY(unusedRoll, unusedPitch, curYaw);
  quaternionMsgToTF(mExpectedPose.orientation, q);
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