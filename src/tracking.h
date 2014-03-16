#pragma once

#include "RRT.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <list>

using namespace std;
using namespace geometry_msgs;
using namespace ros;

class Tracking {
 public:
  Tracking(list<Milestone*>& milestones, Pose& startingPose, NodeHandle& node)
      : mMilestones(milestones),
        mCurMilestone(milestones.begin()),
        mCurCycle(0),
        mExpectedPose(startingPose) {
    mVelocityPublisher = node.advertise<geometry_msgs::Twist>(
      "/cmd_vel_mux/input/navi",
      1);
  }

  bool doCycle(Pose& curPose);

 private:
  list<Milestone*>& mMilestones;
  list<Milestone*>::iterator mCurMilestone;
  int mCurCycle;
  Pose mExpectedPose;

  Publisher mVelocityPublisher;

  Twist getError(Pose& curPose);
};