#pragma once

#include "Map.h"
#include <geometry_msgs/Pose.h>

#include <list>
#include "marker.h"

using namespace std;
using namespace geometry_msgs;

class Milestone {
 public:
  Milestone(
      Milestone* prev,
      Pose& endPose,
      float speed,
      float turnRate,
      int numCycles);

  Pose& getEndPose() { return mEndPosition; }
  Milestone* getPrev() { return mPrevMilestone; }
  float getSpeed() { return mSpeed; }
  float getTurnRate() { return mTurnRate; }
  int getNumCycles() { return mNumCycles; }
  Milestone* makeRandomMilestone(Map& map);

  void draw(MarkerType color);
  float distTo(Pose position);

 private:
  Milestone* mPrevMilestone;
  Pose mEndPosition;
  float mSpeed, mTurnRate;
  int mNumCycles;
};

list<Milestone*> doRRTWaypoints(Pose start, vector<Pose>& waypoints, Map& map);
list<Milestone*> doRRT(Pose start, Pose end, Map& map);
Pose propogateDynamics(Pose start, float speed, float turnRate);
