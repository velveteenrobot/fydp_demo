#include "RRT.h"

#include "turtlebot_example.h"
#include "marker.h"

#include <tf/transform_datatypes.h>
#include <set>

/**
 * Propogates the dynamics for 1 step
 */
Pose propogateDynamics(Pose start, float speed, float turnRate) {
  Pose result = start;
  double roll, pitch, yaw;

  tf::Quaternion bt_q;
  quaternionMsgToTF(start.orientation, bt_q);
  tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);

  result.position.x += CYCLE_TIME * speed * cos(yaw);
  result.position.y += CYCLE_TIME * speed * sin(yaw);
  yaw += CYCLE_TIME * turnRate;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(roll, pitch, yaw),
      result.orientation);
  return result;
}


Milestone::Milestone(
    Milestone* prev,
    Pose& endPose,
    float speed,
    float turnRate,
    int numCycles) :
    mPrevMilestone(prev),
    mEndPosition(endPose),
    mSpeed(speed),
    mTurnRate(turnRate),
    mNumCycles(numCycles) {}

static float randFloat(float min, float max) {
  return (max - min) * ((float)rand()) / ((float) INT_MAX) + min;;
}

/**
 * Use the weighted selction method to pick a milestone to expand
 * @param milestones sorted vector of milestones
 * @return one of the milestones, based on it's position in the list
 */
static vector<Milestone*>::iterator selectRandomMilestone(
    vector<Milestone*>& milestones) {
  const float decay = 0.5;
  for (vector<Milestone*>::iterator i = milestones.begin();
       i != milestones.end();
       ++i) {
    if (randFloat(0, 1) < decay) {
      return i;
    }
  }
  // base exit case
  return --milestones.end();
}

static const int MAX_BRANCH_CREATION_ATTEMPS = 30;

Milestone* Milestone::makeRandomMilestone(Map& map) {
  for (int i = 0; i < MAX_BRANCH_CREATION_ATTEMPS; ++i) {
    // TODO: normal distribution?
    float speed = randFloat(-0.05, 0.3);
    // float speed = randFloat(0.1, 0.3);
    // float speed = randFloat(-0.2, 0.2);
    float turnRateRange = 1 - fabs(speed*2);
    float turnRate = randFloat(-turnRateRange, turnRateRange);
    int numCycles = (rand() % 70) + 30;

    Pose endPose = getEndPose();
    bool failed = false;
    for (int cycle = 0; cycle < numCycles; ++cycle) {
      endPose = propogateDynamics(endPose, speed, turnRate);
      if (map.robotAreaOccupied(endPose)) {
        // this motion path wasn't possible
        failed = true;
        break;
      }
    }
    if (!failed) {
      return new Milestone(this, endPose, speed, turnRate, numCycles);
    }
    // try another set of values
  }
  return NULL;
}

static void insertMilestoneSorted(
    Milestone* newMilestone,
    vector<Milestone*>& milestones,
    Pose goal) {
  // most node expansions should be getting closer to the goal, so linear insert
  vector<Milestone*>::iterator iter = milestones.begin();
  float newDist = newMilestone->distTo(goal);
  while (iter != milestones.end() && (*iter)->distTo(goal) < newDist) {
    iter++;
  }
  milestones.insert(iter, newMilestone);
}

/**
 * Generate and draw a path to the goal
 * @param start The exact pose the path starts at
 * @param end the desired pose to end at
 * @param map reference to the map of the area being traveled
 * @return a list of milestones going form start to near end
 */
list<Milestone*> doRRT(Pose start, Pose end, Map& map) {
  // total exploitation of floats not being bery descrete :)
  vector<Milestone*> milestones;
  Milestone* finalMilestone = NULL;
  while (finalMilestone == NULL) {
    cout<<"trying again"<<endl;
    for (int i = 0; i < milestones.size(); ++i) {
      delete milestones[i];
    }
    milestones.clear();
    milestones.push_back(new Milestone(NULL, start, 0, 0, 0));
    for (int i = 0; i < 1000; ++i) {
      // select a milestone
      vector<Milestone*>::iterator iter = selectRandomMilestone(milestones);
      Milestone* source = *iter;
      // select random motion inputs
      Milestone* newMilestone = source->makeRandomMilestone(map);
      if (newMilestone == NULL) {
        // that milestone didn't have any valid expanding paths try again
        continue;
      }

      if (newMilestone->distTo(end) < TARGET_ERROR) {
        finalMilestone = newMilestone;
        break;
      }
      // insert the new milestone
      insertMilestoneSorted(newMilestone, milestones, end);
      newMilestone->draw(RANDOM_TREE);
    }
  }

  list<Milestone*> result;
  set<Milestone*> dontDelete;
  for (Milestone *cur = finalMilestone;
       cur->getPrev() != NULL;
       cur = cur->getPrev()) {
    result.push_front(cur);
    dontDelete.insert(cur);
    cur->draw(SELECTED_TREE);
  }

  //  free the unused milestones
  for (int i = 0; i < milestones.size(); ++i) {
    if (!dontDelete.count(milestones[i])) {
      delete milestones[i];
    }
  }

  return result;
}

list<Milestone*> doRRTWaypoints(Pose start, vector<Pose>& waypoints, Map& map) {
  list<Milestone*> result;
  Pose* prevEndPose = &start;
  for (int i = 0; i < waypoints.size(); ++i) {
    list<Milestone*> curSection = doRRT(*prevEndPose, waypoints[i], map);
    result.insert(result.end(), curSection.begin(), curSection.end());
    prevEndPose = &(*result.rbegin())->getEndPose();
  }
  return result;
}

float Milestone::distTo(Pose position) {
  float xDist = position.position.x - mEndPosition.position.x;
  float yDist = position.position.y - mEndPosition.position.y;
  return sqrt(xDist*xDist + yDist*yDist);
}

void Milestone::draw(MarkerType color) {
  Pose nextPose = mPrevMilestone->getEndPose();
  bool failed = false;
  vector<Point> points;
  points.push_back(nextPose.position);
  for (int cycle = 0; cycle < mNumCycles; ++cycle) {
    nextPose = propogateDynamics(nextPose, mSpeed, mTurnRate);
    points.push_back(nextPose.position);
  }
  drawLine(color, points);
}
