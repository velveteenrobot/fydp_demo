#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>

using namespace std;
using namespace geometry_msgs;

typedef enum {
  RANDOM_TREE,
  SELECTED_TREE,
  CARROT,
} MarkerType;

void markerInit(ros::NodeHandle& n);
void drawPoint(float x, float y);
void drawLine(Pose pose);
