#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher randomTree;
static ros::Publisher selectedPath;
static ros::Publisher carrotPath;

void markerInit(ros::NodeHandle& n) {
  randomTree = n.advertise<visualization_msgs::Marker>(
      "random_tree",
      1,
      true);
  selectedPath = n.advertise<visualization_msgs::Marker>(
      "selected_path",
      1,
      true);
  carrotPath = n.advertise<visualization_msgs::Marker>(
      "carrot",
      1,
      true);
}

static int lastId = 1;

void drawLine(MarkerType type, vector<Point>& points) {
  double x = 0;
  double y = 0;
  double steps = 50;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = lastId;
  lastId++;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.05;

  for (int i = 0; i < points.size(); ++i) {
    lines.points.push_back(points[i]);
  }

  switch (type) {
    case RANDOM_TREE:
      lines.color.r = 1.0;
      lines.color.b = 1.0;
      lines.color.a = 1.0;
      randomTree.publish(lines);
      break;
    case SELECTED_TREE:
      lines.color.r = 1.0;
      lines.color.a = 1.0;
      selectedPath.publish(lines);
      break;
    case CARROT:
      lines.color.b = 1.0;
      lines.color.a = 1.0;
      carrotPath.publish(lines);
      break;
    default:
      break;
  }
}
