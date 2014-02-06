#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher randomTree;
static ros::Publisher selectedPath;
static ros::Publisher selectedPath2;
static ros::Publisher carrotPath;

static ros::Publisher positionPub;
static visualization_msgs::Marker positions;

void markerInit(ros::NodeHandle& n) {
  randomTree = n.advertise<visualization_msgs::Marker>(
      "random_tree",
      1,
      true);
  selectedPath = n.advertise<visualization_msgs::Marker>(
      "selected_path1",
      0,
      true);
  selectedPath2 = n.advertise<visualization_msgs::Marker>(
      "selected_path2",
      0,
      true);
  carrotPath = n.advertise<visualization_msgs::Marker>(
      "carrot",
      1,
      true);

  positionPub = n.advertise<visualization_msgs::Marker>(
      "selected_path",
      0,
      true);
  positions.header.frame_id = "/map";
  positions.id = 1;
  positions.type = visualization_msgs::Marker::POINTS;
  positions.action = visualization_msgs::Marker::ADD;
  positions.scale.x = 0.1;
  positions.color.g = 1.0;
  positions.color.a = 1.0;
}

static int lastId = 1;
static int prevId = 1;

void drawLine(std::vector<Pose> poses) {

  positions.id++;

  for (int i = 1; i < lastId + 1; i++)
  {
    visualization_msgs::Marker lines;
    lines.header.frame_id = "/map";
    lines.id = i;
    lines.type = visualization_msgs::Marker::SPHERE;
    lines.action = visualization_msgs::Marker::DELETE;
    lines.ns = "curves";
    lines.scale.x = 0.20;
    lines.scale.y = 0.20;
    lines.scale.z = 0.20;

    lines.color.r = 1.0;
    lines.color.a = 1.0;
    selectedPath.publish(lines);
  }

  lastId = 1;

  for (int i = 0; i < poses.size(); i++)
  {
    Point p = Point();
    p.x = poses[i].position.x;
    p.y = poses[i].position.y;
    positions.points.push_back(p);

    visualization_msgs::Marker lines;
    lines.header.frame_id = "/map";
    lines.id = lastId;
    lastId++;
    lines.type = visualization_msgs::Marker::SPHERE;
    lines.action = visualization_msgs::Marker::ADD;
    lines.ns = "curves";
    lines.scale.x = 0.20;
    lines.scale.y = 0.20;
    lines.scale.z = 0.20;
    lines.pose.position.x = poses[i].position.x;
    lines.pose.position.y = poses[i].position.y;

    lines.color.r = 1.0;
    lines.color.a = 1.0;
    selectedPath.publish(lines);
    //ros::Duration(1.0).sleep();
  }

  positionPub.publish(positions);
  positions.points.clear();

  /*double x = 0;
  double y = 0;
  double steps = 50;

  
  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = lastId;
  lastId++;
  lines.type = visualization_msgs::Marker::SPHERE;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.20;
  lines.scale.y = 0.20;
  lines.scale.z = 0.20;
  lines.pose.position.x = pose.position.x;
  lines.pose.position.y = pose.position.y;

  lines.color.r = 1.0;
  lines.color.a = 1.0;
  selectedPath.publish(lines);
  */
  

}

void drawPose(Pose pose) {

    visualization_msgs::Marker lines;
    lines.header.frame_id = "/map";
    lines.id = prevId;
    prevId++;
    lines.type = visualization_msgs::Marker::SPHERE;
    lines.action = visualization_msgs::Marker::ADD;
    lines.ns = "curves";
    lines.scale.x = 0.20;
    lines.scale.y = 0.20;
    lines.scale.z = 0.20;
    lines.pose.position.x = pose.position.x;
    lines.pose.position.y = pose.position.y;

    lines.color.b = 1.0;
    lines.color.a = 1.0;
    selectedPath2.publish(lines);
    //ros::Duration(1.0).sleep();

  /*double x = 0;
  double y = 0;
  double steps = 50;

  
  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = lastId;
  lastId++;
  lines.type = visualization_msgs::Marker::SPHERE;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.20;
  lines.scale.y = 0.20;
  lines.scale.z = 0.20;
  lines.pose.position.x = pose.position.x;
  lines.pose.position.y = pose.position.y;

  lines.color.r = 1.0;
  lines.color.a = 1.0;
  selectedPath.publish(lines);
  */
  

}
