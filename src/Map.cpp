#include "Map.h"
#include "marker.h"
#include <visualization_msgs/Marker.h>

Map::Map(const nav_msgs::OccupancyGrid& msg) {
  mPose = msg.info.origin;
  mWidth = msg.info.width;
  mHeight = msg.info.height;
  mResolution = msg.info.resolution;

  vector<signed char, allocator<signed char> >::const_iterator iter =
      msg.data.begin();

  // copy over the map data
  for (int y = 0; y < mHeight; ++y) {
    mMap.push_back(new vector<bool>());
    for (int x = 0; x < mWidth; ++x) {
      bool occupied = (*iter) > 50;
      mMap[y]->push_back(occupied);
      iter++;
    }
  }
}

bool Map::isOccupied(float x, float y) {
  // convert xy to grid position
  int xGrid = (int)((x - mPose.position.x) / mResolution);
  int yGrid = (int)((y - mPose.position.y) / mResolution);
  if (xGrid < 0 ||
      xGrid >= mWidth ||
      yGrid < 0 ||
      yGrid >= mHeight) {
    std::cout<<"Invalid map position: "<<x<<", "<<y<<std::endl;
  }
  return isCellIndexOccupied(xGrid, yGrid);
}

bool Map::isCellIndexOccupied(unsigned int xGrid, unsigned int yGrid) {
  return (*mMap[yGrid])[xGrid];
}

#define ROBOT_RADIUS 0.21

bool Map::robotAreaOccupied(Pose robotPose) {
  int gridX = (robotPose.position.x - mPose.position.x) / mResolution;
  int gridY = (robotPose.position.y - mPose.position.y) / mResolution;
  int gridDistance = ROBOT_RADIUS / mResolution;

  for (int x = gridX - gridDistance; x <= gridX + gridDistance; x++) {
    for (int y = gridY - gridDistance; y <= gridY + gridDistance; y++) {
      if (x < 0 || x >= mWidth ||
          y < 0 || y >= mHeight ||
          isCellIndexOccupied(x, y)) {
        return true;
      }
    }
  }
  return false;
}
