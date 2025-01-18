


#ifndef ANCHOR_H
#define ANCHOR_H
#include "Map.h"
#include "MapPoint.h"
// #include <vector>

namespace ORB_SLAM3{
class Map;

class Anchor{
public:
Map* map;
std::vector<MapPoint*> refs;
Eigen::Vector3f pos;
Eigen::Vector3f ori;
Eigen::Vector3f update(long unsigned int mapId,std::vector<Eigen::Vector3f> prevPos);
std::vector<Eigen::Vector3f> getRefPositions();

};
}

#endif
