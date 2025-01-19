


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
std::vector<Eigen::Vector3f> prevRefPos;
Eigen::Vector3f pos;
Eigen::Matrix3f ori;
Eigen::Vector3f update(long unsigned int mapId);
std::vector<Eigen::Vector3f> getRefPositions();
Eigen::Matrix3f solveOrthogonalProcrustes(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> Y);
Eigen::Vector3f update1(long unsigned int mapId) ;
//void setPrevRefPos();

};
}

#endif
