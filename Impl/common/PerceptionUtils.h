#ifndef AUTODRIVE_PERCEPTIONUTILS_H
#define AUTODRIVE_PERCEPTIONUTILS_H

#include <vector>
#include <memory>
#include "types.h"
#include "SimOnePNCAPI.h"

// Forward declaration for SimOne_Data_Gps if it's a complex type
struct SimOne_Data_Gps;

std::vector<obstaclestruct> GetObstacleList(const std::unique_ptr<SimOne_Data_Gps> &gps);
std::vector<obstaclestruct> GetObstaclesByOrientation(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &stationaryObstacles);
bool GetValidObstacles(const std::unique_ptr<SimOne_Data_Gps> &gps, SSD::SimString &lane, std::vector<obstaclestruct> &validObstacleList);
int GetNearObstacleIndex(const SSD::SimPoint3D &mainVehiclePos, const double &mainVehiclespeed, const std::vector<obstaclestruct> &obstacleList, const SSD::SimString &currentLaneId);
bool DetectObstacleAheadInZone(const std::unique_ptr<SimOne_Data_Gps> &gps, obstaclestruct &obstacle, double &currentDistance);
bool DetectObstacleAhead(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles, obstaclestruct &obstacle, double &currentDistance);
bool DetectFirstObstacleOnPath(std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3DVector &shortPath, obstaclestruct &firstObstacle, size_t &obstaclePathIndex);
bool DetectMovingObstacleOnPath(std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3DVector &shortPath, obstaclestruct &movingObstacle, size_t &obstaclePathIndex);
bool DetectClosestSign(const std::unique_ptr<SimOne_Data_Gps> &gps, int &warningSignIndex, double &distance);
bool DetectSpeedLimitSignWithinRadius(const SimOne_Data_Gps &gps, int &warningSignIndex, double r, double &speedLimitMps);
bool DetectNearestSpeedLimitSign(const std::unique_ptr<SimOne_Data_Gps> &gps, bool isLongRouteCase, double &speedLimitMps);
bool DetectJunctionObstacle(const std::unique_ptr<SimOne_Data_Gps> &gps, SSD::SimPoint3D &stopLine, double &distance);
bool CrosswalkOccupied(const SSD::SimPoint3DVector &crosswalkBoundary, std::vector<obstaclestruct> &allObstacles);
double CalculateObstacleSpeed(const SimOne_Data_Obstacle_Entry obs);

#endif //AUTODRIVE_PERCEPTIONUTILS_H
