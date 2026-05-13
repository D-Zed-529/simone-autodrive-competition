#ifndef AUTODRIVE_DECISIONUTILS_H
#define AUTODRIVE_DECISIONUTILS_H

#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "types.h"

bool GetDetectingZone(const SSD::SimString &currentLaneId, SSD::SimString &changeToLaneName, const SSD::SimPoint3D &mainVehiclePos, const obstaclestruct &obstacle, SSD::SimPoint3DVector &detectingZone, SSD::SimString &turn);
bool IsChangeLaneOccupied(const std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3DVector &detectingZone);
bool PassedObstacle(const SSD::SimPoint3D &vehiclePos, const obstaclestruct &obstacle, const SSD::SimString &laneId);
bool IsGreenLight(const long &lightId, const SSD::SimString &laneId, const HDMapStandalone::MSignal &light, double speed, double distance);
bool Passed(const SSD::SimPoint3D &vehiclePos, const SSD::SimPoint3D &light, const SSD::SimString &laneId);
SSD::SimString GetTargetSuccessorLane(const SSD::SimStringVector &successorLaneNameList, const SSD::SimVector<HDMapStandalone::MSignalValidity> &validities);
bool DetectStopLine(SSD::SimPoint3D &carPos, SSD::SimVector<long> &roadIdList, SSD::SimPoint3D &stopLine, HDMapStandalone::MSignal &currentLight, HDMapStandalone::MObject &crosswalk, double &currentDistance);
bool DetectNoLightStopLine(const std::unique_ptr<SimOne_Data_Gps> &gps, bool hasTrafficLight, SSD::SimPoint3D &stopLine, SSD::SimPoint3DVector &crosswalk, double &currentDistance);
bool DetectJunction(const std::unique_ptr<SimOne_Data_Gps> &gps);
bool IsJunctionCrowded(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles, const SSD::SimVector<long> &roadIdList);
bool DetectIsTurnable(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles, const obstaclestruct &obstacle, SSD::SimString &turn, SSD::SimString &state);
bool IsChangeable(obstaclestruct &obstacle, std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3D &turnTargetPoint);
bool IsObstacleInJunction(obstaclestruct &obstacle);
bool DetectRightRoad(SSD::SimString &lane, SSD::SimPoint3D &mainVehiclePos);
bool DetectLeftRoad(SSD::SimString &lane, SSD::SimPoint3D &mainVehiclePos);
bool DetectValidCrossing(SSD::SimString &predecessorLane, SSD::SimPoint3D &carPos, std::vector<obstaclestruct> &obstacleList, SSD::SimString &validLane, SSD::SimPoint3D &turnTargetPoint);
double GetLateralDistance(SSD::SimPoint3D &carPos, double &carOriZ, obstaclestruct &obstacle);
bool DetectCross(SSD::SimString &currentLaneId, SSD::SimPoint3D &mainVehiclePos, SimOne_Data_Gps *gpsPtr, HDMapStandalone::MLaneLink lanelink);

#endif //AUTODRIVE_DECISIONUTILS_H
