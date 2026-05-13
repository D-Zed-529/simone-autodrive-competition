#ifndef AUTODRIVE_PATHUTILS_H
#define AUTODRIVE_PATHUTILS_H

#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "types.h"

SSD::SimString GetNearMostLane(const SSD::SimPoint3D &pos);
double GetS(const SSD::SimPoint3D &pos, SSD::SimString laneId);
double GetT(const SSD::SimPoint3D &pos, SSD::SimString laneId);
SSD::SimVector<long> GetNavigateRoadIdList(const SSD::SimPoint3D &startPt, const SSD::SimPoint3D &endPt);
bool GetValidSuccessor(const HDMapStandalone::MLaneId &laneId, const long &currentRoadId, const long &nextRoadId, HDMapStandalone::MLaneId &successor);
void AddSamples(const HDMapStandalone::MLaneId &laneId, SSD::SimPoint3DVector &path);
SSD::SimPoint3DVector GetReferencePath(const SSD::SimPoint3D &startPt, const SSD::SimVector<long> &naviRoadIdList);
void GetLaneSampleFromS(const SSD::SimString &laneId, const double &s, SSD::SimPoint3DVector &targetPath);
SSD::SimPoint3DVector ChangeLanePathWithPoint(const SSD::SimPoint3D &from, const SSD::SimPoint3D &to);
void GetChangeLanePath(const double &vel, const obstaclestruct &obstacle, const SSD::SimPoint3D &mainVehiclePos, const SSD::SimString &changeToLaneName, SSD::SimPoint3DVector &targetPath);
SSD::SimPoint3DVector GetLaneSample(const SSD::SimString &laneId);
SSD::SimPoint3DVector GetDetectionZoneRealtime(const std::unique_ptr<SimOne_Data_Gps> &gps, const double &s_front, const double &s_back, const double &t_left, const double &t_right);
SSD::SimPoint3DVector GetDetectionZoneFixed(const SSD::SimPoint3D &pos, const double &s_front, const double &s_back, const double &t_left, const double &t_right);
double GetSDistance(const std::unique_ptr<SimOne_Data_Gps> &gps, const obstaclestruct &obstacle, const SSD::SimVector<long> &roadIdList);
SSD::SimPoint3D GetTerminalPoint();
HDMapStandalone::MSignal GetTargetLight(const SSD::SimString &laneId, const SSD::SimVector<long> &roadIdList);
SSD::SimPoint3D GetTargetStopLine(const HDMapStandalone::MSignal &light, const SSD::SimString &laneId);
SSD::SimPoint3DVector GenerateForwardPoints(size_t &index, SSD::SimPoint3DVector &targetPath, SSD::SimPoint3D &mainVehiclePos);
size_t IndexNumberOfLongPath(SSD::SimPoint3DVector &longPath, SSD::SimPoint3D &mainVehiclePos);
SSD::SimPoint3DVector ChangeLanePathWithLane(const SSD::SimPoint3D &mainVehiclePos, double speed, const SSD::SimString &changeToLane, SSD::SimPoint3D &changeToPoint);
SSD::SimPoint3DVector ChangeLanePathWithObstacle(const SSD::SimPoint3D &mainVehiclePos, const SSD::SimString &changeToLane, SSD::SimPoint3D &obstaclePos, SSD::SimPoint3D &turnTargetPoint);
void BuildLineWithoutTargetPath(SSD::SimPoint3D &carPos, SSD::SimPoint3DVector &temporaryLine);
void PrintTargetPath(SSD::SimPoint3DVector &targetPath);

#endif //AUTODRIVE_PATHUTILS_H
