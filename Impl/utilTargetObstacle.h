#pragma once

#include <iostream>
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MLaneId.h"
#include "util/GetSignType.h"
#include "SimOneSensorAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "utilTargetLane.h"
#include "SSD/SimPoint2D.h"
#include "util/UtilMath.h"

class utilTargetObstacle {
public:
    struct ObstacleStruct {
        SSD::SimPoint3D pt;
        SSD::SimString ownerLaneId;
    };

    static std::vector<utilTargetObstacle::ObstacleStruct> GetObstacleList() {
        std::vector<utilTargetObstacle::ObstacleStruct> allObstacles;
        std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
        const char *kMainVehicleId = "0";
        if (SimOneAPI::GetGroundTruth(kMainVehicleId, obstaclesPtr.get())) {
            for (int i = 0; i < obstaclesPtr->obstacleSize; i++) {
                float posX = obstaclesPtr->obstacle[i].posX;
                float posY = obstaclesPtr->obstacle[i].posY;
                float posZ = obstaclesPtr->obstacle[i].posZ;
                SSD::SimPoint3D pos(posX, posY, posZ);
                SSD::SimString laneId = utilTargetLane::GetNearMostLane(pos);
                allObstacles.push_back(utilTargetObstacle::ObstacleStruct{SSD::SimPoint3D(posX, posY, posZ), laneId});
            }
        }
        return std::move(allObstacles);
    }

    static bool DetectObstacle(const SSD::SimPoint3D &vehiclePos, const std::vector<ObstacleStruct> &allObstacles,
                               const HDMapStandalone::MLaneType &laneType,
                               const SSD::SimString &laneId, const SSD::SimString &leftNeighborLaneName,
                               SSD::SimPoint3DVector &targetPath, int &warningObstacleIndex) {
        warningObstacleIndex = -1;
        if (int(allObstacles.size()) == 0 || laneType != HDMapStandalone::MLaneType::driving) {
            return false;
        }
        const double kDistanceSafety = 30.0;
        //get closest obstacle
        double minDist = std::numeric_limits<double>::max();
        int obstacleClosestIndex = -1;
        SSD::SimPoint2D vehiclePos2D(vehiclePos.x, vehiclePos.y);
        for (unsigned int i = 0; i < allObstacles.size(); i++) {
            auto &obstacle = allObstacles[i];
            double distanceSign = UtilMath::distance(vehiclePos2D, SSD::SimPoint2D(obstacle.pt.x, obstacle.pt.y));
            if (distanceSign < minDist) {
                minDist = distanceSign;
                obstacleClosestIndex = i;
            }
        }

        auto &obstacleClosest = allObstacles[obstacleClosestIndex];
        if (obstacleClosest.ownerLaneId != laneId) {
            return false;
        }

        double distanceSign = UtilMath::distance(vehiclePos2D,
                                                 SSD::SimPoint2D(obstacleClosest.pt.x, obstacleClosest.pt.y));
        if (distanceSign < kDistanceSafety) {
            warningObstacleIndex = obstacleClosestIndex;
            targetPath.clear();  //Important
            SSD::SimPoint3D changeToPoint;
            SSD::SimPoint3D dir;
            if (SimOneAPI::GetLaneMiddlePoint(obstacleClosest.pt, leftNeighborLaneName, changeToPoint, dir)) {
                std::cout << "changeToPoint x:" << changeToPoint.x << "changeToPoint y:" << changeToPoint.y << "z:"
                          << changeToPoint.z << std::endl;
            }

            targetPath.push_back(vehiclePos);
            SSD::SimPoint3DVector changeLanePath = utilTargetLane::CreateChangeLanePath(vehiclePos, changeToPoint);
            for (auto &knot: changeLanePath) {
                targetPath.push_back(knot);
            }
            targetPath.push_back(changeToPoint);

            double s, t;
            SSD::SimPoint3D changeBackPoint;
            SSD::SimPoint3D dir2;

            bool found = SimOneAPI::GetLaneST(laneId, obstacleClosest.pt, s, t);
            assert(found);
            double s_next = s + kDistanceSafety;
            SimOneAPI::GetInertialFromLaneST(laneId, s_next, t, changeBackPoint, dir2);

            SSD::SimPoint3DVector changeLaneBackPath = utilTargetLane::CreateChangeLanePath(changeToPoint,
                                                                                            changeBackPoint);

            for (auto &knot: changeLaneBackPath) {
                targetPath.push_back(knot);
            }
            targetPath.push_back(changeBackPoint);

            std::cout << "current laneId:" << laneId.GetString() << std::endl;
            HDMapStandalone::MLaneId testLaneId(laneId);
            utilTargetLane::GetLaneSampleFromS(laneId, s_next, targetPath);
            return true;
        }
        return false;
    }

    static bool
    PassedObstacle(const SSD::SimPoint3D &vehiclePos, const ObstacleStruct &obstacle, const SSD::SimString &laneId) {
        double s, t;
        bool found = SimOneAPI::GetLaneST(laneId, obstacle.pt, s, t);
        assert(found);
        double s_vehicle, t_vehicle;
        found = SimOneAPI::GetLaneST(laneId, vehiclePos, s_vehicle, t_vehicle);
        assert(found);
        return s_vehicle > s;
    }

    static bool DetectSpeedLimitSign(SimOne_Data_Gps &gps, int &warningSignIndex) {
        SSD::SimVector<HDMapStandalone::MSignal> trafficSignList;
        SimOneAPI::GetTrafficSignList(trafficSignList);
        float mainVehicleVel = sqrtf(pow(gps.velX, 2) + pow(gps.velY, 2)) * 3.6f;

        const double kAlertDistance = 40.0;
        //get closest sign
        double minDist = std::numeric_limits<double>::max();
        int closestSignIndex = -1;

        SSD::SimPoint2D gps2D(gps.posX, gps.posY);

        for (unsigned int i = 0; i < trafficSignList.size(); i++) {
            auto &sign = trafficSignList[i];
            double distanceSign = UtilMath::distance(gps2D, SSD::SimPoint2D(sign.pt.x, sign.pt.y));
            if (distanceSign < minDist) {
                minDist = distanceSign;
                closestSignIndex = i;
            }
        }
        if (minDist > kAlertDistance) {
            return false;
        }
        warningSignIndex = closestSignIndex;
        auto &targetSign = trafficSignList[closestSignIndex];
        TrafficSignType sign = GetTrafficSignType(targetSign.type);
        if (sign == TrafficSignType::SpeedLimit_Sign) {
            std::string speedLimitValue = targetSign.value.GetString();
            double distanceSign = minDist;

            double speedLimit;
            std::stringstream ss;
            ss << speedLimitValue;
            ss >> speedLimit;

            if (mainVehicleVel > speedLimit) {
                return true;
            }
        }
        return false;
    }
};