
#ifndef AUTODRIVE_PREDICTION_H
#define AUTODRIVE_PREDICTION_H

#include <iostream>
#include "Service/SimOneIOStruct.h"
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "SimOneHDMapAPI.h"
#include "../utilTargetLane.h"


//是否在点之间
bool IsBetween(const double &src, const double &p1, const double &p2)//src是否在p1与p2之间
{
    return (p1 <= src && src <= p2) || (p2 <= src && src <= p1);
}

//是否在正方形内
bool InRectangle(const SSD::SimPoint3D &pt, const SSD::SimPoint3D &vertex1, const SSD::SimPoint3D &vertex2)//pt是否在长方形内
{
    return IsBetween(pt.x, vertex1.x, vertex2.x) && IsBetween(pt.y, vertex1.y, vertex2.y);
}

//判断区域是否被占据
bool IsOccupied2(const SSD::SimPoint3D& obstaclePos, const SSD::SimPoint3DVector& knots)//车位是否被占据
{
        return InRectangle(obstaclePos, knots[0], knots[2]);
}

//判断区域是否被占据
bool IsOccupied1(const SSD::SimPoint3D &obstaclePos, const SSD::SimPoint3DVector &knots, SSD::SimString& turn)//车位是否被占据
{
    if (turn =="left") {
        return InRectangle(obstaclePos, knots[1], knots[3]);
    }
    else if (turn == "right") {
        return InRectangle(obstaclePos, knots[0], knots[2]);
    }
    return false;
}

double getCross(const SSD::SimPoint3D &p1,const SSD::SimPoint3D &p2,const SSD::SimPoint3D &ObsPos){
    return (p2.x -p1.x)*(ObsPos.y-p1.y)-(ObsPos.x-p1.x)*(p2.y-p1.y);
}

bool IsOccupied(const SSD::SimPoint3D &obstaclePos, const SSD::SimPoint3DVector &knots){
    if(getCross(knots[0],knots[1],obstaclePos)*getCross(knots[2],knots[3],obstaclePos)>0&&
        getCross(knots[1],knots[2],obstaclePos)*getCross(knots[3],knots[0],obstaclePos)>0){
        return true;
    }
    return false;
}





bool IsCollision(const std::unique_ptr<SimOne_Data_Gps> &gps,
                 const std::unique_ptr<SimOne_Data_Obstacle> &pSimOne_Data_Obstacle) {
    // 计算未来一段时间内物体1的位置
    double time = 5;//未来5秒
    for (int i = 0; i < pSimOne_Data_Obstacle->obstacleSize; i++) {
        SimOne_Data_Obstacle_Entry obs = pSimOne_Data_Obstacle->obstacle[i];
        for (int j = 0; j < time * 2; j++) {
            //计算未来车的位置
            double futureX1 = gps->posX + gps->velX * float(j) / 2;
            double futureY1 = gps->posY + gps->velY * float(j) / 2;
            // 计算未来障碍物的位置
            double futureX2 = obs.posX + obs.velX * float(j) / 2;
            double futureY2 = obs.posY + obs.velY * float(j) / 2;
            //构建两个方框
            double s, t, T_left, T_right, S_front, S_back;
            SSD::SimPoint3DVector Zone1, Zone2;
            SSD::SimPoint3D Zone, dir, posV(futureX1, futureY1, gps->posZ), posO(futureX2, futureY2, obs.posZ);
            SSD::SimString laneid = utilTargetLane::GetNearMostLane(posV);
            SimOneAPI::GetLaneST(laneid, posV, s, t);
            T_left = t + 1;
            T_right = t - 1;
            S_front = s + 4;
            S_back = s - 2;
            SimOneAPI::GetInertialFromLaneST(laneid, S_front, T_right, Zone, dir);
            Zone1.push_back(Zone);
            SimOneAPI::GetInertialFromLaneST(laneid, S_front, T_left, Zone, dir);
            Zone1.push_back(Zone);
            SimOneAPI::GetInertialFromLaneST(laneid, S_back, T_left, Zone, dir);
            Zone1.push_back(Zone);
            SimOneAPI::GetInertialFromLaneST(laneid, S_back, T_right, Zone, dir);
            Zone1.push_back(Zone);
            laneid = utilTargetLane::GetNearMostLane(posO);
            SimOneAPI::GetLaneST(laneid, posO, s, t);
            T_left = t + obs.length / 2;
            T_right = t - obs.length / 2;
            S_front = s + obs.length / 2;
            S_back = s - obs.length / 2;
            SimOneAPI::GetInertialFromLaneST(laneid, S_front, T_right, Zone, dir);
            Zone2.push_back(Zone);
            SimOneAPI::GetInertialFromLaneST(laneid, S_front, T_left, Zone, dir);
            Zone2.push_back(Zone);
            SimOneAPI::GetInertialFromLaneST(laneid, S_back, T_left, Zone, dir);
            Zone2.push_back(Zone);
            SimOneAPI::GetInertialFromLaneST(laneid, S_back, T_right, Zone, dir);
            Zone2.push_back(Zone);
            for (auto &pointV: Zone1) {
                if (IsOccupied(pointV, Zone2)) {
                    return true;
                }
            }
            for (auto &pointO: Zone2) {
                if (IsOccupied(pointO, Zone2)) {
                    return true;
                }
            }
        }
    }
    return false;
}


#endif //AUTODRIVE_PREDICTION_H
