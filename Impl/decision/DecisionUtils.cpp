#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../common/DecisionUtils.h"
#include <algorithm>
#include <iostream>
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "../perception/prediction.h"
#include "../util/UtilMath.h"
#include "../common/PathUtils.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wtautological-overlap-compare"
#pragma clang diagnostic ignored "-Wuninitialized"
#endif

/**
 * @brief 获取并定义一个用于检测换道区域的矩形区域。
 * @param currentLaneId 主车当前车道ID。
 * @param changeToLaneName 用于接收换道目标车道ID的引用。
 * @param mainVehiclePos 主车当前位置。
 * @param obstacle 目标障碍物。
 * @param detectingZone 用于接收检测区域四个角点的向量引用。
 * @param turn 用于接收换道方向（"left"或"right"）的引用。
 * @return 如果可以确定一个有效的换道方向和区域，则返回true。
 */
bool GetDetectingZone(const SSD::SimString &currentLaneId,
                      SSD::SimString &changeToLaneName,
                      const SSD::SimPoint3D &mainVehiclePos,
                      const obstaclestruct &obstacle,
                      SSD::SimPoint3DVector &detectingZone,
                      SSD::SimString &turn) {
    SSD::SimString turntolane;
    HDMapStandalone::MLaneLink lanelink;
    HDMapStandalone::MLaneType leftlanetype, rightlanetype;
    SimOneAPI::GetLaneLink(currentLaneId, lanelink);
    HDMapStandalone::MRoadMark left, right;

    SimOneAPI::GetRoadMark(mainVehiclePos, currentLaneId, left, right);
    SimOneAPI::GetLaneType(lanelink.leftNeighborLaneName, leftlanetype);
    SimOneAPI::GetLaneType(lanelink.rightNeighborLaneName, rightlanetype);
    if (lanelink.leftNeighborLaneName.Empty() == 0 &&
        leftlanetype == HDMapStandalone::MLaneType::driving &&
        left.type == HDMapStandalone::ERoadMarkType::broken) {
        turntolane = lanelink.leftNeighborLaneName;
        turn = "left";
    } else if (lanelink.rightNeighborLaneName.Empty() == 0 &&
               rightlanetype == HDMapStandalone::MLaneType::driving &&
               right.type == HDMapStandalone::ERoadMarkType::broken) {
        turntolane = lanelink.rightNeighborLaneName;
        turn = "right";
    } else {
        return false;
    }
    changeToLaneName = turntolane;
    SSD::SimPoint3D changeToPoint, changeBackPoint;
    SSD::SimPoint3D dir, dir2;
    double s, t, s_front, s_back, t_left, t_right;
    double distance =
            UtilMath::distance(mainVehiclePos, obstacle.pt);

    SimOneAPI::GetLaneMiddlePoint(
            obstacle.pt, turntolane, changeToPoint,
            dir);
    t_left = t - 1.7;
    t_right = t + 1.7;
    s_front = s + 10;
    s_back = s - distance - 7;

    SimOneAPI::GetInertialFromLaneST(turntolane, s_front, t_right,
                                     changeBackPoint, dir2);
    detectingZone.push_back(changeBackPoint);
    SimOneAPI::GetInertialFromLaneST(turntolane, s_front, t_left, changeBackPoint,
                                     dir2);
    detectingZone.push_back(changeBackPoint);
    SimOneAPI::GetInertialFromLaneST(turntolane, s_back, t_left, changeBackPoint,
                                     dir2);
    detectingZone.push_back(changeBackPoint);
    SimOneAPI::GetInertialFromLaneST(turntolane, s_back, t_right, changeBackPoint,
                                     dir2);
    detectingZone.push_back(changeBackPoint);

    return true;
}

/**
 * @brief 检查一个指定的检测区域是否被任何障碍物占用。
 * @param obstacleList 障碍物列表。
 * @param detectingZone 指定的检测区域（由四个角点定义）。
 * @return 如果区域内有障碍物，则返回true，否则返回false。
 */
bool IsChangeLaneOccupied(const std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3DVector &detectingZone) {
    for (auto &obstacle: obstacleList) {
        if (IsOccupied(obstacle.pt, detectingZone)) {
            std::cout << "obstacle" << obstacle.pt.x << "   " << obstacle.pt.y << std::endl;
            return true;
        }
    }
    return false;
}

/**
 * @brief 判断主车是否已经超过了指定的障碍物。
 * @param vehiclePos 主车位置。
 * @param obstacle 目标障碍物。
 * @param laneId 障碍物所在的车道ID。
 * @return 如果主车已越过障碍物，则返回true。
 */
bool PassedObstacle(const SSD::SimPoint3D &vehiclePos,
                    const obstaclestruct &obstacle,
                    const SSD::SimString &laneId) {
    double s, t;
    bool found = SimOneAPI::GetLaneST(laneId, obstacle.pt, s,
                                      t);
    assert(found);
    double s_vehicle, t_vehicle;
    found = SimOneAPI::GetLaneST(laneId, vehiclePos, s_vehicle,
                                 t_vehicle);
    assert(found);
    return s_vehicle > s;
}

/**
 * @brief 判断指定的交通信号灯是否为绿灯且有足够的时间通过。
 * @param lightId 交通灯ID。
 * @param laneId 主车当前车道ID。
 * @param light 交通灯对象。
 * @param speed 主车速度。
 * @param distance 主车到停止线的距离。
 * @return 如果是绿灯且时间充足，返回true。
 */
bool IsGreenLight(const long &lightId, const SSD::SimString &laneId,
                  const HDMapStandalone::MSignal &light, double speed,
                  double distance) {
    (void)laneId;
    (void)light;
    SimOne_Data_TrafficLight trafficLight;
    if (SimOneAPI::GetTrafficLight(0, lightId,
                                   &trafficLight))
    {
        if (trafficLight.status !=
            ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green) {
            return false;
        }
        else {
            if (trafficLight.countDown != -1) {
                if (distance / speed < (trafficLight.countDown - 2) ||
                    trafficLight.countDown > 10)
                    return true;
            } else if (trafficLight.countDown == -1)
                return true;
            else
                return false;
        }
    }
    return false;
}

/**
 * @brief 根据信号灯的有效性信息，获取目标后继车道。
 * @param successorLaneNameList 后继车道列表。
 * @param validities 信号灯有效性列表。
 * @return 目标后继车道的ID。
 */
SSD::SimString GetTargetSuccessorLane(
        const SSD::SimStringVector &successorLaneNameList,
        const SSD::SimVector<HDMapStandalone::MSignalValidity> &validities) {
    SSD::SimString targetSuccessorLaneId;
    for (auto &successorLane: successorLaneNameList) {
        HDMapStandalone::MLaneId id(
                successorLane);
        for (auto &validity: validities) {
            if ((id.roadId == validity.roadId &&
                 id.sectionIndex == validity.sectionIndex &&
                 id.sectionIndex == validity.fromLaneId) ||
                (id.roadId == validity.roadId &&
                 id.sectionIndex == validity.sectionIndex &&
                 id.sectionIndex == validity.toLaneId)) {
                targetSuccessorLaneId = successorLane;
                break;
            }
        }
    }
    return targetSuccessorLaneId;
}

/**
 * @brief 判断主车是否已经越过了某个点（如停止线）。
 * @param vehiclePos 主车位置。
 * @param light 目标点位置。
 * @param laneId 目标点所在车道ID。
 * @return 如果已越过，返回true。
 */
bool Passed(const SSD::SimPoint3D &vehiclePos, const SSD::SimPoint3D &light,
            const SSD::SimString &laneId) {
    double s, t;
    bool found =
            SimOneAPI::GetLaneST(laneId, light, s, t);
    double sveh, tveh;
    found = SimOneAPI::GetLaneST(laneId, vehiclePos, sveh,
                                 tveh);
    (void)found;
    std::cout << "sveh: " << sveh << " s: " << s << std::endl;
    return sveh > s;
}

/**
 * @brief 检测前方的停止线、交通灯和人行横道信息。
 * @param carPos 主车位置。
 * @param roadIdList 导航路径的道路ID列表。
 * @param stopLine 用于接收停止线位置的引用。
 * @param currentLight 用于接收当前交通灯对象的引用。
 * @param crosswalk 用于接收人行横道对象的引用。
 * @param currentDistance 用于接收到停止线距离的引用。
 * @return 如果检测到有效的停止线信息，则返回true。
 */
bool DetectStopLine(SSD::SimPoint3D &carPos, SSD::SimVector<long> &roadIdList,
                    SSD::SimPoint3D &stopLine,
                    HDMapStandalone::MSignal &currentLight,
                    HDMapStandalone::MObject &crosswalk,
                    double &currentDistance) {
    SSD::SimString laneId = GetNearMostLane(carPos);
    HDMapStandalone::MSignal light1 = GetTargetLight(laneId, roadIdList);
    stopLine = GetTargetStopLine(light1, laneId);
    double distance = UtilMath::distance(carPos, stopLine);
    currentLight = light1;
    SSD::SimVector<HDMapStandalone::MObject> crosswalklist;
    SimOneAPI::GetCrosswalkList(light1, laneId, crosswalklist);
    currentDistance = distance;
    if (distance < 150) {
        crosswalk = crosswalklist.front();
        std::cout << "crosswalk.boundaryKnots 0  " << crosswalk.boundaryKnots[0].x << "  "
                  << crosswalk.boundaryKnots[0].y << std::endl;
        std::cout << "crosswalk.boundaryKnots 1  " << crosswalk.boundaryKnots[1].x << "  "
                  << crosswalk.boundaryKnots[1].y << std::endl;
        std::cout << "crosswalk.boundaryKnots 2  " << crosswalk.boundaryKnots[2].x << "  "
                  << crosswalk.boundaryKnots[2].y << std::endl;
        std::cout << "crosswalk.boundaryKnots 3  " << crosswalk.boundaryKnots[3].x << "  "
                  << crosswalk.boundaryKnots[3].y << std::endl;
    } else {
        std::cout << "No crosswalk" << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 在没有交通灯的情况下，检测停止线和人行横道。
 * @param gps 主车的GPS数据。
 * @param hasTrafficLight 是否有交通灯的标志。
 * @param stopLine 用于接收停止线位置的引用。
 * @param crosswalk 用于接收人行横道边界点的引用。
 * @param currentDistance 用于接收到停止线距离的引用。
 * @return 如果检测到无灯控制的停止线，则返回true。
 */
bool DetectNoLightStopLine(const std::unique_ptr<SimOne_Data_Gps> &gps, bool hasTrafficLight, SSD::SimPoint3D &stopLine,
                           SSD::SimPoint3DVector &crosswalk, double &currentDistance) {
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    if (!hasTrafficLight) {
        SSD::SimVector<HDMapStandalone::MObject> stopLineList;
        SimOneAPI::GetSpecifiedLaneStoplineList(laneId, stopLineList);
        if (!stopLineList.empty()) {
            SSD::SimVector<HDMapStandalone::MObject> crosswalkList;
            SimOneAPI::GetSpecifiedLaneCrosswalkList(laneId, crosswalkList);
            if (!crosswalkList.empty()) {
                SSD::SimPoint3D dir;
                std::cout << "nolightstopline" << std::endl;
                SimOneAPI::GetLaneMiddlePoint(stopLineList[0].pt, laneId, stopLine, dir);
                currentDistance = UtilMath::distance(stopLine, mainVehiclePos);

                crosswalk = crosswalkList.front().boundaryKnots;
                std::cout << "crosswalkList" << std::endl;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 判断前方路口是否拥堵。
 * @param gps 主车的GPS数据。
 * @param allObstacles 场景中所有障碍物的列表。
 * @param roadIdList 导航路径的道路ID列表。
 * @return 如果拥堵，返回true。
 */
bool IsJunctionCrowded(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles,
                      const SSD::SimVector<long> &roadIdList) {
    SSD::SimPoint3DVector centerline;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);

    HDMapStandalone::MLaneId id(laneId);
    HDMapStandalone::MLaneId nextLaneId;
    HDMapStandalone::MLaneId towardsLaneId;

    if (DetectJunction(gps)) {
        int index = std::find(roadIdList.begin(), roadIdList.end(), id.roadId) - roadIdList.begin();
        GetValidSuccessor(id, roadIdList[index], roadIdList[index + 1], nextLaneId);
        GetValidSuccessor(nextLaneId, roadIdList[index + 1], roadIdList[index + 2], towardsLaneId);
        centerline = GetLaneSample(towardsLaneId.ToString());

        SSD::SimPoint3D startLine = centerline.front();
        SSD::SimPoint3DVector crowdedDetectionZone;
        double s, t, s_front, s_back, t_left, t_right;
        SSD::SimPoint3D Zone, dir2;

        SimOneAPI::GetLaneST(towardsLaneId.ToString(), startLine, s, t);
        t_left = t - 1.7;
        t_right = t + 1.8;
        s_front = s + 25;
        s_back = s;

        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_front, t_right, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_back, t_right, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_back, t_left, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_front, t_left, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);

        std::cout << " 0: " << crowdedDetectionZone[0].x << " , " << crowdedDetectionZone[0].y << std::endl;
        std::cout << " 1: " << crowdedDetectionZone[1].x << " , " << crowdedDetectionZone[1].y << std::endl;
        std::cout << " 2: " << crowdedDetectionZone[2].x << " , " << crowdedDetectionZone[2].y << std::endl;
        std::cout << " 3: " << crowdedDetectionZone[3].x << " , " << crowdedDetectionZone[3].y << std::endl;

        for (auto &obstacle: allObstacles) {
            if (obstacle.type == 6 && obstacle.speed <= 1) {
                if (IsOccupied(obstacle.pt, crowdedDetectionZone)) {
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief 检测是否可以进行换道（转向）。
 * @param gps 主车的GPS数据。
 * @param allObstacles 场景中所有障碍物的列表。
 * @param obstacle 目标障碍物。
 * @param turn 用于接收换道方向的引用。
 * @param state 用于接收车辆状态的引用（如 "wait"）。
 * @return 如果可以换道，返回true。
 */
bool DetectIsTurnable(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles,
                      const obstaclestruct &obstacle, SSD::SimString &turn,
                      SSD::SimString &state) {
    double s, t, s_obstacle, t_obstacle;

    SSD::SimPoint3DVector frontDetectionZone, rearDetectionZone;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimPoint3D dir, testpoint;
    SSD::SimString currentlaneid = GetNearMostLane(mainVehiclePos);
    SSD::SimString turntolane;
    HDMapStandalone::MLaneLink lanelink;
    HDMapStandalone::MLaneType leftlanetype, rightlanetype;

    SimOneAPI::GetLaneLink(obstacle.ownerLaneId, lanelink);
    HDMapStandalone::MRoadMark left, right;

    SimOneAPI::GetRoadMark(mainVehiclePos, currentlaneid, left, right);
    SimOneAPI::GetLaneType(lanelink.leftNeighborLaneName, leftlanetype);
    SimOneAPI::GetLaneType(lanelink.rightNeighborLaneName, rightlanetype);
    SimOneAPI::GetLaneST(currentlaneid, mainVehiclePos, s, t);
    SimOneAPI::GetLaneST(currentlaneid, obstacle.pt, s_obstacle, t_obstacle);

    double s_front = s_obstacle + 3;
    double s_obsback = s_obstacle - 2;
    double s_back = s - 3;


    if (lanelink.rightNeighborLaneName.Empty() == 0 &&
        (right.type != HDMapStandalone::ERoadMarkType::solid ||
         right.type != HDMapStandalone::ERoadMarkType::solid_solid ||
         right.type != HDMapStandalone::ERoadMarkType::broken_solid ||
         right.type != HDMapStandalone::ERoadMarkType::curb)) {
        turn = "right";
        std::cout << "right broken" << std::endl;
        turntolane = lanelink.rightNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);

        frontDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.70, +1.70);
        rearDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_obsback, -1.75, +1.75);

        if (!IsChangeLaneOccupied(allObstacles, frontDetectionZone) && !IsChangeLaneOccupied(allObstacles, rearDetectionZone)) {
            turn = "right";
            return true;
        } else {
            state = "wait";
        }
    } else if (lanelink.leftNeighborLaneName.Empty() == 0 &&
               (left.type != HDMapStandalone::ERoadMarkType::solid_broken ||
                left.type != HDMapStandalone::ERoadMarkType::solid ||
                left.type != HDMapStandalone::ERoadMarkType::solid_solid ||
                left.type != HDMapStandalone::ERoadMarkType::curb)) {
        turn = "left";
        std::cout << "left broken" << std::endl;
        turntolane = lanelink.leftNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);

        frontDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.75, +1.75);
        rearDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_obsback, -1.75, +1.75);
        if (!IsChangeLaneOccupied(allObstacles, frontDetectionZone) && !IsChangeLaneOccupied(allObstacles, rearDetectionZone)) {
            turn = "left";
            std::cout << "DetectIsTurnable::left" << std::endl;
            return true;
        } else {
            state = "wait";
            std::cout << "DetectIsTurnable::wait" << std::endl;
        }
    } else {
        state = "cant_change";
        return false;
    }
    PrintTargetPath(frontDetectionZone);
    std::cout << "DetectIsTurnable false " << std::endl;
    return false;
}

/**
 * @brief 检查换道条件是否满足。
 * @param obstacle 目标障碍物。
 * @param obstacleList 场景中所有障碍物的列表。
 * @param turnTargetPoint 用于接收换道目标点的引用。
 * @return 如果可换道，返回true。
 */
bool IsChangeable(obstaclestruct &obstacle, std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3D &turnTargetPoint) {

    double s_obstacle, t_obstacle;
    SSD::SimString currentlaneid = GetNearMostLane(obstacle.pt);
    SSD::SimPoint3DVector detectionZone;
    SSD::SimPoint3D dir, testpoint;
    SSD::SimString turntolane;
    HDMapStandalone::MLaneLink lanelink;
    HDMapStandalone::MLaneType leftlanetype, rightlanetype;

    SimOneAPI::GetLaneLink(currentlaneid, lanelink);

    HDMapStandalone::MRoadMark left, right;

    SimOneAPI::GetRoadMark(obstacle.pt, currentlaneid, left, right);
    SimOneAPI::GetLaneType(lanelink.leftNeighborLaneName, leftlanetype);
    SimOneAPI::GetLaneType(lanelink.rightNeighborLaneName, rightlanetype);
    SimOneAPI::GetLaneST(currentlaneid, obstacle.pt, s_obstacle, t_obstacle);

    double s_front = s_obstacle + 3;
    double s_back = s_obstacle - 3;
    std::cout << "lanelink.leftNeighborLaneName.Empty() ==" << lanelink.leftNeighborLaneName.Empty() << std::endl;
    std::cout << "lanelink.rightNeighborLaneName.Empty() ==" << lanelink.rightNeighborLaneName.Empty() << std::endl;

    if (lanelink.leftNeighborLaneName.Empty() == 0 &&
        leftlanetype == HDMapStandalone::MLaneType::driving &&
        (left.type != HDMapStandalone::ERoadMarkType::solid_broken ||
         left.type != HDMapStandalone::ERoadMarkType::solid ||
         left.type != HDMapStandalone::ERoadMarkType::solid_solid ||
         left.type != HDMapStandalone::ERoadMarkType::curb)) {
        turntolane = lanelink.leftNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);
        detectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.75, +1.75);

        if (!IsChangeLaneOccupied(obstacleList, detectionZone)) {
            turnTargetPoint = changeToPoint;
            return true;
        }
    } else if (lanelink.rightNeighborLaneName.Empty() == 0 &&
               rightlanetype == HDMapStandalone::MLaneType::driving &&
               (right.type != HDMapStandalone::ERoadMarkType::solid ||
                right.type != HDMapStandalone::ERoadMarkType::solid_solid ||
                right.type != HDMapStandalone::ERoadMarkType::broken_solid ||
                right.type != HDMapStandalone::ERoadMarkType::curb)) {
        std::cout << "right broken" << std::endl;
        turntolane = lanelink.rightNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);

        detectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.70,
                                         +1.70);
        if (!IsChangeLaneOccupied(obstacleList, detectionZone)) {
            turnTargetPoint = changeToPoint;
            return true;
        }
    }
    return false;
}

/**
 * @brief 检测障碍物是否在路口内。
 * @param obstacle 目标障碍物。
 * @return 如果在路口内，返回true。
 */
bool IsObstacleInJunction(obstaclestruct &obstacle) {
    SSD::SimString obstacleLaneId = GetNearMostLane(obstacle.pt);
    HDMapStandalone::MLaneInfo info;
    SimOneAPI::GetLaneSample(obstacleLaneId, info);
    SSD::SimPoint3DVector junctionLaneDetectionZone;
    junctionLaneDetectionZone.push_back(info.leftBoundary[0]);
    junctionLaneDetectionZone.push_back(info.leftBoundary[20]);
    junctionLaneDetectionZone.push_back(info.rightBoundary[0]);
    junctionLaneDetectionZone.push_back(info.rightBoundary[20]);
    PrintTargetPath(junctionLaneDetectionZone);
    return IsOccupied(obstacle.pt, junctionLaneDetectionZone);
}

/**
 * @brief 检测右侧是否有可行驶的车道。
 * @param lane 当前车道ID，如果找到右侧车道则会被修改。
 * @param mainVehiclePos 主车位置。
 * @return 如果存在，返回true。
 */
bool DetectRightRoad(SSD::SimString &lane, SSD::SimPoint3D &mainVehiclePos) {
    (void)mainVehiclePos;
    HDMapStandalone::MLaneLink laneLink;
    SimOneAPI::GetLaneLink(lane, laneLink);
    if (laneLink.rightNeighborLaneName.Empty() == 0) {
        lane = laneLink.rightNeighborLaneName;
        return true;
    }
    return false;
}

/**
 * @brief 检测左侧是否有可行驶的车道（且路标线允许变道）。
 * @param lane 当前车道ID，如果找到左侧车道则会被修改。
 * @param mainVehiclePos 主车位置。
 * @return 如果存在，返回true。
 */
bool DetectLeftRoad(SSD::SimString &lane, SSD::SimPoint3D &mainVehiclePos) {
    HDMapStandalone::MLaneLink laneLink;
    SimOneAPI::GetLaneLink(lane, laneLink);
    if (laneLink.leftNeighborLaneName.Empty()) {
        return false;
    }
    HDMapStandalone::MRoadMark left, right;
    SSD::SimPoint3D centerPoint, dir;
    SimOneAPI::GetLaneMiddlePoint(mainVehiclePos, lane, centerPoint, dir);
    SimOneAPI::GetRoadMark(centerPoint, lane, left, right);
    if (laneLink.leftNeighborLaneName.Empty() == 0 &&
        (left.type != HDMapStandalone::ERoadMarkType::solid ||
         left.type != HDMapStandalone::ERoadMarkType::solid_solid)) {
        lane = laneLink.rightNeighborLaneName;
        return true;
    }
    return false;
}

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

/**
 * @brief 在路口处，检测有效的、未被占用的可通行后继车道。
 * @param predecessorLane 进入路口前的车道ID。
 * @param carPos 主车位置。
 * @param obstacleList 障碍物列表。
 * @param validLane 用于接收找到的有效车道ID。
 * @param turnTargetPoint 用于接收转向目标点。
 * @return 如果找到有效通行路径，返回true。
 */
bool DetectValidCrossing(SSD::SimString &predecessorLane, SSD::SimPoint3D &carPos, std::vector<obstaclestruct> &obstacleList,
                         SSD::SimString &validLane, SSD::SimPoint3D &turnTargetPoint) {
    HDMapStandalone::MLaneLink predecessorLaneLink, laneLink;
    SSD::SimStringVector RoadList;
    SSD::SimString roadBuff = predecessorLane;
    SimOneAPI::GetLaneLink(predecessorLane, predecessorLaneLink);

    RoadList.push_back(roadBuff);
    while (DetectRightRoad(roadBuff, carPos)) {
        std::cout << "pushback" << std::endl;
        RoadList.push_back(roadBuff);
    }
    roadBuff = predecessorLane;
    while (DetectLeftRoad(roadBuff, carPos)) {
        RoadList.push_back(roadBuff);
    }


    for (auto &road: RoadList) {
        SimOneAPI::GetLaneLink(road, laneLink);
        for (auto &successorLane: laneLink.successorLaneNameList) {
            size_t n = 0;
            HDMapStandalone::MLaneInfo successorLaneInfo;
            SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
            for (auto &obs: obstacleList) {
                double dis = UtilMath::distance(*successorLaneInfo.centerLine.end(), obs.pt);
                if (dis < 1.75) n++;
            }
            if (n == 0) {
                validLane = successorLane;
                turnTargetPoint = successorLaneInfo.centerLine[1];
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 计算主车与障碍物之间的横向距离。
 * @param carPos 主车位置。
 * @param carOriZ 主车朝向角。
 * @param obstacle 目标障碍物。
 * @return 横向距离（负值表示在左，正值在右）。
 */
double GetLateralDistance(SSD::SimPoint3D &carPos, double &carOriZ, obstaclestruct &obstacle) {
    double distance = UtilMath::distance(obstacle.pt, carPos);
    double headingErrorRad = atan2(obstacle.pt.y - carPos.y, obstacle.pt.x - carPos.x);
    double error = headingErrorRad - carOriZ;

    if (error > M_PI) error = -2 * M_PI + error;
    else if (error < -M_PI) error = 2 * M_PI + error;

    return -distance * sin(error);

}

/**
 * @brief 检测前方是否为交叉路口。
 * @param currentLaneId 主车当前车道ID。
 * @param mainVehiclePos 主车位置。
 * @param gpsPtr 主车GPS数据指针。
 * @param lanelink 主车当前车道的链接信息。
 * @return 如果是交叉路口，返回true。
 */
bool DetectCross(SSD::SimString &currentLaneId, SSD::SimPoint3D &mainVehiclePos, SimOne_Data_Gps *gpsPtr,
                 HDMapStandalone::MLaneLink sourceLaneLink) {
    std::string driveStateName;
    HDMapStandalone::MLaneLink laneLink;
    HDMapStandalone::MLaneInfo successorLaneInfo, currentLaneInfo;

    SimOneAPI::GetLaneLink(currentLaneId, laneLink);
    SimOneAPI::GetLaneSample(currentLaneId, currentLaneInfo);
    size_t num = currentLaneInfo.centerLine.size();

    double lastPointDistance = UtilMath::distance(mainVehiclePos, currentLaneInfo.centerLine[num]);
    SSD::SimString successorLane;
    double laneHeadingRad, leftTurnCount = 0, rightTurnCount = 0, straightCount = 0;
    double carOriZ = gpsPtr->oriZ;
    std::cout << "successorLaneNameList.size():  " << laneLink.successorLaneNameList.size() << std::endl;
    if (lastPointDistance < 20) {
        if (laneLink.successorLaneNameList.size() >= 2) {
            for (const auto &i: laneLink.successorLaneNameList) {
                successorLane = i;
                SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
                laneHeadingRad = atan2(successorLaneInfo.centerLine[10].y - successorLaneInfo.centerLine[0].y,
                                       successorLaneInfo.centerLine[10].x - successorLaneInfo.centerLine[0].x);
                double headingDiffRad = laneHeadingRad - carOriZ;
                if (headingDiffRad < -0.1) {
                    leftTurnCount = 1;
                } else if (headingDiffRad > 0.1) {
                    rightTurnCount = 1;
                } else if (abs(headingDiffRad) <= 0.01) {
                    if (successorLaneInfo.centerLine.size() > 20) {
                        straightCount = 1;
                    }
                }
            }
            if (leftTurnCount == 1 && rightTurnCount != 1) {
                SimOneAPI::GetLaneLink(sourceLaneLink.rightNeighborLaneName, laneLink);
                for (size_t i = 0; i < laneLink.successorLaneNameList.size(); i++) {
                    successorLane = laneLink.successorLaneNameList[i];
                    SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
                    laneHeadingRad = atan2(successorLaneInfo.centerLine[10].y - successorLaneInfo.centerLine[0].y,
                                           successorLaneInfo.centerLine[10].x - successorLaneInfo.centerLine[0].x);
                    double headingDiffRad = laneHeadingRad - carOriZ;
                    std::cout << "laneHeadingRad-carOriZ1:  " << headingDiffRad << std::endl;
                    if (headingDiffRad > 0.1) {
                        rightTurnCount = 1;
                    }
                }
            }
            if (leftTurnCount != 1 && rightTurnCount == 1) {
                SimOneAPI::GetLaneLink(sourceLaneLink.rightNeighborLaneName, laneLink);
                for (size_t i = 0; i < laneLink.successorLaneNameList.size(); i++) {
                    successorLane = laneLink.successorLaneNameList[i];
                    SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
                    laneHeadingRad = atan2(successorLaneInfo.centerLine[10].y - successorLaneInfo.centerLine[0].y,
                                           successorLaneInfo.centerLine[10].x - successorLaneInfo.centerLine[0].x);
                    double headingDiffRad = laneHeadingRad - carOriZ;
                    std::cout << "laneHeadingRad-carOriZ2:  " << headingDiffRad << std::endl;
                    if (headingDiffRad < -0.1) {
                        leftTurnCount = 1;
                    }
                }
            }
        }
        if (leftTurnCount + rightTurnCount + straightCount >= 2) {
            driveStateName = "NearIntersection";
            return true;
        } else {
            driveStateName = "Follow";
        }

    }
    return false;
}
