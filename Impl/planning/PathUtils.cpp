#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../common/PathUtils.h"
#include "SimOneHDMapAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "../util/UtilMath.h"
#include "bezier.h"
#include <iostream>
#include <algorithm>

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

/**
 * @brief 获取离指定位置最近的车道ID。
 * @param pos 指定的3D坐标点。
 * @return 最近车道的ID (SimString)。
 */
SSD::SimString GetNearMostLane(const SSD::SimPoint3D &pos) {
    SSD::SimString laneId;
    double s, t, s_toCenterLine, t_toCenterLine;
    if (!SimOneAPI::GetNearMostLane(pos, laneId, s, t, s_toCenterLine,
                                    t_toCenterLine)) {
        std::cout << "Error: lane is not found." << std::endl;
    }
    return laneId;
}

/**
 * @brief 获取指定点在目标车道上的S值（纵向里程）。
 * @param pos 指定的3D坐标点。
 * @param laneId 目标车道的ID。
 * @return S值 (double)。
 */
double GetS(const SSD::SimPoint3D &pos, SSD::SimString laneId) {
    double s, t;
    if (!SimOneAPI::GetLaneST(laneId, pos, s, t)) {
    }
    return s;
}

/**
 * @brief 获取指定点在目标车道上的T值（横向偏移）。
 * @param pos 指定的3D坐标点。
 * @param laneId 目标车道的ID。
 * @return T值 (double)。
 */
double GetT(const SSD::SimPoint3D &pos, SSD::SimString laneId) {
    double s, t;
    if (!SimOneAPI::GetLaneST(laneId, pos, s, t)) {
    }
    return t;
}

/**
 * @brief 获取从起点到终点的导航路径所经过的道路ID列表。
 * @param startPt 起始点。
 * @param endPt 终点。
 * @return 包含道路ID的向量。
 */
SSD::SimVector<long> GetNavigateRoadIdList(const SSD::SimPoint3D &startPt,
                                           const SSD::SimPoint3D &endPt) {
    SSD::SimVector<long> naviRoadIdList;
    SSD::SimPoint3DVector ptList;
    ptList.push_back(startPt);
    ptList.push_back(endPt);
    SSD::SimVector<int> indexOfValidPoints;
    SimOneAPI::Navigate(ptList, indexOfValidPoints,
                        naviRoadIdList);
    return naviRoadIdList;
}

/**
 * @brief 获取当前车道的有效后继车道。
 * @param laneId 当前车道的ID。
 * @param currentRoadId 当前道路的ID。
 * @param nextRoadId 下一个道路的ID。
 * @param successor 用于接收后继车道ID的引用。
 * @return 如果找到有效的后继车道，则返回true，否则返回false。
 */
bool GetValidSuccessor(const HDMapStandalone::MLaneId &laneId,
                       const long &currentRoadId, const long &nextRoadId,
                       HDMapStandalone::MLaneId &successor) {
    if (nextRoadId == -1)
    {
        return false;
    }
    HDMapStandalone::MLaneLink laneLink;
    bool valid =
            SimOneAPI::GetLaneLink(laneId.ToString(), laneLink);
    assert(valid);
    if (laneLink.successorLaneNameList.empty()) {

        return false;
    }

    for (auto &successorLane: laneLink.successorLaneNameList) {
        HDMapStandalone::MLaneId successorId(successorLane);
        if (successorId.roadId != currentRoadId)
        {
            if (successorId.roadId == nextRoadId)
            {
                successor = successorId;
                return true;
            }
        } else
        {
            HDMapStandalone::MLaneId successorOfSuccessor;
            if (successorId.sectionIndex ==
                laneId.sectionIndex + 1)
            {
                successor = successorId;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 将指定车道的采样点添加到路径向量中。
 * @param laneId 目标车道的ID。
 * @param path 用于接收采样点的路径向量。
 */
void AddSamples(const HDMapStandalone::MLaneId &laneId,
                SSD::SimPoint3DVector &path) {
    HDMapStandalone::MLaneInfo laneInfo;
    if (SimOneAPI::GetLaneSample(laneId.ToString(), laneInfo)) {
        path.reserve(path.size() +
                     laneInfo.centerLine.size());
        for (auto &pt: laneInfo.centerLine) {
            path.push_back(pt);
        }
    }
}

/**
 * @brief 根据导航路网生成参考路径。
 * @param startPt 起始点。
 * @param naviRoadIdList 导航路径的道路ID列表。
 * @return 生成的参考路径（3D点向量）。
 */
SSD::SimPoint3DVector
GetReferencePath(const SSD::SimPoint3D &startPt,
                 const SSD::SimVector<long> &naviRoadIdList) {
    SSD::SimPoint3DVector path;
    SSD::SimString laneName;
    double s = 0, t = 0, s_toCenterLine, t_toCenterLine;
    HDMapStandalone::MLaneInfo info;

    if (!SimOneAPI::GetNearMostLane(startPt, laneName, s, t, s_toCenterLine,
                                    t_toCenterLine)) {
        return path;
    }
    SSD::SimString laneId;
    HDMapStandalone::MLaneId currentLaneId(laneId);
    HDMapStandalone::MLaneId nextLaneId;

    AddSamples(currentLaneId, path);

    for (size_t index = 0; index < naviRoadIdList.size(); ++index) {
        long roadId = naviRoadIdList[index];
        long nextRoadId =
                (index + 1 < naviRoadIdList.size()) ? naviRoadIdList[index + 1] : -1;

        if (GetValidSuccessor(currentLaneId, roadId, nextRoadId, nextLaneId)) {
            AddSamples(nextLaneId, path);
            currentLaneId = nextLaneId;
        } else {
            break;
        }
    }

    return path;
}

/**
 * @brief 从指定车道的指定S值开始，获取采样点并添加到目标路径。
 * @param laneId 目标车道的ID。
 * @param s 起始S值。
 * @param targetPath 用于接收采样点的路径向量。
 */
void GetLaneSampleFromS(const SSD::SimString &laneId, const double &s,
                        SSD::SimPoint3DVector &targetPath) {
    HDMapStandalone::MLaneInfo info;
    if (SimOneAPI::GetLaneSample(laneId, info)) {
        double accumulated = 0.0;
        int startIndex = -1;
        for (unsigned int i = 0; i < info.centerLine.size() - 1; i++) {
            auto &pt = info.centerLine[i];
            auto &ptNext = info.centerLine[i + 1];
            double d = UtilMath::distance(pt, ptNext);
            accumulated += d;
            if (accumulated >= s) {
                startIndex = i + 1;
                break;
            }
        }
        for (unsigned int i = startIndex; i < info.centerLine.size(); i++) {
            SSD::SimPoint3D item = info.centerLine[i];
            targetPath.push_back(info.centerLine[i]);
        }
    }
}

/**
 * @brief 使用贝塞尔曲线生成两点间的换道路径。
 * @param from 起始点。
 * @param to 目标点。
 * @return 生成的换道路径（3D点向量）。
 */
SSD::SimPoint3DVector ChangeLanePathWithPoint(const SSD::SimPoint3D &from,
                                              const SSD::SimPoint3D &to) {
    SSD::SimPoint3DVector path;
    double s, t, s_c, t_c;
    SSD::SimPoint3D PointNextFrom, PointNextTo, dir;
    SSD::SimString lanefrom, laneto;
    SimOneAPI::GetNearMostLane(from, lanefrom, s, t, s_c, t_c);
    SimOneAPI::GetInertialFromLaneST(lanefrom, s + 1, t, PointNextFrom, dir);
    SimOneAPI::GetNearMostLane(to, laneto, s, t, s_c, t_c);
    SimOneAPI::GetInertialFromLaneST(laneto, s + 1, t, PointNextTo, dir);
    BuildLaneChangePath(int(UtilMath::distance(from, to)), from, PointNextFrom, to,
                        PointNextTo, path);
    return path;
}

/**
 * @brief 生成绕行障碍物的换道路径。
 * @param vel 当前速度。
 * @param obstacle 目标障碍物。
 * @param mainVehiclePos 主车位置。
 * @param changeToLaneName 换道的目标车道ID。
 * @param targetPath 用于接收生成的路径。
 */
void GetChangeLanePath(const double &vel, const obstaclestruct &obstacle,
                       const SSD::SimPoint3D &mainVehiclePos,
                       const SSD::SimString &changeToLaneName,
                       SSD::SimPoint3DVector &targetPath) {
    targetPath.clear();
    SSD::SimString laneNow;
    SSD::SimPoint3D changeToPoint, changeToNextPoint, FromNextPoint;
    SSD::SimPoint3D dir;
    double tos, tot;
    if (SimOneAPI::GetLaneMiddlePoint(
            obstacle.pt, changeToLaneName, changeToPoint,
            dir))
    {
    }

    SimOneAPI::GetLaneST(changeToLaneName, changeToPoint, tos, tot);
    SimOneAPI::GetInertialFromLaneST(changeToLaneName, tos + vel * 4, tot,
                                     changeToPoint, dir);
    targetPath = ChangeLanePathWithPoint(mainVehiclePos, changeToPoint);
}

/**
 * @brief 获取指定车道的所有中心线采样点。
 * @param laneId 目标车道的ID。
 * @return 包含车道中心线采样点的向量。
 */
SSD::SimPoint3DVector GetLaneSample(const SSD::SimString &laneId) {
    SSD::SimPoint3DVector targetPath;
    HDMapStandalone::MLaneInfo info;
    if (SimOneAPI::GetLaneSample(laneId, info)) {
        for (auto &pt: info.centerLine) {
            targetPath.push_back(SSD::SimPoint3D(pt.x, pt.y, pt.z));
        }
    }
    return targetPath;
}

/**
 * @brief 获取主车周围的实时检测区域。
 * @param gps 主车的GPS数据。
 * @param s_front 向前的纵向距离。
 * @param s_back 向后的纵向距离。
 * @param t_left 向左的横向距离。
 * @param t_right 向右的横向距离。
 * @return 定义了检测区域四个角的3D点向量。
 */
SSD::SimPoint3DVector
GetDetectionZoneRealtime(const std::unique_ptr<SimOne_Data_Gps> &gps,
                       const double &s_front, const double &s_back,
                       const double &t_left, const double &t_right) {
    SSD::SimPoint3DVector detectZone;
    SSD::SimPoint3D Zone, dir;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    double s, t, S_front, S_back, T_left, T_right;

    SimOneAPI::GetLaneST(laneId, mainVehiclePos, s, t);
    T_left = t + t_left;
    T_right = t + t_right;
    S_front = s + s_front;
    S_back = s + s_back;

    SimOneAPI::GetInertialFromLaneST(laneId, S_front, T_right, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, S_front, T_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, S_back, T_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, S_back, T_right, Zone, dir);
    detectZone.push_back(Zone);

    return detectZone;
}

/**
 * @brief 获取固定位置的检测区域。
 * @param pos 区域中心的3D坐标点。
 * @param s_front 向前的纵向距离。
 * @param s_back 向后的纵向距离。
 * @param t_left 向左的横向距离。
 * @param t_right 向右的横向距离。
 * @return 定义了检测区域四个角的3D点向量。
 */
SSD::SimPoint3DVector GetDetectionZoneFixed(const SSD::SimPoint3D &pos,
                                          const double &s_front,
                                          const double &s_back,
                                          const double &t_left,
                                          const double &t_right) {
    SSD::SimPoint3DVector detectZone;
    SSD::SimPoint3D Zone, dir;
    SSD::SimString laneId = GetNearMostLane(pos);
    std::cout << "laneId==" << laneId.GetString() << std::endl;

    SimOneAPI::GetInertialFromLaneST(laneId, s_front, t_right, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, s_front, t_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, s_back, t_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, s_back, t_right, Zone, dir);
    detectZone.push_back(Zone);

    return detectZone;
}

/**
 * @brief 计算主车与障碍物之间的纵向S距离。
 * @param gps 主车的GPS数据。
 * @param obstacle 目标障碍物。
 * @param roadIdList 导航路径的道路ID列表。
 * @return 纵向S距离。
 */
double GetSDistance(const std::unique_ptr<SimOne_Data_Gps> &gps,
                    const obstaclestruct &obstacle,
                    const SSD::SimVector<long> &roadIdList) {
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    SSD::SimString obstacleLaneId = GetNearMostLane(obstacle.pt);

    HDMapStandalone::MLaneId carLaneId(laneId);
    HDMapStandalone::MLaneId obstacleLaneInfo(obstacleLaneId);

    double sDistance;
    double s, s_obstacle, t, z;
    SimOneAPI::GetRoadST(laneId, mainVehiclePos, s, t, z);
    SimOneAPI::GetRoadST(obstacleLaneId, obstacle.pt, s_obstacle, t, z);

    sDistance = s_obstacle - s;

    if (carLaneId.roadId <= obstacleLaneInfo.roadId) {
        HDMapStandalone::MLaneId current_id = carLaneId;
        while (current_id.roadId <= obstacleLaneInfo.roadId) {
            SSD::SimPoint3DVector centerline = GetLaneSample(current_id.ToString());
            double tempS;
            SimOneAPI::GetRoadST(current_id.ToString(), centerline.back(), tempS, t,
                                 z);
            sDistance += tempS;

            auto it =
                    std::find(roadIdList.begin(), roadIdList.end(), current_id.roadId);
            if (it != roadIdList.end() && std::next(it) != roadIdList.end()) {
                current_id.roadId = *(std::next(it));
            } else {
                break;
            }
        }
    } else {
        HDMapStandalone::MLaneId current_id = carLaneId;
        while (current_id.roadId >= obstacleLaneInfo.roadId) {
            SSD::SimPoint3DVector centerline = GetLaneSample(current_id.ToString());
            double tempS;
            SimOneAPI::GetRoadST(current_id.ToString(), centerline.back(), tempS, t,
                                 z);
            sDistance -= tempS;

            auto it =
                    std::find(roadIdList.begin(), roadIdList.end(), current_id.roadId);
            if (it != roadIdList.begin()) {
                current_id.roadId = *(std::prev(it));
            } else {
                break;
            }
        }
    }

    return sDistance;
}

/**
 * @brief 获取仿真环境中的终点。
 * @return 终点的3D坐标。
 */
SSD::SimPoint3D GetTerminalPoint() {
    SimOne_Data_WayPoints wayPoints;
    SSD::SimPoint3D endPt;
    const char *kMainVehicleId = "0";
    if (SimOneAPI::GetWayPoints(kMainVehicleId, &wayPoints)) {
        int waySize = wayPoints.wayPointsSize;
        endPt.x = wayPoints.wayPoints[waySize - 1].posX;
        endPt.y = wayPoints.wayPoints[waySize - 1].posY;
        endPt.z = 0;
    }
    return endPt;
}

/**
 * @brief 获取与当前路径相关的交通信号灯。
 * @param laneId 当前车道ID。
 * @param roadIdList 导航路径的道路ID列表。
 * @return 目标交通信号灯对象。
 */
HDMapStandalone::MSignal GetTargetLight(const SSD::SimString &laneId, const SSD::SimVector<long> &roadIdList) {
    (void)laneId;
    HDMapStandalone::MSignal light;
    SSD::SimVector<HDMapStandalone::MSignal> lightList;
    SimOneAPI::GetTrafficLightList(lightList);


    for (auto &item: lightList) {
        int num = 0;

        for (auto &ptValidities: item.validities) {
            auto it = std::find(roadIdList.begin(), roadIdList.end(), ptValidities.roadId);
            if (it != roadIdList.end()) {
                ++num;
            }
        }

        if (num >= 2) {
            light = item;
            break;
        }
    }
    return light;
}

/**
 * @brief 根据交通信号灯获取关联的停止线。
 * @param light 交通信号灯对象。
 * @param laneId 当前车道ID。
 * @return 停止线的中心点3D坐标。如果找不到则返回一个最大值坐标。
 */
SSD::SimPoint3D GetTargetStopLine(const HDMapStandalone::MSignal &light,
                                  const SSD::SimString &laneId) {
    SSD::SimVector<HDMapStandalone::MObject> stoplineList;
    SimOneAPI::GetStoplineList(light, laneId, stoplineList);
    if (stoplineList.empty()) {
        return SSD::SimPoint3D(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    }
    SSD::SimPoint3D pt, dir;
    SimOneAPI::GetLaneMiddlePoint(stoplineList[0].pt, laneId, pt, dir);
    return pt;
}

/**
 * @brief 在长路径中查找离主车最近的点的索引。
 * @param longPath 长路径的点向量。
 * @param mainVehiclePos 主车位置。
 * @return 最近点的索引。
 */
size_t IndexNumberOfLongPath(SSD::SimPoint3DVector &longPath, SSD::SimPoint3D &mainVehiclePos) {
    size_t Index = 0;
    std::vector<float> pts;
    for (auto &i: longPath) {
        pts.push_back(
                pow((float(mainVehiclePos.x) - (float) i.x), 2) + pow((float(mainVehiclePos.y) - (float) i.y), 2));
    }
    Index = std::min_element(pts.begin(), pts.end()) - pts.begin();
    return Index;
}

/**
 * @brief 从长路径中截取前方的一段作为短期规划路径。
 * @param index 主车在长路径上的当前索引。
 * @param targetPath 完整的长路径。
 * @param mainVehiclePos 主车位置。
 * @return 截取出的前方路径点向量。
 */
SSD::SimPoint3DVector
GenerateForwardPoints(size_t &index, SSD::SimPoint3DVector &targetPath, SSD::SimPoint3D &mainVehiclePos) {
    SSD::SimPoint3DVector ForwardPoints;
    index = IndexNumberOfLongPath(targetPath, mainVehiclePos);
    const size_t endIndex = static_cast<size_t>(std::min(int(targetPath.size()), int(index + 100)));
    for (size_t i = index; i <= endIndex; i++) {
        ForwardPoints.push_back(targetPath[i]);
    }
    return ForwardPoints;
}

/**
 * @brief 生成从当前位置到目标车道上某点的换道路径。
 * @param mainVehiclePos 主车当前位置。
 * @param speed 主车当前速度。
 * @param changeToLane 目标车道ID。
 * @param changeToPoint 用于接收换道目标点的引用。
 * @return 生成的换道路径点向量。
 */
SSD::SimPoint3DVector
ChangeLanePathWithLane(const SSD::SimPoint3D &mainVehiclePos, double speed,
                       const SSD::SimString &changeToLane,
                       SSD::SimPoint3D &changeToPoint) {
    double s, t, s_changePath;
    SSD::SimPoint3DVector changelane_path;
    SSD::SimPoint3D forwardPoint, dir;
    SSD::SimString currentlaneid = GetNearMostLane(mainVehiclePos);
    SimOneAPI::GetLaneST(currentlaneid, mainVehiclePos, s, t);
    s_changePath = s + std::max(speed * 3, 5.);
    SimOneAPI::GetInertialFromLaneST(currentlaneid, s_changePath, t, forwardPoint,
                                     dir);
    SimOneAPI::GetLaneMiddlePoint(forwardPoint, changeToLane, changeToPoint, dir);
    changelane_path = ChangeLanePathWithPoint(mainVehiclePos, changeToPoint);
    return changelane_path;
}

/**
 * @brief 生成绕行障碍物的换道路径。
 * @param mainVehiclePos 主车当前位置。
 * @param changeToLane 目标车道ID。
 * @param obstaclePos 障碍物位置。
 * @param turnTargetPoint 用于接收换道目标点的引用。
 * @return 生成的换道路径点向量。
 */
SSD::SimPoint3DVector ChangeLanePathWithObstacle(const SSD::SimPoint3D &mainVehiclePos,
                                                 const SSD::SimString &changeToLane,
                                                 SSD::SimPoint3D &obstaclePos,
                                                 SSD::SimPoint3D &turnTargetPoint) {
    SSD::SimPoint3DVector changelane_path;
    SSD::SimPoint3D dir;
    SimOneAPI::GetLaneMiddlePoint(obstaclePos, changeToLane, turnTargetPoint, dir);
    changelane_path = ChangeLanePathWithPoint(mainVehiclePos, turnTargetPoint);
    return changelane_path;
}

/**
 * @brief 在没有目标路径时，根据当前位置生成一条沿着当前车道中心线的临时路径。
 * @param carPos 主车当前位置。
 * @param temporaryLine 用于接收生成的临时路径。
 */
void BuildLineWithoutTargetPath(SSD::SimPoint3D &carPos, SSD::SimPoint3DVector &temporaryLine) {
    HDMapStandalone::MLaneInfo laneinfo;
    SimOneAPI::GetLaneSampleByLocation(carPos, laneinfo);
    temporaryLine = laneinfo.centerLine;
}

/**
 * @brief (Debug) 打印目标路径中的部分点用于调试。
 * @param targetPath 目标路径的点向量。
 */
void PrintTargetPath(SSD::SimPoint3DVector &targetPath) {
    std::cout << "targetPath.size()==  " << targetPath.size() << std::endl;
    for (size_t i = 0; i < targetPath.size(); i += 20) {
        std::cout << "Point[" << i << "]==  " << targetPath[i].x << ","
                  << targetPath[i].y << "," << targetPath[i].z << std::endl;
    }
}
