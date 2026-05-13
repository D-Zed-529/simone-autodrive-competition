#pragma once

#include <cmath>
#include <iostream>
#include "assert.h"
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "SimOneHDMapAPI.h"
#include "public/common/MLaneInfo.h"
#include "util/UtilMath.h"

class utilTargetLane {
public:
    static SSD::SimPoint3D CreateKnot(const SSD::SimPoint3D &from, const SSD::SimPoint3D &to, const double &k) {
        return SSD::SimPoint3D(from.x + k * (to.x - from.x), from.y + k * (to.y - from.y),
                               from.z + k * (to.z - from.z));
    }

    static SSD::SimString GetNearMostLane(const SSD::SimPoint3D &pos) {
        SSD::SimString laneId;
        double s, t, s_toCenterLine, t_toCenterLine;
        if (!SimOneAPI::GetNearMostLane(pos, laneId, s, t, s_toCenterLine, t_toCenterLine)) {
            std::cout << "Error: lane is not found." << std::endl;
        }
        return laneId;
    }

    static SSD::SimString GetLeftNeighborLane(const SSD::SimString &laneId) {
        SSD::SimString leftNeighborLaneName;
        HDMapStandalone::MLaneLink laneLink;
        if (SimOneAPI::GetLaneLink(laneId, laneLink)) {
            leftNeighborLaneName = laneLink.leftNeighborLaneName;
        }
        return leftNeighborLaneName;
    }

    static void GetLaneSampleFromS(const SSD::SimString &laneId, const double &s, SSD::SimPoint3DVector &targetPath) {
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

    static SSD::SimPoint3DVector CreateChangeLanePath(const SSD::SimPoint3D &from, const SSD::SimPoint3D &to) {
        SSD::SimPoint3DVector path;
        double d = UtilMath::distance(from, to);
        const double kDistancePerSample = 1;  //every 1 meter, add a knot
        double k = kDistancePerSample / d;
        int addKnotCount = (int) std::floor(d / kDistancePerSample);
        for (int i = 1; i <= addKnotCount; i++) {
            path.push_back(CreateKnot(from, to, i * k));
        }
        return path;
    }

    static SSD::SimPoint3DVector GetLaneSample(const SSD::SimString &laneId) {
        SSD::SimPoint3DVector targetPath;
        HDMapStandalone::MLaneInfo info;
        if (SimOneAPI::GetLaneSample(laneId, info)) {
            for (auto &pt: info.centerLine) {
                targetPath.push_back(SSD::SimPoint3D(pt.x, pt.y, pt.z));
            }
        }
        return targetPath;
    }
};
