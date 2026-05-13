#pragma once
#define USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <iostream>
#include <memory>
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "Service/SimOneIOStruct.h"

#ifndef M_PI
#define M_PI 3.1415926
#endif

class UtilDriver
{
public:
	static void setDriver(long long& timestamp, const float& throttle, const float& brake, const float& steering)
	{
		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
		pControl->timestamp = timestamp;
		pControl->throttle = throttle;
		pControl->brake = brake;
		pControl->steering = steering;
                pControl->throttleMode=ESimOne_Throttle_Mode_Accel;
		pControl->handbrake = false;
		pControl->isManualGear = false;
		pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
		SimOneAPI::SetDrive(0, pControl.get());
	}

    static double calculateSteering(const SSD::SimPoint3DVector& targetPath, SimOne_Data_Gps *pGps,double &headingErrorRad,double &turnHeadingErrorRad)
    {
        double ld,forwardIndex2;
        size_t forwardIndex;
        size_t index;
        //首先，对于目标路径上的每个点，计算当前车辆位置到该点的距离的平方，并将这些距离平方保存在名为 pts 的数组中
        std::vector<float> pts;
        for (size_t i = 0; i < targetPath.size(); ++i){
            pts.push_back(pow((pGps->posX - (float)targetPath[i].x), 2) + pow((pGps->posY - (float)targetPath[i].y), 2));
        }

        //然后，找到 pts 数组中的最小值对应的索引，即找到最接近当前车辆位置的目标路径上的点的索引。这个索引被存储在变量 index 中
        index = std::min_element(pts.begin(), pts.end()) - pts.begin();

        //接下来，根据一些预设的参数和车辆的当前速度，计算一个“前进距离”（progDist），这个距离用来选择目标路径上的一个前方点
        forwardIndex = 0;
        forwardIndex2 = 0;
        float minProgDist = 5.f;
        float progTime1 = 0.8f;
        float progTime2 = 4.f;
        float mainVehicleSpeed = sqrtf(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
        float progDist1 = mainVehicleSpeed * progTime1 > minProgDist ? mainVehicleSpeed * progTime1 : minProgDist;
        float progDist2 = mainVehicleSpeed * progTime2 > minProgDist ? mainVehicleSpeed * progTime2 : minProgDist;
        //从找到的 index 开始，遍历目标路径，找到第一个距离大于等于前进距离 progDist 的点的索引，这个点的索引被存储在变量 forwardIndex 中
        for (; index < targetPath.size(); ++index)
        {
            float distance = sqrtf(((float)pow(targetPath[index].x - pGps->posX, 2) + pow((float)targetPath[index].y - pGps->posY, 2)));
            if(distance<=progDist1) {
              forwardIndex = index;
            }
            forwardIndex2 = index;
            if (distance >= progDist2)
            {
                break;
            }
        }

        //根据车辆的位置、朝向角以及找到的前方点，计算角度 headingErrorRad，表示车辆与目标路径上的前方点之间的夹角
        auto psi = (double)pGps->oriZ;
        headingErrorRad = atan2(targetPath[forwardIndex].y - pGps->posY, targetPath[forwardIndex].x - pGps->posX) - psi;
        turnHeadingErrorRad=atan2(targetPath[forwardIndex2].y - pGps->posY, targetPath[forwardIndex2].x - pGps->posX) - psi;
        if (headingErrorRad > M_PI) {
            headingErrorRad = -2*M_PI + headingErrorRad;
        } else if (headingErrorRad < -M_PI) {
            headingErrorRad = 2*M_PI + headingErrorRad;
        }
        if (turnHeadingErrorRad > M_PI) {
            turnHeadingErrorRad = -2*M_PI + turnHeadingErrorRad;
        } else if (turnHeadingErrorRad < -M_PI) {
            turnHeadingErrorRad = 2*M_PI + turnHeadingErrorRad;
        }

        //计算变量 ld，表示车辆到前方点的距离
        ld = sqrt(pow(targetPath[forwardIndex].y - pGps->posY, 2) + pow(targetPath[forwardIndex].x - pGps->posX, 2));

        //最后，根据一些几何和物理参数，计算车辆的方向盘转角 steering，并将其返回
        double steering = -atan2(2. * (1.3 + 1.55) * sin(headingErrorRad), ld) * 36. / (7. * M_PI ) *1.7;

        return steering;
    }
};
