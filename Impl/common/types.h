#ifndef AUTODRIVE_TYPES_H
#define AUTODRIVE_TYPES_H

#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "SimOnePNCAPI.h"

struct obstaclestruct {
    SSD::SimPoint3D pt;
    SSD::SimString ownerLaneId;
    ESimOne_Obstacle_Type type;
    double speed = 100;
    int id{};
    double width = 0;
    double oriZ = 100;
};

#endif //AUTODRIVE_TYPES_H
