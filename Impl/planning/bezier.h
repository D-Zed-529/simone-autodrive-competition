#ifndef AUTODRIVE_BEZIER_H
#define AUTODRIVE_BEZIER_H

#include "SSD/SimPoint3D.h"

inline SSD::SimPoint3D CatmullRomInterpolation(
        const SSD::SimPoint3D &p0,
        const SSD::SimPoint3D &p1,
        const SSD::SimPoint3D &p2,
        const SSD::SimPoint3D &p3,
        double t) {
    double t2 = t * t;
    double t3 = t2 * t;

    double x = 0.5 * ((2.0 * p1.x) + (-p0.x + p2.x) * t
                      + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
                      + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

    double y = 0.5 * ((2.0 * p1.y) + (-p0.y + p2.y) * t
                      + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
                      + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

    SSD::SimPoint3D point;
    point.x = x;
    point.y = y;
    point.z = p0.z + p3.z;
    return point;
}

inline void BuildLaneChangePath(
        int num,
        const SSD::SimPoint3D &p0,
        const SSD::SimPoint3D &p1,
        const SSD::SimPoint3D &p2,
        const SSD::SimPoint3D &p3,
        SSD::SimPoint3DVector &path) {
    (void)p0;
    (void)p3;
    if (num <= 0) {
        return;
    }

    for (int i = 0; i < num; i++) {
        const double ratio = static_cast<double>(i) / static_cast<double>(num);
        SSD::SimPoint3D point;
        point.x = (p2.x - p1.x) * ratio + p1.x;
        point.y = (p2.y - p1.y) * ratio + p1.y;
        point.z = (p2.z - p1.z) * ratio + p1.z;
        path.push_back(point);
    }
}

#endif // AUTODRIVE_BEZIER_H
