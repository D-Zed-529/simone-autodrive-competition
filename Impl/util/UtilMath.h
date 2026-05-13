#pragma once

#include "SSD/SimPoint3D.h"
#include "SSD/SimPoint2D.h"
#include <cmath>

class UtilMath
{
public:
	static double distance(const SSD::SimPoint3D& pt1, const SSD::SimPoint3D& pt2)
	{
		return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
	}

	static double distance(const SSD::SimPoint2D& pt1, const SSD::SimPoint2D& pt2)
	{
		return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
	}

	static double planarDistance(const SSD::SimPoint3D& pt1, const SSD::SimPoint3D& pt2)
	{
		return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
	}

	static double calculateSpeed(const double velX, const double velY, const double velZ)
	{
		return std::sqrt(std::pow(velX, 2) + std::pow(velY, 2) + std::pow(velZ, 2));
	}
};
