#pragma once

#include "SSD/SimString.h"
#include "UtilString.h"

enum TrafficSignType {
	SpeedLimit_Sign = 1010203800001413,
	CrossWalk_Sign = 1010301800002616,
	TurnRight_Sign = 1010300300002413,
	TurnLeft_Sign = 1010300200002413,
	NoTooting_Sign = 1010203300001413,
	NoPassing_Sign = 1010202900001413,
	MinimumSpeed_Sign = 1010301500002413
};

TrafficSignType GetTrafficSignType(const SSD::SimString& typeStr)
{
	long long value = UtilString::FromString<long long>(typeStr.GetString());
	TrafficSignType sign = TrafficSignType(value);
	return sign;
}