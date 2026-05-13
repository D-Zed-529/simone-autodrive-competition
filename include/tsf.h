#pragma once


#ifdef _MSC_VER
#ifdef DLLTSF_EXPORTS
#define _IMP_EXP_CLASS_ _declspec( dllexport )
#else
#define _IMP_EXP_CLASS_ _declspec(dllimport)
#endif
#elif defined(__GNUC__)
#ifdef DLLTSF_EXPORTS
#define _IMP_EXP_CLASS_ __attribute__((visibility("default")))
#else
#define _IMP_EXP_CLASS_
#endif
#endif 

struct Point
{
	double x;
	double y;
	double z;
};

extern "C"
{
	_IMP_EXP_CLASS_ void SetGPSBase(double lat, double lon, double alt);
	_IMP_EXP_CLASS_ void GetLocal(double lat, double lon, double alt, Point& local);
	_IMP_EXP_CLASS_ Point GetWGS(const Point& pos);
}