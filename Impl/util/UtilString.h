#pragma once

#include <sstream>

#define SOINFO(f_, ...) printf(("SOAPI[I]:" f_ "\n"), __VA_ARGS__)
#define SOWARNING(f_, ...) printf(("SOAPI[W]:" f_ "\n"), __VA_ARGS__)
#define SOERROR(f_, ...) printf(("SOAPI[E]:"  f_ "\n"), __VA_ARGS__)

#define SOINFO_N(f_, ...) printf(("SOAPI[I]:" f_ "\n"))
#define SOWARNING_N(f_, ...) printf(("SOAPI[W]:" f_ "\n"))
#define SOERROR_N(f_, ...) printf(("SOAPI[E]:" f_ "\n"))

using namespace std;

class UtilString
{
public:
	template<typename T>
	static std::string ToString(const T& t)
	{
		return std::to_string(t);
	}

	template<typename T>
	static T FromString(const std::string& str)
	{
		std::stringstream ss;
		ss << str;
		T output;
		ss >> output;
		return output;
	}
};
