#include "nyr/solutions/route.h"

using namespace std;
using namespace nlohmann;

namespace nyr
{

// Serialization / Deserialization
void to_json(json& j, const RouteMakespan& r)
{
	j["path"] = r.path;
	j["makespan"] = r.value;
}
void from_json(const json& j, RouteMakespan& r)
{
	vector<int> p = j["path"];
	r.path = p;
	r.value = j["makespan"];
}
void to_json(json& j, const RouteDuration& r)
{
	j["path"] = r.path;
	j["t0"] = r.t0;
	j["duration"] = r.value;
}
void from_json(const json& j, RouteDuration& r)
{
	vector<int> p = j["path"];
	r.path = p;
	r.t0 = j["t0"];
	r.value = j["duration"];
}
void to_json(json& j, const RouteTravelTime& r)
{
	j["path"] = r.path;
	j["t0s"] = r.t0s;
	j["travel_time"] = r.value;
}
void from_json(const json& j, RouteTravelTime& r)
{
	vector<int> p = j["path"];
	r.path = p;
	vector<double> t0s = j["t0s"];
	r.t0s = t0s;
	r.value = j["travel_time"];
}

// Returns: if two routes are equal.
bool operator==(const RouteMakespan& r1, const RouteMakespan& r2)
{
	return r1.path == r2.path && r1.value == r2.value;
}
bool operator==(const RouteDuration& r1, const RouteDuration& r2)
{
	return r1.path == r2.path && r1.t0 == r2.t0 && r1.value == r2.value;
}
bool operator==(const RouteTravelTime& r1, const RouteTravelTime& r2)
{
    return r1.path == r2.path && 
		   r1.t0s == r2.t0s && 
		   r1.value == r2.value;
}

} // namespace nyr