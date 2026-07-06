#include "preprocess/load_igp.h"

#include "preprocess/preprocess_travel_times.h"
#include "preprocess/preprocess_capacity.h"
#include "preprocess/preprocess_time_windows.h"
#include "preprocess/preprocess_service_waiting.h"
#include "preprocess/preprocess_triangle_depot.h"
#include "preprocess/preprocess_validity.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

const vector<string>COMMON_REQUIRED_KEYS = {
    "instance_basename",
    "benchmark_basename",
    "problem_type",
    "nb_vertices",
    "vehicle_capacity",
    "demands",
    "time_windows",
    "service_times",
    "start_depot",
    "end_depot",
    "nb_vehicles",
    "horizon",
    "arc_count",
    "arcs",   
};
const vector<string>SOLONOM_REQUIRED_KEYS = {
    "coordinates",
};
const vector<string>DABIA_REQUIRED_KEYS = {
    "nb_speeds",
    "speeds",
    "zones",
    "nb_time_steps",
    "time_steps",
    "distances",
};
const vector<string>RIFKI_REQUIRED_KEYS = {
    "td_cost_matrix",
    "nb_time_steps",
    "time_steps",
};

namespace solver
{
namespace
{
void check_required_keys(
    nlohmann::json& instance,
    const vector<string>& required_keys
) {
    for (const string& key: required_keys)
        if (!has_key(instance, key))
            throw runtime_error("The JSON instance is missing the key: " + key);
}

void load_solomon(nlohmann::json& instance)
{
    check_required_keys(instance, SOLONOM_REQUIRED_KEYS);
    
    clog << "Preprocessing..." << endl;
    preprocess_capacity(instance);
    preprocess_constant_travel_times(instance);
    preprocess_service_waiting(instance);
    preprocess_time_windows(instance);
    preprocess_triangle_depot(instance);
}

void load_dabia(nlohmann::json& instance)
{
    // Dabia2013 is based on Solomon1987.
    check_required_keys(instance, SOLONOM_REQUIRED_KEYS);
    check_required_keys(instance, DABIA_REQUIRED_KEYS);
    
    clog << "Preprocessing..." << endl;
    preprocess_capacity(instance);
    preprocess_igp_travel_times(instance);
    preprocess_service_waiting(instance);
    preprocess_time_windows(instance);
    preprocess_triangle_depot(instance);
}

void load_ari(nlohmann::json& instance)
{
    // Like Dabia's benchmark, Ari's instances are IGPs.
    check_required_keys(instance, DABIA_REQUIRED_KEYS);
    
    clog << "Preprocessing..." << endl;
    preprocess_capacity(instance);
    preprocess_igp_travel_times(instance);
    preprocess_service_waiting(instance);
    preprocess_time_windows(instance);
    preprocess_triangle_depot(instance);
}

void load_rifki(nlohmann::json& instance)
{
    // Rifki2020 is based on Solomon1987.
    check_required_keys(instance, RIFKI_REQUIRED_KEYS);
    
    clog << "Preprocessing..." << endl;
    preprocess_capacity(instance);
    preprocess_piecewise_constant_travel_times(instance);
    preprocess_service_waiting(instance);
    preprocess_time_windows(instance);
    preprocess_triangle_depot(instance);
}
} // anonymous namespace

void preprocess_instance_from_json(nlohmann::json& instance)
{
    clog << "Loading..." << endl;
    check_required_keys(instance, COMMON_REQUIRED_KEYS);
    
    string benchmark_basename = instance["benchmark_basename"];
    if (benchmark_basename == "Dabia2013")
        load_dabia(instance);
    else if (benchmark_basename == "Solomon1987")
        load_solomon(instance);
    else if (
        benchmark_basename == "Ari2018" ||
        benchmark_basename == "Vu2020"
    )
        load_ari(instance);
    else if (benchmark_basename == "Rifki2020")
        load_rifki(instance);
    else
        throw runtime_error("The benchmark_basename is not supported: " + benchmark_basename + ". If you need to add support for this benchmark, please modify the load.cpp file.");
}

nyr::VRPInstance load_instance_from_json(nlohmann::json const& instance) {
    // make a local copy, since preprocessing mutates the JSON
    nlohmann::json inst = instance;

    // run exactly the same two steps you were doing in main:
    preprocess_instance_from_json(inst);

    #ifndef NDEBUG
    preprocess_validity(inst);
    #endif

    // then deserialize via from_json overload into a VRPInstance
    return inst.get<nyr::VRPInstance>();
}

} // namespace