//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/runner/runner_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/string/string_utils.h"
#include "goc/lib/json.hpp"

#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using namespace nlohmann;

namespace goc
{
namespace
{
// The input that will be added to STDIN.
stringstream custom_cin;
}

void simulate_runner_input(
	const string& instance_dirpath, 
	const string& instance_filename,
	const string& experiment_filepath,
	const string& experiment_name
) {
	// Read experiments_old from file.
	ifstream experiment_file(experiment_filepath);
	if (!experiment_file.good()) fail("The experiment file does not exist.");
	json experiment_set, experiment;
	experiment_file >> experiment_set;
	for (auto& e: experiment_set["experiments"]) if (e["name"] == experiment_name) experiment = e;
	if (experiment == json({})) fail("The experiment named " + STR(experiment_name) + " does not exist.");
	custom_cin << experiment;
	experiment_file.close();
	
	// Read instance file.
	json instance;
	string instance_filepath = instance_dirpath + "/" + instance_filename;
	ifstream instance_stream(instance_filepath);
	if (!instance_stream.good()) fail("The instance file could not be found at " + instance_filepath);
	instance_stream >> instance;
	custom_cin << instance;
	instance_stream.close();
	
	// Read instance solutions if exists, otherwise output empty array.
	ifstream solutions_file(instance_dirpath + "/solutions.json");
	json solutions = {};
	if (solutions_file.good()) solutions_file >> solutions;
	solutions_file.close();
	json instance_solutions = vector<json>();
	for (auto& solution: solutions) if (solution["instance_filename"] == instance_filename) instance_solutions.push_back(solution);
	custom_cin << instance_solutions;
	
	// Move the stream custom_cin to cin.
	cin.rdbuf(custom_cin.rdbuf());
}
} // namespace goc