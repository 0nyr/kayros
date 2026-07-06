//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "preprocess/preprocess_travel_times.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace solver
{
namespace
{
// Calculates the time to depart to traverse arc e arriving at tf.
// Returns: INFTY if it is infeasible to depart inside the horizon.
double igp_departing_time(const json& instance, Arc e, double tf)
{
	int c = instance["zones"][e.tail][e.head]; // cluster of arc e.
	vector<Interval> T = instance["time_steps"]; // T[k] = speed zone k.
	vector<double> speed = instance["speeds"][c]; // speed[k] = speed of traversing e in speed zone k.
	double d = instance["distances"][e.tail][e.head]; // distance of arc e.
	double t = tf;
	for (int k = (int)T.size()-1; k >= 0; --k)
	{
		if (epsilon_equal(d, 0.0)) break;
		if (epsilon_bigger(T[k].left, tf)) continue;
		double remaining_time_in_k = min(T[k].right, tf) - T[k].left;
		double time_to_complete_d_in_k = d / speed[k];
		double time_in_k = min(remaining_time_in_k, time_to_complete_d_in_k);
		t -= time_in_k;
		d -= time_in_k * speed[k];
	}
	if (epsilon_bigger(d, 0.0)) return INFTY;
	return t;
}


// Calculates the travel time to traverse arc e departing at t0.
// Returns: INFTY if it is infeasible to arrive inside the horizon.
double igp_travel_time(const json& instance, Arc e, double t0)
{
	int c = instance["zones"][e.tail][e.head]; // cluster of arc e.
	vector<Interval> T = instance["time_steps"]; // T[k] = speed zone k.
	vector<double> speed = instance["speeds"][c]; // speed[k] = speed of traversing e in speed zone k.
	double d = instance["distances"][e.tail][e.head]; // distance of arc e.
	double t = t0;
	for (int k = 0; k < T.size(); ++k)
	{
		if (epsilon_equal(d, 0.0)) break;
		if (epsilon_smaller(T[k].right, t0)) continue;
		double remaining_time_in_k = T[k].right - max(T[k].left, t0);
		double time_to_complete_d_in_k = d / speed[k];
		double time_in_k = min(remaining_time_in_k, time_to_complete_d_in_k);
		t += time_in_k;
		d -= time_in_k * speed[k];
	}
	if (epsilon_bigger(d, 0.0)) return INFTY;
	return t-t0;
}

// Returns the time when we arrive at the end of arc e if departing at t0.
double igp_ready_time(const json& instance, Arc e, double t0)
{
	double tt = igp_travel_time(instance, e, t0);
	return tt == INFTY ? tt : t0 + tt;
}

// Precondition: no speeds are 0.
PWLFunction compute_igp_travel_time_function(const json& instance, Arc e)
{
	// Calculate speed breakpoints.
	vector<Interval> speed_zones = instance["time_steps"];
	vector<double> speed_breakpoints;
	for (auto& z: speed_zones) speed_breakpoints.push_back(z.left);
	speed_breakpoints.push_back(speed_zones.back().right);
	
	// Travel time breakpoints are two sets
	// 	- B1: speed breakpoints which are feasible to depart
	// 	- B2: times t such that we arrive to head(e) at a speed breakpoint.
	vector<double> B1;
	for (double t: speed_breakpoints)
		if (igp_travel_time(instance, e, t) != INFTY)
			B1.push_back(t);
		
	vector<double> B2;
	for (double t: speed_breakpoints)
		if (igp_departing_time(instance, e, t) != INFTY)
			B2.push_back(igp_departing_time(instance, e, t));
	
	// Merge breakpoints in order in a set B.
	vector<double> B(B1.size()+B2.size());
	merge(B1.begin(), B1.end(), B2.begin(), B2.end(), B.begin());
	
	// Remove duplicates from B.
	B.resize(distance(B.begin(), unique(B.begin(), B.end())));
	
	// Calculate travel times for each t \in B.
	vector<double> T;
	for (double t: B) T.push_back(igp_travel_time(instance, e, t));
	
	// Create travel time function.
	PWLFunction tau;
	for (int i = 0; i < (int)B.size()-1; ++i)
		tau.AddPiece(LinearFunction(Point2D(B[i], T[i]), Point2D(B[i+1], T[i+1])));
	
	return tau;
}

// Computes the euclidean distance between two points.
double euclidean_distance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

inline double get_raw_travel_time_from_td_cost_matrix(
	const json& instance,
	int nb_vertices, 
    int i, 
    int j, 
    size_t time_step
) {
	return instance["td_cost_matrix"][i*nb_vertices + j][time_step];
}

void check_tau(
	const PWLFunction& tau, const Arc& e
) {
	if (tau.check_invariant())
		clog << "*";
	else
	{
		std::ostringstream oss;
		oss << "Invariant error for tau[" << e.tail << "][" << e.head << "]: " << to_string(tau);
		throw runtime_error(oss.str());
	}
}

PWLFunction compute_piecewise_constant_travel_time_function(
	const json& instance, Arc e
) {
	vector<double> B; // x-axis breakpoints (departure times)
	vector<double> T; // y-axis breakpoints (travel times)
	double t1, t2;
	const vector<Interval> time_steps = instance["time_steps"];
	const size_t nb_time_steps = time_steps.size();
	const int nb_vertices = instance["nb_vertices"];
	Interval horizon = instance["horizon"];
	const double epsilon_time_step_duration = (horizon.right - horizon.left) / (100 * nb_time_steps);

	// Add start point
	B.push_back(time_steps[0].left);
	T.push_back(get_raw_travel_time_from_td_cost_matrix(instance, nb_vertices, e.tail, e.head, 0));

	for (size_t ts = 0; ts < nb_time_steps - 1; ++ts) 
	{
		// handling time step transitions
		// NOTE: We are sure that there is a next time step
		const double travel_time = get_raw_travel_time_from_td_cost_matrix(instance, nb_vertices, e.tail, e.head, ts);
		const double next_travel_time = get_raw_travel_time_from_td_cost_matrix(instance, nb_vertices, e.tail, e.head, ts + 1);

		// x-axis breakpoints
		t1 = t2 = time_steps[ts].right;
		const double time_step_end = time_steps[ts + 1].left;
		assert(t1 == time_step_end  && "Time steps must be contiguous");

		if (next_travel_time < travel_time) 
		{
			// travel time decrease must be limited by a
			// slope of -1. Do the math
			t1 = time_step_end + next_travel_time - travel_time;
		}
		else if (next_travel_time > travel_time)
		{
			// modify x-axis breakpoints to avoid discontinuity
			t1 -= epsilon_time_step_duration;
			t2 += epsilon_time_step_duration;
		}
		else
		{
			// same slope, do not add extra breakpoints
			continue;
		}

		// check that t1 is within its time step range
		if (t1 < time_steps[ts].left || t1 > time_steps[ts].right)
		{
			std::ostringstream oss;
			oss << "t1 = " << t1 << " is not within the time step range [" << time_steps[ts].left << ", " << time_steps[ts].right << "]";
			throw runtime_error(oss.str());
		}
		// do same for t2
		if (t2 < time_steps[ts + 1].left || t2 > time_steps[ts + 1].right)
		{
			std::ostringstream oss;
			oss << "t2 = " << t2 << " is not within the time step range [" << time_steps[ts].left << ", " << time_steps[ts].right << "]";
			throw runtime_error(oss.str());
		}
		
		// save breakpoints
		B.push_back(t1);
		T.push_back(travel_time);
		B.push_back(t2);
		T.push_back(next_travel_time);
	}

	// Add end point
	B.push_back(time_steps.back().right);
	T.push_back(get_raw_travel_time_from_td_cost_matrix(instance, nb_vertices, e.tail, e.head, nb_time_steps - 1));

	#ifdef PRINT_TRAVEL_TIMES_PREPROCESSING
	clog << "B and T vectors for arc " << e.tail << " -> " << e.head << endl;
	print_padded_vectors(clog, B, T);
	#endif

	// Create travel time function.
	PWLFunction tau;
	for (int i = 0; i < (int)B.size()-1; ++i)
		tau.AddPiece(LinearFunction(Point2D(B[i], T[i]), Point2D(B[i+1], T[i+1])));	
	
	return tau;
}
} // anonymous namespace

void preprocess_constant_travel_times(nlohmann::json& instance)
{
	clog << " - Constant Travel Times" << endl;
	Interval horizon = instance["horizon"];

	Digraph D = instance;
	Matrix<PWLFunction> tau(D.NbVertices(), D.NbVertices());
	for (Arc e: D.Arcs())
	{
		// We consider the distance as the travel time.
		double distance = euclidean_distance(
			instance["coordinates"][e.head][0], instance["coordinates"][e.head][1],
			instance["coordinates"][e.tail][0], instance["coordinates"][e.tail][1]
		); 
		tau[e.tail][e.head] = PWLFunction::ConstantFunction(distance, Interval(horizon.left, horizon.right - distance));
		
		#ifdef PRINT_TRAVEL_TIMES_PREPROCESSING
		check_tau(tau[e.tail][e.head], e);
		clog << "   - Arc " << e.tail << " -> " << e.head << " = " << to_string(tau[e.tail][e.head]) << endl;
		#endif
	}
	instance["travel_times"] = tau;
}

void preprocess_igp_travel_times(json& instance)
{
	clog << " - IGP Travel Times" << endl;

	Digraph D = instance;
	Matrix<PWLFunction> tau(D.NbVertices(), D.NbVertices());
	for (Arc e: D.Arcs()) 
	{
		tau[e.tail][e.head] = compute_igp_travel_time_function(instance, e);
		
		#ifdef PRINT_TRAVEL_TIMES_PREPROCESSING
		check_tau(tau[e.tail][e.head], e);
		clog << "   - Arc " << e.tail << " -> " << e.head << " = " << to_string(tau[e.tail][e.head]) << endl;
		#endif
	}
	instance["travel_times"] = tau;
}

void preprocess_piecewise_constant_travel_times(json& instance)
{
	clog << " - Piecewise Constant Travel Times" << endl;

	Digraph D = instance;
	Matrix<PWLFunction> tau(D.NbVertices(), D.NbVertices());
	for (Arc e: D.Arcs()) 
	{
		tau[e.tail][e.head] = compute_piecewise_constant_travel_time_function(instance, e);
		#ifdef PRINT_TRAVEL_TIMES_PREPROCESSING
		check_tau(tau[e.tail][e.head], e);
		clog << "   - Arc " << e.tail << " -> " << e.head << " = " << to_string(tau[e.tail][e.head]) << endl;
		#endif
	}
	instance["travel_times"] = tau;
}

} // namespace