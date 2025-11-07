#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <complex>
#include <list>

#include "Solution.h"
#include "PatrollingInput.h"
#include "Utilities.h"
#include "Roots.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "Pathfinder.h"
#include "SimulationState.h" // ÐÂÔö
#include <boost/numeric/conversion/cast.hpp>


#define DEBUG_SOLVER	DEBUG || 0


// Data structure for when a drone arrives at the UGV
struct Arrival {
	double time;
	int ID;
	Arrival(double t, int id) : time(t), ID(id) {}
};

// Comparison function to order Arrivals by time
struct CompareArrival {
	bool operator()(const Arrival& a1, const Arrival& a2) {
		return a1.time > a2.time;
	}
};

struct TSPVertex {
	int nID;
	double x;
	double y;
};


class Solver {
public:
	Solver(Pathfinder* pathfinder);
	virtual ~Solver();

	// *** MODIFICATION: Change Solve interface ***
	virtual void Solve(PatrollingInput* input, SimulationState* sim_state) = 0;

protected:
	// *** MODIFICATION: Add two distinct planning functions ***
	void GeneratePatrolPlan(PatrollingInput* input, Solution* sol_final);
	void SolveTaskAssignment(PatrollingInput* input, const std::vector<Node>& tasks, Solution* sol_final);

	void solverTSP_LKH(std::vector<TSPVertex>& lst, std::vector<TSPVertex>& result, double multiplier);

	Pathfinder* mPathfinder;

private:
	KMeansSolver mKMeansSolver;
	VRPSolver mVRPSolver;
};

#endif // SOLVER_H