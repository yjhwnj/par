#ifndef SOLVER_OPTLAUNCH_H
#define SOLVER_OPTLAUNCH_H

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "LaunchOptimizer.h"
#include "gurobi_c++.h"

#define DEBUG_OPTLAUNCH	DEBUG || 0


class OptLaunchSolver : public Solver {
public:
	OptLaunchSolver(Pathfinder* pathfinder);
	~OptLaunchSolver();

	// *** MODIFICATION: Update to new Solve interface ***
	void Solve(PatrollingInput* input, SimulationState* sim_state) override;

protected:
private:
	LaunchOptimizer optimizer;
};

#endif // SOLVER_OPTLAUNCH_H