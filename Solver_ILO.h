#ifndef SOLVER_ILO_H
#define SOLVER_ILO_H

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "LaunchOptimizer.h"
#include "gurobi_c++.h"

#define DEBUG_ILO	DEBUG || 0


class Solver_ILO : public Solver {
public:
	Solver_ILO(Pathfinder* pathfinder);
	~Solver_ILO();

	// *** MODIFICATION: Update to new Solve interface ***
	void Solve(PatrollingInput* input, SimulationState* sim_state) override;

protected:
private:
	LaunchOptimizer optimizer;
	bool updateSubtours(int drone_id, Solution* sol_final);
};

#endif // SOLVER_ILO_H