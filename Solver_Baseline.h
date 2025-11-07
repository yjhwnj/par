#ifndef SOLVER_BASELINE_H
#define SOLVER_BASELINE_H

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"

#define DEBUG_GREEDY	DEBUG || 0


class BaselineSolver : public Solver {
public:
	BaselineSolver(Pathfinder* pathfinder);

	// *** MODIFICATION: Update to new Solve interface ***
	void Solve(PatrollingInput* input, SimulationState* sim_state) override;

protected:
private:
};

#endif // SOLVER_BASELINE_H