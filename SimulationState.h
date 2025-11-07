#ifndef SIMULATION_STATE_H
#define SIMULATION_STATE_H

#include "Solution.h"
#include "PatrollingInput.h"
#include <vector> // °üº¬ vector

/*
 * SimulationState.h
 * ...
 */

class SimulationState {
public:
    double current_time;
    Solution current_plan;

    // *** MODIFICATION HERE: Add list for discovered tasks ***
    std::vector<Node> discovered_tasks;

    SimulationState(PatrollingInput* input, Pathfinder* pathfinder)
        : current_time(0.0), current_plan(input, pathfinder) {
        // Initially, the discovered tasks are the ones visible from the start
        const auto& initial_nodes = input->GetNodes();
        discovered_tasks.assign(initial_nodes.begin(), initial_nodes.end());
    }
};

#endif // SIMULATION_STATE_H