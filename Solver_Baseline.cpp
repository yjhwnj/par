#include "Solver_Baseline.h"

BaselineSolver::BaselineSolver(Pathfinder* pathfinder) : Solver(pathfinder) {
    if (SANITY_PRINT)
        printf("Hello from Baseline Solver!\n");
}

// *** MODIFICATION HERE: Update function signature and logic ***
void BaselineSolver::Solve(PatrollingInput* input, SimulationState* sim_state) {
    if (SANITY_PRINT)
        printf("\nStarting Baseline solver\n\n");

    // This solver now acts as a dispatcher based on discovered tasks.
    if (sim_state->discovered_tasks.empty()) {
        // No tasks known, generate patrol plan
        GeneratePatrolPlan(input, &sim_state->current_plan);
    }
    else {
        // Tasks have been discovered, run the task assignment
        SolveTaskAssignment(input, sim_state->discovered_tasks, &sim_state->current_plan);
    }
}