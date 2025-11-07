#include "Solver_OptLaunch.h"
#include <cstdlib>
#include <cstring>

static bool env_disable_gurobi() {
    const char* v = std::getenv("NO_GUROBI_OPT");
    if (!v) return false;
    if (std::strcmp(v, "1") == 0) return true;
    if (std::strcmp(v, "true") == 0) return true;
    if (std::strcmp(v, "TRUE") == 0) return true;
    if (std::strcmp(v, "True") == 0) return true;
    return false;
}

static void dump_plan_yaml(const char* tag, double sim_time, Solution& plan) {
    int t = static_cast<int>(sim_time + 0.5);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "output_plan_%s_t%04d.yaml", tag, t);
    plan.GenerateYAML(std::string(buf));
    printf("[DUMP] Plan exported -> %s\n", buf);
}

OptLaunchSolver::OptLaunchSolver(Pathfinder* pathfinder) : Solver(pathfinder) {
    if (SANITY_PRINT)
        printf("Hello from Opt-Launch Solver!\n");
}

OptLaunchSolver::~OptLaunchSolver() { }

// *** MODIFICATION HERE: Update function signature and logic ***
void OptLaunchSolver::Solve(PatrollingInput* input, SimulationState* sim_state) {
    if (SANITY_PRINT)
        printf("\nStarting OptLaunch solver\n\n");

    if (sim_state->discovered_tasks.empty()) {
        GeneratePatrolPlan(input, &sim_state->current_plan);
        printf("NOTE: No tasks discovered. Generated patrol plan.\n");
        dump_plan_yaml("patrol", sim_state->current_time, sim_state->current_plan);
        return;
    }

    printf("NOTE: Tasks discovered. Running assignment and Launch Optimization.\n");
    printf("      Discovered tasks: %zu\n", sim_state->discovered_tasks.size());

    // 1. Run the task assignment logic first
    SolveTaskAssignment(input, sim_state->discovered_tasks, &sim_state->current_plan);

    // 如果禁用 Gurobi，则不进入 Launch 优化
    if (env_disable_gurobi()) {
        printf("[INFO] OptLaunch: NO_GUROBI_OPT is set -> skip launch optimization. Using assignment-only plan.\n");
        dump_plan_yaml("replan", sim_state->current_time, sim_state->current_plan);
        return;
    }

    // 2. Then, run the launch optimizer on the result
    std::vector<std::vector<int>> drones_to_UGV;
    input->AssignDronesToUGV(drones_to_UGV);

    try {
        for (int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
            optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, &sim_state->current_plan, mPathfinder);
        }
    }
    catch (GRBException& e) {
        fprintf(stderr,
            "[WARN] OptLaunch: Gurobi exception caught (code=%d, msg=%s). "
            "Skipping launch optimization and using assignment-only plan.\n",
            e.getErrorCode(), e.getMessage().c_str());
    }
    catch (const std::exception& e) {
        fprintf(stderr,
            "[WARN] OptLaunch: std::exception caught in launch optimization (%s). "
            "Skipping launch optimization and using assignment-only plan.\n",
            e.what());
    }
    catch (...) {
        fprintf(stderr,
            "[WARN] OptLaunch: Unknown exception in launch optimization. "
            "Skipping launch optimization and using assignment-only plan.\n");
    }

    dump_plan_yaml("replan", sim_state->current_time, sim_state->current_plan);
}
