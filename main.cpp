#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>
#include <list>
#include <cstdlib>
#include <cmath>
#include <algorithm>

#include "defines.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver.h"
#include "Solver_Baseline.h"
#include "Solver_ILO.h"
#include "Solver_OptLaunch.h"
#include "Pathfinder.h"
#include "SimulationState.h"
#include "Utilities.h"   // for distAtoB

#define DEBUG_MAIN               (DEBUG || 0)
#define DEFAULT_PRINT_ACTIONS    false
#define DEFAULT_PRINT_RESULTS    0
#define DATA_LOG_FORMAT          "alg_%d.dat"
#define DEFAULT_DATA_LOG_PATH    ""
#define DEFAULT_RUN_NUM          0
#define DEFAULT_VEHICLE_DEFINE_LOCATION "../../VehicleInputs/StandardDefinitionVehicle.yaml"

static double getEnvDouble(const char* key, double fallback) {
    if (const char* v = std::getenv(key)) {
        try { return std::stod(std::string(v)); }
        catch (...) { return fallback; }
    }
    return fallback;
}

int main(int argc, char* argv[]) {
    const char* inputPath;
    int algorithm;
    bool print_actions;
    int runnum;
    const char* vehiclePath;

    if (argc >= 3) {
        inputPath = argv[1];
        algorithm = atoi(argv[2]);
        print_actions = (argc > 3) ? atoi(argv[3]) : DEFAULT_PRINT_ACTIONS;
        runnum = (argc > 4) ? atoi(argv[4]) : DEFAULT_RUN_NUM;
        vehiclePath = (argc > 5) ? argv[5] : DEFAULT_VEHICLE_DEFINE_LOCATION;
        if (argc < 6) vehiclePath = DEFAULT_VEHICLE_DEFINE_LOCATION;
        if (argc < 5) runnum = DEFAULT_RUN_NUM;
    }
    else {
        printf("Received %d args, expected 2 or more.\nExpected use:\t./find-assignment <file path> <algorithm> [print actions] [run number] [vehicle path]\n\n", argc - 1);
        return 1;
    }

    srand(runnum);

    PatrollingInput input(inputPath, vehiclePath);

    Pathfinder pathfinder;
    pathfinder.initialize(&input);

    SimulationState sim_state(&input, &pathfinder);

    // 选择算法
    Solver* solver = NULL;
    switch (algorithm) {
    case e_Algo_GREEDY:   solver = new BaselineSolver(&pathfinder); break;
    case e_Algo_OPTLAUNCH:solver = new OptLaunchSolver(&pathfinder); break;
    case e_Algo_ILO:      solver = new Solver_ILO(&pathfinder); break;
    default:
        fprintf(stderr, "[ERROR][main] : \n\tInvalid algorithm identifier!\n");
        exit(1);
    }

    printf("--- Initial Planning ---\n");
    solver->Solve(&input, &sim_state);
    printf("--- Initial Planning Complete ---\n\n");

    // === 仿真配置（保持你现有行为，可从环境变量覆写） ===
    double detection_radius = getEnvDouble("DETECTION_RADIUS", 500.0); // 500 不改
    double simulation_duration = getEnvDouble("SIM_DURATION", 10000.0);
    double time_step = getEnvDouble("TIME_STEP", 10.0);

    printf("[SIM CONFIG] detection_radius=%.1f, simulation_duration=%.1f, time_step=%.1f\n",
        detection_radius, simulation_duration, time_step);

    // 隐藏点池（静态 ground truth，用于逐步“被发现”）
    std::list<Node> hidden_nodes_remaining(input.GetHiddenNodes().begin(), input.GetHiddenNodes().end());

    // UAV 位置与动作索引（用于发现判定）
    std::vector<Point> agent_positions(input.GetMa());
    std::vector<size_t> next_action_idx(input.GetMa(), 0);
    for (int i = 0; i < input.GetMa(); ++i) {
        double start_x, start_y;
        input.GetDroneInitLocal(i, &start_x, &start_y);
        agent_positions[i] = { start_x, start_y };
    }

    printf("--- Starting Simulation ---\n");
    bool needs_replan = false;

    while (sim_state.current_time < simulation_duration) {
        sim_state.current_time += time_step;
        printf("Simulation time: %.1f / %.1f seconds\n", sim_state.current_time, simulation_duration);

        // ===== 更新 UAV 位置（沿当前计划段做线性插值）=====
        for (int i = 0; i < input.GetMa(); ++i) {
            std::vector<DroneAction> drone_plan;
            sim_state.current_plan.GetDroneActionList(i, drone_plan);
            if (drone_plan.empty()) continue;

            while (next_action_idx[i] < drone_plan.size()
                && sim_state.current_time >= drone_plan[next_action_idx[i]].fCompletionTime) {
                next_action_idx[i]++;
            }

            if (next_action_idx[i] < drone_plan.size()) {
                size_t current_idx = next_action_idx[i];

                Point start_pos;
                double start_time;
                if (current_idx == 0) {
                    input.GetDroneInitLocal(i, &start_pos.x, &start_pos.y);
                    start_time = 0.0;
                }
                else {
                    start_pos = { drone_plan[current_idx - 1].fX, drone_plan[current_idx - 1].fY };
                    start_time = drone_plan[current_idx - 1].fCompletionTime;
                }

                Point end_pos = { drone_plan[current_idx].fX, drone_plan[current_idx].fY };
                double end_time = drone_plan[current_idx].fCompletionTime;
                double segment_duration = end_time - start_time;

                if (segment_duration > EPSILON) {
                    double progress = (sim_state.current_time - start_time) / segment_duration;
                    if (progress > 1.0) progress = 1.0;
                    agent_positions[i].x = start_pos.x + (end_pos.x - start_pos.x) * progress;
                    agent_positions[i].y = start_pos.y + (end_pos.y - start_pos.y) * progress;
                }
                else {
                    agent_positions[i] = end_pos;
                }
            }
            else {
                if (!drone_plan.empty()) {
                    agent_positions[i] = { drone_plan.back().fX, drone_plan.back().fY };
                }
            }
        }

        // ===== 在线发现：UAV 在 detection_radius 内发现隐藏点 =====
        auto it_hidden = hidden_nodes_remaining.begin();
        while (it_hidden != hidden_nodes_remaining.end()) {
            bool discovered = false;
            for (int i = 0; i < input.GetMa(); ++i) {
                double dist = distAtoB(agent_positions[i].x, agent_positions[i].y,
                    it_hidden->location.x, it_hidden->location.y);
                if (dist <= detection_radius) {
                    printf("  >>> DISCOVERY! UAV %d discovered hidden node %s at time %.1f <<<\n",
                        i, it_hidden->ID.c_str(), sim_state.current_time);
                    sim_state.discovered_tasks.push_back(*it_hidden);
                    it_hidden = hidden_nodes_remaining.erase(it_hidden);
                    discovered = true;
                    needs_replan = true;
                    break;
                }
            }
            if (!discovered) ++it_hidden;
        }

        // =====（新增）UGV 完成判定：当 UGV 抵达动作点时，消除对应任务 =====
        // 做法：扫描所有 UGV 的动作；对 type==e_MoveToNode 且完成时间<=当前时间的动作，
        //       在 discovered_tasks 中按坐标匹配（容差），匹配到则视为已处置 -> 从 discovered_tasks 移除并打印。
        {
            const int Mg = input.GetMg();
            const double XY_TOL = 1.0;   // 坐标匹配容差（可按需要调大/调小）
            for (int g = 0; g < Mg; ++g) {
                std::vector<UGVAction> ugv_actions;
                sim_state.current_plan.GetUGVActionList(g, ugv_actions);
                if (ugv_actions.empty()) continue;

                for (const auto& act : ugv_actions) {
                    if (act.mActionType != E_UGVActionTypes::e_MoveToNode) continue;
                    if (act.fCompletionTime > sim_state.current_time + 1e-6) continue;

                    // 与未完成任务列表匹配（按坐标近似）
                    bool serviced_any = false;
                    for (auto it = sim_state.discovered_tasks.begin(); it != sim_state.discovered_tasks.end();) {
                        double dxy = distAtoB(act.fX, act.fY, it->location.x, it->location.y);
                        if (dxy <= XY_TOL) {
                            printf("  >>> SERVICED! UGV %d serviced node %s at time %.1f <<<\n",
                                g, it->ID.c_str(), sim_state.current_time);
                            it = sim_state.discovered_tasks.erase(it);
                            serviced_any = true;
                        }
                        else {
                            ++it;
                        }
                    }
                    // 不触发重规划；完成后下一轮若无新发现，UGV 继续按现有表走
                    (void)serviced_any;
                }
            }
        }

        // ===== 若有新任务发现 -> 触发滚动重规划 =====
        if (needs_replan) {
            printf("\n--- !!! New Tasks Discovered, Triggering Re-planning !!! ---\n");
            solver->Solve(&input, &sim_state);
            printf("--- Re-planning Complete. New plan generated. ---\n\n");

            // 重置 UAV 动作游标
            for (size_t i = 0; i < next_action_idx.size(); ++i) next_action_idx[i] = 0;
            needs_replan = false;
        }
    }

    printf("--- Simulation Finished ---\n\n");

    if (print_actions) {
        sim_state.current_plan.GenerateYAML("output_plan_initial.yaml");
        printf("Initial plan has been saved to 'output_plan_initial.yaml'\n");
    }

    delete solver;
    printf("Done!\n");
    return 0;
}
