#include "Solver.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>

namespace {

double getEnvDouble(const char* key, double fallback) {
    if (const char* v = std::getenv(key)) {
        try { return std::stod(std::string(v)); }
        catch (...) { return fallback; }
    }
    return fallback;
}

double safePathDistance(Pathfinder* pathfinder, Point start, Point end) {
    if (!pathfinder) {
        return distAtoB(start.x, start.y, end.x, end.y);
    }
    double dist = pathfinder->get_path_distance(start, end);
    if (!(dist < INF)) {
        dist = distAtoB(start.x, start.y, end.x, end.y);
    }
    return dist;
}

} // namespace

Solver::Solver(Pathfinder* pathfinder) {
    mPathfinder = pathfinder;
}

Solver::~Solver() {}

void Solver::GeneratePatrolPlan(PatrollingInput* input, Solution* sol_final) {
    if (DEBUG_SOLVER) {
        printf("\n--- Generating adaptive coverage patrol ---\n");
    }

    sol_final->ClearSolution();

    const int ugv_count = input->GetMg();
    std::vector<std::vector<int>> ugv_to_drones;
    input->AssignDronesToUGV(ugv_to_drones);
    std::vector<int> drone_home(input->GetMa(), 0);
    for (int g = 0; g < static_cast<int>(ugv_to_drones.size()); ++g) {
        for (int drone_id : ugv_to_drones[g]) {
            if (drone_id >= 0 && drone_id < input->GetMa()) {
                drone_home[drone_id] = g;
            }
        }
    }

    for (int g = 0; g < ugv_count; ++g) {
        double depot_x, depot_y;
        input->GetDepot(g, &depot_x, &depot_y);
        sol_final->PushUGVAction(g, { E_UGVActionTypes::e_AtDepot, depot_x, depot_y, 0.0 });
    }

    const BoundingBox bounds = input->GetMapBounds();
    const double grid_res = std::max(10.0, getEnvDouble("COVERAGE_GRID", 400.0));
    const double obstacle_buffer = std::max(0.0, getEnvDouble("COVERAGE_OBS_BUFFER", grid_res * 0.25));

    const double width = std::max(grid_res, bounds.max_x - bounds.min_x);
    const double height = std::max(grid_res, bounds.max_y - bounds.min_y);
    const int num_cols = std::max(1, static_cast<int>(std::ceil(width / grid_res)));
    const int num_rows = std::max(1, static_cast<int>(std::ceil(height / grid_res)));
    const double col_step = width / num_cols;
    const double row_step = height / num_rows;

    std::vector<std::vector<Point>> column_cells(num_cols);
    for (int c = 0; c < num_cols; ++c) {
        double cx = bounds.min_x + col_step * (c + 0.5);
        cx = std::clamp(cx, bounds.min_x, bounds.max_x);
        for (int r = 0; r < num_rows; ++r) {
            double cy = bounds.min_y + row_step * (r + 0.5);
            cy = std::clamp(cy, bounds.min_y, bounds.max_y);
            if (input->IsLocationInObstacle(cx, cy, obstacle_buffer)) {
                continue;
            }
            column_cells[c].push_back({ cx, cy });
        }
        std::sort(column_cells[c].begin(), column_cells[c].end(),
            [](const Point& a, const Point& b) { return a.y < b.y; });
    }

    const int drone_count = input->GetMa();
    std::vector<double> drone_finish_times(drone_count, 0.0);

    if (drone_count == 0) {
        for (int g = 0; g < ugv_count; ++g) {
            double depot_x, depot_y;
            input->GetDepot(g, &depot_x, &depot_y);
            sol_final->PushUGVAction(g, { E_UGVActionTypes::e_ReceiveDrone, depot_x, depot_y, 0.0 });
            sol_final->PushUGVAction(g, { E_UGVActionTypes::e_KernelEnd, depot_x, depot_y, 0.0 });
        }
        return;
    }

    const int cols_per_drone = std::max(1, static_cast<int>(std::ceil(static_cast<double>(num_cols) /
        std::max(1, drone_count))));

    for (int j = 0; j < drone_count; ++j) {
        int start_col = j * cols_per_drone;
        int end_col = (j == drone_count - 1) ? num_cols : std::min(num_cols, start_col + cols_per_drone);
        if (start_col >= num_cols) {
            start_col = num_cols;
            end_col = num_cols;
        }

        std::vector<Point> sweep_points;
        bool go_up = true;
        for (int c = start_col; c < end_col; ++c) {
            auto cells = column_cells[c];
            if (cells.empty()) {
                continue;
            }
            if (!go_up) {
                std::reverse(cells.begin(), cells.end());
            }
            sweep_points.insert(sweep_points.end(), cells.begin(), cells.end());
            go_up = !go_up;
        }

        int home_ugv = (j < static_cast<int>(drone_home.size())) ? drone_home[j] : 0;
        if (home_ugv < 0 || home_ugv >= ugv_count) {
            home_ugv = 0;
        }

        double depot_x, depot_y;
        input->GetDepot(home_ugv, &depot_x, &depot_y);
        sol_final->PushDroneAction(j, { E_DroneActionTypes::e_AtUGV, depot_x, depot_y, 0.0 });

        if (sweep_points.empty()) {
            sol_final->PushDroneAction(j, { E_DroneActionTypes::e_KernelEnd, depot_x, depot_y, 0.0 });
            continue;
        }

        UAV uav = input->getUAV(j);
        const double speed = std::max(1.0, input->GetDroneVMax(j));
        double current_time = 0.0;
        Point current_pos{ depot_x, depot_y };

        current_time += uav.timeNeededToLaunch;
        sol_final->PushDroneAction(j, { E_DroneActionTypes::e_LaunchFromUGV, depot_x, depot_y, current_time });

        for (const auto& target : sweep_points) {
            double dist = safePathDistance(mPathfinder, current_pos, target);
            if (!(dist < INF)) {
                if (DEBUG_SOLVER) {
                    printf("[WARN] UAV %d cannot reach coverage point (%.1f, %.1f). Skipping.\n",
                        j, target.x, target.y);
                }
                continue;
            }
            current_time += dist / speed;
            sol_final->PushDroneAction(j, { E_DroneActionTypes::e_MoveToNode, target.x, target.y, current_time, -1 });
            current_pos = target;
        }

        double dist_home = safePathDistance(mPathfinder, current_pos, { depot_x, depot_y });
        if (dist_home < INF) {
            current_time += dist_home / speed;
            sol_final->PushDroneAction(j, { E_DroneActionTypes::e_MoveToUGV, depot_x, depot_y, current_time });
        }
        current_time += uav.timeNeededToLand;
        sol_final->PushDroneAction(j, { E_DroneActionTypes::e_LandOnUGV, depot_x, depot_y, current_time });
        sol_final->PushDroneAction(j, { E_DroneActionTypes::e_KernelEnd, depot_x, depot_y, current_time });

        drone_finish_times[j] = current_time;
    }

    double max_finish = 0.0;
    for (double t : drone_finish_times) {
        if (t > max_finish) {
            max_finish = t;
        }
    }

    for (int g = 0; g < ugv_count; ++g) {
        double depot_x, depot_y;
        input->GetDepot(g, &depot_x, &depot_y);
        sol_final->PushUGVAction(g, { E_UGVActionTypes::e_ReceiveDrone, depot_x, depot_y, max_finish });
        sol_final->PushUGVAction(g, { E_UGVActionTypes::e_KernelEnd, depot_x, depot_y, max_finish });
    }

    if (DEBUG_SOLVER) {
        printf("--- Coverage patrol generated with %d drones over %d x %d grid ---\n",
            drone_count, num_cols, num_rows);
    }
}

void Solver::SolveTaskAssignment(PatrollingInput* input, const std::vector<Node>& tasks, Solution* sol_final) {
    if (DEBUG_SOLVER) {
        printf("\n--- Solving task assignment for %zu tasks ---\n", tasks.size());
    }

    GeneratePatrolPlan(input, sol_final);
    if (tasks.empty()) {
        return;
    }

    const int ugv_count = input->GetMg();
    if (ugv_count <= 0) {
        if (DEBUG_SOLVER) {
            printf("[WARN] No UGVs available to service tasks.\n");
        }
        return;
    }

    double latest_drone_time = 0.0;
    for (int j = 0; j < input->GetMa(); ++j) {
        std::vector<DroneAction> actions;
        sol_final->GetDroneActionList(j, actions);
        if (!actions.empty()) {
            latest_drone_time = std::max(latest_drone_time, actions.back().fCompletionTime);
        }
    }

    std::vector<std::vector<int>> ugv_task_indices(ugv_count);
    std::vector<Point> ugv_positions(ugv_count);
    for (int g = 0; g < ugv_count; ++g) {
        double sx, sy;
        input->GetUGVInitLocal(g, &sx, &sy);
        ugv_positions[g] = { sx, sy };
    }

    for (size_t idx = 0; idx < tasks.size(); ++idx) {
        const Node& task = tasks[idx];
        Point task_pos{ task.location.x, task.location.y };
        int best_g = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (int g = 0; g < ugv_count; ++g) {
            double dist = safePathDistance(mPathfinder, ugv_positions[g], task_pos);
            if (dist < best_dist) {
                best_dist = dist;
                best_g = g;
            }
        }
        ugv_task_indices[best_g].push_back(static_cast<int>(idx));
        ugv_positions[best_g] = task_pos;
    }

    const double service_time = std::max(0.0, getEnvDouble("UGV_SERVICE_TIME", 120.0));

    for (int g = 0; g < ugv_count; ++g) {
        sol_final->ClearUGVSolution(g);
        double start_x, start_y;
        input->GetUGVInitLocal(g, &start_x, &start_y);
        sol_final->PushUGVAction(g, { E_UGVActionTypes::e_AtDepot, start_x, start_y, 0.0 });

        double current_time = 0.0;
        Point current_pos{ start_x, start_y };
        double speed = input->getUGV(g).maxDriveSpeed;
        if (speed <= 1e-6) {
            speed = 5.0;
        }

        for (int task_idx : ugv_task_indices[g]) {
            if (task_idx < 0 || task_idx >= static_cast<int>(tasks.size())) {
                continue;
            }
            const Node& node = tasks[task_idx];
            Point dst{ node.location.x, node.location.y };
            double dist = safePathDistance(mPathfinder, current_pos, dst);
            if (!(dist < INF)) {
                if (DEBUG_SOLVER) {
                    printf("[WARN] UGV %d cannot reach task %s. Skipping.\n", g, node.ID.c_str());
                }
                continue;
            }
            current_time += dist / speed;
            sol_final->PushUGVAction(g, { E_UGVActionTypes::e_MoveToNode, dst.x, dst.y, current_time, task_idx });
            current_time += service_time;
            current_pos = dst;
        }

        double dist_home = safePathDistance(mPathfinder, current_pos, { start_x, start_y });
        if (dist_home < INF) {
            current_time += dist_home / speed;
            sol_final->PushUGVAction(g, { E_UGVActionTypes::e_MoveToDepot, start_x, start_y, current_time });
        }

        current_time = std::max(current_time, latest_drone_time);
        sol_final->PushUGVAction(g, { E_UGVActionTypes::e_ReceiveDrone, start_x, start_y, current_time });
        sol_final->PushUGVAction(g, { E_UGVActionTypes::e_KernelEnd, start_x, start_y, current_time });
    }

    if (DEBUG_SOLVER) {
        printf("--- UGV assignments generated with horizon %.1f seconds ---\n", latest_drone_time);
    }
}

void Solver::solverTSP_LKH(std::vector<TSPVertex>& lst, std::vector<TSPVertex>& result, double multiplier) {
    if (lst.empty()) return;
    int depot_id = lst.front().nID;
    int terminal_id = lst.back().nID;
    int depot_index = 0;
    int terminal_index = boost::numeric_cast<int>(lst.size() - 1);

    FILE* pParFile = fopen("FixedHPP.par", "w");
    fprintf(pParFile, "PROBLEM_FILE = FixedHPP.tsp\n");
    fprintf(pParFile, "TOUR_FILE = LKH_output.dat\n");
    fprintf(pParFile, "TRACE_LEVEL = 0\n");
    fclose(pParFile);

    FILE* pDataFile = fopen("FixedHPP.tsp", "w");
    fprintf(pDataFile, "NAME : FixedHPP \nTYPE : TSP \nDIMENSION : %ld \n", lst.size());
    fprintf(pDataFile, "EDGE_WEIGHT_TYPE : EXPLICIT \nEDGE_WEIGHT_FORMAT : FULL_MATRIX\n");
    fprintf(pDataFile, "EDGE_WEIGHT_SECTION\n");
    for (const auto& v : lst) {
        for (const auto& u : lst) {
            double dist = mPathfinder->get_path_distance({ v.x, v.y }, { u.x, u.y });
            if ((v.nID == depot_id && u.nID == terminal_id) || (v.nID == terminal_id && u.nID == depot_id)) {
                fprintf(pDataFile, "%f\t", 0.0);
            }
            else if ((v.nID == depot_id) || (u.nID == depot_id) || (u.nID == terminal_id) || (v.nID == terminal_id)) {
                fprintf(pDataFile, "%f\t", dist * multiplier);
            }
            else {
                fprintf(pDataFile, "%.5f\t", dist);
            }
        }
        fprintf(pDataFile, "\n");
    }
    fprintf(pDataFile, "EOF\n");
    fclose(pDataFile);

    int sys_output = std::system("LKH FixedHPP.par > trash.out");
    (void)sys_output;

    std::ifstream file("LKH_output.dat");
    if (!file.is_open()) return;

    std::string line;
    for (int i = 0; i < 6; i++) std::getline(file, line);

    std::list<int> totalPath;
    for (size_t i = 0; i < lst.size(); i++) {
        std::getline(file, line);
        std::stringstream lineStreamN(line);
        int n;
        lineStreamN >> n;
        totalPath.push_back(n - 1);
    }
    file.close();

    if (!totalPath.empty() && !((totalPath.front() == depot_index) && (totalPath.back() == terminal_index))) {
        if ((totalPath.front() == terminal_index) && (totalPath.back() == depot_index)) {
            totalPath.reverse();
        }
        else {
            bool reverseList = true;
            for (auto it = totalPath.begin(); it != totalPath.end() && *it != depot_index; ++it) {
                if (*it == terminal_index) reverseList = false;
            }
            while (totalPath.front() != depot_index) {
                totalPath.push_back(totalPath.front());
                totalPath.pop_front();
            }
            if (reverseList) {
                totalPath.push_back(totalPath.front());
                totalPath.pop_front();
                totalPath.reverse();
            }
        }
    }

    if (totalPath.empty() || totalPath.front() != depot_index || totalPath.back() != terminal_index) {
        fprintf(stderr, "[ERROR] : Solver::solverTSP_LKH() : totalPath order is not as expected\n");
        result.clear();
        return;
    }

    result.clear();
    for (int i : totalPath) result.push_back(lst.at(i));
}
