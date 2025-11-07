#include "Solver.h"

Solver::Solver(Pathfinder* pathfinder) {
    mPathfinder = pathfinder;
}

Solver::~Solver() {}

/**
 * 生成“左右分区 + S 形全覆盖”的巡航路线
 * 约定：
 * - 地图边界：[-5000, 5000] × [-5000, 5000]
 * - 竖直分割线 x = 0；UAV_0 负责左半区（[-5000,0]），UAV_1 负责右半区（[0,5000]）
 * - 其他 UAV 若存在：继续按左右交替分区（偶数编号左、奇数编号右）
 * - 每个 UAV 在自己分区内按 S 形往返扫描；默认条带宽度 stripe_w = 700（可用环境变量 STRIPE_W 覆盖）
 * - UAV 从 UGV_0 的 depot 起降；用 mPathfinder 计算路径长度，时间 = 距离 / 速度
 */
void Solver::GeneratePatrolPlan(PatrollingInput* input, Solution* sol_final) {
    if (DEBUG_SOLVER)
        printf("\n--- Generating Patrol Routes (Split map + S-sweep) ---\n");

    sol_final->ClearSolution();

    // 1) UGV 固定在库位：保持原有风格
    for (int j = 0; j < input->GetMg(); ++j) {
        double depot_x, depot_y;
        input->GetDepot(j, &depot_x, &depot_y);
        sol_final->PushUGVAction(j, { E_UGVActionTypes::e_AtDepot, depot_x, depot_y, 0.0 });
        sol_final->PushUGVAction(j, { E_UGVActionTypes::e_KernelEnd, depot_x, depot_y, 3600.0 });
    }

    // 2) 地图与扫描参数
    const double minX = -5000.0, maxX = 5000.0;
    const double minY = -5000.0, maxY = 5000.0;
    const double splitX = 0.0; // 左右分界线

    // ---- 新增：从环境变量读取条带宽（默认 700），不改变其余逻辑 ----
    double stripe_w = 700.0;
    if (const char* env = std::getenv("STRIPE_W")) {
        char* endp = nullptr;
        double v = std::strtod(env, &endp);
        if (endp != env && v > 1.0) stripe_w = v;
    }

    auto build_sweep_waypoints = [&](double xL, double xR, double yB, double yT) {
        // 生成从左到右（或右到左）按条带摆动的 S 形路径
        std::vector<Point> wps;
        bool up = true; // 第一条从下(yB)扫到上(yT)
        // 计算条带中心线（沿 X 方向步进）
        double width = std::max(1.0, std::fabs(xR - xL));
        int cols = std::max(1, (int)std::ceil(width / stripe_w));
        double dx = (xR - xL) / cols;

        double x = xL + dx * 0.5; // 第一条带的“中心”x
        for (int c = 0; c < cols; ++c, x += dx) {
            if (up) {
                wps.push_back({ x, yB });
                wps.push_back({ x, yT });
            }
            else {
                wps.push_back({ x, yT });
                wps.push_back({ x, yB });
            }
            up = !up;
        }
        return wps;
    };

    // 3) 为每个 UAV 设定分区并生成路径
    //    偶数 idx（0,2,4...）→ 左区；奇数 idx（1,3,5...）→ 右区
    for (int j_a = 0; j_a < input->GetMa(); ++j_a) {
        // 起降点（用 UGV_0 depot）
        double depot_x, depot_y;
        input->GetDepot(0, &depot_x, &depot_y);

        // 分区
        bool left_side = (j_a % 2 == 0);
        double xL = left_side ? minX : splitX;
        double xR = left_side ? splitX : maxX;
        double yB = minY, yT = maxY;

        // 生成 S 形路径点
        std::vector<Point> waypoints = build_sweep_waypoints(xL, xR, yB, yT);

        // 推动作（时间用距离/速度推进）
        UAV uav = input->getUAV(j_a);
        double current_time = 0.0;
        Point  current_pos = { depot_x, depot_y };

        // 起始：在车上
        sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_AtUGV, depot_x, depot_y, 0.0 });

        // 起飞
        current_time += uav.timeNeededToLaunch;
        sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_LaunchFromUGV, depot_x, depot_y, current_time });

        // 从库位飞到本 UAV 分区的第一个点（若存在）
        if (!waypoints.empty()) {
            double dist0 = mPathfinder->get_path_distance(current_pos, waypoints.front());
            if (dist0 < INF) {
                current_time += dist0 / input->GetDroneVMax(j_a);
                sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_MoveToNode, waypoints.front().x, waypoints.front().y, current_time, -1 });
                current_pos = waypoints.front();
            }
            else if (DEBUG_SOLVER) {
                printf("Warning: UAV %d cannot reach its zone start point.\n", j_a);
            }
        }

        // 扫描各条带
        for (size_t k = 1; k < waypoints.size(); ++k) {
            const Point& nxt = waypoints[k];
            double dist = mPathfinder->get_path_distance(current_pos, nxt);
            if (dist >= INF) {
                if (DEBUG_SOLVER) printf("Warning: path blocked for UAV %d between sweeps.\n", j_a);
                continue;
            }
            current_time += dist / input->GetDroneVMax(j_a);
            sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_MoveToNode, nxt.x, nxt.y, current_time, -1 });
            current_pos = nxt;
        }

        // 回库并降落
        double dist_back = mPathfinder->get_path_distance(current_pos, { depot_x, depot_y });
        if (dist_back < INF) {
            current_time += dist_back / input->GetDroneVMax(j_a);
            sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_MoveToUGV, depot_x, depot_y, current_time });
            current_time += uav.timeNeededToLand;
            sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_LandOnUGV, depot_x, depot_y, current_time });
        }
        // 结尾
        sol_final->PushDroneAction(j_a, { E_DroneActionTypes::e_KernelEnd, depot_x, depot_y, current_time });
    }

    if (DEBUG_SOLVER) {
        printf("\n--- Generated Patrol Plan (S-sweep, stripe_w=%.1f) ---\n", stripe_w);
        sol_final->PrintSolution();
    }
}


// *** 原“RunBaseline”保留为任务分派入口（未动） ***
void Solver::SolveTaskAssignment(PatrollingInput* input, const std::vector<Node>& tasks, Solution* sol_final) {
    if (DEBUG_SOLVER) printf("\n--- Solving Task Assignment for %zu tasks ---\n", tasks.size());

    std::vector<ClusteringPoint> vctrPoIPoint;
    for (size_t i = 0; i < tasks.size(); ++i) {
        vctrPoIPoint.emplace_back(i, tasks[i].location.x, tasks[i].location.y, 0);
    }

    std::vector<VRPPoint> depots;
    std::vector<std::vector<int>> depot_order;
    std::vector<std::vector<std::vector<int>>> depots_tours;

    int K = 1;
    int Mp = input->GetMg();

    bool valid_solution = false;
    while (!valid_solution) {
        depots.clear();
        depot_order.clear();
        depots_tours.clear();
        sol_final->ClearSolution();
        valid_solution = true;

        K = std::max(Mp, K);
        if (K > boost::numeric_cast<int>(tasks.size())) {
            K = tasks.size();
        }
        if (K == 0) break; // No tasks to assign

        for (int k = 0; k < K; k++) {
            depots_tours.emplace_back();
        }

        std::vector<std::vector<ClusteringPoint>> clusters;
        if (K > 1) {
            mKMeansSolver.SolveKMeans(vctrPoIPoint, K);
            clusters.resize(K);
            for (const auto& n : vctrPoIPoint) {
                clusters[n.cluster].push_back(n);
            }
        }
        else {
            clusters.push_back(vctrPoIPoint);
        }

        for (int i = 0; i < K; i++) {
            double x = 0, y = 0;
            for (const auto& n : clusters.at(i)) {
                x += n.x;
                y += n.y;
            }
            if (!clusters.at(i).empty()) {
                depots.emplace_back(i, x / clusters.at(i).size(), y / clusters.at(i).size());
            }
            else {
                depots.emplace_back(i, 0, 0);
            }
        }

        double depot_x, depot_y;
        input->GetDepot(0, &depot_x, &depot_y);
        depots.emplace_back(-1, depot_x, depot_y);

        std::vector<std::vector<int>> drones_to_UGV;
        input->AssignDronesToUGV(drones_to_UGV);

        mVRPSolver.SolveVRP(depots, Mp, depot_order, mPathfinder);

        for (int j_p = 0; j_p < Mp && j_p < boost::numeric_cast<int>(depot_order.size()); j_p++) {
            int j_actual = j_p % input->GetMg();
            for (int k : depot_order.at(j_p)) {
                std::vector<VRPPoint> nodes;
                for (const auto& n : clusters.at(k)) {
                    nodes.emplace_back(n.ID, n.x, n.y);
                }
                nodes.emplace_back(k, depots.at(k).x, depots.at(k).y);

                std::vector<std::vector<int>> subtours;
                mVRPSolver.SolveVRP(nodes, boost::numeric_cast<int>(drones_to_UGV.at(j_actual).size()), subtours, mPathfinder);
                depots_tours.at(k) = subtours;

                for (const auto& subtour : subtours) {
                    if (!subtour.empty()) {
                        double tour_length = mPathfinder->get_path_distance({ depots.at(k).x, depots.at(k).y }, { tasks.at(subtour.front()).location.x, tasks.at(subtour.front()).location.y });
                        for (size_t i = 0; i < subtour.size() - 1; ++i) {
                            tour_length += mPathfinder->get_path_distance({ tasks.at(subtour[i]).location.x, tasks.at(subtour[i]).location.y }, { tasks.at(subtour[i + 1]).location.x, tasks.at(subtour[i + 1]).location.y });
                        }
                        tour_length += mPathfinder->get_path_distance({ tasks.at(subtour.back()).location.x, tasks.at(subtour.back()).location.y }, { depots.at(k).x, depots.at(k).y });

                        if (tour_length > input->GetDroneMaxDist(DRONE_I)) {
                            K++;
                            valid_solution = false;
                            break;
                        }
                    }
                }
                if (!valid_solution) break;
            }
            if (!valid_solution) break;
        }
    }

    printf("Task assignment logic is complex. For now, we will skip generating the detailed action list.\n");
    printf("The important part is that the logic was triggered.\n");

    // 为保持仿真可跑，仍生成巡航（此处会再次调用上面的 S 扫描）
    GeneratePatrolPlan(input, sol_final);
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
