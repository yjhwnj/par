#include "Solver_ILO.h"
#include <cstdlib>
#include <stdexcept>
#include <limits>
#include <fstream>
#include <sstream>
#include <iomanip>

// 说明：本文件保留“UGV 任务分派 + 导出 YAML”能力；
// 巡航路径仍调用原有 GeneratePatrolPlan（不改 UAV 动作）。
// 之后要做“左右分区 + S 形”巡航，请在 Solver 基类里实现 GeneratePatrolPlan 的蛇形版本。

Solver_ILO::Solver_ILO(Pathfinder* pathfinder)
    : Solver(pathfinder) {}

Solver_ILO::~Solver_ILO() {}

// ---------------- UGV replanning 导出 ----------------
static void dumpUGVPlanYAML(PatrollingInput* input,
    SimulationState* sim_state,
    const std::string& tag_prefix = "output_plan_replan_ugv_")
{
    const int Mg = input->GetMg();

    // 用最近一次重规划触发时刻（四舍五入秒）做文件后缀
    const double t_now = sim_state ? sim_state->current_time : 0.0;
    const int t_sec = static_cast<int>(std::round(t_now));
    char fname[256];
    std::snprintf(fname, sizeof(fname), "%st%04d.yaml", tag_prefix.c_str(), t_sec);
    std::string out_file(fname);

    YAML::Emitter out;
    out << YAML::Comment("API Version: 0.9.1");
    out << YAML::BeginMap;
    out << YAML::Key << "ID" << YAML::Value << "plan_every_UAV_action_02";
    out << YAML::Key << "state_ID" << YAML::Value << "state_every_UAV_action_02";
    out << YAML::Key << "description" << YAML::Value << "UGV Assignment Only";
    out << YAML::Key << "start_time" << YAML::Value << 0.0;
    out << YAML::Key << "end_time" << YAML::Value << 3600.0;

    out << YAML::Key << "individual_plans" << YAML::Value << YAML::BeginSeq;

    for (int g = 0; g < Mg; ++g)
    {
        std::vector<UGVAction> acts;
        sim_state->current_plan.GetUGVActionList(g, acts);

        out << YAML::BeginMap;
        char agid[32];
        std::snprintf(agid, sizeof(agid), "UGV_%02d", g + 1);
        out << YAML::Key << "agent_ID" << YAML::Value << agid;
        out << YAML::Key << "actions" << YAML::Value << YAML::BeginSeq;

        {
            out << YAML::BeginMap;
            out << YAML::Key << "type" << YAML::Value << "start";
            out << YAML::Key << "start_time" << YAML::Value << 0.0;
            out << YAML::Key << "end_time" << YAML::Value << 0.0;
            out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;

            double sx = 0.0, sy = 0.0;
            input->GetUGVInitLocal(g, &sx, &sy);
            out << YAML::Key << "location" << YAML::Value
                << YAML::BeginMap
                << YAML::Key << "x" << YAML::Value << sx
                << YAML::Key << "y" << YAML::Value << sy
                << YAML::EndMap;

            out << YAML::EndMap;
            out << YAML::EndMap;
        }

        double last_x = 0.0, last_y = 0.0;
        input->GetUGVInitLocal(g, &last_x, &last_y);

        for (size_t k = 0; k < acts.size(); ++k)
        {
            const UGVAction& a = acts[k];
            if (a.mActionType != E_UGVActionTypes::e_MoveToNode) {
                continue;
            }

            double dx = a.fX;
            double dy = a.fY;

            double seg_end = a.fCompletionTime;
            if (seg_end <= 0.0) {
                const double dist = distAtoB(last_x, last_y, dx, dy); // 用 Utilities.h 的声明/实现
                const double v = 5.0; // 若有 input->getUGV(g).maxDriveSpeed，可替换为真实速度
                seg_end = (sim_state ? sim_state->current_time : 0.0)
                    + (v > 1e-9 ? dist / v : 0.0);
            }

            out << YAML::BeginMap;
            out << YAML::Key << "type" << YAML::Value << "move_to_location";
            out << YAML::Key << "start_time" << YAML::Value << (sim_state ? sim_state->current_time : 0.0);
            out << YAML::Key << "end_time" << YAML::Value << seg_end;
            out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;

            out << YAML::Key << "origin" << YAML::Value
                << YAML::BeginMap
                << YAML::Key << "x" << YAML::Value << last_x
                << YAML::Key << "y" << YAML::Value << last_y
                << YAML::EndMap;

            out << YAML::Key << "destination" << YAML::Value
                << YAML::BeginMap
                << YAML::Key << "x" << YAML::Value << dx
                << YAML::Key << "y" << YAML::Value << dy
                << YAML::EndMap;

            out << YAML::EndMap;
            out << YAML::EndMap;

            last_x = dx; last_y = dy;
        }

        {
            out << YAML::BeginMap;
            out << YAML::Key << "type" << YAML::Value << "end";
            out << YAML::Key << "start_time" << YAML::Value << 3600.0;
            out << YAML::Key << "end_time" << YAML::Value << 3600.0;
            out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;

            out << YAML::Key << "location" << YAML::Value
                << YAML::BeginMap
                << YAML::Key << "x" << YAML::Value << last_x
                << YAML::Key << "y" << YAML::Value << last_y
                << YAML::EndMap;

            out << YAML::EndMap;
            out << YAML::EndMap;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;
    }

    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(out_file);
    fout << out.c_str();
    fout.close();

    std::printf("[DEBUG] UGV replanned YAML written: %s\n", out_file.c_str());
}

// ---------------- 主入口：保持与之前一致（不改 UAV 动作） ----------------
void Solver_ILO::Solve(PatrollingInput* input, SimulationState* sim_state) {
    if (SANITY_PRINT) {
        printf("\n[ILO] Start Solve at t=%.1f, discovered=%zu\n",
            sim_state->current_time, sim_state->discovered_tasks.size());
    }

    // 无任务：调用原有巡逻生成（此处不再尝试直接插入 UAV 动作）
    if (sim_state->discovered_tasks.empty()) {
        GeneratePatrolPlan(input, &sim_state->current_plan);
        if (DEBUG_ILO) printf("[ILO] No discovered tasks -> Patrol plan only.\n");
        return;
    }

    // 有任务：先做任务分派（保持与原版兼容）
    SolveTaskAssignment(input, sim_state->discovered_tasks, &sim_state->current_plan);

    bool skip_gurobi = false;
    if (const char* env = std::getenv("NO_GUROBI_OPT")) {
        if (std::string(env) == "1") skip_gurobi = true;
    }

    auto getUGVCurrentXY = [&](int ugv_idx, double& cx, double& cy) {
        std::vector<UGVAction> acts;
        sim_state->current_plan.GetUGVActionList(ugv_idx, acts);
        if (!acts.empty()) {
            cx = acts.back().fX;
            cy = acts.back().fY;
        }
        else {
            input->GetUGVInitLocal(ugv_idx, &cx, &cy);
        }
    };

    auto getUGVCurrentT = [&](int ugv_idx) -> double {
        return sim_state->current_plan.GetTotalTourTime(ugv_idx);
    };

    auto appendMoveToNodeForUGV = [&](int ugv_idx, double dst_x, double dst_y) {
        double cx, cy;
        getUGVCurrentXY(ugv_idx, cx, cy);
        double t0 = getUGVCurrentT(ugv_idx);

        double dist = 0.0;
        if (mPathfinder) {
            dist = mPathfinder->get_path_distance({ cx, cy }, { dst_x, dst_y });
        }
        else {
            dist = distAtoB(cx, cy, dst_x, dst_y);
        }

        // 尽量使用真实速度；没有则降级为 5 m/s
        double v = 5.0;
        if (ugv_idx >= 0 && ugv_idx < input->GetMg()) {
            UGV ugv = input->getUGV(ugv_idx);
            if (ugv.maxDriveSpeed > 1e-6) v = ugv.maxDriveSpeed;
        }

        const double dt = (v > 1e-9) ? (dist / v) : 0.0;

        UGVAction mv(E_UGVActionTypes::e_MoveToNode, dst_x, dst_y, t0 + dt, -1);
        sim_state->current_plan.PushUGVAction(ugv_idx, mv);

        if (DEBUG_ILO) {
            printf("[ILO] UGV_%02d move: (%.1f,%.1f)->(%.1f,%.1f), dist=%.1f, v=%.2f, dt=%.1f, end_t=%.1f\n",
                ugv_idx, cx, cy, dst_x, dst_y, dist, v, dt, t0 + dt);
        }
    };

    // ====== 新：用 Fast-VRP 对每辆 UGV 的任务顺序进行求解 ======
    auto planWithFastVRP = [&]() {
        printf("[INFO] ILO(FastVRP): building per-UGV tours using VRPSolver::SolveFastVRP.\n");

        const int M = input->GetMg();
        if (M <= 0) {
            fprintf(stderr, "[WARN] No UGV available; cannot service tasks.\n");
            return;
        }

        // 1) 读取每个 UGV 的“当前点”（动作序列末尾，或初始化位置）
        std::vector<std::pair<double, double>> ugv_pos(M);
        for (int g = 0; g < M; ++g) {
            double x0, y0;
            getUGVCurrentXY(g, x0, y0);
            ugv_pos[g] = { x0, y0 };
        }

        // 2) 先用最近距离把任务分给各 UGV（简单稳健；下一步可替换为 k-means）
        std::vector<std::vector<int>> tasks_per_ugv(M);
        for (int idx = 0; idx < (int)sim_state->discovered_tasks.size(); ++idx) {
            const Node& task = sim_state->discovered_tasks[idx];
            double tx = task.location.x, ty = task.location.y;

            int best_g = 0;
            double best_d = std::numeric_limits<double>::infinity();
            for (int g = 0; g < M; ++g) {
                const double d = (mPathfinder)
                    ? mPathfinder->get_path_distance({ ugv_pos[g].first, ugv_pos[g].second }, { tx, ty })
                    : distAtoB(ugv_pos[g].first, ugv_pos[g].second, tx, ty);
                if (d < best_d) { best_d = d; best_g = g; }
            }
            tasks_per_ugv[best_g].push_back(idx);
        }

        // 3) 对每辆 UGV：构造 VRPPoint 列表（最后一个是该 UGV 当前点作为“depot”），用 FastVRP 求巡回顺序
        VRPSolver vrp;
        for (int g = 0; g < M; ++g) {
            if (tasks_per_ugv[g].empty()) continue;

            std::vector<VRPPoint> nodes;
            nodes.reserve(tasks_per_ugv[g].size() + 1);

            // 任务点：ID 用“在 discovered_tasks 向量中的下标”，确保能反查坐标
            for (int global_idx : tasks_per_ugv[g]) {
                const Node& t = sim_state->discovered_tasks[global_idx];
                nodes.emplace_back(global_idx, t.location.x, t.location.y);
            }

            // depot：该 UGV 当前点，ID 用 -1（SolveFastVRP 约定最后一个为 depot）
            nodes.emplace_back(-1, ugv_pos[g].first, ugv_pos[g].second);

            std::vector<std::vector<int>> tours;
            // 这里对单辆车求解，num_vehicles=1
            vrp.SolveFastVRP(nodes, 1, tours, mPathfinder);

            if (tours.empty()) continue;
            const std::vector<int>& order = tours.front(); // 该 UGV 的到访顺序（元素是 global_idx）

            if (DEBUG_ILO) {
                printf("[ILO][UGV_%02d] VRP order:", g);
                for (int id : order) printf(" %d", id);
                printf("\n");
            }

            // 4) 把顺序写回 UGV 动作：依次 MoveToNode
            for (int global_idx : order) {
                if (global_idx < 0 || global_idx >= (int)sim_state->discovered_tasks.size()) continue;
                const Node& t = sim_state->discovered_tasks[global_idx];
                appendMoveToNodeForUGV(g, t.location.x, t.location.y);
                ugv_pos[g] = { t.location.x, t.location.y };
            }
        }

        // 5) 打印动作并导出 YAML（沿用你原有打印）
        const int Mg = input->GetMg();
        for (int g = 0; g < Mg; ++g) {
            std::vector<UGVAction> acts;
            sim_state->current_plan.GetUGVActionList(g, acts);
            printf("[DEBUG] UGV_%02d has %zu actions after replanning.\n", g, acts.size());
            for (size_t i = 0; i < acts.size(); ++i) {
                const auto& a = acts[i];
                printf("        #%zu: type=%d, (x=%.1f,y=%.1f), end_t=%.1f, details=%d\n",
                    i, (int)a.mActionType, a.fX, a.fY, a.fCompletionTime, a.mDetails);
            }
        }
        dumpUGVPlanYAML(input, sim_state, "output_plan_replan_ugv_");
    };

    if (skip_gurobi) {
        // 直接走 Fast-VRP（不依赖 Gurobi）
        planWithFastVRP();
        return;
    }

    try {
        // 这里仍然先用 Fast-VRP（稳态可用）；后续你需要时，我们再把 Gurobi 的 SOC 上去
        planWithFastVRP();
    }
    catch (GRBException& e) {
        fprintf(stderr, "[ERROR] Gurobi exception %d: %s\n", e.getErrorCode(), e.getMessage().c_str());
        planWithFastVRP();
    }
    catch (std::exception& e) {
        fprintf(stderr, "[ERROR] Exception during ILO optimization: %s\n", e.what());
        planWithFastVRP();
    }
    catch (...) {
        fprintf(stderr, "[ERROR] Unknown error during ILO optimization.\n");
        planWithFastVRP();
    }
}
