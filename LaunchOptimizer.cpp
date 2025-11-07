#include "LaunchOptimizer.h"
#include "UAV.h"
#include "UGV.h"


LaunchOptimizer::LaunchOptimizer() { }

LaunchOptimizer::~LaunchOptimizer() { }


void LaunchOptimizer::OptLaunching(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_final, Pathfinder* pathfinder) {
	if (DEBUG_LAUNCHOPT)
		printf("Optimizing UGV %d route\n", ugv_num);

	std::vector<Node> vctrPOINodes = input->GetNodes();
	std::vector<std::vector<DroneAction>> drone_action_lists;
	std::vector<int> drone_action_lists_i;
	for (int j_a = 0; j_a < input->GetMa(); j_a++) {
		std::vector<DroneAction> action_list_j;
		sol_final->GetDroneActionList(j_a, action_list_j);
		drone_action_lists.push_back(action_list_j);
		drone_action_lists_i.push_back(1);
	}

	std::vector<UGVAction> ugv_final_actions;
	std::vector<std::vector<DroneAction>> drone_final_actions_j;
	for (int j_a = 0; j_a < input->GetMa(); j_a++) {
		std::vector<DroneAction> drone_actions;
		drone_final_actions_j.push_back(drone_actions);
	}

	std::map<int, std::queue<DroneAction>> old_action_queues;
	for (int drone_j : drones_on_UGV) {
		std::vector<DroneAction> action_list;
		sol_final->GetDroneActionList(drone_j, action_list);
		std::queue<DroneAction, std::deque<DroneAction>> action_queue(std::deque<DroneAction>(action_list.begin(), action_list.end()));
		old_action_queues.insert(std::pair<int, std::queue<DroneAction>>(drone_j, action_queue));
	}

	double ugv_tour_start = 0.0;
	bool process_next_team_tour = true;
	do {
		try {
			bool has_valid_tours = false;
			for (int drone_id : drones_on_UGV) {
				bool has_launch = false, has_land = false;
				for (size_t i = drone_action_lists_i.at(drone_id); i < drone_action_lists.at(drone_id).size(); ++i) {
					if (drone_action_lists.at(drone_id).at(i).mActionType == E_DroneActionTypes::e_LaunchFromUGV) has_launch = true;
					if (drone_action_lists.at(drone_id).at(i).mActionType == E_DroneActionTypes::e_LandOnUGV) has_land = true;
				}
				if (has_launch && has_land) {
					has_valid_tours = true;
					break;
				}
			}

			if (!has_valid_tours) {
				if (DEBUG_LAUNCHOPT) printf("No valid drone tours found for this optimization round. Skipping.\n");
				process_next_team_tour = false;
				break;
			}

			GRBEnv env = GRBEnv();
			if (!DEBUG_LAUNCHOPT) env.set(GRB_INT_PAR_OUTPUTFLAG, "0");
			GRBModel model = GRBModel(env);

			std::priority_queue<SOCAction, std::vector<SOCAction>, CompareSOCAction> action_queue;
			std::vector<SubTour> sub_tours;
			int subtour_counter = 0;

			for (int drone_id : drones_on_UGV) {
				int start_idx = -1;
				for (size_t i = drone_action_lists_i.at(drone_id); i < drone_action_lists.at(drone_id).size(); ++i) {
					DroneAction current_action = drone_action_lists.at(drone_id).at(i);
					if (current_action.mActionType == E_DroneActionTypes::e_LaunchFromUGV) start_idx = i;
					else if (current_action.mActionType == E_DroneActionTypes::e_LandOnUGV && start_idx != -1) {
						DroneAction launch_action = drone_action_lists.at(drone_id).at(start_idx);
						double tour_dist = 0.0, prev_x = 0.0, prev_y = 0.0, strt_x = 0.0, strt_y = 0.0;
						bool first_node = true;
						for (size_t j = start_idx + 1; j < i; ++j) {
							DroneAction inner_action = drone_action_lists.at(drone_id).at(j);
							if (inner_action.mActionType == E_DroneActionTypes::e_MoveToNode) {
								if (first_node) {
									strt_x = prev_x = inner_action.fX;
									strt_y = prev_y = inner_action.fY;
									first_node = false;
								}
								else {
									tour_dist += pathfinder->get_path_distance({ prev_x, prev_y }, { inner_action.fX, inner_action.fY });
									prev_x = inner_action.fX;
									prev_y = inner_action.fY;
								}
							}
						}
						if (first_node) {
							strt_x = launch_action.fX; strt_y = launch_action.fY;
							prev_x = current_action.fX; prev_y = current_action.fY;
						}
						sub_tours.emplace_back(tour_dist, subtour_counter, strt_x, strt_y, prev_x, prev_y);
						action_queue.emplace(launch_action.fCompletionTime, drone_id, E_SOCActionType::e_LaunchDrone, subtour_counter);
						action_queue.emplace(current_action.fCompletionTime, drone_id, E_SOCActionType::e_ReceiveDrone, subtour_counter);
						subtour_counter++;
						start_idx = -1;
					}
					else if (current_action.mActionType == E_DroneActionTypes::e_AtUGV || current_action.mActionType == E_DroneActionTypes::e_KernelEnd) {
						start_idx = -1;
					}
				}
				drone_action_lists_i.at(drone_id) = drone_action_lists.at(drone_id).size();
			}

			if (sub_tours.empty()) {
				if (DEBUG_LAUNCHOPT) printf("No valid drone tours found for this optimization round after detailed parsing. Skipping.\n");
				process_next_team_tour = false;
				break;
			}

			std::vector<std::vector<GRBVar>> sub_tour_dist_vars(subtour_counter);
			std::vector<std::vector<GRBVar>> sub_tour_pos_vars(subtour_counter);
			for (int i = 0; i < subtour_counter; ++i) {
				sub_tour_dist_vars[i] = { model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_s_" + itos(i)), model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_e_" + itos(i)) };
				sub_tour_pos_vars[i] = { model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_s_" + itos(i)), model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_s_" + itos(i)), model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_e_" + itos(i)), model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_e_" + itos(i)) };
			}

			std::vector<std::vector<GRBVar>> action_coord_vars;
			std::vector<GRBVar> action_time_vars;
			std::vector<SOCAction> ordered_action_list;
			int action_cout = 0;
			{
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);
				action_coord_vars.push_back({ model.addVar(x_b, x_b, 0.0, GRB_CONTINUOUS, "x_0"), model.addVar(y_b, y_b, 0.0, GRB_CONTINUOUS, "y_0") });
				action_time_vars.push_back(model.addVar(0.0, 0.0, 0.0, GRB_CONTINUOUS, "t_0"));
				ordered_action_list.emplace_back(0.0, -1, E_SOCActionType::e_BaseStation, -1);
				action_cout++;
			}
			std::vector<int> prev_action_id(input->GetMa(), -1);
			std::vector<int> first_action_id(input->GetMa(), -1);
			while (!action_queue.empty()) {
				SOCAction action = action_queue.top();
				action_queue.pop();
				ordered_action_list.push_back(action);
				action_coord_vars.push_back({ model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_" + itos(action_cout)), model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_" + itos(action_cout)) });
				GRBVar t_i = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t_" + itos(action_cout));
				if (action.action_type == E_SOCActionType::e_LaunchDrone) {
					if (prev_action_id.at(action.ID) >= 0) {
						model.addConstr(t_i >= action_time_vars.at(prev_action_id.at(action.ID)) + input->GetTMax(DRONE_I));
					}
					else {
						first_action_id.at(action.ID) = action_cout;
					}
					sub_tours.at(action.subtour_index).launch_ID = action_cout;
				}
				else if (action.action_type == E_SOCActionType::e_ReceiveDrone) {
					prev_action_id.at(action.ID) = action_cout;
					sub_tours.at(action.subtour_index).land_ID = action_cout;
				}
				action_time_vars.push_back(t_i);
				action_cout++;
			}
			{
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);
				action_coord_vars.push_back({ model.addVar(x_b, x_b, 0.0, GRB_CONTINUOUS, "x_" + itos(action_cout)), model.addVar(y_b, y_b, 0.0, GRB_CONTINUOUS, "y_" + itos(action_cout)) });
				ordered_action_list.emplace_back(0.0, -1, E_SOCActionType::e_BaseStation, -1);
			}
			GRBVar t_base = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "t_" + itos(action_cout));
			action_time_vars.push_back(t_base);
			for (size_t i = 0, j = 1; j < action_coord_vars.size(); j++, i++) {
				GRBVar d_j = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_" + std::to_string(i) + "_" + std::to_string(j));
				model.addQConstr(d_j * d_j >= (action_coord_vars[i][0] - action_coord_vars[j][0]) * (action_coord_vars[i][0] - action_coord_vars[j][0]) + (action_coord_vars[i][1] - action_coord_vars[j][1]) * (action_coord_vars[i][1] - action_coord_vars[j][1]));
				double action_time = 0.0;
				if (ordered_action_list.at(j).action_type == E_SOCActionType::e_LaunchDrone) action_time = input->getUAV(ordered_action_list.at(j).ID).timeNeededToLaunch;
				else if (ordered_action_list.at(j).action_type == E_SOCActionType::e_ReceiveDrone) action_time = input->getUAV(ordered_action_list.at(j).ID).timeNeededToLand;
				model.addConstr(action_time_vars.at(j) >= action_time_vars.at(i) + d_j / input->getUGV(ugv_num).ugv_v_crg + action_time);
			}
			for (size_t tour = 0; tour < sub_tours.size(); tour++) {
				int i = sub_tours.at(tour).launch_ID;
				int j = sub_tours.at(tour).land_ID;
				if (i < 0 || j < 0) continue;
				UAV uav = input->getUAV(ordered_action_list.at(j).ID);
				model.addConstr(action_time_vars.at(j) == action_time_vars.at(i) + sub_tour_dist_vars[tour][0] / input->GetDroneVMax(DRONE_I) + sub_tours[tour].tour_dist / input->GetDroneVMax(DRONE_I) + sub_tour_dist_vars[tour][1] / input->GetDroneVMax(DRONE_I) + uav.timeNeededToLand);
				model.addQConstr(sub_tour_dist_vars[tour][0] * sub_tour_dist_vars[tour][0] >= (action_coord_vars[i][0] - sub_tour_pos_vars[tour][0]) * (action_coord_vars[i][0] - sub_tour_pos_vars[tour][0]) + (action_coord_vars[i][1] - sub_tour_pos_vars[tour][1]) * (action_coord_vars[i][1] - sub_tour_pos_vars[tour][1]));
				model.addQConstr(sub_tour_dist_vars[tour][1] * sub_tour_dist_vars[tour][1] >= (action_coord_vars[j][0] - sub_tour_pos_vars[tour][2]) * (action_coord_vars[j][0] - sub_tour_pos_vars[tour][2]) + (action_coord_vars[j][1] - sub_tour_pos_vars[tour][3]) * (action_coord_vars[j][1] - sub_tour_pos_vars[tour][3]));
				model.addConstr(input->GetDroneMaxDist(DRONE_I) >= sub_tour_dist_vars[tour][0] + sub_tours[tour].tour_dist + sub_tour_dist_vars[tour][1]);
			}

			// *** MODIFICATION HERE: Explicitly create GRBLinExpr ***
			GRBLinExpr obj = t_base;
			model.setObjective(obj, GRB_MINIMIZE);
			model.optimize();

			double ugv_x_i, ugv_y_i, ugv_t_i;
			std::map<int, std::vector<double>> drone_pos_x_y_t;
			double x_b, y_b;
			input->GetDepot(ugv_num, &x_b, &y_b);
			ugv_final_actions.emplace_back(E_UGVActionTypes::e_AtDepot, x_b, y_b, ugv_tour_start);
			ugv_x_i = x_b; ugv_y_i = y_b; ugv_t_i = ugv_tour_start;
			for (int drone_j : drones_on_UGV) drone_pos_x_y_t[drone_j] = { x_b, y_b, ugv_tour_start };
			for (size_t a_i = 1; a_i < ordered_action_list.size() - 1; a_i++) {
				SOCAction action_i = ordered_action_list.at(a_i);
				double x_j = action_coord_vars[a_i][0].get(GRB_DoubleAttr_X);
				double y_j = action_coord_vars[a_i][1].get(GRB_DoubleAttr_X);
				double dist_i_j = pathfinder->get_path_distance({ ugv_x_i, ugv_y_i }, { x_j, y_j });
				double t_j = ugv_t_i + dist_i_j / input->getUGV(ugv_num).ugv_v_crg;
				ugv_final_actions.emplace_back(E_UGVActionTypes::e_MoveToWaypoint, x_j, y_j, t_j);
				ugv_x_i = x_j; ugv_y_i = y_j; ugv_t_i = t_j;
				int drone_id = action_i.ID;
				UAV uav = input->getUAV(drone_id);
				if (action_i.action_type == E_SOCActionType::e_LaunchDrone) {
					t_j = ugv_t_i + uav.timeNeededToLaunch;
					ugv_final_actions.emplace_back(E_UGVActionTypes::e_LaunchDrone, ugv_x_i, ugv_y_i, t_j, drone_id);
					drone_final_actions_j.at(drone_id).emplace_back(E_DroneActionTypes::e_LaunchFromUGV, ugv_x_i, ugv_y_i, t_j, ugv_num);
					ugv_t_i = t_j;
					double drone_x_i = ugv_x_i, drone_y_i = ugv_y_i, drone_t_i = ugv_t_i;
					while (!old_action_queues.at(drone_id).empty() && old_action_queues.at(drone_id).front().mActionType != E_DroneActionTypes::e_LandOnUGV) {
						DroneAction next_action = old_action_queues.at(drone_id).front();
						if (next_action.mActionType == E_DroneActionTypes::e_MoveToNode) {
							double x_j_node = vctrPOINodes.at(next_action.mDetails).location.x;
							double y_j_node = vctrPOINodes.at(next_action.mDetails).location.y;
							double dist = pathfinder->get_path_distance({ drone_x_i, drone_y_i }, { x_j_node, y_j_node });
							double t_j_node = drone_t_i + dist / input->GetDroneVMax(DRONE_I);
							drone_final_actions_j.at(drone_id).emplace_back(E_DroneActionTypes::e_MoveToNode, x_j_node, y_j_node, t_j_node, next_action.mDetails);
							drone_pos_x_y_t.at(drone_id) = { x_j_node, y_j_node, t_j_node };
							drone_x_i = x_j_node; drone_y_i = y_j_node; drone_t_i = t_j_node;
						}
						old_action_queues.at(drone_id).pop();
					}
					if (!old_action_queues.at(drone_id).empty()) old_action_queues.at(drone_id).pop();
				}
				else if (action_i.action_type == E_SOCActionType::e_ReceiveDrone) {
					double dist_drone_j = pathfinder->get_path_distance({ drone_pos_x_y_t.at(drone_id)[0], drone_pos_x_y_t.at(drone_id)[1] }, { x_j, y_j });
					double drone_arrives = std::max(t_j, drone_pos_x_y_t.at(drone_id)[2] + dist_drone_j / input->GetDroneVMax(DRONE_I));
					drone_final_actions_j.at(drone_id).emplace_back(E_DroneActionTypes::e_MoveToUGV, ugv_x_i, ugv_y_i, drone_arrives, ugv_num);
					t_j = drone_arrives + uav.timeNeededToLand;
					ugv_final_actions.emplace_back(E_UGVActionTypes::e_ReceiveDrone, ugv_x_i, ugv_y_i, t_j, drone_id);
					drone_final_actions_j.at(drone_id).emplace_back(E_DroneActionTypes::e_LandOnUGV, ugv_x_i, ugv_y_i, t_j, ugv_num);
					ugv_t_i = t_j;
				}
			}
			double dist_i_b = pathfinder->get_path_distance({ ugv_x_i, ugv_y_i }, { x_b, y_b });
			double t_b = ugv_t_i + dist_i_b / input->getUGV(ugv_num).ugv_v_crg;
			ugv_final_actions.emplace_back(E_UGVActionTypes::e_MoveToDepot, x_b, y_b, t_b);
			double kernel_complete_time = t_b + input->getUGV(ugv_num).batterySwapTime;
			ugv_final_actions.emplace_back(E_UGVActionTypes::e_AtDepot, x_b, y_b, kernel_complete_time);
			for (int drone : drones_on_UGV) drone_final_actions_j.at(drone).emplace_back(E_DroneActionTypes::e_AtUGV, x_b, y_b, kernel_complete_time);
			ugv_tour_start = kernel_complete_time;
		}
		catch (GRBException& e) {
			printf("[ERROR] %d: %s\n", e.getErrorCode(), e.getMessage().c_str());
		}
		catch (const std::exception& e) {
			printf("Exception during optimization: %s\n", e.what());
		}

		process_next_team_tour = false;
		for (int drone_j : drones_on_UGV) {
			if (drone_action_lists_i.at(drone_j) < boost::numeric_cast<int>(drone_action_lists.at(drone_j).size()) - 1) {
				process_next_team_tour = true;
			}
		}

	} while (process_next_team_tour);

	sol_final->ClearUGVSolution(ugv_num);
	for (int drone_j : drones_on_UGV) {
		sol_final->ClearDroneSolution(drone_j);
	}
	for (UGVAction a : ugv_final_actions) {
		sol_final->PushUGVAction(ugv_num, a);
	}
	if (!ugv_final_actions.empty()) {
		UGVAction last_action = sol_final->GetLastUGVAction(ugv_num);
		sol_final->PushUGVAction(ugv_num, { E_UGVActionTypes::e_KernelEnd, last_action.fX, last_action.fY, last_action.fCompletionTime });
	}
	for (int drone_id : drones_on_UGV) {
		double x_b, y_b;
		input->GetDepot(ugv_num, &x_b, &y_b);
		sol_final->PushDroneAction(drone_id, { E_DroneActionTypes::e_AtUGV, x_b, y_b, 0.0, ugv_num });
		for (DroneAction a : drone_final_actions_j.at(drone_id)) {
			sol_final->PushDroneAction(drone_id, a);
		}
		if (!drone_final_actions_j.at(drone_id).empty()) {
			auto last_action = sol_final->GetLastDroneAction(drone_id);
			sol_final->PushDroneAction(drone_id, { E_DroneActionTypes::e_KernelEnd, last_action.fX, last_action.fY, last_action.fCompletionTime });
		}
	}
}