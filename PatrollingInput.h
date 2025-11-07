/*
 * MASInput.h
 * ...
 */

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <boost/numeric/conversion/cast.hpp>
#include <yaml-cpp/yaml.h>

#include "Input.h"
#include "Roots.h"
#include "UGV.h"
#include "UAV.h"
#include "Agent.h"


#define DEBUG_PATROLINPUT	DEBUG || 0


 // *** MODIFICATION HERE: Add status to Node ***
struct Node {
        std::string ID;
        std::string type;
        Location location;
        double time_last_service;
        bool is_hidden;
};

struct BoundingBox {
        double min_x;
        double max_x;
        double min_y;
        double max_y;
};

using Obstacle = std::vector<Location>;


class PatrollingInput : public Input {
public:
	PatrollingInput(std::string scenario_input, std::string vehicle_input);
	virtual ~PatrollingInput();

	/// Getters
	int GetN() { return nodes.size(); }
	int GetMa() { return mRa.size(); }
	int GetMg() { return mRg.size(); }

	const std::vector<Node>& GetNodes() const { return nodes; }

	// *** MODIFICATION HERE: Add getter for hidden nodes ***
        const std::vector<Node>& GetHiddenNodes() const { return hidden_nodes; }

        const BoundingBox& GetMapBounds() const { return map_bounds; }

        const std::vector<Obstacle>& GetObstacles() const { return obstacles; }

        bool IsLocationInObstacle(double x, double y, double buffer = 0.0) const;

	// ... (rest of the getters remain the same) ...
	void GetDepot(int j, double* x, double* y);
	void GetDroneInitLocal(int j, double* x, double* y);
	void GetUGVInitLocal(int j, double* x, double* y);
	std::string GetNodeID(int i) { return nodes.at(i).ID; }
	std::string GetDroneID(int j) { return mRa.at(j).ID; }
	std::string GetUGVID(int j) { return mRg.at(j).ID; }
	double GetDroneBatCap(int j) { return mRa.at(j).battery_state.max_battery_energy; }
	double GetUGVBatCap(int j) { return mRg.at(j).battery_state.max_battery_energy; }
	double GetDroneMaxDist(int j);
	double GetUGVMaxDist(int j);
	double calcChargeTime(int drone_j, double J);
	double GetTMax(int drone_j);
	double GetDroneVMax(int drone_j);
	void AssignDronesToUGV(std::vector<std::vector<int>>& drones_to_UGV);
	UAV getUAV(int ID) { return mRa.at(ID); }
	UGV getUGV(int ID) { return mRg.at(ID); }
	double LowerBound();

private:
        std::vector<UAV> mRa;
        std::vector<UGV> mRg;

        std::vector<Node> nodes; // This will now only store VISIBLE nodes

        // *** MODIFICATION HERE: Add storage for hidden nodes ***
        std::vector<Node> hidden_nodes;

        BoundingBox map_bounds;
        bool bounds_initialized = false;

        std::vector<Obstacle> obstacles;
        double depot_x;
        double depot_y;
        void parseAgents(const YAML::Node& agents);
        void parseScenario(const YAML::Node& scenario);
        void parseUAVs(const YAML::Node& UAVs);
        void parseUGVs(const YAML::Node& UGVs);
        void updateBounds(double x, double y);
        void GenerateRandomHiddenTasks(size_t additional_count);
};