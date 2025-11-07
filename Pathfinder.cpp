#include "Pathfinder.h"
#include <iostream>
#include <cmath>
#include <queue>
#include <map>
#include <algorithm>

#include "defines.h"

/*
 * Pathfinder.cpp
 *
 * Created by: Gemini AI Assistant (based on user request)
 * On:         [Current Date]
 *
 * Description: Implementation of the Pathfinder class.
 */


Pathfinder::Pathfinder() {
    m_initialized = false;
    m_grid_resolution = 50.0;
}

Pathfinder::~Pathfinder() {}

void Pathfinder::initialize(const PatrollingInput* input) {
    if (m_initialized) return;

    m_map_min_x = -5000; m_map_max_x = 5000;
    m_map_min_y = -5000; m_map_max_y = 5000;

    for (const auto& node : input->GetNodes()) {
        if (node.location.x < m_map_min_x) m_map_min_x = node.location.x;
        if (node.location.x > m_map_max_x) m_map_max_x = node.location.x;
        if (node.location.y < m_map_min_y) m_map_min_y = node.location.y;
        if (node.location.y > m_map_max_y) m_map_max_y = node.location.y;
    }
    m_map_min_x -= 200; m_map_max_x += 200;
    m_map_min_y -= 200; m_map_max_y += 200;

    m_grid_width = static_cast<int>(std::ceil((m_map_max_x - m_map_min_x) / m_grid_resolution));
    m_grid_height = static_cast<int>(std::ceil((m_map_max_y - m_map_min_y) / m_grid_resolution));

    m_grid_map.assign(m_grid_height, std::vector<int>(m_grid_width, 0));

    rasterize_obstacles(input->GetObstacles());

    m_initialized = true;

    std::cout << "Pathfinder initialized. Map bounds: [("
        << m_map_min_x << ", " << m_map_min_y << "), ("
        << m_map_max_x << ", " << m_map_max_y << ")]. Grid size: "
        << m_grid_width << "x" << m_grid_height << std::endl;
}

double Pathfinder::get_path_distance(Point start, Point end) {
    if (!m_initialized) {
        std::cerr << "ERROR: Pathfinder not initialized!" << std::endl;
        return distAtoB(start.x, start.y, end.x, end.y);
    }

    Point start_grid = world_to_grid(start);
    Point end_grid = world_to_grid(end);
    if (m_grid_map[static_cast<int>(start_grid.y)][static_cast<int>(start_grid.x)] == 1 ||
        m_grid_map[static_cast<int>(end_grid.y)][static_cast<int>(end_grid.x)] == 1) {
        return INF;
    }

    Path path = find_path_astar(start, end);
    if (path.empty()) {
        return INF;
    }

    double distance = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        distance += distAtoB(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
    }
    return distance;
}

Path Pathfinder::get_path(Point start, Point end) {
    if (!m_initialized) {
        std::cerr << "ERROR: Pathfinder not initialized!" << std::endl;
        return { start, end };
    }
    return find_path_astar(start, end);
}

Point Pathfinder::world_to_grid(Point world_pos) {
    int grid_x = static_cast<int>((world_pos.x - m_map_min_x) / m_grid_resolution);
    int grid_y = static_cast<int>((world_pos.y - m_map_min_y) / m_grid_resolution);
    if (grid_x < 0) grid_x = 0;
    if (grid_x >= m_grid_width) grid_x = m_grid_width - 1;
    if (grid_y < 0) grid_y = 0;
    if (grid_y >= m_grid_height) grid_y = m_grid_height - 1;
    return { static_cast<double>(grid_x), static_cast<double>(grid_y) };
}

Point Pathfinder::grid_to_world(Point grid_pos) {
    double world_x = grid_pos.x * m_grid_resolution + m_map_min_x;
    double world_y = grid_pos.y * m_grid_resolution + m_map_min_y;
    return { world_x, world_y };
}

void Pathfinder::rasterize_obstacles(const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (obstacle.size() < 2) continue;
        for (size_t i = 0; i < obstacle.size(); ++i) {
            Point p1 = { obstacle[i].x, obstacle[i].y };
            Point p2 = { obstacle[(i + 1) % obstacle.size()].x, obstacle[(i + 1) % obstacle.size()].y };
            Point grid_p1 = world_to_grid(p1);
            Point grid_p2 = world_to_grid(p2);
            draw_line_on_grid(grid_p1, grid_p2);
        }
    }
}

void Pathfinder::draw_line_on_grid(Point start_grid, Point end_grid) {
    int x0 = static_cast<int>(start_grid.x);
    int y0 = static_cast<int>(start_grid.y);
    int x1 = static_cast<int>(end_grid.x);
    int y1 = static_cast<int>(end_grid.y);
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (x0 >= 0 && x0 < m_grid_width && y0 >= 0 && y0 < m_grid_height) {
            m_grid_map[y0][x0] = 1;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

double Pathfinder::heuristic(Point a, Point b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)) * m_grid_resolution;
}

Path Pathfinder::find_path_astar(Point start_world, Point end_world) {
    Point start_node_pos = world_to_grid(start_world);
    Point end_node_pos = world_to_grid(end_world);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::map<Point, AStarNode> all_nodes;

    AStarNode start_node;
    start_node.pos = start_node_pos;
    start_node.g = 0;
    start_node.h = heuristic(start_node_pos, end_node_pos);
    start_node.f = start_node.g + start_node.h;
    start_node.parent = { -1, -1 };

    open_set.push(start_node);
    all_nodes[start_node_pos] = start_node;

    int dx[] = { -1, 1, 0, 0, -1, 1, -1, 1 };
    int dy[] = { 0, 0, -1, 1, -1, -1, 1, 1 };
    double move_cost[] = { 1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414 };

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        if (static_cast<int>(current.pos.x) == static_cast<int>(end_node_pos.x) &&
            static_cast<int>(current.pos.y) == static_cast<int>(end_node_pos.y)) {

            // *** MODIFICATION HERE: Add success log ***
            if (DEBUG) { // Only print if DEBUG is enabled
                printf("Success: A* path found from (%.1f, %.1f) to (%.1f, %.1f)\n",
                    start_world.x, start_world.y, end_world.x, end_world.y);
            }

            Path path;
            AStarNode path_node = current;
            while (path_node.parent.x != -1) {
                path.push_back(grid_to_world(path_node.pos));
                path_node = all_nodes[path_node.parent];
            }
            path.push_back(start_world);
            std::reverse(path.begin(), path.end());
            path.push_back(end_world);
            return path;
        }

        for (int i = 0; i < 8; ++i) {
            Point neighbor_pos = { current.pos.x + dx[i], current.pos.y + dy[i] };
            int nx = static_cast<int>(neighbor_pos.x);
            int ny = static_cast<int>(neighbor_pos.y);

            if (nx < 0 || nx >= m_grid_width || ny < 0 || ny >= m_grid_height) continue;
            if (m_grid_map[ny][nx] == 1) continue;

            double new_g = current.g + (move_cost[i] * m_grid_resolution);

            if (all_nodes.find(neighbor_pos) == all_nodes.end() || new_g < all_nodes[neighbor_pos].g) {
                AStarNode neighbor_node;
                neighbor_node.pos = neighbor_pos;
                neighbor_node.parent = current.pos;
                neighbor_node.g = new_g;
                neighbor_node.h = heuristic(neighbor_pos, end_node_pos);
                neighbor_node.f = neighbor_node.g + neighbor_node.h;

                open_set.push(neighbor_node);
                all_nodes[neighbor_pos] = neighbor_node;
            }
        }
    }

    // Warning message for failure remains
    std::cout << "Warning: A* path not found from (" << start_world.x << ", " << start_world.y
        << ") to (" << end_world.x << ", " << end_world.y << ")" << std::endl;
    return {};
}