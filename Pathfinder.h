#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include "Utilities.h"
#include "PatrollingInput.h"

/*
 * Pathfinder.h
 *
 * Created by: Gemini AI Assistant (based on user request)
 * On:         [Current Date]
 *
 * Description: A module responsible for all pathfinding and distance calculations.
 *              This abstracts the pathfinding logic, allowing for future integration
 *              of obstacle avoidance algorithms like A*.
 */


 // 一个简单的数据结构，用来表示一个二维点或坐标
struct Point {
    double x;
    double y;

    // 为 A* 算法中的 map 提供比较函数
    bool operator<(const Point& other) const {
        if (x < other.x) return true;
        if (x > other.x) return false;
        return y < other.y;
    }
};

// 一个简单的数据结构，用来表示一条路径，即一系列点的序列
using Path = std::vector<Point>;


// *** 新增 A* 节点结构体 ***
struct AStarNode {
    Point pos;      // 栅格坐标
    double g, h, f; // A* 的评价值
    Point parent;   // 父节点坐标

    bool operator>(const AStarNode& other) const {
        return f > other.f; // 用于优先队列的比较
    }
};


class Pathfinder {
public:
    Pathfinder();
    void initialize(const PatrollingInput* input);
    ~Pathfinder();

    double get_path_distance(Point start, Point end);
    Path get_path(Point start, Point end);

private:
    bool m_initialized = false;
    double m_map_min_x, m_map_max_x, m_map_min_y, m_map_max_y;
    double m_grid_resolution;
    int m_grid_width, m_grid_height;
    std::vector<std::vector<int>> m_grid_map;

    Point world_to_grid(Point world_pos);
    Point grid_to_world(Point grid_pos);
    void rasterize_obstacles(const std::vector<Obstacle>& obstacles);
    void draw_line_on_grid(Point start_grid, Point end_grid);

    // *** 新增 A* 相关的私有函数 ***
    Path find_path_astar(Point start_world, Point end_world);
    double heuristic(Point a, Point b); // 启发函数
};

#endif // PATHFINDER_H