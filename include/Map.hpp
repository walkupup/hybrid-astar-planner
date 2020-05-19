#ifndef MAP_HPP
#define MAP_HPP

#include "State.hpp"
#include "Utils.hpp"

class Map{
public:
    int** obs_map;			// 800x800的障碍物地图
    int** acc_obs_map;		// 800x800的累积地图，从原点到当前位置的矩形区域内obs_map值的和
    int** nearest_obstacle; // 800x800的最近障碍物地图，即每个点距离最近的障碍物的距离
    int obs_dist_max;

    Map();
    void initCollisionChecker();
    bool checkCollision(State pos);
    void find_near_obs();
    int  nearest_obstacle_distance(State pos);
    bool is_boundary_obstacle(int i, int j);
};

class node{
public:
	int x,y,nearest_obstacle;
};

#endif
