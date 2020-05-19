#ifndef COMPARE_HPP
#define COMPARE_HPP

#include "State.hpp"
#include "Utils.hpp"

class Compare
{
public:
	static State target;
	static int** obs_map;			// 障碍物地图，与map.obs_map是同一张地图
	static int** grid_obs_map;		// 240x240的grid map
	static float** shortest_2d;		// dijkstra算法计算出的最短路径代价，即每个点离起始点的最短路径代价

    bool operator() (const State s1, const State s2);
    float non_holonomic_without_obs(State src);
    float holonomic_with_obs(State src);
    void runDijkstra();
};

#endif COMPARE_HPP