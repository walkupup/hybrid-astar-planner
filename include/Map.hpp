#ifndef MAP_HPP
#define MAP_HPP

#include "State.hpp"
#include "Utils.hpp"

class Map{
public:
    int** obs_map;			// 800x800���ϰ����ͼ
    int** acc_obs_map;		// 800x800���ۻ���ͼ����ԭ�㵽��ǰλ�õľ���������obs_mapֵ�ĺ�
    int** nearest_obstacle; // 800x800������ϰ����ͼ����ÿ�������������ϰ���ľ���
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
