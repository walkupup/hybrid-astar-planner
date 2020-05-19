#include "../include/Planner.hpp"


State previous[GX][GY][Theta];
int vis[GX][GY][Theta];

void Planner::plan(State &start, State &target, Map &map){

	//initialize variables for the Compare class
	Compare::target=target;
	Compare::obs_map=map.obs_map;
	Compare::grid_obs_map=new int*[DX];
	for(int i=0;i<DX;i++)
	{
		Compare::grid_obs_map[i]=new int[DY];
		for(int j=0;j<DY;j++)
			Compare::grid_obs_map[i][j]=0;
	}
	for(int i=0;i<MAPX;i++)
		for(int j=0;j<MAPY;j++)
		{
			if(Compare::obs_map[i][j])
				Compare::grid_obs_map[i*DX/MAPX][j*DY/MAPY]=1;
		}

	// 运行dijkstra算法，目的是得到shortest_2d，即每个点到起始点的代价，后续要用到
	Compare cmp;
	cmp.runDijkstra();


	map.initCollisionChecker();

	// 为每个栅格找到距离最近的障碍物的距离，输出为 nearest_obstacle
	map.find_near_obs();

	// 此处定义优先级队列，元素大小比较是由 Compare::operator()定义的
	priority_queue<State, vector<State>, Compare> pq;
	start.cost3d=0;
	pq.push(start);

	GUI display(800, 800);
	display.drawObs(map);
	display.drawCar(start);
	display.drawCar(target);

	memset(vis, 0, sizeof(int)*GX*GY*Theta);
	
	int iter=0;
	while(pq.size()>0)
	{
		State current=pq.top();
		pq.pop();

		// 如果到达目的地，则把全部车轨迹画出来，结束路径规划
		if(abs(current.gx-target.gx)<=1 && abs(current.gy-target.gy)<=1 && abs(current.gtheta-target.gtheta)<=5)
		{
			cout<<"Reached target."<<endl;

			State Dummy;
			current.change=PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(current))/(float)(map.obs_dist_max-1)+
						   fabs(current.theta)/BOT_M_ALPHA+1; 
				
			while(current.x!=start.x || current.y!=start.y || current.theta!=start.theta){
				current.velocity=VELOCITY_MAX/current.change;
				display.drawCar(current);
			    display.show(2000/current.velocity);//This can be removed while executing the algo
				//display.show(0);
				Dummy=previous[current.gx][current.gy][current.gtheta];
				Dummy.change=PRIORITY_MOVEMENT*fabs(Dummy.theta-current.theta)/(2.0*BOT_M_ALPHA)+
					     PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(Dummy))/(float)(map.obs_dist_max-1)+
					     fabs(Dummy.theta)/BOT_M_ALPHA+1;
				current=Dummy;
			}
			break;
		}

		if(vis[current.gx][current.gy][current.gtheta]){
			continue;
		}

		vis[current.gx][current.gy][current.gtheta]=1;

		// 为当前状态扩展出若干个（3个）后续状态：左转30度，右转30度，直行。行进长度为40.
		vector<State> next=current.getNextStates();

		for(int i=0;i<next.size();i++){
			//display.drawCar(next[i]);
			// 检查在当前位置是否发生碰撞，考虑了质点加上车轮廓之后的碰撞情况
			if(!map.checkCollision(next[i])){

				if(!vis[next[i].gx][next[i].gy][next[i].gtheta]){
					//display.drawCar(next[i]);
					current.next=&(next[i]);
					next[i].previous=&(current);
					
					// 如果直行，cost加5，否则加7
					if(i==1)
						next[i].cost3d=current.cost3d+5;
					else
						next[i].cost3d=current.cost3d+7;
					//next[i].cost3d=current.cost3d+1;
					pq.push(next[i]);

					previous[next[i].gx][next[i].gy][next[i].gtheta]=current;
				}
			}
		}
	}
	cout<<"Done."<<endl;
	display.show(0);
	
	return;
}
