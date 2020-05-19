#include "include/Planner.hpp"

State getStartState();
State getTargetState();

int main(){

	Map *map = new Map();
	State *start = new State(700, 100, 36);// getStartState();
	State *target = new State(100, 600, 18);// getTargetState();

	Planner astar;
	
	astar.plan(*start, *target, *map);
	delete map;
	delete start;
	delete target;
}

State getStartState()
{
	//to do: read from yml file
	return State(700, 100, 36);
}

State getTargetState()
{
	//to do: read from yml file
	return State(100, 600, 18);
}