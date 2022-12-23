#pragma once
#include <vector>
#include "Location.h"


class Agents
{
public:
	Location pos;
	Location goal;

	void set_pos(Location pos_) {
		pos = pos_;
	};
	void set_goal(Location goal_) {
		goal = goal_;
	}
	Agents() {};

};


class Obstacle
{
public:
	Location pos;
	void set_pos(Location pos_) {
		pos = pos_;
	};
};

