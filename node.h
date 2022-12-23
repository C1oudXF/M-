#pragma once
#include "Location.h"
#include <string>

class Node {
public:
	Location position;
	float move_cost;
	float prev_act;
	Location* prev_pos;

	Node() {};
	Node(Location pos,float mc,float pa,Location* pp) {
		move_cost = mc;
		position = pos;
		prev_act = pa;
		prev_pos = pp;
	};
	bool Biger(Node n2) { return move_cost > n2.move_cost; };
	bool BigerORequal(Node n2) { return move_cost >= n2.move_cost; };
	bool smaller(Node n2) { return move_cost < n2.move_cost; };
	bool samllerORequal(Node n2) { return move_cost <= n2.move_cost; };
	bool Requal(Node n2) { return move_cost == n2.move_cost; };
	bool operator==(const Node obj) const
	{
		if (position == obj.position && move_cost == obj.move_cost && prev_act == obj.prev_act && prev_pos == obj.prev_pos)
		{
			return true;
		}
	}
};
class next_pos_cost
{
public:
	Location position;
	float cost;
};

class Location2Node {
public:
	Location location;
	Node node;

	Location2Node() {};
	Location2Node(Location location_,Node node_) {
		location = location_;
		node = node_;
	};
	bool operator==(const Location2Node obj) const
	{
		if (location == obj.location/* && node == obj.node*/)
		{
			return true;
		}
	}
};

class Location2int {
public:
	Location location;
	int i;
	Location2int() {};
	Location2int(Location location_,int i_) {
		location = location_;
		i = i_;
	};
};


class OBJ {
public:
	std::vector<int> color; // RGB
	Location pos; //格子标号
	std::vector<std::string> type;		//是否是障碍物

	OBJ() {};
	OBJ(std::vector<int> color_, Location pos_) {
		color = color_;
		pos = pos_;
	};
	void set_type(std::vector<std::string> type_) {
		type = type;
	}
};
