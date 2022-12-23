#pragma once
#include <map>
#include <vector>
#include <functional>
#include "Location.h"
#include "set.h"
#include "ODmstar.h"
#include <queue>
#include "agents.h"


class v_id2Vertex;
class ALLVertex;
class Vertex;
class ODmstar;

class Vertex {
public:
	std::vector<std::vector<Location>> v_id;
	Set<int> collision_set;//碰撞的agent编号
	float g = 1000000;
	float f = 0;
	float move_cost = 0;

	//std::vector<v_id2Vertex> back_set;
	std::vector<Vertex*> back_set;
	Vertex* back_ptr;
	//Vertex() {};
	Vertex(std::vector<std::vector<Location>> v_id_) {
		v_id = v_id_;
	}
	bool is_standard() {
		return v_id[0] == v_id[1];
	}
	void add_collision(Set<int> other_collision_set) {
		collision_set = this->collision_set.union_(other_collision_set);
	}
	bool is_col_subset(Set<int> other_set) {
		return other_set.issubset(collision_set);
	}
	void add_back_set(std::vector<v_id2Vertex> back_set_);
	void add_back_set(Vertex* back_set_);
	std::vector<Vertex*> get_back_set(int i = 0);
	bool operator==(const Vertex obj) const {
		return g == obj.g;
	}
	bool operator>(const Vertex obj) const {
		return g > obj.g;
	}
	bool operator>=(const Vertex obj) const {
		return g >= obj.g;
	}
	bool operator<(const Vertex obj) const {
		return g < obj.g;
	}
	bool operator<=(const Vertex obj) const {
		return g <= obj.g;
	}



};

class v_id2Vertex {
public:
	std::vector<int> v_id;
	Vertex v;
};

void Vertex::add_back_set(Vertex* back_set_)
{
	/*
	for (size_t i = 0; i < back_set_.size(); i++)
	{
		back_set = back_set_;
		back_set.push_back(back_set_[i]);
	}
	*/

};
std::vector<Vertex*> Vertex::get_back_set(int i) {
	return back_set;
};



class ALLVertex {
public:
	std::vector<Vertex*> all_v;

	Vertex* get(std::vector<std::vector<Location>> v_id_)
	{
		std::vector<std::vector<Location>> v_id_2 = v_id_;
		bool isNotIn = true;
		for (size_t i = 0; i < all_v.size(); i++)
		{
			if (all_v[i]->v_id == v_id_2)
			{
				isNotIn = false;
				return all_v[i];
			}
		}
		if (isNotIn)
		{
			Vertex* v_ = new Vertex(v_id_2);
			//v_->v_id = v_id_;
			all_v.push_back(v_);
			return all_v[all_v.size()-1];//刚加进去，最后一个就是当前新建的
		}
	};
};

class VertexInOpen {
public:
	Vertex* vertex;
	float vertex_f;
	VertexInOpen() {};
	VertexInOpen(Vertex* v, float v_f) {
		this->vertex = v;
		this->vertex_f = v_f;
	};

	bool operator<(const VertexInOpen other) const
	{ return this->vertex_f < other.vertex_f; };
};

template<class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type> >
class My_Priority_Queue : public std::priority_queue<T, Container, Compare>
{
public:
	typedef typename std::priority_queue<T, Container, Compare>::container_type::const_iterator const_iterator;

	/* 查找队列中是否存在 t，若存在则返回指向该元素的迭代器，否则返回 std::priority_queue::c.cend() */
	const_iterator find(const T& t) const
	{
		auto first = this->c.cbegin();
		auto last = this->c.cend();

		while (first != last)
		{
			if (*first == t)
				return first;

			++first;
		}

		return last;
	}
};

class Mstar_OD {
public:
	std::vector<Location> M_startset;
	std::vector<Location> M_endset;
	std::vector<Agents*> M_agents;
	std::vector<std::vector<Location>> start_v, end_v;
	My_Priority_Queue<VertexInOpen> open;
	//std::priority_queue<VertexInOpen,std::vector<VertexInOpen>> open;
	std::vector<std::vector<std::vector<Location>>> vl_ids;
	ALLVertex all_v;

	float cost_all;

	ODmstar* ODmstar;

	Mstar_OD(std::vector<Agents*> agents)
	{
		M_agents = agents;
		for (size_t i = 0; i < agents.size(); i++)
		{
			M_startset.push_back(agents[i]->pos);
			M_endset.push_back(agents[i]->goal);
		}
	};
	
	/*
	Mstar_OD(std::vector<Location> startset, std::vector<Location> endset, HSPC* function) :
		heuristic_shortest_path_cost(function),
		M_startset(startset),
		M_endset(endset){};
	*/
	/*
	Mstar_OD(std::vector<Location> startset, std::vector<Location> endset)
	{
		M_startset = startset;
		M_endset = endset;
	};
	*/
	/**
	float call_HSPCfunc(float(ODmstar::*pf)(int, Location),int i ,Location pos) {
		return ODmstar::pf(i, pos);
	}
	*/
	std::vector<std::vector<Location>> Search();
	float heuristic_SIC(std::vector<std::vector<Location>> v_id_);
	std::vector<std::vector<std::vector<Location>>> expand_function(Vertex* v1);
	Set<int> _is_pos_colliding(std::vector<Location> v_pos);
	My_Priority_Queue<VertexInOpen> _backprop(Vertex* v_k,
		Set<int> c_l,
		My_Priority_Queue<VertexInOpen> open);
	float get_move_cost(Vertex* vk, Vertex* vn);
	//std::vector<std::vector<int>> _back_track(Vertex* goal_v);
	std::vector<std::vector<Location>> _back_track(Vertex* goal_v);
	Location _add_tup(Location a,Location b);
	Location _mult_tup(Location a, float m);

};
