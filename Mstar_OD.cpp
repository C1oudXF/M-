#include "Mstar_OD.h"


std::vector<std::vector<Location>> Mstar_OD::Search() {


	start_v.push_back(M_startset);
	start_v.push_back(M_startset);
	end_v.push_back(M_endset);
	end_v.push_back(M_endset);

	Vertex* all_v_get_start_v;
	all_v_get_start_v = all_v.get(start_v);
	all_v_get_start_v->f = 5;
	Vertex* vs = all_v_get_start_v;
	vs->g = 0;
	vs->f = vs->g + heuristic_SIC(vs->v_id);
	VertexInOpen vsInOpen(vs,vs->f);
	open.push(vsInOpen);

	while (!open.empty())
	{
		VertexInOpen vk_open = open.top();
		open.pop();
		for (size_t i = 0; i < vk_open.vertex->v_id[0].size(); i++)
		{
			std::cout << i << ":" << vk_open.vertex->v_id[0][i].x << ',' << vk_open.vertex->v_id[0][i].y << std::endl;

		}

		if (vk_open.vertex->v_id == end_v)
		{
			return _back_track(vk_open.vertex);
		}
		vl_ids = expand_function(vk_open.vertex);
		for (size_t i = 0; i < vl_ids.size(); i++)
		{
			Vertex* vl = this->all_v.get(vl_ids[i]);
			std::vector<Location> v_pos = vl->v_id[vl->v_id.size()-1];
			Set<int> col = this->_is_pos_colliding(v_pos);

			if (vl->is_standard())
			{
				vl->back_set.push_back(vk_open.vertex);
				//vl->add_back_set(vk_open.vertex);
				vl->add_collision(col);
				My_Priority_Queue<VertexInOpen> openlist = this->_backprop(vk_open.vertex, vl->collision_set, open);
				
				while (!open.empty())
				{
					open.pop();
				}
				while(!openlist.empty())
				{
					open.push(openlist.top());
					openlist.pop();
				}
			}
			//std::cout << col.len() << std::endl;
			if ((col.size() == 0 || vl->is_standard()==false) && vk_open.vertex->g + this->get_move_cost(vk_open.vertex,vl) < vl->g)
			{
				vl->g = vk_open.vertex->g + this->get_move_cost(vk_open.vertex, vl);
				vl->f = vl->g + this->heuristic_SIC(vl->v_id);
				vl->back_ptr = vk_open.vertex;
				open.push(VertexInOpen(vl, vl->f));
			}
		}
	}
//	return NULL;
};

//std::vector<std::vector<int>> Mstar_OD::_back_track(Vertex* goal_v)
std::vector<std::vector<Location>> Mstar_OD::_back_track(Vertex* goal_v) {
	std::vector<Location2int> pos_act;
	//std::map<Location, int> pos_act; //可采取动作

	pos_act.push_back(Location2int(Location(0, 1), 2));

	pos_act.push_back(Location2int(Location(1, 0), 3));

	pos_act.push_back(Location2int(Location(0, -1), 4));

	pos_act.push_back(Location2int(Location(-1, 0), 1));

	pos_act.push_back(Location2int(Location(0, 0), 0));

	std::vector<std::vector<Location>> all_v2;
	all_v2.push_back(goal_v->v_id[1]);
	Vertex* next_v = goal_v->back_ptr;

	while (next_v != NULL)
	{
		if (next_v->is_standard())
		{
			all_v2.push_back(next_v->v_id[1]);
			cost_all += next_v->g;

		}
		next_v = next_v->back_ptr;
	}

	std::vector<std::vector<int>> all_actions;
	std::vector<std::vector<Location>> all_agents_position;
	std::vector<Location> agent1_positions,agent2_positions,stepI_position;
	std::vector<Location> prev_v = all_v2[1];
	//去掉数组最后一位 然后反转吗
	all_v2.pop_back();
	reverse(all_v2.begin(), all_v2.end());
	//这里有个action字典
	std::vector<int> action;

	for (size_t i = 0; i < all_v2.size(); i++)
	{
		std::vector<Location> v = all_v2[i];
		for (size_t j = 0; j < all_v2[i].size(); j++)
		{
			Location previous_position;
			Location next_position;
			previous_position = prev_v[j];
			next_position = v[j];
			stepI_position.push_back(next_position);
			Location position_diff = this->_add_tup(next_position, this->_mult_tup(previous_position, -1));
			for (size_t k = 0; k < pos_act.size(); k++)
			{
				if (pos_act[k].location == position_diff)
				{
					action.push_back(pos_act[k].i);

				}
			}
		}
		all_agents_position.push_back(stepI_position);
		stepI_position.clear();
		prev_v = v;
		all_actions.push_back(action);

	}

	return all_agents_position;
};


Location Mstar_OD::_add_tup(Location a,Location b) {
	Location ans;
	ans.x = a.x + b.x;
	ans.y = a.y + b.y;
	return ans;
};

Location Mstar_OD::_mult_tup(Location a,float m) {
	Location ans;
	ans.x = a.x * m;
	ans.y = a.y * m;
	return ans;
};


float Mstar_OD::get_move_cost(Vertex* vk, Vertex* vn) {
	std::vector<Location> end = this->M_endset;
	float cost;
	int num_pos_cange = 0;
	if (vk->is_standard())
	{
		if (vn->is_standard())
		{
			cost = this->M_startset.size();
			int num_agents_stay_on_goal = 0;
			for (size_t i = 0; i < this->M_startset.size(); i++)
			{
				Location gp = end[i];
				Location pk = vk->v_id[0][i];
				Location pn = vn->v_id[0][i];
				if (pk==gp&&pn==gp)
				{
					num_agents_stay_on_goal += 1;
				}
			}
			cost -= num_agents_stay_on_goal;
		}
		else
		{
			int cnt_vn = 0;
			for (size_t i = 0; i < end.size(); i++)
			{
				Location g = end[i];
				Location pn = vn->v_id[0][i];
				Location pk = vk->v_id[0][i];

				if (pn != Location(-1,-1))
				{
					if (pn == g && pk == g)
					{
						cnt_vn += 0;
					}
					else
					{
						cnt_vn += 1;
					}
				}
			}
			cost = cnt_vn;
		}
	}
	else
	{
		if (vn->is_standard())
		{

			cost = 0;
			for (size_t i = 0; i < end.size(); i++)
			{
				Location gp, pk, pn, pk_root;
				gp = end[i];
				pk = vk->v_id[0][i];
				pn = vn->v_id[0][i];
				pk_root = vk->v_id[1][i];
				if (pk == Location(-1,-1))
				{
					num_pos_cange += 1;
					if (pn == gp && pk_root == gp)
					{
						cost += 0;
					}
					else
					{
						cost += 1;
					}
				}
			}
		}
		else
		{
			num_pos_cange = 0;
			cost = 0;
			for (size_t i = 0; i < end.size(); i++)
			{
				Location gp, pk, pn, pk_root;
				gp = end[i];
				pk = vk->v_id[0][i];
				pn = vn->v_id[0][i];
				pk_root = vk->v_id[1][i];
				if (pk == Location(-1,-1) && pn != Location(-1, -1))
				{
					num_pos_cange += 1;
					if (pn == gp && pk_root == gp)
					{
						cost += 0;
					}
					else
					{
						cost += 1;
					}

				}
			}
		}
	}
	return cost;
};

My_Priority_Queue<VertexInOpen> Mstar_OD::_backprop(Vertex* v_k,
	Set<int> c_l,
	My_Priority_Queue<VertexInOpen> open) {

	bool isInopen = false;
	if (v_k->is_standard())
	{
		if (!c_l.issubset(v_k->collision_set))
		{
			v_k->add_collision(c_l);

			//find v_k in openlist
			My_Priority_Queue<VertexInOpen> open_re;
			for (size_t i = 0; i < open.size(); i++)
			{

				VertexInOpen open_front = open.top();//top是第一个元素吗?
				open_re.push(open_front);
				open.pop();
				if (v_k == open_front.vertex)
				{
					//v_k 在 open里面
					isInopen = true;
					break;
				}
			}
			//把抽出来的openre中元素再加回去
			if (!open_re.empty())
			{
				open.push(open_re.top());
				open_re.pop();
			}
			//如果不在openlist中
			if (!isInopen)
			{
				float priority = v_k->g + this->heuristic_SIC(v_k->v_id);
				VertexInOpen priority_inopen(v_k, priority);
				open.push(priority_inopen);
			}
			std::vector<Vertex*> v_m_list = v_k->get_back_set();
			for (size_t i = 0; i < v_m_list.size(); i++)
			{
				Vertex* v_m = v_m_list[i];
				this->_backprop(v_m, v_k->collision_set, open);
			}
		}
	}
	return open;
};

Set<int> Mstar_OD::_is_pos_colliding(std::vector<Location> v_pos) {
	Set<int> hldr;
	for (size_t i = 0; i < v_pos.size(); i++)
	{
		for (size_t j = 0; j < v_pos.size(); j++)
		{
			if (i != j)
			{
				if (v_pos[i] == v_pos[j])
				{
					hldr.add(i);
					hldr.add(j);
				}
			}
		}
	}
	return hldr;
};


std::vector<std::vector<std::vector<Location>>> Mstar_OD::expand_function(Vertex* v1) {
	std::vector<Location> inter_tup, vertex_pos_tup;
	inter_tup = v1->v_id[0];
	vertex_pos_tup = v1->v_id[1];

	std::vector<Location> next_inter_tup,n_pos;
	Location pos1;
	bool standard_ = true;
	//is there a collision?
	for (size_t i = 0; i < inter_tup.size(); i++)
	{
		if (inter_tup[i] == Location(-1,-1))
		{
			standard_ = false;
		}
	}
	if (standard_)//if not 
	{
		Set<int> collision_set = v1->collision_set;
		for (size_t j = 0; j < inter_tup.size(); j++)//every agent
		{
			if (collision_set.is_exist(j)) //if this agent collision
			{
				next_inter_tup.push_back(Location(-1, -1));
			}
			else//if this agent not collision
			{
				pos1 = this->ODmstar->get_next_joint_polict_position(j, inter_tup[j], this->M_endset[j]);
				n_pos.push_back(pos1);
				next_inter_tup.push_back(n_pos[n_pos.size() - 1]);
			}
		}
	}
	else// if is
	{
		next_inter_tup = inter_tup;
	}

    //定位碰撞发生位置
	int this_inter_level = -1;

	for (size_t i = 0; i < next_inter_tup.size(); i++)
	{
		if (next_inter_tup[i] == Location(-1,-1))
		{
			this_inter_level = i;
			break;
		}
	}

	std::vector<std::vector<Location>> all_next_inter_tup;
	std::vector<Location> positions_taken;
	if (this_inter_level != -1)
	{
		Location pos = vertex_pos_tup[this_inter_level];
		for (size_t i = 0; i < next_inter_tup.size(); i++)
		{
			if (next_inter_tup[i] != Location(-1,-1))
			{
				positions_taken.push_back(next_inter_tup[i]);
			}
		}
		n_pos = this->ODmstar->expand_position(this_inter_level, pos);
		bool isInPosition_taken = false;
		std::vector<Location> valid_n_pos;
		// valid_n_pos = [p for p in n_pos if not p in positions_taken]
		for (size_t i = 0; i < n_pos.size(); i++)
		{
			for (size_t j = 0; j < positions_taken.size(); j++)
			{
				if (n_pos[i] == positions_taken[j])
				{
					isInPosition_taken = true;
				}
			}
			if (!isInPosition_taken)
			{
				valid_n_pos.push_back(n_pos[i]);
			}
		}
		/*
		if (valid_n_pos.empty())
		{
			return NULL;//好像可以不要
		}
		*/
		for (size_t i = 0; i < valid_n_pos.size(); i++)
		{
			next_inter_tup[this_inter_level] = valid_n_pos[i];
			all_next_inter_tup.push_back(next_inter_tup);
		}
	}
	else
	{
		all_next_inter_tup.push_back(next_inter_tup);
	}
	std::vector<std::vector<std::vector<Location>>> v_ids_;
	std::vector<std::vector<Location>> v_ids_1;
	bool is_ = false;
	for (size_t i = 0; i < all_next_inter_tup.size(); i++)
	{
		for (size_t j = 0; j < all_next_inter_tup[i].size(); j++)
		{
			if (all_next_inter_tup[i][j] == Location(-1,-1) )
			{
				is_ = true;
			}
		}
		if (!is_)
		{
			v_ids_1.clear();
			v_ids_1.push_back(all_next_inter_tup[i]);
			v_ids_1.push_back(all_next_inter_tup[i]);
			v_ids_.push_back(v_ids_1);
		}
		else
		{
			v_ids_1.clear();
			v_ids_1.push_back(all_next_inter_tup[i]);
			v_ids_1.push_back(vertex_pos_tup);
			v_ids_.push_back(v_ids_1);
		}
	}
	return v_ids_;


};


float Mstar_OD::heuristic_SIC(std::vector<std::vector<Location>> v_id_) {
	std::vector<Location> inter_tup, vertex_pos_tup;
	inter_tup = v_id_[0];
	vertex_pos_tup = v_id_[1];

	float total_cost = 0;

	for (size_t i = 0; i < inter_tup.size(); i++)
	{
		if (inter_tup[i] == Location(-1,-1))
		{
			total_cost += this->ODmstar->heuristic_shortest_path_cost(i, vertex_pos_tup[i]);
			//float h_cost = std::sqrt((this->M_endset[i].x - inter_tup[i].x) * (this->M_endset[i].x - inter_tup[i].x)
			//	- (this->M_endset[i].y - inter_tup[i].y) * (this->M_endset[i].y - inter_tup[i].y));
			//total_cost += h_cost;
		}
		else
		{
			total_cost += this->ODmstar->heuristic_shortest_path_cost(i, inter_tup[i]);
			//float h_cost = std::sqrt((this->M_endset[i].x - inter_tup[i].x) * (this->M_endset[i].x - inter_tup[i].x)
			//	- (this->M_endset[i].y - inter_tup[i].y) * (this->M_endset[i].y - inter_tup[i].y));
			//total_cost += h_cost;

		}
	}
	return total_cost;
}
