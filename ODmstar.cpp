#include "ODmstar.h"
#include "env.h"
/*
* 这更像是一个全局的管理器。
*/

using heuristic_shortest_path_cost = float(int, Location);


std::vector<std::vector<Location>> ODmstar::ODmstarStart(std::vector<Agents*> agents_)
{
	std::vector<Agents*> agents = agents_;
	if (dijkstra_graph.empty())
	{
		init_joint_policy_graphs(agents);
	}

	//Mstar_OD* mstar = Mstar_OD(startset, endset, func);
	Mstar_OD mstar = Mstar_OD(agents);
	mstar.ODmstar = this;
	all_action = mstar.Search();
	cost_all = mstar.cost_all;
	/*
	Mstar_OD* mstar = new Mstar_OD(startset, endset);
	all_action = mstar->Search();//也是个字典｛编号：动作｝
	return all_action;
	*/
	return all_action;
};



float ODmstar::heuristic_shortest_path_cost(int i, Location pos) {

	for (size_t j = 0; j < this->dijkstra_graph[i].size(); j++)
	{
		if (this->dijkstra_graph[i][j].location == pos)
		{
			return this->dijkstra_graph[i][j].node.move_cost;
		}
	}
};

void ODmstar::init_joint_policy_graphs(std::vector<Agents*> agents)
{
	for (size_t i = 0; i < agents.size(); i++)
	{
		d_a_map = BFS(agents[i]);
		//dijkstra_graph.insert(std::pair<int, std::map<std::vector<float>, Node>>(i, d_a_map));
		dijkstra_graph.push_back(d_a_map);
	}
};


std::vector<Location2Node> ODmstar::BFS(Agents* agent)
{

	Node start_node(agent->pos, 0, NULL, NULL);
	q.push(start_node);

	while (!q.empty())
	{
		n = q.front();
		q.pop();

		//判断有没有被访问过。

			if (std::find(visited.begin(), visited.end(), Location2Node(n.position,n)) == visited.end())
			{
				//未被访问：
				//先把他加进去
				visited.push_back(Location2Node(n.position, n));
				//visited[0].node.move_cost += std::sqrt((visited[0].location.x - endpos.x) * (visited[0].location.x - endpos.x)
					//+ (visited[0].location.y - endpos.y) * (visited[0].location.y - endpos.y));
				next_Nodes_pos = get_neighbor(n.position);
				for (size_t i=0;i < next_Nodes_pos.size(); i++)
				{
					//Location hldr = next_Nodes_pos[i].location;
					Node hldr(next_Nodes_pos[i].location, n.move_cost + 1, next_Nodes_pos[i].i, &n.position);
					if (std::find(visited.begin(),visited.end(),Location2Node(hldr.position,hldr)) == visited.end())
					{
						q.push(Node(next_Nodes_pos[i].location, n.move_cost + 1, next_Nodes_pos[i].i, &n.position));
					}
				}
			}
	}
	return visited;
};

std::vector<Location2int> ODmstar::get_neighbor(Location position) {

	std::vector<Location2int> n_pos; //位置 ，动作
	bool isObstalce = false;
	for (auto it : pos_act)
	{
		int n = 0;
		pos = add_tup(position,it.second);
		if (assert_in_bounds(pos))
		{
			OBJ obj = grid[pos.x][pos.y]; //将地图中的这一个格子单独取出
			//判断是否为障碍物
			for (size_t i = 0; i < this->obstacle_pos.size(); i++)
			{
				if (obj.pos == obstacle_pos[i]->pos)
				{
					isObstalce = true;
				}
			}
			//如果bu是障碍物
			if (!isObstalce && !isIn(obj))
			{
				n_pos.push_back(Location2int(pos, it.first));
			}
		}
		n++;
	}
	return n_pos;
};

Location ODmstar::add_tup(Location position, std::vector<int> pos_act_) 
{
	Location ans;
	ans.x = position.x + pos_act_[0];
	ans.y = position.y + pos_act_[1];

	return ans;
}

bool ODmstar::isIn(OBJ obj_) {
	for (size_t i = 0; i < obj_.type.size(); i++)
	{
		for (size_t j = 0; j < obstacle_type.size(); j++)
		{
			if (obj_.type[i] == obstacle_type[j])
			{
				return true;
			}
		}
	}
	return false;
}

bool ODmstar::assert_in_bounds(Location l) {
	float x = l.x;
	float y = l.y;
	if (x >= 0 && x < x_bound && y >= 0 && y < y_bound)
	{
		return true;
	}
	else
	{
		return false;
	}
}

Location ODmstar::get_next_joint_polict_position(int agent_handle, Location position, Location goal_pos) {
	std::vector<Location2Node> this_graph = this->dijkstra_graph[agent_handle];
	std::vector<Location> next_position1;
	bool isN_pos_in_graph = false;
	std::vector<next_pos_cost> next_position_cost;

	//next_position
	for (size_t i = 0; i < this->pos_act.size(); i++)
	{
		next_position1.push_back(this->add_tup(position,this->pos_act[i]));

	}
	//next_position_cost
	for (size_t i = 0; i < next_position1.size(); i++)
	{
		Location n_pos = next_position1[i];
		int index;
		for (size_t j = 0; j < this_graph.size(); j++)
		{
			if (this_graph[j].location == position)
			{
				index = j;
			}
		}
		for (size_t j = 0; j < this_graph.size(); j++)
		{
			if (this_graph[j].location == n_pos)
			{
				//这里有点混乱
				isN_pos_in_graph = true;
				next_pos_cost n_p_c;
				n_p_c.position = n_pos;
				n_p_c.cost = this_graph[index].node.move_cost + 1   + std::sqrt((goal_pos.x - n_pos.x) * (goal_pos.x - n_pos.x)
						+ (goal_pos.y - n_pos.y) * (goal_pos.y - n_pos.y));
				next_position_cost.push_back(n_p_c);
			}
		}
	}
	//mini_cost
	float min_cost =99999;
	int min_index=0;
	min_cost = next_position_cost[0].cost;
	for (size_t i = 1; i < next_position_cost.size(); i++)
	{
		if (min_cost >next_position_cost[i].cost)
		{
			min_cost = next_position_cost[i].cost;
			min_index = i;
		}
	}
	Location min_cost_next_pos = next_position_cost[min_index].position;

	return min_cost_next_pos;
};

std::vector<Location> ODmstar::expand_position(int agent_handle, Location position) {
	std::vector<Location2Node> this_graph;
	std::vector<Location> neighbours;
	this_graph = this->dijkstra_graph[agent_handle];
	std::vector<Location> next_position;
	for (size_t i = 0; i < this->pos_act.size(); i++)
	{
		next_position.push_back(this->add_tup(position, pos_act[i]));
	}
	bool ingraph = false;
	for (size_t i = 0; i < next_position.size(); i++)
	{
		for (size_t j = 0; j < this_graph.size(); j++)
		{
			if (next_position[i] == this_graph[j].location)
			{
				neighbours.push_back(next_position[i]);
				break;
			}
		}
	}
	return neighbours;
};

void ODmstar::init_astar_map() {//compute astar cost for every node;
	
	/*for (size_t i = 0; i < this->dijkstra_graph.size(); i++) //agent num
	{
		size_t agent_goal_id = i;
		Location goal = this->EndSet[agent_goal_id];
		this->path_grid[i] = this->BFS(this->StartSet[i], this->EndSet[i]);
		std::queue<Location2Node> openlist;

	}
	*/


	for (size_t i = 0; i < StartSet.size(); i++)
	{
		MyPoint EndPos = { EndSet[i].x,EndSet[i].y };
		std::vector<std::vector<bool>> pathMap;
		for (size_t i = 0; i < x_bound; i++)
		{
			for (size_t j = 0; j < y_bound; j++)
			{
				pathMap[i][j] = { 0 };
			}
		}

		pathMap[StartSet[i].x][StartSet[i].y] = true;

		//准备一颗树
		treeNode* pRoot = NULL;
		pRoot = createTreeNode(StartSet[i].x, StartSet[i].y);
		vector<treeNode*> buff;//准备一个数组
		bool isFindEnd = false;
		treeNode* current = pRoot;

		vector<treeNode*>::iterator it;
		vector<treeNode*>::iterator itMin;

		while (1) {
			//1 把当前点周围能走的点找出来
			for (int i = 0; i < 8; i++) {
				treeNode* pChild = createTreeNode(current->pos.row,
					current->pos.col);
				switch (i) {
				case p_up:
					pChild->pos.row--;
					pChild->pos.g += ZXDJ;
					break;
				case p_down:
					pChild->pos.row++;
					pChild->pos.g += ZXDJ;
					break;
				case p_left:
					pChild->pos.col--;
					pChild->pos.g += ZXDJ;
					break;
				case p_right:
					pChild->pos.col++;
					pChild->pos.g += ZXDJ;
					break;
				case p_lup:
					pChild->pos.col--;
					pChild->pos.row--;
					pChild->pos.g += XXDJ;
					break;
				case p_ldown:
					pChild->pos.col--;
					pChild->pos.row++;
					pChild->pos.g += XXDJ;
					break;
				case p_rup:
					pChild->pos.col++;
					pChild->pos.row--;
					pChild->pos.g += XXDJ;
					break;
				case p_rdown:
					pChild->pos.col++;
					pChild->pos.row++;
					pChild->pos.g += XXDJ;
					break;
				}

				//2 判断能不能走
				if ( //不是障碍物
					pathMap[pChild->pos.row][pChild->pos.col] == false
					) {//能走
					//的计算出f值 入树，存入数组
					pChild->pos.h = getH(pChild->pos, EndPos);
					pChild->pos.f = pChild->pos.g + pChild->pos.h;
					for (size_t kl = 0; kl < this->dijkstra_graph[i].size(); kl++)
					{
						if (this->dijkstra_graph[i][kl].location.x == pChild->pos.row &&
							this->dijkstra_graph[i][kl].location.y == pChild->pos.col)
						{
							this->dijkstra_graph[i][kl].node.move_cost = pChild->pos.f;

						}
					}

					//入树
					current->child.push_back(pChild);
					pChild->pParent = current;

					//存入数组
					buff.push_back(pChild);
				}
				else {//不能走
					//delete pChild;
				}
			}
		}

	}
};


