#pragma once
#include <vector>
#include "node.h"
#include <map>
#include "Mstar_OD.h"
#include <queue>
#include "agents.h"

class ODmstar {

	//��Ա��
public:
	std::vector<std::vector<Location2Node>> dijkstra_graph; //����agents��cost�ĵ�ͼ
	std::vector<Location2Node> d_a_map; //����agent �ĵ�ͼ
	std::vector<std::vector<Location2Node>> path_grid;
	float cost_all;

	//std::map<int, int> all_action;
	std::vector<std::vector<Location>> all_action;
	float x_bound, y_bound;
	std::vector<Location> StartSet;
	std::vector<Location> EndSet;

	//BFS
	std::vector<Location2Node> visited,n2;
	std::queue<Node> q;
	Node n;
	std::vector<Location2int> next_Nodes_pos;  //����֮��õ���ȫ��position



    // get_neighbor
	std::map<int, std::vector<int>> pos_act; //�ɲ�ȡ����
	std::vector<std::vector<OBJ>> grid; //ԭʼ��ͼ ����һ�����еĶ�����
	std::vector<std::string> obstacle_type;
	std::vector<Obstacle*> obstacle_pos;

	Location pos;//����֮��õ���ĳ��position
	//������
	void add_obstacle(std::vector<Obstacle*> _obstacle_pos) {
		obstacle_pos = _obstacle_pos;
	};
	ODmstar() {};
	ODmstar(float maxX,float maxY) {

		x_bound = maxX;
		y_bound = maxY;

		//������ maxX * maxY��grid
		for (size_t i = 0; i < x_bound; i++)
		{
			std::vector<OBJ> gridy;
			for (size_t j = 0; j < y_bound; j++)
			{
				OBJ gridx;
				gridx.pos = Location(i, j);
	
				gridy.push_back(gridx);
			}
			grid.push_back(gridy);
		}

		std::vector<int> action_part;
		action_part.push_back(0);
		action_part.push_back(1);
		pos_act.insert(std::pair<int, std::vector<int>>(2, action_part));
		action_part.clear();

		action_part.push_back(1);
		action_part.push_back(0);
		pos_act.insert(std::pair<int, std::vector<int>>(3, action_part));
		action_part.clear();

		action_part.push_back(0);
		action_part.push_back(-1);
		pos_act.insert(std::pair<int, std::vector<int>>(4, action_part));
		action_part.clear();

		action_part.push_back(-1);
		action_part.push_back(0);
		pos_act.insert(std::pair<int, std::vector<int>>(1, action_part));
		action_part.clear();

		action_part.push_back(0);
		action_part.push_back(0);
		pos_act.insert(std::pair<int, std::vector<int>>(0, action_part));
		action_part.clear();

		obstacle_type.push_back("obstacle");
		obstacle_type.push_back("agent");
	};
	void init_astar_map();//graph��cost���ԣ��������ʼ��Ϊastar��cost��search��ֻ����movecost������ʽcost�����Ǵ������ȡ�ġ�
	void init_joint_policy_graphs(std::vector<Agents*> agents);
	std::vector<std::vector<Location>> ODmstarStart(std::vector<Agents*> agents);
	//std::map<float[2], Node> BFS(float startpos, float endpos);
	std::vector<Location2Node> BFS(Agents* agent);
	
	std::vector<Location2int> get_neighbor(Location pos);
	
	Location add_tup(Location position, std::vector<int> pos_act_);
	bool assert_in_bounds(Location);
	bool isIn(OBJ obj_);
	float heuristic_shortest_path_cost(int i, Location pos);
	Location get_next_joint_polict_position(int j, Location inter_tup, Location end);
	std::vector<Location> expand_position(int i, Location pos);

};
