#include "ODmstar.h"
#include <iostream>
#include <vector>
#include "agents.h"


const float M_PI = 3.14159265358979323846f;

int main()
{
	std::vector<Agents*> agents;
	Agents* agent1 = new Agents();
	agent1->set_pos(Location(1.0, 1.0));
	agent1->set_goal(Location(10.0, 10.0));
	agents.push_back(agent1);
	Agents* agent2 = new Agents();
	agent2->set_pos(Location(1.0, 9.0));
	agent2->set_goal(Location(9.0, 1.0));
	agents.push_back(agent2);
	Agents* agent3 = new Agents();
	agent3->set_pos(Location(20.0, 9.0));
	agent3->set_goal(Location(9.0, 20.0));
	agents.push_back(agent3);
	Agents* agent4 = new Agents();
	agent4->set_pos(Location(15.0, 13.0));
	agent4->set_goal(Location(30.0, 29.0));
	agents.push_back(agent4);
	Agents* agent5 = new Agents();
	agent5->set_pos(Location(1.0, 23.0));
	agent5->set_goal(Location(30.0, 1.0));
	agents.push_back(agent5);
	/*
	std::vector<Location> agent;
	Location pos;
	pos.x = 1.0;
	pos.y = 1.0;
	agent.push_back(pos);
	pos.x = 1.0;
	pos.y = 9.0;
	agent.push_back(pos);
	pos.x = 20.0;
	pos.y = 9.0;
	agent.push_back(pos);
	pos.x = 15.0;
	pos.y = 13.0;
	agent.push_back(pos);
	pos.x = 1.0;
	pos.y = 23.0;
	agent.push_back(pos);
	
	std::vector<Location> agentend;
	Location end(10.0,10.0);
	agentend.push_back(end);
	end.x = 9.0;
	end.y = 1.0;
	agentend.push_back(end); 
	end.x = 9.0;
	end.y = 20.0;
	agentend.push_back(end);
	end.x = 30.0;
	end.y = 29.0;
	agentend.push_back(end);
	end.x = 30.0;
	end.y = 1.0;
	agentend.push_back(end);
	*/

	for (size_t i = 0; i < agents.size(); i++)
	{
		std::cout << agents[i]->pos.x  <<',' << agents[i]->pos.y << std::endl;
	}
	ODmstar  ODm(32.0,32.0); //地图大小

	std::vector<Obstacle*> obstalce_list;
	Obstacle* obstacle = new Obstacle();
	obstacle->set_pos(Location(15, 19));
	obstalce_list.push_back(obstacle);
	ODm.add_obstacle(obstalce_list);
	//ODm.init_astar_map();

	std::vector<std::vector<Location>> all_action;
	std::vector<std::vector<float>> Yaw, Pitch, Roll;
	std::vector<float> Yaw_1_step, Pitch_1_step, Roll_1_step;
	float yaw, pitch, roll;
	all_action = ODm.ODmstarStart(agents);
	float cost;
	cost = ODm.cost_all;
	for (size_t i = 0; i < all_action.size(); i++)
	{
		std::cout << "step:" << i << std::endl; 
		for (size_t j = 0; j < all_action[i].size(); j++)
		{
			//yaw
			yaw = atan2(all_action[i][j].y, all_action[i][j].x) * (180.f / M_PI);

			//pitch
			pitch = atan2(all_action[i][j].z, sqrt(all_action[i][j].x * all_action[i][j].x + all_action[i][j].y * all_action[i][j].y)) * (180.f / M_PI);
			//roll
			roll = 0;
			Yaw_1_step.push_back(yaw);
			Pitch_1_step.push_back(pitch);
			Roll_1_step.push_back(roll);
			std::cout << "agentID:" << j << "action:" << all_action[i][j].x << "," << all_action[i][j].y 
				<< "角度：(" << yaw << "," << pitch << "," << roll << ")" << std::endl;
		}
		Yaw.push_back(Yaw_1_step);
		Pitch.push_back(Pitch_1_step);
		Roll.push_back(Roll_1_step);
	}
	std::cout << cost << std::endl;

}