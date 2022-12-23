# M-
c++ m* code

ODmstar 是一个m*的路径规划器，初始化时需要指定地图大小
例如在一个32*32的地图上： ODmstar odm(32,32)

如果地图中有障碍物，给障碍物赋值一个Location类型的坐标点，再把所有障碍物放入一个vector容器中，使用odm.add_obstacle(obstacle_list)更新地图中的障碍物信息

调用odm.ODmstarstart(agents_list)函数进行路径规划，返回值类型为 vector<vector<Location>>，第一个vector是agents顺序，第二个vector是某个agent的从起点到终点的位置。

ODm.cost_all获取这次路径规划的总cost，cost值为所有agents此次规划的A*启发式函数的总和。
