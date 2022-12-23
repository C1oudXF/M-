#pragma once
#include "node.h"
#include <vector>

class Grid {
	int x_len;
	int y_len;
	std::vector<std::vector<OBJ>> grid;

	Grid(int x, int y) {
		x_len = x;
		y_len = y;
	}


	OBJ get(int x, int y) {
		return grid[x][y];
	}
};

class Grid_Env : public Grid {

};



#include <stdio.h>
#include <vector>//动态数组
using namespace std;


//直线代价
#define ZXDJ 10
//斜线代价
#define XXDJ 14

enum direct {
	p_up, p_down, p_left, p_right,
	p_lup, p_ldown, p_rup, p_rdown
};

//存下标
struct MyPoint {
	int row;
	int col;

	int f;
	int g;
	int h;
};
//树的节点类型
struct treeNode
{
	MyPoint				pos;
	vector<treeNode*>	child;
	treeNode* pParent;
};


//创建树节点
treeNode* createTreeNode(int row, int col) {
	treeNode* pNew = new treeNode;//开内存
	memset(pNew, 0, sizeof(treeNode));//全部赋值为NULL
	pNew->pos.row = row;
	pNew->pos.col = col;
	return pNew;
}

//计算h值并返回
int getH(MyPoint pos, MyPoint endPos) {
	int x = ((pos.col > endPos.col) ? (pos.col - endPos.col) : (endPos.col - pos.col));
	int y = ((pos.row > endPos.row) ? (pos.row - endPos.row) : (endPos.row - pos.row));

	return 10 * (x + y);
}