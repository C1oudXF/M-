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
#include <vector>//��̬����
using namespace std;


//ֱ�ߴ���
#define ZXDJ 10
//б�ߴ���
#define XXDJ 14

enum direct {
	p_up, p_down, p_left, p_right,
	p_lup, p_ldown, p_rup, p_rdown
};

//���±�
struct MyPoint {
	int row;
	int col;

	int f;
	int g;
	int h;
};
//���Ľڵ�����
struct treeNode
{
	MyPoint				pos;
	vector<treeNode*>	child;
	treeNode* pParent;
};


//�������ڵ�
treeNode* createTreeNode(int row, int col) {
	treeNode* pNew = new treeNode;//���ڴ�
	memset(pNew, 0, sizeof(treeNode));//ȫ����ֵΪNULL
	pNew->pos.row = row;
	pNew->pos.col = col;
	return pNew;
}

//����hֵ������
int getH(MyPoint pos, MyPoint endPos) {
	int x = ((pos.col > endPos.col) ? (pos.col - endPos.col) : (endPos.col - pos.col));
	int y = ((pos.row > endPos.row) ? (pos.row - endPos.row) : (endPos.row - pos.row));

	return 10 * (x + y);
}