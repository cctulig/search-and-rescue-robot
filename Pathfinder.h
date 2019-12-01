/*
 * Pathfinder.h
 *
 *  Created on: Nov 20, 2019
 *      Author: Conrad Tulig
 */
#include <iostream>
#include <list>
#include "Node.h"
using namespace std;

#ifndef PATHFINDER_H_
#define PATHFINDER_H_

class Pathfinder {
private:
	Node nodes[6][6];

public:
	Pathfinder();

	void createNodes();
	void setBuildings();
	void setAdjacencies();
	Node* checkBounds(int x, int y);
	list<Node*> pathfind(Node *start, Node *target);
	list<Node*> getVisitedList(Node *start, Node *target);
	bool haveVisited(Node *current, list<Node*> visisted);
	void printNodes(list<Node*> path);
};

#endif /* PATHFINDER_H_ */