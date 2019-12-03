/*
 * Pathfinder.h
 *
 *  Created on: Nov 20, 2019
 *      Author: Conrad Tulig
 */
#include <Arduino.h>
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
	list<Node*> pathFindTest(int startX, int startY, int endX, int endY);
	list<Node*> generateInitialPath();
	list<Node*> addBuildingSearch(Node* current, Node* building);
	list<Node*> Pathfinder::pushListBack(list<Node*> orig, list<Node*> added);
};

#endif /* PATHFINDER_H_ */
