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
	void addBuildingsAndRoadBlock();
	list<Node*> generateInitialPath();
	list<Node*> addBuildingSearch(list<Node*> final_path, Node* building);
	list<Node*> newAddBuildingSearch(Node* building);
	list<Node*> pushListBack(list<Node*> orig, list<Node*> added);
	Node* getNode(int x, int y);
	list<Node*> priorityQueue();
	int getAdjacentDirection(Node* current, Node* adj);
};

#endif /* PATHFINDER_H_ */
