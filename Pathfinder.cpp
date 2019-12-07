/*
 * Pathfinder.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Conrad Tulig
 */

#include "Pathfinder.h"

Pathfinder::Pathfinder() {
	// TODO Auto-generated constructor stub
	createNodes();
	setBuildings();
	setAdjacencies();
}

list<Node*> Pathfinder::pathfind(Node *start, Node *target) {
	list<Node*> visited = getVisitedList(start, target);
	list<Node*> path;

	Node *current = visited.front();
	visited.pop_front();
	path.push_front(current);

	int length = visited.size();
	for (int i = 0; i < length; i++) {
		Node *next = visited.front();
		visited.pop_front();
		for (int n = 0; n < 4; n++) {
			if (current->nodes[n] == next) {
				path.push_front(next);
				current = next;
			}
		}
	}

	return path;
}

list<Node*> Pathfinder::getVisitedList(Node *start, Node *target) {
	list<Node*> queue;
	list<Node*> visited;

	queue.push_back(start);

	while (!queue.empty()) {
		Node *current = queue.front();
		queue.pop_front();

		visited.push_front(current);
		if (current == target) {
			return visited;
		}

		for (int i = 0; i < 4; i++) {
			if (current->nodes[i]->street
					&& !haveVisited(current->nodes[i], visited)) {
				queue.push_back(current->nodes[i]);
			}
		}
	}

	return visited;
}

bool Pathfinder::haveVisited(Node *current, list<Node*> visited) {
	int length = visited.size();
	for (int i = 0; i < length; i++) {
		Node *temp = visited.front();
		visited.pop_front();
		if (current == temp) {
			return true;
		}
	}
	return false;
}

void Pathfinder::createNodes() {
	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {
			nodes[x][y] = new Node(true);
			nodes[x][y].xPos = x;
			nodes[x][y].yPos = y;
		}
	}
}

void Pathfinder::setBuildings() {
	for (int x = 1; x < 6; x += 2) {
		for (int y = 0; y < 6; y += 2) {
			nodes[x][y].street = false;
			nodes[x][y].buildingLot = true;
		}
	}
}

void Pathfinder::setAdjacencies() {
	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {
			Node *N = checkBounds(x, y + 1);
			Node *E = checkBounds(x + 1, y);
			Node *S = checkBounds(x, y - 1);
			Node *W = checkBounds(x - 1, y);
			nodes[x][y].setAdjencies(N, E, S, W);
		}
	}
}

Node* Pathfinder::checkBounds(int x, int y) {
	if (x >= 6 || y >= 6 || x < 0 || y < 0) {
		return new Node(false);
	}
	return &nodes[x][y];
}

void Pathfinder::printNodes(list<Node*> path) {
	int length = path.size();
	for (int i = 0; i < length; i++) {
		Node* temp = path.front();
		path.pop_front();

		Serial.print("(");
		Serial.print(temp->xPos);
		Serial.print(",");
		Serial.print(temp->yPos);
		Serial.println(")");

	}
}

list<Node*> Pathfinder::pathFindTest(int startX, int startY, int endX,
		int endY) {

	//Road block:
	nodes[3][1].street = false;

	Serial.print("Finding path from (");
	Serial.print(startX);
	Serial.print(",");
	Serial.print(startY);
	Serial.print(") to (");
	Serial.print(endX);
	Serial.print(",");
	Serial.print(endY);
	Serial.println(").");

	list<Node*> path = pathfind(&nodes[startX][startY], &nodes[endX][endY]);

	Serial.print("Length of path: ");
	Serial.println(path.size());

	printNodes(path);
	return path;
}

void Pathfinder::addBuildingsAndRoadBlock() {
	nodes[3][1].street = false;
	nodes[3][1].roadBlock = true;

	nodes[1][2].building = true;
	nodes[1][4].building = true;
	nodes[5][0].building = true;
}


list<Node*> Pathfinder::generateInitialPath() {
	list<Node*> path;

	//path.push_back(&nodes[0][0]);
	path.push_back(&nodes[0][1]);
	path.push_back(&nodes[1][1]);
	path.push_back(&nodes[2][1]);
	path.push_back(&nodes[3][1]);
	path.push_back(&nodes[4][1]);
	path.push_back(&nodes[4][0]);
	path.push_back(&nodes[4][1]);
	path.push_back(&nodes[4][2]);
	path.push_back(&nodes[4][3]);
	path.push_back(&nodes[5][3]);
	path.push_back(&nodes[4][3]);
	path.push_back(&nodes[3][3]);
	path.push_back(&nodes[2][3]);
	path.push_back(&nodes[1][3]);
	path.push_back(&nodes[0][3]);
	path.push_back(&nodes[0][2]);
	path.push_back(&nodes[0][1]);
	path.push_back(&nodes[0][0]);



	return path;
}

list<Node*> Pathfinder::addBuildingSearch(list<Node*> final_path,
		Node* building) {
	list<Node*> path;
	Node* start = final_path.front();
	Node* current = start;
	final_path.pop_front();
	Node* next = final_path.front();
	final_path.push_front(start);
	Node* target;

	for (int i = 2; i < 6; i++) {
		int pos = i % 4;
		if (building->nodes[pos]->street) {
			target = building->nodes[pos];
			path = pushListBack(path, pathfind(current, target));
			current = target;
		}
	}

	path = pushListBack(path, pathfind(current, next));
	final_path = pushListBack(path, final_path);
	final_path.push_front(start);

	return final_path;
}

list<Node*> Pathfinder::pushListBack(list<Node*> orig, list<Node*> added) {
	added.pop_front();
	int length = added.size();
	for (int i = 0; i < length; i++) {
		orig.push_back(added.front());
		added.pop_front();
	}

	return orig;
}
