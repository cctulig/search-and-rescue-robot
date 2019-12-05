/*
 * Node.h
 *
 *  Created on: Nov 16, 2019
 *      Author: Robobot
 */

#ifndef NODE_H_
#define NODE_H_

class Node {

private:

public:
	Node();
	Node(bool isStreet);
	Node *nodes[4];
	int xPos;
	int yPos;

	bool street = true;
	bool buildingLot = false;
	bool building = false;
	bool hasChecked = false;

	void setAdjencies(Node *N, Node *E, Node *S, Node *W);
};

#endif /* NODE_H_ */

