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

	bool street;

public:
	Node();
	Node(bool isStreet);
	Node *nodes[4];
	float weights[4];
	int xPos;
	int yPos;

	void setAdjencies(Node *N, Node *E, Node *S, Node *W);
	void setWeights(float N, float E, float S, float W);
	void setType(bool isStreet);
	bool isStreet();

};

#endif /* NODE_H_ */

