/*
 * Node.cpp
 *
 *  Created on: Nov 16, 2019
 *      Author: Robobot
 */

#include "Node.h"

Node::Node() {

}

Node::Node(bool isStreet) {
	street = isStreet;
}

void Node::setAdjencies(Node *N, Node *E, Node *S, Node *W) {
	nodes[0] = N;
	nodes[1] = E;
	nodes[2] = S;
	nodes[3] = W;
}

int Node::findAdj(Node* adj){
	for(int i = 0; i < 4; i++) {
		if(nodes[i] == adj) {
			return i;
		}
	}
	return 0;
}


