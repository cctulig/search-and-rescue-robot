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

void Node::setType(bool isStreet) {
	street = isStreet;
}

bool Node::isStreet() {
	return street;
}
