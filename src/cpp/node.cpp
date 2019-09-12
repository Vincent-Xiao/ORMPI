/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/*!
 *  Copyright (c) 2018 by Contributors
 * \file node.cpp
 * \brief OctoNode and OctoTree Construction.
 * @authors VincentXiao
 * @date    2018-08-21 17:40:19
 */
#include "../include/node.h"

OctoNode::OctoNode()
{
	for (int i = 0; i < 3; i++) this->center.push_back((DataType)0);
	this->halfWidth = 0;
	for (int i = 0; i < 8; i++)this->child.push_back(nullptr);
	this->state = 0;
	this->isLeaf = true;
	this->parent = nullptr;
	this->childIndex = 0;
}

OctoNode::OctoNode(vector<DataType> center, DataType halfWidth, int state, vector<OctoNode*> child, OctoNode* parent)
{
	this->center = center;
	this->halfWidth = halfWidth;
	this->state = state;
	this->child = child;
	this->isLeaf = this->child[0] == nullptr&&this->child[1] == nullptr&&this->child[2] == nullptr&&this->child[3] == nullptr
				 &&this->child[4] == nullptr&&this->child[5] == nullptr&&this->child[6] == nullptr&&this->child[7] == nullptr;
	this->parent = parent;
	this->childIndex = 0;
}




OctoTree::OctoTree()
{
	this->octotree = nullptr;
}

void OctoTree::clear()
{
	//this->deleteChild(this->octotree);
	this->octonum = 0;
	this->leafnum = 0;
	this->octotree->child.clear();
}
//Post-Order Traversal
void OctoTree::deleteChild(OctoNode * node)
{
	if (node != nullptr) {
		for (int i = 0; i < 8; i++)
		{
			deleteChild(node->child[i]);
		}
		delete node;		
	}
}
// delete node and childs octonum
//Pre-Order Traversal
void OctoTree::OctoNum(OctoNode * node)
{
	if (node != nullptr) {
		this->octonum++;
		if (node->isLeaf == true) this->leafnum++;
		else {
			for (int i = 0; i < 8; i++) //traverse child
			{
				this->OctoNum(node->child[i]);
			}
		}
	}
}
void OctoTree::buildTree(vector<box> objectBox)
{
	cout << "bounds: " << this->bounds << endl;
	cout << "maxDepth: " << this->maxDepth << endl;
	cout << "start building OctoTree" << endl;
	this->objectBox = objectBox;
	vector<DataType> center;
	for (int i = 0; i < 3; i++) center.push_back((DataType)0);
	DataType halfWidth = this->bounds;
	this->octotree = this->buildOctotree(center, halfWidth,this->maxDepth);
	this->OctoNum(this->octotree);
}

void OctoTree::buildTree(vector<box> objectBox, vector<DataType> center, DataType halfWidth)
{
	cout << "bounds: " << halfWidth << endl;
	cout << "maxDepth: " << this->maxDepth << endl;
	cout << "start building OctoTree" << endl;
	this->objectBox = objectBox;
	this->octotree = this->buildOctotree(center, halfWidth,this->maxDepth);
	this->OctoNum(this->octotree);

}
void OctoTree::buildTree(vector<box> objectBox, vector<DataType> center, DataType halfWidth, int depth)
{
	cout << "bounds: " << halfWidth << endl;
	cout << "maxDepth: " << this->maxDepth << endl;
	cout << "start building OctoTree" << endl;
	this->objectBox = objectBox;
	this->maxDepth = depth;
	this->octotree = this->buildOctotree(center, halfWidth, this->maxDepth);
	this->OctoNum(this->octotree);

}
//Post-Order Traversal
OctoNode * OctoTree::buildOctotree(vector<DataType> center, DataType halfWidth,int depth)
{
	OctoNode *rootNode = new OctoNode();
	rootNode->center = center;
	rootNode->halfWidth = halfWidth;
	rootNode->state= this->AABBstateCheck(rootNode);
	// if node is empty or occupied depth==0,no need to build child tree
	//return node with null child
	if (depth==0 || rootNode->state == 0)//rootNode->state ==0 empty
	{
		rootNode->isLeaf = true;
	}
	else {
		rootNode->isLeaf = false;
		vector<DataType> childCenter;
		DataType x, y, z;
		DataType childHalfWidth = halfWidth*(DataType)0.5;
		int childDepth = depth-1;
		bool intersect = false;
		bool occupancy = true;
		for (int i = 0; i < 8; i++)//child
		{
			x = (i & 1) ? (center[0] + childHalfWidth) : (center[0] - childHalfWidth);
			y = (i & 2) ? (center[1] + childHalfWidth) : (center[1] - childHalfWidth);
			z = (i & 4) ? (center[2] + childHalfWidth) : (center[2] - childHalfWidth);
			childCenter.push_back(x);//x
			childCenter.push_back(y);//y
			childCenter.push_back(z);//z
			rootNode->child[i] = buildOctotree(childCenter, childHalfWidth,childDepth);
			rootNode->child[i]->parent = rootNode;
			rootNode->child[i]->childIndex = i;
			childCenter.clear();
			if (rootNode->child[i]->state > 0) intersect = true;
			if (rootNode->child[i]->state !=1) occupancy = false;
		}
		 if (intersect == false) {
			 rootNode->state = 0;//empty
			 rootNode->isLeaf = true;
		 }
		else if(occupancy == true){
			rootNode->state = 1;//occupancied
			rootNode->isLeaf = true;
		}
		else rootNode->state = 2;//inner
	}
	return rootNode;
}
//check if overlap between leaf of Octree and box;AABB and AABB
bool OctoTree::AABBstateCheck(OctoNode * node)
{
	for (auto box : this->objectBox)
	{
		bool overlap = true;
		for (int i = 0; i < 3; i++)//x,y,z
		{
			DataType minA= node->center[i] - node->halfWidth;
			DataType maxA = node->center[i] + node->halfWidth;
			DataType minB = box.center[i] - box.halfWidth[i];
			DataType maxB = box.center[i] + box.halfWidth[i];
			if (minA > maxB || minB > maxA)
			{
				overlap =false;
				break;
			}
		}
		if (overlap) return true;
	}
	return false;
}

void OctoTree::printOctoTree2() {
	queue<OctoNode *> unvisited;//queue fifo
	OctoNode *current;

	unvisited.push(this->octotree); 

	while (!unvisited.empty()) { 
		current = (unvisited.front());
		if (current->isLeaf == false) {
			for (int i = 0; i < 8; i++) {
				if (current->child[i] != nullptr) unvisited.push(current->child[i]);
			}
		}
		cout << "center: ";
		for (int i = 0; i < 3; i++) cout << current->center[i] << " ";
		cout << "halfWidth: " << current->halfWidth << " state: " << current->state << " isLeaf: " << current->isLeaf << endl;
		unvisited.pop();
	}
}
void OctoTree::printOctoTree()
{
	this->printOcto(this->octotree);
}

//preorder traversal
void OctoTree::printOcto(OctoNode * node)
{

	if (node != nullptr) {
		cout << "center: ";
		for (int i = 0; i < 3; i++) cout << node->center[i] << " ";
		cout << "halfWidth: " << node->halfWidth << " state: " << node->state << " isLeaf: " << node->isLeaf << endl;
		if (node->isLeaf == false) {
			for (int i = 0; i < 8; i++) //traverse child
			{
				this->printOcto(node->child[i]);
			}
		}
	}
}
//preorder traversal
void OctoTree::GetOctoTree(OctoNode * node, vector<OctoNode*> *nodeVector)
{

	if (node != nullptr) {
		nodeVector->push_back(node);
		if (node->isLeaf == false) {
			for (int i = 0; i < 8; i++) //traverse child
			{
				this->GetOctoTree(node->child[i], nodeVector);
			}
		}
	}
}
//preorder traversal
void OctoTree::printOctoFile(string filename)
{	
	vector<OctoNode*> nodeVector;
	this->GetOctoTree(this->octotree, &nodeVector);
	ofstream out(filename.c_str(), ios::trunc);
	if (!out.is_open()) cout << "Error writting " << filename << endl;
	else {
		for (auto node :nodeVector) {
			for (int i = 0; i < 3; i++) out << node->center[i] << " ";
			out << node->halfWidth << " " << node->state << " " << node->isLeaf << endl;
		}
	}
	out.close();
	cout << "writing path to " << filename << " success" << endl;
}
void OctoTree::printOctoFileMatlab(string filename)
{
	vector<OctoNode*> nodeVector;
	this->GetOctoTree(this->octotree, &nodeVector);
	ofstream out(filename.c_str(), ios::trunc);
	if (!out.is_open()) cout << "Error writting " << filename << endl;
	else {
		for (auto node : nodeVector) {
			int nodeLeaf=node->isLeaf;
			int nodeIndex = GetIndexOctoTree(nodeVector, node);
			int childIndex = nodeLeaf == 1 ? -1 : GetIndexOctoTree(nodeVector, node->child[0] );
			int parentIndex= nodeIndex==0 ? -1: GetIndexOctoTree(nodeVector, node->parent);
			int nearestIndex;
			OctoNode* current = node;
			OctoNode* parent = node->parent;
			if (nodeIndex == 0) nearestIndex = -1;
			else {
				while (1) {
					if (current->childIndex == 7 && GetIndexOctoTree(nodeVector, parent) == 0) {// parent is rootNode
						nearestIndex = -1;
						break;
					}
					else if (current->childIndex == 7) {
						current = parent;
						parent = parent->parent;
					}
					else {
						nearestIndex = GetIndexOctoTree(nodeVector, parent->child[current->childIndex + 1]);// the next child of parent
						break;
					}
				}
			}
			 out << nodeIndex+1 << " "<< nodeLeaf<<" "<< childIndex+1<<" "<< nearestIndex+1<<endl;
			
		}
	}
	out.close();
	cout << "writing path to " << filename << " success" << endl;
}
void OctoTree::printOctoComFile(string filename, vector<OctoComNode> octoComVector)
{
	OctoComNode node;
	ofstream out(filename.c_str(), ios::trunc);
	if (!out.is_open()) cout << "Error writting " << filename << endl;
	else {
		for (auto node : octoComVector) {
			for (int i = 0; i < 3; i++) out << node.center[i] << " ";
			out << node.halfWidth << " " << node.state << " "<<node.parent<<" ";
			for (int i = 0; i < 8; i++) out << node.child[i] << " ";
			for (int i = 0; i < 8; i++) out << node.childState[i] << " ";
			out << endl;
		}
	}
	out.close();
	cout << "writing path to " << filename << " success" << endl;
}

int OctoTree::GetIndexOctoTree(vector<OctoNode*> nodeVector, OctoNode * node)
{
	for (int i = 0; i < nodeVector.size(); i++) {
		if (node == nodeVector[i]) return i;
	}
	return 0;
}
vector<OctoComNode> OctoTree::buildOctoComTree()
{
	vector<OctoComNode> octoComVector;
	vector<OctoNode*> OctoVector;
	vector<OctoNode*> OctoNonLeafVector;
	this->GetOctoTree(this->octotree, &OctoVector);
	// formalize non-leaf OctoTree vector
	for (int i = 0; i < OctoVector.size(); i++) {
		if (OctoVector[i]->isLeaf == false) {
			OctoNonLeafVector.push_back(OctoVector[i]);
		}
	}
	// formalize node index
	for (int i = 0; i < OctoNonLeafVector.size(); i++) {
		OctoComNode ComNode;
		ComNode.center = OctoNonLeafVector[i]->center;
		ComNode.halfWidth = OctoNonLeafVector[i]->halfWidth;
		ComNode.state = OctoNonLeafVector[i]->state;
		ComNode.parent = this->GetIndexOctoTree(OctoNonLeafVector, OctoNonLeafVector[i]->parent);
		for (int j = 0; j < 8; j++) {
			ComNode.child[j] = this->GetIndexOctoTree(OctoNonLeafVector, OctoNonLeafVector[i]->child[j]);
			ComNode.childState[j] = OctoNonLeafVector[i]->child[j]->state;
		}
		octoComVector.push_back(ComNode);
	}

	return octoComVector;
}

OctoComNode::OctoComNode()
{
	for (int i = 0; i < 3; i++) this->center.push_back((DataType)0);
	this->halfWidth = 0;
	this->parent=0;
	this->state = 0;
	for (int i = 0; i < 8; i++)this->child.push_back((DataType)0);
	for (int i = 0; i < 8; i++)this->childState.push_back((DataType)0);
	this->childAccessed = (DataType)0;
}
OctoNode * OctoTree::OctoTreeDecoder(vector<OctoComNode> octoComVector,OctoComNode ComNode, int index)
{
	OctoNode * octo = new OctoNode;
	if (ComNode.childState[index] == 2) {
		int childIndex = ComNode.child[index];
		OctoComNode Node = octoComVector[childIndex];
		octo->center = Node.center;
		octo->halfWidth = Node.halfWidth;
		octo->state = Node.state;
		octo->isLeaf = false;
		for (int i = 0; i < 8; i++) octo->child[i] = this->OctoTreeDecoder(octoComVector, Node, i);
	}
	else { //leaf node
		int childIndex = ComNode.child[index];
		DataType halfWidth = ComNode.halfWidth*0.5;
		octo->center[0] = (index & 1) ? (ComNode.center[0] + halfWidth) : (ComNode.center[0] - halfWidth);
		octo->center[1] = (index & 2) ? (ComNode.center[1] + halfWidth) : (ComNode.center[1] - halfWidth);
		octo->center[2] = (index & 4) ? (ComNode.center[2] + halfWidth) : (ComNode.center[2] - halfWidth);
		octo->halfWidth = halfWidth;
		octo->state = ComNode.childState[index];
		octo->isLeaf = true;
	}
	return octo;
}

OctoNode * OctoTree::OctoTreeDecompression(vector<OctoComNode> octoComVector)
{
	if (octoComVector.size() == 0) return nullptr;
	else {
		OctoNode * octo = new OctoNode;
		OctoComNode Node = octoComVector[0];
		octo->center = Node.center;
		octo->halfWidth = Node.halfWidth;
		octo->state = Node.state;
		octo->isLeaf = false;
		for (int i = 0; i < 8; i++) {
			octo->child[i] = this->OctoTreeDecoder(octoComVector, Node, i);
		}

		return octo;
	}
}
