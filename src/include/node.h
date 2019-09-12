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
 * \file node.h
 * \brief OctoNode and OctoTree Construction.
 * @authors VincentXiao
 * @date    2018-08-21 17:40:04
 */

#ifndef _NODE_H
#define _NODE_H

#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include "queue"
#include <string>
//#include <time.h>
#include "../../config/config.h"
#include "../../3rdparty/half-1.12.0/include/half.hpp"
using half_float::half;
#define half_max (half)numeric_limits<half>::max()
#define half_min (half)numeric_limits<half>::min()

using namespace std;
class OctoNode {
public:
	vector<DataType> center;
	DataType halfWidth;
	int   state;//0:empty 1:occupied;2:inner node
	vector<OctoNode*> child;//size=8;
	OctoNode* parent;// used to compression
	bool isLeaf;//is leaf node(contain no child) or not
	int childIndex;// the child number of parent; for matlab experiment
	OctoNode();
	OctoNode(vector<DataType> center, DataType halfWidth, int state, vector<OctoNode*> child, OctoNode* parent);

};
// compression Leaf OctoNode in preoder storage
class OctoComNode {
public:
	vector<DataType> center;
	DataType halfWidth;
	int   state;//0:empty 1:occupied;2:inner node
	int parent;// parent index in storage
	vector<int> child;//size=8; child index in storage
	vector<int> childState;
	int childAccessed;// how many child are accessed;// used to collision detection
	OctoComNode();

};
class OBBNode {
public:
	DataType center[3] = { (DataType)0 };
	DataType halfWidth[3] = { (DataType)0 };
	DataType angle[9] = { (DataType)0 };
};
class box
{
public:
	DataType center[3] = { (DataType)0 };
	DataType halfWidth[3] = { (DataType)0 };
};
class Point
{
public:
	DataType center[3] = { (DataType)0 };
};
class OctoTree
{
public:
	OctoNode *octotree;
	DataType bounds= (DataType)OctoTreeBounds;//environment halfwidth
	long int octonum = 0;
	long int leafnum = 0;
	int maxDepth = OctoTreeMaxDepth;
	vector<box> objectBox;
	OctoTree();
	void clear();//free memory
	void deleteChild(OctoNode *node);
	void OctoNum(OctoNode *node);
	void buildTree(vector<box> objectBox);
	void buildTree(vector<box> objectBox, vector<DataType> center, DataType halfWidth);
	void buildTree(vector<box> objectBox, vector<DataType> center, DataType halfWidth, int depth);
	OctoNode* buildOctotree(vector<DataType> center, DataType halfWidth,int depth);
	//check if overlap between leaf of Octree and box;AABB and AABB
	bool AABBstateCheck(OctoNode * node);
	void printOctoTree();//preorder traversal
	void printOctoTree2(); //Breadth - First - Search
	void printOcto(OctoNode *node);//preorder traversal
	void GetOctoTree(OctoNode * node, vector<OctoNode*> *nodeVector);// queue OctoTree in preorder traversal
	void printOctoFile(string filename);
	void printOctoFileMatlab(string filename);// for matlab experiment
	vector<OctoComNode> buildOctoComTree();
	int GetIndexOctoTree(vector<OctoNode*> nodeVector, OctoNode* node);//get index of node in vector
	void printOctoComFile(string filename, vector<OctoComNode> octoComVector);
	OctoNode* OctoTreeDecoder(vector<OctoComNode> octoComVector,OctoComNode ComNode, int index);
	OctoNode* OctoTreeDecompression(vector<OctoComNode> octoComVector);
};

#endif
