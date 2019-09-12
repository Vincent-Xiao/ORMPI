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
 * \file RRT.h
 * \brief Motion Planning Algotirhm base RRT.
 * @authors VincentXiao
 * @date    2018-08-13 20:14:03
 */

#ifndef _RRT_H
#define _RRT_H

#include "KinematicControl.h" 
#include <random>
#include <float.h>
#include <algorithm> 
#include "../../config/config.h"
#define REACHED 0
#define ADVANCED 1
#define TRAPPED -1

class RRTNode
{
	public:
		vector<DataType> config;
		DataType distance = (DataType)0;//distance from parent
		RRTNode *parent;
		DataType getDistance(RRTNode *node0, vector<DataType> config);//get distance between nodes and config
		DataType getDistance(RRTNode *node0, RRTNode *node1);//get distance between two nodes
		RRTNode(vector<DataType> config, RRTNode *parent);
		RRTNode(vector<DataType> config);
		RRTNode();
};

class RRTTree
{

	public:
		
		vector<RRTNode*> rrtTree;
		void addNode(RRTNode *node);
		void addNode(vector<DataType> config);
		void addNode(vector<DataType> config, RRTNode *parent);
		RRTNode* getNode(int index) { return this->rrtTree[index]; }
		RRTNode* getNearest(vector<DataType> config);
		RRTTree();
		RRTTree(vector<DataType> config);//construct a init RRTree with root RRTNode config
		vector<RRTNode*> getPath(vector<DataType> config);//get navie path from root node to config node;
		void printpath(vector<RRTNode*> path);//print path
		void printRRTree(string filename);//print tree
		int GetIndexRRTree(RRTNode* node);//get index of node in vector
		vector<DataType> ReadPath(string filename);//read path from file
		void WritePath(vector<RRTNode*> path, string filename);//write path to file
	    
		
	    
};
class RRT: public RRTTree
{
public:
	vector<DataType> StartConfig;
	vector<DataType> GoalConfig;
	DataType GoalBias = (DataType)RRTBias;
	DataType stepSize = (DataType)RRTStepSize;
	long int MaxIteration = RRTMaxIteration;
	long int count = 0;
	bool  IsGoal = false;//indicating if node reach goalconfig
	bool  Success = false;//indicating if RRT successed
	vector<OBBNode> ObbVector ;
	vector<OBBNode> ObbVectorInit;
	OctoTree octoTree;
	RRT();
	void rrtTreeInit();
	void OBBInit();//init ObbVectorInit according robot model
	void setMaxIteration(long int num) { this->MaxIteration = num; }
	void setGoalBias(DataType bias);
	void setStart(vector<DataType> config);
	void setGoal(vector<DataType> config);
	void setStepSize(DataType size) { this->stepSize = size; }
	void setOBB(vector<OBBNode> ObbVector) { this->ObbVector = ObbVector; }
	void RRTClear();//clear parameters;
	vector<DataType> SamleConfig();//random config
	int extend(RRTNode *near, vector<DataType> config);//extend from nearest to random 
	int connect(vector<DataType> config);//continue generating new RRTnode to rrttree until reach random
	bool checkCollision(RRTNode *node);
	bool checkCollision(vector<DataType> config);
	bool startRRT();
};

int randomint(int min, int max); // min<=rand<=max
DataType randomdouble(int min, int max);

#endif
