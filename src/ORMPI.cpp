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
 * \file nn.cc
 * \brief The Open Robotic Motion Planning Implementation.
 * @authors VincentXiao
 * @date    2018-08-13 15:57:15
 */

#include "./include/RRT.h"
#include "../config/config.h"
#include <string>
#include <fstream>
using namespace std;
using namespace chrono;

vector<box> environmentInit();

#define PickPlace 1

int main()
{
	//TestHardware();

	cout << "RRT PLAN TEST" << endl;
	RRT rrt;
	vector<box> objectBox = environmentInit();
	vector<DataType>center;
	center.push_back((DataType)1);
	center.push_back((DataType)0);
	center.push_back((DataType)0.32);//= table center

	rrt.octoTree.buildTree(objectBox, center, (DataType)1, (DataType)5);//place
	
	
#if PickPlace==0
	//pick
	DataType startconfig0[FreeDom] = { (DataType)0,(DataType)0,(DataType)0,(DataType)-1.13565,(DataType)0,(DataType)-1.05,(DataType)0 };
	DataType goalconfig0[FreeDom] = { (DataType)-1.38464, (DataType)0.575081 ,(DataType)-1.29135, (DataType)-1.21074 ,(DataType)-0.758836, (DataType)-0.25828 ,(DataType)1.69313 };
#else
	//place
	DataType startconfig0[FreeDom]={ (DataType)-1.38464, (DataType)0.575081 ,(DataType)- 1.29135, (DataType)- 1.21074 ,(DataType)- 0.758836, (DataType)- 0.25828 ,(DataType)1.69313 };
	DataType goalconfig0[FreeDom]={ (DataType)-0.60892, (DataType)1.296, (DataType)- 1.99498, (DataType)- 1.47596, (DataType)1.7886, (DataType)- 1.36201, (DataType)- 1.36774 };
#endif

	vector<DataType> startconfig;
	vector<DataType> goalconfig;
	for(int i=0;i<FreeDom;i++)
	{
		startconfig.push_back(startconfig0[i]);
		goalconfig.push_back(goalconfig0[i]);
	}

	cout << "startconfig [";
	for (auto config : startconfig) cout << config << " ";
	cout << "]"<<endl;
	cout << "goalconfig [";
	for (auto config : goalconfig) cout << config << " ";
	cout << "]" << endl;
	rrt.setStart(startconfig);
	rrt.setGoal(goalconfig);
	cout << "bias: " << rrt.GoalBias << endl;
	cout << "step_size:" << rrt.stepSize << endl;
	cout << "MaxIteration:" << rrt.MaxIteration << endl;

	cout << "start RRT planning" << endl;
	//kinematic transformation debug
	//rrt.ObbVector = kinematicMoveit(startconfig, rrt.ObbVectorInit);
	int SuccessCount = 0;
	double TimeCosts = 0;
	for (int i = 0; i < loopNum; i++) {
		auto start = system_clock::now();
		rrt.startRRT();
		auto end = system_clock::now();
		auto duration = duration_cast<microseconds>(end - start);
		double currentTime = double(duration.count()) * microseconds::period::num / microseconds::period::den;
		//double currentTime = (double)(end - start) / CLOCKS_PER_SEC;
		if (rrt.Success)
		{
			TimeCosts += currentTime;

		}
#if TESTMode
		if (rrt.Success) {
			vector<RRTNode*> path = rrt.getPath(goalconfig);
			rrt.printpath(path);
			//rrt.octoTree.printOctoTree();
			cout << "octoTree including " << rrt.octoTree.octonum << " nodes" << endl;
			cout << "time costs "<< double(duration.count()) * microseconds::period::num / microseconds::period::den<< " seconds" << endl;			
			rrt.WritePath(path, "../../PATH.txt"); 
			rrt.WritePath(path, "./PATH.txt");
			break;
		}
		//vector<DataType> pathFile = ReadPath("../../PATH.txt");
#endif
		SuccessCount += rrt.Success;
		rrt.RRTClear();
	}
	cout << "success count : " << SuccessCount<<" of "<<loopNum<<" iterations" << endl;
	cout << "time cost average: " << double(TimeCosts / SuccessCount)<<"s" << endl;
	rrt.RRTClear();
	return 0;
}
vector<box> environmentInit()
{
	vector<box> BoxVector;
	box object;
	//table0
	object.center[0] = (DataType)1;
	object.center[1] = (DataType)0;
	object.center[2] = (DataType)0.32;
	object.halfWidth[0] = (DataType)1.2*0.5;
	object.halfWidth[1] = (DataType)1.6*0.5;
	object.halfWidth[2] = (DataType)0.04*0.5;
	BoxVector.push_back(object);
	//table 1
	object.center[0] = (DataType)1;
	object.center[1] = (DataType)-0.58;
	object.center[2] = (DataType)0.15;
	object.halfWidth[0] = (DataType)1.2*0.5;
	object.halfWidth[1] = (DataType)0.04*0.5;
	object.halfWidth[2] = (DataType)0.3*0.5;
	BoxVector.push_back(object);
	//table 2
	object.center[0] = (DataType)1;
	object.center[1] = (DataType)0.58;
	object.center[2] = (DataType)0.15;
	object.halfWidth[0] = (DataType)1.2*0.5;
	object.halfWidth[1] = (DataType)0.04*0.5;
	object.halfWidth[2] = (DataType)0.3*0.5;
	BoxVector.push_back(object);
	//pole
	object.center[0] = (DataType)0.55;
	object.center[1] = (DataType)-0.4;
	object.center[2] = (DataType)0.64;
	object.halfWidth[0] = (DataType)0.3*0.5;
	object.halfWidth[1] = (DataType)0.1*0.5;
	object.halfWidth[2] = (DataType)0.5*0.5;
	BoxVector.push_back(object);
	//pole1
	object.center[0] = (DataType)1.0;
	object.center[1] = (DataType)-0.2;
	object.center[2] = (DataType)0.44;
	object.halfWidth[0] = (DataType)0.3*0.5;
	object.halfWidth[1] = (DataType)0.1*0.5;
	object.halfWidth[2] = (DataType)0.2*0.5;
	BoxVector.push_back(object);
	//pole2
	object.center[0] = (DataType)1.0;
	object.center[1] = (DataType)0.5;
	object.center[2] = (DataType)0.54;
	object.halfWidth[0] = (DataType)0.2*0.5;
	object.halfWidth[1] = (DataType)0.2*0.5;
	object.halfWidth[2] = (DataType)0.4*0.5;
	BoxVector.push_back(object);
	//pole3
	object.center[0] = (DataType)1.4;
	object.center[1] = (DataType)-0.5;
	object.center[2] = (DataType)0.44;
	object.halfWidth[0] = (DataType)0.2*0.5;
	object.halfWidth[1] = (DataType)0.2*0.5;
	object.halfWidth[2] = (DataType)0.2*0.5;
	BoxVector.push_back(object);
	//pole4
	object.center[0] = (DataType)1.4;
	object.center[1] = (DataType)0.5;
	object.center[2] = (DataType)0.44;
	object.halfWidth[0] = (DataType)0.2*0.5;
	object.halfWidth[1] = (DataType)0.2*0.5;
	object.halfWidth[2] = (DataType)0.2*0.5;
	BoxVector.push_back(object);
	return BoxVector;
}

