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
 * \file KinematicControl.h
 * \brief Kinematic Computation for Robotics reference to Moviet.
 * @authors VincentXiao 
 * @date    2018-08-11 21:00:45
 */

#ifndef _KINEMATICCONTROL_H
#define _KINEMATICCONTROL_H

#include "node.h"
#include "../../config/config.h"

class TransNode//transformationMatrix of Link node
{
public:
	DataType translation[3] = { (DataType)0 };
	DataType rotation[9] = { (DataType)0 };
	
};
 
/******************END*********************************/

bool CollisionDectection(vector<OBBNode> ObbVector, OctoNode octo);
bool OBBCollisionDectection(OBBNode obb, OctoNode octo);
bool OctoCollisionDectection(vector<OBBNode> ObbVector, OctoNode *rootnode);
bool OBBCollisionDectectionAccurate(OBBNode obb, OctoNode octo);
bool CollisionDectectionOBB_OBB(vector<OBBNode> ObbVector, OctoNode octo);
/***************************transform DH methods***************/
vector<OBBNode> kinematic(vector<DataType> angle, vector<OBBNode>ObbVectorInit);
/***************************transform of moveit not DH methods***************/
vector<OBBNode> kinematicMoveit(vector<DataType> angle, vector<OBBNode>ObbVectorInit);//LInk transfrom
int transformJoint(int index, DataType angle, DataType *out);//Joint transfrom
int transformOrign(int index , DataType *out);//Origin transfrom
int RPY2rotation(DataType roll, DataType pitch, DataType yaw,DataType *out);
int MatixMult(DataType *in0,DataType *in1,DataType *out);//4*4 matrix multiplier
bool checkConfig(vector<DataType> config);

#endif