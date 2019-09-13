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
 * \file config.h
 * \brief Configuration for ORMPI.
 * @authors VincentXiao
 * @date    2019-09-12 20:00:03
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#define _USE_MATH_DEFINES
#include <math.h>

/************DataType Config for float or half ************/
typedef float  DataType;//float or half	
/************Debug Info Set ************/
#define TESTMode 0 //1:print debug info;0:off info
/************OctoTree Config ************/
#define OctoTreeMaxDepth 5
#define OctoTreeBounds 2
/************RRT Config ************/									   
#define RRTBias 0.1 
#define RRTStepSize 0.2 
#define RRTMaxIteration 1000 
/************Itration Limit ************/
#define loopNum 100

/******************Robot config*********************************/
#define FreeDom 7

const DataType OBB_HalfWidth[FreeDom][3] = { { (DataType)0.1778,(DataType)0.1270,(DataType)0.3166 },
											 { (DataType)0.1242,(DataType)0.0764,(DataType)0.1270 },
											 { (DataType)0.0115,(DataType)0.0691,(DataType)0.0721 },
											 { (DataType)0.0864,(DataType)0.0464,(DataType)0.0801 },
											 { (DataType)0.0655,(DataType)0.0414,(DataType)0.0421 },
											 { (DataType)0.0323,(DataType)0.0213,(DataType)0.0390 },
											 { (DataType)0.0085,(DataType)0.0189,(DataType)0.0189 } };
const DataType OBB_AngleUpperLimit[FreeDom] = { (DataType)0.564601836603,(DataType)1.2963,(DataType)0.65,(DataType)-0.15,(DataType)M_PI,- (DataType)0.10,(DataType)M_PI };
const DataType OBB_AngleLowerLimit[FreeDom] = { (DataType)-2.1353981634,(DataType)-0.3536,(DataType)-3.75,(DataType)-2.1213,-(DataType)M_PI,(DataType)-2.0,(DataType)-M_PI };

//OBB center coordinate related to joint origin;
const DataType OBB_CenterInit[FreeDom][3] = { { (DataType)0.0509, (DataType)0.0001, (DataType)-0.1747     },
											  { (DataType)-0.0027,(DataType)-0.0003,(DataType)0.0004      },
											  { (DataType)0.1289, (DataType)0.0030, (DataType)-2.0418e-05 },
											  { (DataType)0.0061, (DataType)-0.0006,(DataType)0.0001      },
											  { (DataType)0.0410, (DataType)-0.0001,(DataType)0.0007      },
											  { (DataType)-0.0047,(DataType)0,      (DataType)-0.0015     },
											  { (DataType)0.0332, (DataType)0,      (DataType)0           } };
;
//transformation matrix for base 
const DataType T0[16] = {(DataType)1,(DataType)0,(DataType)0,(DataType)-0.05,
					     (DataType)0,(DataType)1,(DataType)0,(DataType)0,
					     (DataType)0,(DataType)0,(DataType)1,(DataType)0.958925,
					     (DataType)0,(DataType)0,(DataType)0,(DataType)1};
//transformation matrix/rotation/angle for obb init 
const DataType OBB_AngleInit[9] = { (DataType)1,(DataType)0,(DataType)0,
								    (DataType)0,(DataType)1,(DataType)0,
							        (DataType)0,(DataType)0,(DataType)1};
/***************************transform of moveit not DH methods***************/
//Joint origin transform from ROS pareURDF()
const DataType AxisXYZ[FreeDom][3] = { {(DataType)0, (DataType)0, (DataType)-1},
						 		       {(DataType)0, (DataType)-1,(DataType)0 },
						 		       {(DataType)-1,(DataType)0, (DataType)0 },
						 		       {(DataType)0, (DataType)-1,(DataType)0 },
						 		       {(DataType)-1,(DataType)0, (DataType)0 },
						 		       {(DataType)0, (DataType)-1,(DataType)0 },
						 		       {(DataType)-1,(DataType)0, (DataType)0 } };
const DataType OrignRPY[FreeDom][3] = {(DataType)0};
const DataType OrignXYZ[FreeDom][3]= {{(DataType)0,    (DataType)-0.188,(DataType)0},
									  {(DataType)0.1,  (DataType)0,     (DataType)0},
									  {(DataType)0,    (DataType)0,     (DataType)0},
									  {(DataType)0.4,  (DataType)0,     (DataType)0},
									  {(DataType)0,    (DataType)0,     (DataType)0},
									  {(DataType)0.321,(DataType)0,     (DataType)0},
									  {(DataType)0,    (DataType)0,     (DataType)0}};
//TransOrigin is const in term of OrignRPY and OrignXYZ is const
const DataType TransOriginT[FreeDom][16] = { {1,0,0,0,0,1,0,-0.188,0,0,1,0,0,0,0,1},
										   {1,0,0,0.1,0,1,0,0,0,0,1,0,0,0,0,1},
										   {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},
										   {1,0,0,0.4,0,1,0,0,0,0,1,0,0,0,0,1},
										   {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},
										   {1,0,0,0.321,0,1,0,0,0,0,1,0,0,0,0,1},
										   {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1} };
/*********************end*******************************************************/


#endif