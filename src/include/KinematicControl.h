/**
 * 
 * @authors VincentXiao 
 * @date    2018-08-11 21:00:45
 * @version $Id$
 */

#include "node.h"
#define _USE_MATH_DEFINES
#include <math.h>

/******************Robot config*********************************/
#define FreeDom 7
#define PickPlace 0 //0:pick 1 place
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
/*********************end********************/
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