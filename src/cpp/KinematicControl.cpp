
/**
 * 
 * @authors VincentXiao (you@example.org)
 * @date    2018-08-11 18:40:25
 * @version $Id$
 * RRT soft including RRT,collision dectection
 */

#include "../include/KinematicControl.h"
TransNode  transformationMatrix[FreeDom];//transformationMatrix of each link
vector<OBBNode> kinematicMoveit(vector<DataType> angle, vector<OBBNode> ObbVectorInit)
{
	vector<OBBNode> ObbVectorOut;
	OBBNode obbnode;
	int size = ObbVectorInit.size();//freedom
	DataType transM[16];//transformation matrix of T0
	DataType transOrigin[16];
	DataType transJoint[16];
	for (int i = 0; i < 16; i++) transM[i] = T0[i];

	for (int i = 0; i < size; i++)//T=T0*T1*....Ti
	{		
		transformJoint(i, angle[i], transJoint);
		//transformOrign(i, transOrigin);//TransOrigin is const in term of OrignRPY and OrignXYZ is const
		for (int k = 0; k < 16; k++) transOrigin[k] = TransOriginT[i][k];
		DataType transM0[16];
		MatixMult(transM, transOrigin, transM0);
		MatixMult(transM0, transJoint, transM);//T=T0*T1*...Ti
		obbnode.angle[0] = transM[0 * 4 + 0];
		obbnode.angle[1] = transM[0 * 4 + 1];
		obbnode.angle[2] = transM[0 * 4 + 2];
		obbnode.angle[3] = transM[1 * 4 + 0];
		obbnode.angle[4] = transM[1 * 4 + 1];
		obbnode.angle[5] = transM[1 * 4 + 2];
		obbnode.angle[6] = transM[2 * 4 + 0];
		obbnode.angle[7] = transM[2 * 4 + 1];
		obbnode.angle[8] = transM[2 * 4 + 2];
		obbnode.center[0] = transM[0 * 4 + 3];
		obbnode.center[1] = transM[1 * 4 + 3];
		obbnode.center[2] = transM[2 * 4 + 3];
		obbnode.halfWidth[0] = ObbVectorInit[i].halfWidth[0];
		obbnode.halfWidth[1] = ObbVectorInit[i].halfWidth[1];
		obbnode.halfWidth[2] = ObbVectorInit[i].halfWidth[2];
		ObbVectorOut.push_back(obbnode);
	}

	return ObbVectorOut;
}
int transformJoint(int index, DataType angle, DataType * out)
{
	//RevoluteJointModel
	DataType x, y, z;
	x = AxisXYZ[index][0];
	y = AxisXYZ[index][1];
	z = AxisXYZ[index][2];
	DataType c = cos(angle);
	DataType s = sin(angle);
	DataType t = (DataType)1.0 - c;
	DataType txy = t * x*y;
	DataType txz = t * x*z;
	DataType tyz = t * y*z;
	DataType zs = z * s;
	DataType ys = y * s;
	DataType xs = x * s;

	out[0]= t * x*x + c;
	out[1]= txy + zs;
	out[2]= txz - ys;
	out[3]= (DataType)0.0;
	out[4]= txy - zs;
	out[5]= t * y*y + c;
	out[6]= tyz + xs;
	out[7]= (DataType)0.0;
	out[8]= txz + ys;
	out[9] = tyz - xs;
	out[10] = t * z*z + c;
	out[11] = (DataType)0.0;
	out[12] = (DataType)0.0;
	out[13] = (DataType)0.0;
	out[14] = (DataType)0.0;
	out[15] = (DataType)1.0;

	return 0;
}
int transformOrign(int index, DataType * out)
{
	DataType rotation[9] = { (DataType)0 };
	RPY2rotation(OrignRPY[index][0], OrignRPY[index][1], OrignRPY[index][2],rotation );
	out[0] = rotation[0];
	out[1] = rotation[1];
	out[2] = rotation[2];
	out[3] = OrignXYZ[index][0];
	out[4] = rotation[3];
	out[5] = rotation[4];
	out[6] = rotation[5];
	out[7] = OrignXYZ[index][1];
	out[8] = rotation[6];
	out[9] = rotation[7];
	out[10] = rotation[8];
	out[11] = OrignXYZ[index][2];
	out[12] = (DataType)0;
	out[13] = (DataType)0;
	out[14] = (DataType)0;
	out[15] = (DataType)1;
	return 0;
}
int RPY2rotation(DataType roll, DataType pitch, DataType yaw, DataType * out)
{
	DataType ca1, cb1, cc1, sa1, sb1, sc1;
	ca1 = cos(yaw); sa1 = sin(yaw);
	cb1 = cos(pitch); sb1 = sin(pitch);
	cc1 = cos(roll); sc1 = sin(roll);
	out[0] = ca1*cb1; 
	out[1] = ca1*sb1*sc1 - sa1*cc1;
	out[2] = ca1*sb1*cc1 + sa1*sc1;
	out[3] = sa1*cb1;
	out[4] = sa1*sb1*sc1 + ca1*cc1;
	out[5] = sa1*sb1*cc1 - ca1*sc1;
	out[6] = -sb1;
	out[7] = cb1*sc1;
	out[8]=cb1*cc1;
	return 0;
}
int MatixMult(DataType *in0, DataType *in1, DataType *out) {//4*4 matrix multiplier
	for(int i=0;i<4;i++)
	{
		*(out + i * 4 + 0) = *(in0 + i * 4 + 0)**(in1 + 0 * 4 + 0) + *(in0 + i * 4 + 1)**(in1 + 1 * 4 + 0) + *(in0 + i * 4 + 2)**(in1 + 2 * 4 + 0) + *(in0 + i * 4 + 3)**(in1 + 3 * 4 + 0);
		*(out + i * 4 + 1) = *(in0 + i * 4 + 0)**(in1 + 0 * 4 + 1) + *(in0 + i * 4 + 1)**(in1 + 1 * 4 + 1) + *(in0 + i * 4 + 2)**(in1 + 2 * 4 + 1) + *(in0 + i * 4 + 3)**(in1 + 3 * 4 + 1);
		*(out + i * 4 + 2) = *(in0 + i * 4 + 0)**(in1 + 0 * 4 + 2) + *(in0 + i * 4 + 1)**(in1 + 1 * 4 + 2) + *(in0 + i * 4 + 2)**(in1 + 2 * 4 + 2) + *(in0 + i * 4 + 3)**(in1 + 3 * 4 + 2);
		*(out + i * 4 + 3) = *(in0 + i * 4 + 0)**(in1 + 0 * 4 + 3) + *(in0 + i * 4 + 1)**(in1 + 1 * 4 + 3) + *(in0 + i * 4 + 2)**(in1 + 2 * 4 + 3) + *(in0 + i * 4 + 3)**(in1 + 3 * 4 + 3);
	}
	return 0;
}
bool checkConfig(vector<DataType> config)
{
	for(int i=0;i<FreeDom;i++)
	{
		if (config[i] > OBB_AngleUpperLimit[i] || config[i] < OBB_AngleLowerLimit[i])
		{
			cout << "Error:config beyond limit" << endl;
			return false;
		}
	}
	return true;
}
//Pre-Order Traversal
bool OctoCollisionDectection(vector<OBBNode> ObbVector, OctoNode *rootnode)
{
	bool CollisionState = CollisionDectection(ObbVector, *rootnode);
	if (CollisionState == false) return false;
	else if (CollisionState == true && rootnode->isLeaf) return true;
	else 
	{
		for (int i = 0; i < 8; i++)//traversal octree
		{
			if (rootnode->child[i] != nullptr)
			{
				if (OctoCollisionDectection(ObbVector, rootnode->child[i]) == true) return true;
			}
		}
		return false;
	}
	return false;
}
bool CollisionDectection(vector<OBBNode> ObbVector, OctoNode octo)
{

	if (octo.state == 0) return false;
	else for (auto obb:ObbVector)
	{
		bool centerEqual=true;
		//check if octo center inside obb
		for (int i = 0; i < 3; i++) {
			DataType minAixs = obb.center[i] - obb.halfWidth[i];
			DataType maxAxis = obb.center[i] + obb.halfWidth[i];
			if(octo.center[i]<minAixs||octo.center[i]>maxAxis)
			{
				centerEqual = false;
				break;
			}
		}
		if (centerEqual && octo.state == 1) return true;
		else {
			bool state = OBBCollisionDectection(obb, octo);
			//bool state = OBBCollisionDectectionAccurate(obb, octo);
			if (state == true) return true;
		}
	}

	return false;
}
bool CollisionDectectionOBB_OBB(vector<OBBNode> ObbVector, OctoNode octo)
{
	 for (auto obb : ObbVector)
	{

		bool stateAccurate = OBBCollisionDectectionAccurate(obb, octo);
		if (stateAccurate == true) return true;
	}

	return false;
}
bool OBBCollisionDectection(OBBNode obb, OctoNode octo)
{
	DataType T[3];
	DataType ra, rb, L;
	for (int i = 0; i<3; i++) T[i] = obb.center[i] - octo.center[i];

	for (int i = 0; i<3; i++)
	{
		ra = octo.halfWidth;
		rb = obb.halfWidth[0] * fabs(obb.angle[0 * 3 + i]) + obb.halfWidth[1] * fabs(obb.angle[1 * 3 + i])
			+ obb.halfWidth[2] * fabs(obb.angle[2 * 3 + i]);
		L = fabs(T[i]);
		if (L > ra + rb) return false;
	}
	//axis B0,B1,B2
	for (int i = 0; i<3; i++)
	{
		ra = octo.halfWidth*fabs(obb.angle[i * 3 + 0]) + octo.halfWidth*fabs(obb.angle[i * 3 + 1])
			+ octo.halfWidth*fabs(obb.angle[i * 2 + 2]);
		rb = obb.halfWidth[i];
		L = fabs(T[0] * obb.angle[i * 3 + 0] + T[1] * obb.angle[i * 3 + 1] + T[2] * obb.angle[i * 3 + 2]);
		if (L > ra + rb) return false;
	}
	
	return true;
}
#include<stdlib.h>
bool OBBCollisionDectectionAccurate(OBBNode obb, OctoNode octo)
{
	DataType R[3][3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = obb.angle[i * 3 + j] * rand();
	DataType T[3];
	DataType ra, rb, L;
	for (int i = 0; i<3; i++) T[i] = obb.center[i] - octo.center[i];
	DataType t[3];
	for (int i = 0; i < 3; i++) t[i] = T[i] * R[i][0];
	for (int i = 0; i<3; i++)
	{
		ra = octo.halfWidth;
		rb = obb.halfWidth[0] * fabs(obb.angle[0 * 3 + i]) + obb.halfWidth[1] * fabs(obb.angle[1 * 3 + i])
			+ obb.halfWidth[2] * fabs(obb.angle[2 * 3 + i]);
		L = fabs(T[i]);
		if (L > ra + rb) return false;
	}
	//axis B0,B1,B2
	for (int i = 0; i<3; i++)
	{
		ra = octo.halfWidth*fabs(obb.angle[i * 3 + 0]) + octo.halfWidth*fabs(obb.angle[i * 3 + 1])
			+ octo.halfWidth*fabs(obb.angle[i * 2 + 2]);
		rb = obb.halfWidth[i];
		L = fabs(T[0] * obb.angle[i * 3 + 0] + T[1] * obb.angle[i * 3 + 1] + T[2] * obb.angle[i * 3 + 2]);
		if (L > ra + rb) return false;
	}

	//axis A-B
		for (int i=0;i<3;i++)
	{
	for(int j=0;j<3;j++)
	{
	ra = octo.halfWidth*fabs(obb.angle[j * 3 + (i == 2 ? 1 : 2)]) + octo.halfWidth*fabs(obb.angle[j * 3 + (i == 0 ? 1 : 0)]);
	rb = obb.halfWidth[j == 0 ? 1 : 0] * fabs(obb.angle[(j == 2 ? 1 : 2) * 3 + i]) + obb.halfWidth[j == 2 ? 1 : 2] * fabs(obb.angle[(j == 0 ? 1 : 0) * 3 + i]);
	L = fabs(T[i == 0 ? 2 : i == 1 ? 0 : 1] * obb.angle[j * 3 + (i == 0 ? 1 : i == 1 ? 2 : 0)] - T[i == 0 ? 1 : i == 1 ? 2 : 0] * obb.angle[j * 3 + (i == 0 ? 2 : i == 1 ? 0 : 1)]);
	if (L > ra + rb) return false;
	}
	}
	return true;
}