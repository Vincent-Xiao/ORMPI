/**
 * 
 * @authors VincentXiao (you@example.org)
 * @date    2018-08-13 20:14:03
 * @version $RRT Planner$
 */
#include "KinematicControl.h" 
#include <random>
#include <float.h>
#include <algorithm> 
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
	    
		
	    
};
class RRT: public RRTTree
{
public:
	vector<DataType> StartConfig;
	vector<DataType> GoalConfig;
	DataType GoalBias = (DataType)0.1;
	DataType stepSize = (DataType)0.2;
	long int MaxIteration = 1000;
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
