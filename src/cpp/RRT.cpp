#include "../include/RRT.h"
using namespace chrono;
/**
 * 
 * @authors VincentXiao (you@example.org)
 * @date    2018-08-13 20:14:36
 * @version $Id$
 */

int sampleIndex = -1;
vector<DataType> ReadConfig(string filename)//read config from file
{
	vector<DataType> path;
	ifstream in(filename.c_str(), ios::in);
	if (!in.is_open()) cout << "Error reading " << filename << endl;
	else
	{
		char line[128] = { '\0' };
		string num = "\0";
		while (!in.eof())
		{
			in.getline(line, 128);
			for (int i = 0; i<128; i++)
			{
				if (line[i] == '\0') {
					break;
				}
				else if (line[i] == ' ')
				{
					path.push_back((DataType)atof(num.c_str()));
					num = "\0";
				}
				else
				{
					num += line[i];
				}
			}

		}
	}
	in.close();
	cout << "Reading config from " << filename << " success" << endl;
	return path;
}
 /*************************RRTNode*****************/
DataType RRTNode::getDistance(RRTNode * node0, vector<DataType> config)
{
	DataType distance = (DataType)0;
	for (int i = 0; i<FreeDom; i++)
	{
		distance = distance + pow(node0->config[i] - config[i], 2);
	}
	return sqrt(distance);
}

DataType RRTNode::getDistance(RRTNode * node0, RRTNode * node1)
{
	return this->getDistance(node0, node1->config);
}

RRTNode::RRTNode(vector<DataType> config, RRTNode * parent)
{
	this->config = config;
	this->parent = parent;
	//this->distance = this->getDistance(this->parent, config);
	this->distance = 0;
}
RRTNode::RRTNode(vector<DataType> config)
{
	this->config = config;
	this->parent = nullptr;
	this->distance = 0;
}
RRTNode::RRTNode()
{
	for (int i = 0; i < FreeDom; i++) this->config.push_back(half_max);
	this->distance = 0;
	this->parent = nullptr;
}
/***************RRTTree**********************/
void RRTTree::addNode(RRTNode *node)
{
	this->rrtTree.push_back(node);
}

void RRTTree::addNode(vector<DataType> config)
{
	this->addNode(new RRTNode(config));
}

void RRTTree::addNode(vector<DataType> config, RRTNode * parent)
{
	this->addNode(new RRTNode(config,parent));
}

RRTNode* RRTTree::getNearest(vector<DataType> config)
{
	int index=0;
	//DataType min = DBL_MAX;
	DataType min = half_max; //max half 
	int size = this->rrtTree.size();
	//cout << "nearest hardware clocks " << hardwareClocks << endl;
	for(int i=0;i<size;i++)
	{
		double distance=this->rrtTree[i]->getDistance(this->rrtTree[i],config);
		if(distance<min)
		{
			min = distance;
			index = i;
		}

	}
	return this->rrtTree[index];
}

RRTTree::RRTTree()
{
	this->addNode(new RRTNode);
}

RRTTree::RRTTree(vector<DataType> config)
{
	this->addNode(new RRTNode(config));
}
vector<RRTNode*> RRTTree::getPath(vector<DataType> config)
{
	vector<RRTNode*> path;
	RRTNode *node=getNearest(config);
	while(node->parent!=nullptr)
	{
		path.push_back(node);
		node = node->parent;		
	}
	path.push_back(node);
	reverse(path.begin(), path.end());
	return path;
	
}
void RRTTree::printpath(vector<RRTNode*> path)
{
	int size = path.size();
	cout << "Path including " << size << " waypoints" << endl;
	for(int i=0;i<size;i++)
	{
		//cout << "[";
		for (int j = 0; j < FreeDom; j++)cout << path[i]->config[j] << " ";
		cout << endl;
		//cout << path[i]->distance << "]" << endl;
	}
}

void RRTTree::printRRTree(string filename)
{
	ofstream out(filename.c_str(), ios::trunc);
	if (!out.is_open()) cout << "Error writting " << filename << endl;
	else
	{
		int size = this->rrtTree.size();
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < FreeDom; j++) out << this->rrtTree[i]->config[j] << " ";
			out <<GetIndexRRTree(this->rrtTree[i]->parent)<<" "<< endl;
		}
		cout << "writing path to " << filename << " success" << endl;
	}
	out.close();
	
}

int RRTTree::GetIndexRRTree(RRTNode * node)
{
	for (int i = 0; i < this->rrtTree.size(); i++) {
		if (node == this->rrtTree[i]) return i;
	}
	return 0;
}

RRT::RRT()
{
	OBBInit();
}

void RRT::rrtTreeInit()
{
	this->rrtTree[0]->config = this->StartConfig; 
}

void RRT::OBBInit()
{
	OBBNode node;
	for(int i=0;i<FreeDom;i++)
	{
		for(int j=0;j<3;j++)
		{
			node.center[j] = OBB_CenterInit[i][j];
			node.halfWidth[j] = OBB_HalfWidth[i][j];
		}
		for (int j = 0; j < 9; j++) node.angle[j] = OBB_AngleInit[j];
		this->ObbVectorInit.push_back(node);
	}
}

/*****************RRT***************************/
void RRT::setGoalBias(DataType bias)
{
	this->GoalBias = bias;
}
void RRT::setStart(vector<DataType> config)
{
	checkConfig(config);
	this->StartConfig = config;
	
}
void RRT::setGoal(vector<DataType> config)
{
	checkConfig(config);
	this->GoalConfig = config;
}
void RRT::RRTClear()
{
	this->count = 0;
	this->ObbVector.clear();
	//this->octoTree.clear();
	for (auto rrt : this->rrtTree) delete rrt;//free node space
	this->rrtTree.clear();
	this->IsGoal = false;
	this->Success = false;
	sampleIndex = -1;
	//this->ObbVectorInit.clear();

}
vector<DataType> RRT::SamleConfig()
{
	//cout << "sampling hardware clocks " << hardwareClocks << endl;
	DataType goalb = randomdouble(0, 1);//rand betweeen 0~1;
	vector<DataType> config;
	if(goalb<this->GoalBias)
	//if(1)
	{
		this->IsGoal = true;
		config = this->GoalConfig;
	}
	else {
		for (int i = 0; i < FreeDom; i++)
		{
			DataType jointrange = OBB_AngleUpperLimit[i] - OBB_AngleLowerLimit[i];
			DataType r = randomdouble(0,1)*jointrange;
			config.push_back(OBB_AngleLowerLimit[i] + r) ;
		}
		this->IsGoal = false;
	}
	//cout << "IsGoal " << this->IsGoal<< endl;
	sampleIndex++;
	return config;

}


int RRT::extend(RRTNode * near, vector<DataType> config)
{
	this->count = this->count + 1;
	DataType distance = near->getDistance(near,config);
	if(distance<this->stepSize)//Reach random config ;adding config node
	{
		if(!checkCollision(config))
		{
			this->addNode(config,near);
			//cout << "REACHED" << endl;
			return REACHED;
		}
		else return TRAPPED;
	}
	else//extend a stepsize from near to config
	{
		vector<DataType> newconfig;
		for(int i=0;i<FreeDom;i++)
		{
			DataType jointrange = config[i] - near->config[i];
			DataType r = (jointrange/distance)*stepSize;
			newconfig.push_back(near->config[i] + r);
		}
		if(!checkCollision(newconfig))
		{
			this->addNode(newconfig,near);
			//cout << "collision free" << endl;
			return  ADVANCED;
			
		}
		else
		{
			//cout << "collision happen" << endl;
			return TRAPPED;
		}
	}
}

int RRT::connect(vector<DataType> config)
{

	RRTNode* near = this->getNearest(config);
	int s = this->extend(near, config);
	while (s != TRAPPED) {
		if (s == REACHED) {
			return REACHED;
		}
		near = this->rrtTree.back();//update near node to new node
		s = this->extend(near, config);

	}
	return TRAPPED;
}

bool RRT::checkCollision(RRTNode *node)
{
	
	auto start = system_clock::now();
	//this->ObbVector = kinematic(node->config, this->ObbVectorInit);
	this->ObbVector = kinematicMoveit(node->config, this->ObbVectorInit);
	auto end = system_clock::now();
	auto duration = duration_cast<microseconds>(end - start);
	auto start2 = system_clock::now();
	bool state=OctoCollisionDectection(ObbVector, this->octoTree.octotree);
	auto end2 = system_clock::now();
	auto duration2 = duration_cast<microseconds>(end2 - start2);
	return  state;
}

bool RRT::checkCollision(vector<DataType> config)
{
	//this->ObbVector = kinematic(config, this->ObbVectorInit);
	this->ObbVector = kinematicMoveit(config, this->ObbVectorInit);
	bool state=OctoCollisionDectection(ObbVector, this->octoTree.octotree);
	return  state;
}

bool RRT::startRRT()
{
	this->addNode(this->StartConfig);
	while (this->count < this->MaxIteration)
	{
		vector<DataType> config;
		config = this->SamleConfig();
		if (this->connect(config) == REACHED&&this->IsGoal == true)
		{
			this->Success = true;
			break;
			
		}
	}
#if TESTMode
	cout << "RRT:building " << this->count << " states" << " isSuccess "<< this->Success <<endl;
	//cout << "hardware clocks " << hardwareClocks << endl;
	cout << " sampleIndex " << sampleIndex << endl;
#endif	
	return this->count < this->MaxIteration;
}

int randomint(int min, int max)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<> dis(min, max);
	return dis(gen);
}

DataType randomdouble(int min, int max)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<> dis(min, max);
	return (DataType)dis(gen);
}
