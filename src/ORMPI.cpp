/**
 * 
 * @authors VincentXiao (you@example.org)
 * @date    2018-08-13 15:57:15
 * @version $main$
 */
#include "./include/RRT.h"
#include <string>
#include <fstream>
using namespace std;
using namespace chrono;
vector<box> environmentInit();
void WritePath(vector<RRTNode*> path, string filename);//write path to file
vector<DataType> ReadPath(string filename);//read path from file
#define loopNum 100

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
			WritePath(path, "../../PATH.txt"); 
			WritePath(path, "./PATH.txt");
			break;
		}
		//vector<DataType> pathFile = ReadPath("../../PATH.txt");
#endif
		SuccessCount += rrt.Success;
		rrt.RRTClear();
	}
	cout << "success count: " << SuccessCount << endl;
	cout << "time cost average: " << double(TimeCosts / SuccessCount) << endl;
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
void WritePath(vector<RRTNode*> path,string filename)//write path to file
{
	ofstream out(filename.c_str(), ios::trunc);
	if (!out.is_open()) cout << "Error writting " << filename << endl;
	else
	{
		int size = path.size();
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < FreeDom; j++) out << path[i]->config[j]<<" ";
			out << endl;
		}
	}
	out.close();
	cout << "writing path to " << filename << " success" << endl;
}
vector<DataType> ReadPath(string filename)//read path from file
{
	vector<DataType> path;
	ifstream in(filename.c_str(), ios::in);
	if (!in.is_open()) cout << "Error reading " << filename << endl;
	else
	{
		char line[100]={'\0'};
		string num="\0";
		while (!in.eof())
		{
			in.getline(line, 100);
			for(int i=0;i<100;i++)
			{
				if (line[i] == '\0') break;
				else if(line[i]==' ')
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
	cout << "Reading path from " << filename << " success" << endl;
	return path;
}
