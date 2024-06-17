#pragma once
#include "RRTConnectPlusPlus.h"

class BiMRRTs : public RRTConnectPlusPlus
{
public:
	BiMRRTs(Problem* p);
	

public:
	virtual ~BiMRRTs(void);

public:
    // Extend操作返回值
	enum ExtendStatus
	{
		REACHTARGET,	// 到达采样点
		REACHGOAL,	// 到达或路过目标点
		REACHNEWSTATE,	// 到达半道的新节点
		NOEXTEND
	};

	// 可视化
	bool bVisual_;
	visualization_msgs::Marker markerVertexes_, markerEdges_, markerPath_;
	geometry_msgs::Point vertex_;
	ros::Publisher marker_pub_;
	void DisplayTreeNodeAndEdge(const MSLVector& x1, const MSLVector& x2);

	MSLVector NullInput;
	// T树中入侵目标任务空间的节点
	vector< MSLNode* > m_kIntruders;
	// T2树中入侵起始任务空间的节点
	vector< MSLNode* > m_kIntruders2;
	double m_dCheckIntruderTime;
	double m_dAddRootProb;
	bool m_bUseIntruders;
	bool m_bGuass;

	// 规划函数
	virtual bool Plan();
	//bool PlanStar();
	// 向起始树集增加新树
	bool AddInitRoot();
	// 向目标树集增加新树
	bool AddGoalRoot();
	// 以正常方式扩展树，若两个树集相遇则路径找到
	bool ExtendTrees();
	// 扩展进入站位环的节点（入侵者），以当时下车位姿获得随机位形，若能到达则路径找到
	bool ExtendIntruders();
	virtual bool Connect( const MSLVector &x, MSLTree *t, MSLNode *&nn, bool forward = true );
	bool ConnectLine( MSLNode* pGrowingNode, const MSLVector &x, MSLTree *t, MSLNode *&nn, bool forward = true );
	virtual void Reset();
	//void OptimizeTrees();
	//BiMRRTs::ExtendStatus ExtendStar( MSLTree *t, const MSLVector &rTarget, MSLNode *&rpNewNode, bool bForward );

	// 辅助函数
	// 从任务空间采样，获得满足条件的随机点
	MSLVector SamplingFromInitTaskSpace();
	MSLVector SamplingFromGoalTaskSpace();
	// 增加T树入侵者
	bool AddInitIntruder( MSLVector kIntruder );
	// 增加T2树入侵者
	bool AddGoalIntruder( MSLVector kIntruder );
	// 生成路径
	void RecoverSolution(MSLNode *n1, MSLNode *n2);

	// 写论文用的功能
	void ConstructTrees();
};
