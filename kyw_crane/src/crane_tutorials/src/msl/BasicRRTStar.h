
#pragma once
#include "rrt.h"
class CBasicRRTStar: public RRTGoalBias
{
protected:
	// Extend操作返回值
	enum ExtendType
	{
		REACHTARGET,	// 到达采样点
		REACHGOAL,		// 到达或路过目标点
		REACHNEWSTATE,	// 到达半道的新节点
		NOEXTEND
	};
	// 带偏向的无碰撞采样
	MSLVector ChooseState();
	// 向采样点扩展
	ExtendType Extend( MSLTree *t, const MSLVector &rTarget, MSLNode *&rpNewNode );
	// 采样点方向生成新状态
	void GenerateNewState( const MSLNode *pParent, const MSLVector &rTarget, 
		std::vector< MSLVector > &rvTraj, 
		std::vector< MSLVector > &rvInput, std::vector< double > &rvCost );
	// 选择优秀父节点
	void SelectBestParent( const MSLVector &rNewState, const std::vector< MSLNode* > &vNodes, 
		double &rdCost, MSLNode *&rpParent, std::vector< MSLVector > &rvTraj, 
		std::vector< MSLVector > &rvInput, std::vector< double > &rvCost );
	// 根据局部轨迹扩展树，路过目标点返回true
	bool TreeExtend( MSLNode *pParent, const std::vector< MSLVector > &rvTraj, 
		const std::vector< MSLVector > &rvInput,
		const std::vector< double > &rvCost, MSLNode *&rpNewNode );
	// 对邻近节点进行检测、改道，路过目标点返回true
	bool RewireNodes( MSLNode *&pNewNode, std::vector< MSLNode* > &rvNodes );
	// 从State1到State1生成无碰撞轨迹，包含出发点
	void Steer( const MSLVector &rState1, const MSLVector &rState2,		// 输入
		std::vector< MSLVector > &rvTraj,		// 输出
		std::vector< MSLVector > &rvInput, 		// 输出
		std::vector< double > &rvCost );		// 输出
	// 检测轨迹是否路过目标点
	bool IsPassbyGoal( const std::vector< MSLVector > &rvTraj, int &riSuccessIdx );
	// 更新子树节点代价
	void UpdateSubtreeCost( MSLNode* pNode, double dDiff );

	//// 选择最好的父节点
	//MSLNode* SelectBestParent( const MSLVector &x, std::vector< MSLNode* > &vNodes, MSLNode* &pBestParent, double &dMinCost );
	//// 邻近节点改道
	//void RewireNodes( MSLTree *t, MSLNode* pNewNode, std::vector< MSLNode* > &vNodes, bool bForward );
	//// 更新子树节点代价
	//void UpdateSubtreeCost( MSLNode* pNode, double dDiff );
	//// 判断一序列状态是否有与障碍物碰撞，并将发生碰撞之前的状态返回
	//void ExtractCollisionFreeStates( std::vector< MSLVector > &vStates );

	////! Incrementally extend the RRT
	//virtual bool Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	//	bool forward = true);
public:
	double m_dD;

	int m_nIterNum;
	double m_dCurPathCost;
	std::vector< double > m_vPathCost;
	std::vector< double > m_vPathTime;
	std::vector< list< MSLVector > > m_vPaths;
	MSLNode* m_pkGoalNode;

	// 改道统计
	int m_nRewireNum;
	double m_dRewireTime;

	// Steer操作统计
	int m_nSteerNum;
	double m_dSteerTime;

	CBasicRRTStar(Problem *p);
	virtual ~CBasicRRTStar() {};
	
	virtual bool Plan();

	virtual void Statistics( string path );

	virtual void Reset();
};
