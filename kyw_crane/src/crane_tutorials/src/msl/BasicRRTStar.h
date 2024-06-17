
#pragma once
#include "rrt.h"
class CBasicRRTStar: public RRTGoalBias
{
protected:
	// Extend��������ֵ
	enum ExtendType
	{
		REACHTARGET,	// ���������
		REACHGOAL,		// �����·��Ŀ���
		REACHNEWSTATE,	// ���������½ڵ�
		NOEXTEND
	};
	// ��ƫ�������ײ����
	MSLVector ChooseState();
	// ���������չ
	ExtendType Extend( MSLTree *t, const MSLVector &rTarget, MSLNode *&rpNewNode );
	// �����㷽��������״̬
	void GenerateNewState( const MSLNode *pParent, const MSLVector &rTarget, 
		std::vector< MSLVector > &rvTraj, 
		std::vector< MSLVector > &rvInput, std::vector< double > &rvCost );
	// ѡ�����㸸�ڵ�
	void SelectBestParent( const MSLVector &rNewState, const std::vector< MSLNode* > &vNodes, 
		double &rdCost, MSLNode *&rpParent, std::vector< MSLVector > &rvTraj, 
		std::vector< MSLVector > &rvInput, std::vector< double > &rvCost );
	// ���ݾֲ��켣��չ����·��Ŀ��㷵��true
	bool TreeExtend( MSLNode *pParent, const std::vector< MSLVector > &rvTraj, 
		const std::vector< MSLVector > &rvInput,
		const std::vector< double > &rvCost, MSLNode *&rpNewNode );
	// ���ڽ��ڵ���м�⡢�ĵ���·��Ŀ��㷵��true
	bool RewireNodes( MSLNode *&pNewNode, std::vector< MSLNode* > &rvNodes );
	// ��State1��State1��������ײ�켣������������
	void Steer( const MSLVector &rState1, const MSLVector &rState2,		// ����
		std::vector< MSLVector > &rvTraj,		// ���
		std::vector< MSLVector > &rvInput, 		// ���
		std::vector< double > &rvCost );		// ���
	// ���켣�Ƿ�·��Ŀ���
	bool IsPassbyGoal( const std::vector< MSLVector > &rvTraj, int &riSuccessIdx );
	// ���������ڵ����
	void UpdateSubtreeCost( MSLNode* pNode, double dDiff );

	//// ѡ����õĸ��ڵ�
	//MSLNode* SelectBestParent( const MSLVector &x, std::vector< MSLNode* > &vNodes, MSLNode* &pBestParent, double &dMinCost );
	//// �ڽ��ڵ�ĵ�
	//void RewireNodes( MSLTree *t, MSLNode* pNewNode, std::vector< MSLNode* > &vNodes, bool bForward );
	//// ���������ڵ����
	//void UpdateSubtreeCost( MSLNode* pNode, double dDiff );
	//// �ж�һ����״̬�Ƿ������ϰ�����ײ������������ײ֮ǰ��״̬����
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

	// �ĵ�ͳ��
	int m_nRewireNum;
	double m_dRewireTime;

	// Steer����ͳ��
	int m_nSteerNum;
	double m_dSteerTime;

	CBasicRRTStar(Problem *p);
	virtual ~CBasicRRTStar() {};
	
	virtual bool Plan();

	virtual void Statistics( string path );

	virtual void Reset();
};
