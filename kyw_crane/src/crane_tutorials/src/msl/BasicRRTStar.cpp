#include "BasicRRTStar.h"
#include "defs.h"

// *********************************************************************
// *********************************************************************
// CLASS:     CBasicRRTStar
// 
// 林远山添加，在采样点处选择邻居集合，并通过新的节点修改邻居节点的连接
// *********************************************************************
// *********************************************************************
CBasicRRTStar::CBasicRRTStar(Problem *p): RRTGoalBias(p)
{
	m_strName = "CBasicRRTStar";

	MSLVector nx = p->Integrate( p->InitialState, p->GetInputs().front(), 1.0 );
	double dOneStepDist = p->Metric( p->InitialState, nx );
	m_dD = dOneStepDist;

	m_dCurPathCost = INFINITY;
	m_pkGoalNode = NULL;
	m_nIterNum = 100000;

	m_nRewireNum = 0;
	m_dRewireTime = 0;

	// Steer操作统计
	m_nSteerNum = 0;
	m_dSteerTime = 0;
}

MSLVector CBasicRRTStar::ChooseState()
{
	m_iChooseStateNum++;

	// 林远山于2011.6.6增加时间统计
	float time;
	time = used_time();

	double rv;
	MSLVector zx;
	int i;

	int nMaxCount = 20;
	i = 0;
	R >> rv;
	if( rv > GoalProb )
	{
		while( i < nMaxCount )
		{
			zx = RandomState();
			if( P->CollisionFree( zx ) )
			{
				break;
			}
			i++;
		}
		if( i >= nMaxCount )
		{
			zx = P->GoalState;
		}
	}
	else
	{
		zx = P->GoalState;
	}
	
	// 林远山于2011.6.6增加时间统计
	this->m_dChooseStateTime += ((double)used_time(time));

	return zx;
}

bool CBasicRRTStar::Plan()
{
	if (!T)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T = new MSLTree(P->InitialState);
			T->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T = new MSLTree( P->InitialState, this->m_pTopology, this->m_pScaling );
			T->Root()->SetCost( 0 );
		}
#endif

	}

	// Keep track of time
	float t;
	t = used_time();

	int i = 0;
	MSLVector Target;
	MSLNode* pNewNode = NULL;
	list<MSLNode*> pathNodeList;
	while ( i < m_nIterNum ) 
	{
		Target = ChooseState();
		m_Samples.push_back( Target );
		ExtendType eRes = Extend( T, Target, pNewNode );

		switch( eRes )
		{
		case ExtendType::REACHGOAL:

			CumulativePlanningTime += ((double)used_time(t));
			m_bSuccess = true;
			P->m_dMinCost = m_dCurPathCost;

			pathNodeList = T->PathToRoot( m_pkGoalNode );
			pathNodeList.reverse();
			RecordSolution( pathNodeList ); // Write to Path and Policy
			// 应该将生成树和路径也存储起来
			// .....

			cout << "Success\n";
			PrintInfo();

			cout << "m_nRewireNum: " << m_nRewireNum << endl;

			return m_bSuccess;
			break;
		case ExtendType::REACHTARGET:		// 暂没有操作
			{
			int a = 10;
			break;
			}
		case ExtendType::REACHNEWSTATE:		// 暂没有操作
			{
			int a = 10;
			break;
			}
		case ExtendType::NOEXTEND:			// 暂没有操作
			{
			int a = 10;
			break;
			}
		}
		i++;
	}

	return m_bSuccess;
}

// 向采样点扩展
CBasicRRTStar::ExtendType CBasicRRTStar::Extend( MSLTree *t, const MSLVector &rTarget, MSLNode *&rpNewNode )
{
	ExtendType eRes = ExtendType::NOEXTEND;
	bool bReachGoal = false;
	MSLNode *pBestParent = NULL;
	MSLVector kNewState;
	double dCost;
	std::vector< MSLNode* > vNodes;
	std::vector< MSLVector > vTraj;
	std::vector< MSLVector > vInput;
	std::vector< double > vCost;

	MSLNode *pInitParent = NULL;
	std::vector< MSLVector > vInitTraj;
	std::vector< MSLVector > vInitInput;
	std::vector< double > vInitCost;

	// 1.生成新状态
	// 1.1 选择最近节点
	pBestParent = SelectNode( rTarget, t, true );
	// 1.2 生成状态
	GenerateNewState( pBestParent, rTarget, vInitTraj, vInitInput, vInitCost );

	// 2.选择kNewState的邻近节点集
	if( vInitTraj.size() <= 1 )		// 没生成新的节点
	{
		return ExtendType::NOEXTEND;
	}
	dCost = vInitCost.back();
	kNewState = vInitTraj.back();
	vNodes = SelectNearNodes( kNewState, t, true );

	// 3.挑选优秀的父节点（生长点）
	pInitParent = pBestParent;
	SelectBestParent( kNewState, vNodes, dCost, pBestParent, vTraj, vInput, vCost );

	
	if( pBestParent == pInitParent )
	{
		vTraj = vInitTraj;
		vInput = vInitInput;
		vCost = vInitCost;
	}
	int iNum = vTraj.size();
	if( iNum > 1 )
	{
		// 4.扩展树
		bReachGoal = TreeExtend( pBestParent, vTraj, vInput, vCost, rpNewNode );
		if( rpNewNode && rpNewNode->State() == rTarget )
		{
			eRes = ExtendType::REACHTARGET;
		}
		else
		{
			eRes = ExtendType::REACHNEWSTATE;
		}

		// 5.改道
		bool bReachGoal2 = false;
		if( iNum > 1 && rpNewNode )
		{
			bReachGoal2 = RewireNodes( rpNewNode, vNodes );
		}

		// 6.处理返回值
		if( bReachGoal || bReachGoal2 )
		{
			eRes = ExtendType::REACHGOAL;
		}
	}
	else
	{
		eRes = ExtendType::NOEXTEND;
	}

	return eRes;
}

// 采样点方向生成新状态
void CBasicRRTStar::GenerateNewState( const MSLNode *pParent, const MSLVector &rTarget, 
									 std::vector< MSLVector > &rvTraj, 
									 std::vector< MSLVector > &rvInput, 
									 std::vector< double > &rvCost )
{
	std::vector< MSLVector > vTraj;
	std::vector< MSLVector > vInput;
	std::vector< double > vCost;

	// 返回的vTraj,vInput,vCost包括出发点
	//P->GenerateTraj( pParent->State(), rTarget, vTraj, vInput, vCost );

	MSLVector newTarget = rTarget;
	// 控制单步生长的距离
	const double dD = 15;
	double dDist = P->Metric( pParent->State(), rTarget );
	if( dDist > dD )	// 大于一定的距离
	{
		newTarget = P->InterpolateState( pParent->State(), rTarget, dD / dDist );
	}
	Steer( pParent->State(), newTarget, vTraj, vInput, vCost );

	if( vTraj.size() > 0 )		// 按常理一定大于零
	{
		rvTraj = vTraj;
		rvInput = vInput;
		rvCost = vCost;
	}
}

// 选择优秀父节点
void CBasicRRTStar::SelectBestParent( const MSLVector &rNewState, const std::vector< MSLNode* > &vNodes, 
		double &rdCost, MSLNode *&rpParent, std::vector< MSLVector > &rvTraj, 
		std::vector< MSLVector > &rvInput, std::vector< double > &rvCost )
{
	std::vector< MSLVector > vTraj;
	std::vector< MSLVector > vInput;
	std::vector< double > vCost;
	rvTraj.push_back( rpParent->State() );
	rvInput.push_back( MSLVector( this->P->InputDim ) );
	rvCost.push_back( 0 );

	MSLVector xnew;
	double dTrajCost;
	int iNum = vNodes.size();
	int i;
	for( i = 0; i < iNum; ++i )
	{
		//if( vNodes[i] == rpParent )		// 原父节点
		//{
		//	continue;
		//}

		// 返回的rvTraj, rvInput, rvCost包含出发点，所以一定不会空
		Steer( vNodes[i]->State(), rNewState, vTraj, vInput, vCost );
		if( rvTraj.back() != rNewState )		// 没有操作可以到达
		{
			continue;
		}

		// 计算代价
		double dCost = vNodes[i]->Cost() + rvCost.back();
		if( dCost < rdCost )
		{
			rdCost = dCost;
			rpParent = vNodes[i];
			rvTraj = vTraj;
			rvInput = vInput;
			rvCost = vCost;
		}
	}
}

// 根据局部轨迹扩展树，路过目标点返回true
bool CBasicRRTStar::TreeExtend( MSLNode *pParent, const std::vector< MSLVector > &rvTraj, 
							   const std::vector< MSLVector > &rvInput,
							   const std::vector< double > &rvCost, MSLNode *&rpNewNode )
{
	//bool bReachGoal = false;
	//if( rvCost.back() < 0.0001 )	// 几乎与父节点一样
	//{
	//	return bReachGoal;
	//}

	//double dCost = pParent->Cost() + rvCost.back();

	//// 判断局部路径rvTraj是否路过目标点
	//int iSuccessIdx = 0;
	//if( IsPassbyGoal( rvTraj, iSuccessIdx ) )
	//{	// 如果路过，则只扩展到目标点，并放弃此次的改道
	//	dCost = pParent->Cost() + rvCost[iSuccessIdx];
	//	if( dCost < this->m_dCurPathCost )
	//	{
	//		// 扩展树
	//		rpNewNode = T->Extend( pParent, rvTraj[iSuccessIdx], 
	//			rvInput[iSuccessIdx], (iSuccessIdx-1) * PlannerDeltaT );
	//		rpNewNode->SetCost( dCost );

	//		// 善后
	//		m_pkGoalNode = rpNewNode;
	//		this->m_dCurPathCost = rpNewNode->Cost();
	//		bReachGoal = true;

	//		return bReachGoal;
	//	}
	//}
	//else
	//{
	//	// 下面的输入是暂定的，没有太大意义
	//	rpNewNode = T->Extend( pParent, rvTraj.back(), rvInput.back(), (rvInput.size() - 1) * PlannerDeltaT );
	//	rpNewNode->SetCost( dCost );
	//	return bReachGoal;
	//}

	bool bReachGoal = false;
	if( rvCost.back() < 0.0001 )	// 几乎与父节点一样
	{
		return bReachGoal;
	}

	bool bForward = true;
	MSLTree *pTree = bForward ? T : T2;

	double dCost;

	// 判断局部路径rvTraj是否路过目标点
	int iSuccessIdx = 0;
	int i;
	if( IsPassbyGoal( rvTraj, iSuccessIdx ) )
	{	// 如果路过，则只扩展到目标点，并放弃此次的改道
		dCost = pParent->Cost() + rvCost[iSuccessIdx];
		if( dCost < this->m_dCurPathCost )
		{
			MSLNode *pNewParent = pParent;
			double dBaseCost = pNewParent->Cost();
			for( i = 1; i < iSuccessIdx; ++i )
			{
				dCost = dBaseCost + rvCost[i];
				rpNewNode = pTree->Extend( pNewParent, rvTraj[i], 
					rvInput[i], PlannerDeltaT );
				rpNewNode->SetCost( dCost );

				pNewParent = rpNewNode;
			}

			// 善后
			if( bForward )
			{
				m_pkGoalNode = rpNewNode;
			}
			this->m_dCurPathCost = rpNewNode->Cost();
			bReachGoal = true;

			return bReachGoal;
		}
	}
	else
	{
		MSLNode *pNewParent = pParent;
		int iCounts = rvTraj.size();
		double dBaseCost = pNewParent->Cost();
		for( i = 1; i < iCounts; ++i )
		{
			dCost = dBaseCost + rvCost[i];
			if( dCost < this->m_dCurPathCost )
			{
				rpNewNode = pTree->Extend( pNewParent, rvTraj[i], 
					rvInput[i], PlannerDeltaT );
				rpNewNode->SetCost( dCost );

				pNewParent = rpNewNode;
			}
		}

		return bReachGoal;
	}
}

// 对邻近节点进行检测、改道，路过目标点返回true，并返回新节点
bool CBasicRRTStar::RewireNodes( MSLNode *&pNewNode, std::vector< MSLNode* > &rvNodes )
{
	float time = used_time();

	bool bReachGoal = false;
	std::vector< MSLVector > vTraj;
	std::vector< MSLVector > vInput;
	std::vector< double > vCost;
	MSLVector xnew;
	double dTrajCost;
	int iNum = rvNodes.size();
	int i;
	for( i = 0; i < iNum; ++i )
	{
		// 检测是否已是父子关系
		if( rvNodes[i] == pNewNode->Parent() )
		{
			continue;
		}

		// 返回的rvTraj, rvInput, rvCost包含出发点，所以一定不会空
		Steer( pNewNode->State(), rvNodes[i]->State(), vTraj, vInput, vCost );
		if( vTraj.back() != rvNodes[i]->State() )		// 没有操作可以到达
		{
			continue;
		}

		// 计算代价
		// 我们认为只有改道走的代价小于原来的95%才值得，因为改道后，其子孙节点的代价也需要更新，
		// 耗费一定的时间
		double dCost = pNewNode->Cost() + vCost.back();
		if( dCost > 0.95 * rvNodes[i]->Cost() )
		{
			continue;
		}

		// 判断局部路径vTraj是否路过目标点
		int iSuccessIdx = 0;
		if( IsPassbyGoal( vTraj, iSuccessIdx ) )
		{	// 如果路过，则只扩展到目标点，并放弃此次的改道
			dCost = pNewNode->Cost() + vCost[iSuccessIdx];
			if( dCost < this->m_dCurPathCost )
			{
				// 扩展树
				MSLNode * pNode = T->Extend( pNewNode, vTraj[iSuccessIdx], 
					vInput[iSuccessIdx], (iSuccessIdx-1) * PlannerDeltaT );
				pNode->SetCost( dCost );

				// 善后
				m_pkGoalNode = pNode;
				this->m_dCurPathCost = pNode->Cost();
				bReachGoal = true;
				pNewNode = pNode;
				break;

				//bReachGoal = TreeExtend( pNewNode, vTraj, vInput, vCost, pNewNode );
				//break;
			}
			continue;	// 放弃此次改道机会，继续对其它节点进行改道
		}

		// 与前任父节点脱离关系
		rvNodes[i]->Parent()->DetachChild( rvNodes[i] );
		// 与新的父节点建立父子关系
		pNewNode->AddChild( rvNodes[i] );
		rvNodes[i]->SetParent( pNewNode );
		// 更新信息
		MSLVector kInput = vInput.back();
		rvNodes[i]->SetInputandTime( kInput, (vInput.size() - 1) * PlannerDeltaT );
		double dOldCost = rvNodes[i]->Cost();
		double dDiff = dOldCost - dCost;
		rvNodes[i]->SetCost( dCost );

		m_nRewireNum++;

		//if( GapSatisfied( rvNodes[i]->State(), P->GoalState ) )
		//{
		//	if( rvNodes[i]->Cost() < this->m_dCurPathCost )
		//	{
		//		m_pkGoalNode = rvNodes[i];
		//		this->m_dCurPathCost = rvNodes[i]->Cost();
		//		bReachGoal = true;
		//	}
		//}

		// 更新其子孙节点的代价
		if( dDiff < 0.001 )		// 与不更新一样
		{
			continue;
		}
		UpdateSubtreeCost( rvNodes[i], dDiff );
	}

	m_dRewireTime += (double) used_time( time );

	return bReachGoal;
}

// 从x1到x2生成无碰撞轨迹，包含出发点
void CBasicRRTStar::Steer( const MSLVector &rState1, const MSLVector &rState2,		// 输入
						  std::vector< MSLVector > &rvTraj,			// 输出
						  std::vector< MSLVector > &rvInput, 		// 输出
						  std::vector< double > &rvCost )			// 输出
{
	float time = used_time();

	if (Holonomic)
	{
		rvTraj.clear();
		rvInput.clear();
		rvCost.clear();
		rvTraj.push_back( rState1 );
		rvInput.push_back( MSLVector( P->StateDim) );
		rvCost.push_back( 0 );

		if( P->CollisionFreeLine( rState1, rState2 ) )
		{
			rvTraj.push_back( rState2 );
			rvInput.push_back( rState2 - rState1 );
			rvCost.push_back( P->Metric( rState1, rState2 ) );
		}

		//const double dDetaD = 5.0;
		//MSLVector x, u;
		//double c = 0;
		//double dStep = 0;
		//double dA = 0;
		//double dDist = P->Metric( rState1, rState2 );
		//int n = dDist / dDetaD;
		//n = 0 == n ? 1 : n;		// 不够一步
		//dStep = (double)dDist / n;
		//u = P->InterpolateState( rState1, rState2, 1.0 / (double)n );
		//for( int i = 1; i < n + 1; ++i )	// 出发点已添加，从下一个点开始
		//{
		//	dA = ( i * dStep ) / dDist;
		//	x = P->InterpolateState( rState1, rState2, dA );
		//	if( !P->CollisionFree( x ) )
		//	{
		//		return;
		//	}
		//	c = i * dStep;
		//	rvTraj.push_back( x );
		//	rvInput.push_back( u );
		//	rvCost.push_back( c );
		//}
	}
	else
	{
		P->GenerateTraj( rState1, rState2, rvTraj, rvInput, rvCost );
	}
	m_dSteerTime += used_time( time );
}

void CBasicRRTStar::UpdateSubtreeCost( MSLNode* pNode, double dDiff )
{
	list<MSLNode*> children = pNode->Children();
	list<MSLNode*>::iterator it;
	double dCost;
	for( it = children.begin(); it != children.end(); it++ )
	{
		dCost = (*it)->Cost() - dDiff;
		(*it)->SetCost( dCost );

		if( GapSatisfied( (*it)->State(), P->GoalState ) )
		{
			if( (*it)->Cost() < this->m_dCurPathCost )
			{
				m_pkGoalNode = (*it);
				this->m_dCurPathCost = (*it)->Cost();
			}
		}

		// 递归更新其子孙
		UpdateSubtreeCost( *it, dDiff );
	}
}

// 检测轨迹是否路过目标点
bool CBasicRRTStar::IsPassbyGoal( const std::vector< MSLVector > &rvTraj, int &riSuccessIdx )
{
	int iNum = rvTraj.size();
	for( int i = 1; i < iNum; ++i )		// 第一个节点不考虑，它为生长点
	{
		if( this->GapSatisfied( rvTraj[i], P->GoalState ) )
		{
			riSuccessIdx = i;
			return true;
		}
	}
	return false;
}


void CBasicRRTStar::Statistics( string path )
{
	string strFileName = path + this->m_strName +  ".Statistics.csv";

	ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
	
	if( m_bFirstWrite )
	{
		fout << "找到路径" << ",路径长度" << ",总规划时间" << ",树节点数" << ",碰撞检测次数";
		
		fout << ",采样次数";
		fout << ",采样时间";
		fout << ",采样时间百分比";
		fout << ",选择节点次数";
		fout << ",选择节点时间";
		fout << ",选择节点时间百分比";
		fout << ",选择输入次数";
		fout << ",选择输入时间";
		fout << ",选择输入时间百分比";	
		fout << ",操作树时间";
		fout << ",操作树时间百分比";
		fout << ",改道次数";
		fout << ",改道时间";
		fout << ",改道时间百分比";
		fout << ",Steer操作次数";
		fout << ",Steer操作时间";
		fout << ",Steer操作时间百分比";
		fout << ",碰撞检测次数";
		fout << ",碰撞检测时间";
		fout << ",碰撞检测时间百分比";
		fout << ",距离度量次数";
		fout << ",距离度量时间";
		fout << ",距离度量时间百分比";

		m_bFirstWrite = false;
	}

	fout.seekp( 0, std::ios::end );

	int nCount = T->Size();
	if( T2 )
	{
		nCount += T2->Size();
	}

	fout << "\n" << m_bSuccess << "," << P->m_dMinCost << "," << CumulativePlanningTime 
		<< "," << nCount << "," << SatisfiedCount;

	fout << "," << m_iChooseStateNum;
	fout << "," << m_dChooseStateTime;
	fout << "," << m_dChooseStateTime * 100 / CumulativePlanningTime << "%";
	fout << "," << m_iSelectNodeNum;
	fout << "," << m_dSelectNodeTime;
	fout << "," << m_dSelectNodeTime * 100 / CumulativePlanningTime << "%";
	fout << "," << m_iSelectInputNum;
	fout << "," << m_dSelectInputTime;
	fout << "," << m_dSelectInputTime * 100 / CumulativePlanningTime << "%";
	fout << "," << m_dTreeOperTime;
	fout << "," << m_dTreeOperTime * 100 / CumulativePlanningTime << "%";
	fout << "," << m_nRewireNum;
	fout << "," << m_dRewireTime;
	fout << "," << m_dRewireTime * 100 / CumulativePlanningTime << "%";
	fout << "," << m_nSteerNum;
	fout << "," << m_dSteerTime;
	fout << "," << m_dSteerTime * 100 / CumulativePlanningTime << "%";
	fout << "," << P->m_iCollisionNum;
	fout << "," << P->m_dCumulativeCollisionTime;
	fout << "," << P->m_dCumulativeCollisionTime * 100 / CumulativePlanningTime << "%";
	fout << "," << P->m_iMetricNum;
	fout << "," << P->m_dMetricTime;
	fout << "," << P->m_dMetricTime * 100 / CumulativePlanningTime << "%";

	fout.close();
}


void CBasicRRTStar::Reset()
{
	RRT::Reset();

	m_nRewireNum = 0;
	m_dRewireTime = 0;
	m_nSteerNum = 0;
	m_dSteerTime = 0;
	m_dCurPathCost = INFINITY;
}

/*
#include <algorithm>
extern int Compare( pair<MSLNode*,double> a, pair<MSLNode*,double> b);

// 选择最好的父节点
MSLNode* CBasicRRTStar::SelectBestParent( const MSLVector &x, std::vector< MSLNode* > &vNodes, MSLNode* &pBestParent, double &dMinCost )
{
	//MSLNode* pBestParent = NULL;
	int i;
	int iNum = vNodes.size();
	//vector<std::pair< MSLNode*, double >> vNodeCostPairs( iNum );
	//
	//double dCost = INFINITY;
	//// 1.计算经过vNodes节点到x的代价，并根据此代价对vNodes从小到大排序，采用STL中的algorithm库
	//// 1.1计算到x的代价，并构造Pairs
	//for( i = 0; i < iNum; ++i )
	//{
	//	dCost = P->Metric( vNodes[i]->State(), x );
	//	vNodeCostPairs[i].first = vNodes[i];
	//	vNodeCostPairs[i].second = dCost;
	//}
	//// 1.2排序
	//::sort( vNodeCostPairs.begin(), vNodeCostPairs.end(), Compare );
	//
	//pBestParent = vNodeCostPairs[0].first;
	
	std::vector< MSLVector > vStates;
	std::vector< MSLVector > vInputs;
	MSLVector xnew;
	double dTrajCost;
	for( i = 0; i < iNum; ++i )
	{
		// 检测是否可达
		xnew = P->GetModel()->Steer( vNodes[i]->State(), x, vStates, vInputs, dTrajCost, true );
		if( xnew != x )		// 没有操作可以到达
		{
			continue;
		}
		// 提取发生干涉之前的状态
		ExtractCollisionFreeStates( vStates );
		int iSize = vStates.size();
		if( iSize <= 0 )		// 第一个就碰到障碍
		{
			continue;
			
		}
		xnew = vStates[iSize-1];
		if( xnew != x )		// 有障碍，道路不通
		{
			continue;
		}

		// 计算代价
		double dCost = vNodes[i]->Cost() + dTrajCost;
		if( dCost < dMinCost )
		{
			dMinCost = dCost;
			pBestParent = vNodes[i];
		}
	}

	return pBestParent;
}


// 邻近节点改道
void CBasicRRTStar::RewireNodes( MSLTree *t, MSLNode* pNewNode, std::vector< MSLNode* > &vNodes, bool bForward )
{
	double dTrajCost, dTime;
	MSLVector kInput, xnew;
	MSLNode *pNode;
	int i;
	int iNum = vNodes.size();
	vector<std::pair< MSLNode*, double >> vNodeCostPairs( iNum );
	
	if( pNewNode == NULL || iNum == 0 )
	{
		return;
	}

	double dCost = INFINITY;
	// 1.计算经过vNodes节点到x的代价，并根据此代价对vNodes从小到大排序，采用STL中的algorithm库
	// 1.1计算到x的代价，并构造Pairs
	for( i = 0; i < iNum; ++i )
	{
		dCost = vNodes[i]->Cost();
		vNodeCostPairs[i].first = vNodes[i];
		vNodeCostPairs[i].second = dCost;
	}
	//// 1.2排序
	//::sort( vNodeCostPairs.begin(), vNodeCostPairs.end(), Compare );

	//iNum = min( 15, iNum );
	for( i = 0; i < iNum; ++i )
	{
		// 检测是否可改道
		std::vector< MSLVector > vStates;
		std::vector< MSLVector > vInputs;
		xnew = P->GetModel()->Steer( pNewNode->State(), vNodeCostPairs[i].first->State(), vStates, vInputs, dTrajCost, bForward );
		if( xnew != vNodeCostPairs[i].first->State() )		// 改道失败
		{
			continue;
		}
		// 提取发生干涉之前的状态
		ExtractCollisionFreeStates( vStates );
		int iSize = vStates.size();
		if( iSize <= 0 )
		{
			continue;
			
		}
		xnew = vStates[iSize-1];
		if( xnew != vNodeCostPairs[i].first->State() )		// 改道失败
		{
			continue;
		}

		// 计算代价
		dCost = pNewNode->Cost() + dTrajCost;
		if( dCost >= 0.95 * vNodeCostPairs[i].first->Cost() )
		{
			continue;
		}

		
		// 我们认为只有改道走的代价小于原来的95%才值得，因为改道后，其子孙节点的代价也需要更新，
		// 耗费一定的时间
		MSLNode *pParentNode = NULL;
		if( dCost < 0.95 * vNodeCostPairs[i].first->Cost() )	
		{
			m_nRewireNum++;
			// 将该路段加入树中
			double dNewNodeCost;
			int iTrajLen = vStates.size();
			pParentNode = pNewNode;
			//for( int i = 0; i < iTrajLen - 1; ++i )
			//{
			//	xnew = vStates[i];
			//	kInput = vInputs[i];
			//	dNewNodeCost = pParentNode->Cost() + P->Metric( pParentNode->State(), xnew );
			//	if( dNewNodeCost < P->m_dMinCost )
			//	{	
			//		pNode = t->Extend( pParentNode, xnew, kInput, PlannerDeltaT );
			//		pNode->SetCost( dNewNodeCost );
			//		pParentNode = pNode;
			//		if( GapSatisfied( xnew, P->GoalState ) )
			//		{
			//			m_pkGoalNode = pNode;
			//			this->m_dCurPathCost = dNewNodeCost;
			//		}
			//	}
			//}
			
			// 最后一个节点不需重新生成，只需更新即可
			kInput = vInputs[iTrajLen - 1];
			
			// 与前任父节点脱离关系
			vNodeCostPairs[i].first->Parent()->DetachChild( vNodeCostPairs[i].first );
			// 与新的父节点建立父子关系
			pParentNode->AddChild( vNodeCostPairs[i].first );
			vNodeCostPairs[i].first->SetParent( pParentNode );
			// 更新信息
			vNodeCostPairs[i].first->SetInputandTime( kInput, iTrajLen * PlannerDeltaT );
			
			double dOldCost = vNodeCostPairs[i].first->Cost();
			double dDiff = dOldCost - dCost;
			vNodeCostPairs[i].first->SetCost( dCost );

			if( GapSatisfied( vNodeCostPairs[i].first->State(), P->GoalState ) )
			{
				if( vNodeCostPairs[i].first->Cost() < this->m_dCurPathCost )
				{
					m_pkGoalNode = vNodeCostPairs[i].first;
					this->m_dCurPathCost = vNodeCostPairs[i].first->Cost();
				}
			}

			// 更新其子孙节点的代价
			UpdateSubtreeCost( vNodeCostPairs[i].first, dDiff );
		}
	}
}


void CBasicRRTStar::ExtractCollisionFreeStates( std::vector< MSLVector > &vStates )
{
	std::vector< MSLVector >::iterator it;
	it = vStates.begin();
	while( it != vStates.end() )
	{
		if( !P->CollisionFree( *it ) )
		{
			vStates.erase( it, vStates.end() );
			break;
		}
		it++;
	}
}

//! Incrementally extend the RRT
bool CBasicRRTStar::Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	bool forward)
{
	// 先将输出参数设初值
	nn = NULL;

	m_iExtendNum++;

	std::vector< MSLNode* > vNodes;
	MSLNode* pParentNode = NULL;
	double dNewNodeCost = 0;
	MSLVector kInput;
	MSLVector xnew;
	double dTime = 0;
	double dTrajCost;

	std::vector< MSLVector > vStates;
	std::vector< MSLVector > vInputs;
	// 1.按常规方法生成一个新节点
	pParentNode = SelectNode( x, t, forward );

	vStates.clear();
	vInputs.clear();
	xnew = P->GetModel()->Steer( pParentNode->State(), x, vStates, vInputs, dTrajCost, forward );
	if( xnew == pParentNode->State() )		// 未能扩展出去
	{
		return false;
	}
	// 提取发生干涉之前的状态
	ExtractCollisionFreeStates( vStates );
	if( vStates.size() <= 0 )
	{
		return false;
	}

	// 2.选择最好的父节点
	xnew = vStates.back();
	vNodes = SelectNearNodes( xnew, t, forward );
	double dMinCost = pParentNode->Cost() + dTrajCost;
	pParentNode = SelectBestParent( xnew, vNodes, pParentNode, dMinCost );
	if( !pParentNode )
	{
		return false;
	}
	

	// 3.将边和节点添加到树中
	MSLVector xtmp = P->GetModel()->Steer( pParentNode->State(), xnew, vStates, vInputs, dTrajCost, forward );
	if( xtmp == pParentNode->State() )		// 未能扩展出去
	{
		return false;
	}
	// 提取发生干涉之前的状态
	ExtractCollisionFreeStates( vStates );
	if( vStates.size() > 0 && vStates.back() == xnew )
	{
		dNewNodeCost = pParentNode->Cost() + dTrajCost;
		if( dNewNodeCost < P->m_dMinCost )
		{
			kInput = vInputs.front();	// 暂设，不起作用
			nn = t->Extend( pParentNode, xnew, kInput, vStates.size() * PlannerDeltaT );
			nn->SetCost( dNewNodeCost );
			if( GapSatisfied( xnew, P->GoalState ) )
			{
				m_pkGoalNode = nn;
				this->m_dCurPathCost = dNewNodeCost;
			}
		}
			
	    
		//// 扩展生成树
		//int iTrajLen = vStates.size();
		//for( int i = 0; i < iTrajLen; ++i )
		//{
		//	xnew = vStates[i];
		//	kInput = vInputs[i];
		//	dNewNodeCost = pParentNode->Cost() + P->Metric( pParentNode->State(), xnew );
		//	if( dNewNodeCost < P->m_dMinCost )
		//	{	

		//		nn = t->Extend( pParentNode, xnew, kInput, PlannerDeltaT );
		//		nn->SetCost( dNewNodeCost );
		//		pParentNode = nn;
		//		if( GapSatisfied( xnew, P->GoalState ) )
		//		{
		//			m_pkGoalNode = nn;
		//			this->m_dCurPathCost = dNewNodeCost;
		//		}
		//	}
		//}

	}

	//// 扩展生成树
	//int iTrajLen = vStates.size();
	//for( int i = 0; i < iTrajLen; ++i )
	//{
	//	xnew = vStates[i];
	//	kInput = vInputs[i];
	//	dNewNodeCost = pParentNode->Cost() + P->Metric( pParentNode->State(), xnew );
	//	if( dNewNodeCost < P->m_dMinCost )
	//	{	
	//		
	//		nn = t->Extend( pParentNode, xnew, kInput, PlannerDeltaT );
	//		nn->SetCost( dNewNodeCost );
	//		pParentNode = nn;
	//		if( !m_pkGoalNode && GapSatisfied( xnew, P->GoalState ) )
	//		{
	//			m_pkGoalNode = nn;
	//			this->m_dCurPathCost = dNewNodeCost;
	//		}
	//	}
	//}

	// 4.邻近节点改道
	if( vNodes.size() > 0 )
	{
		bool bForward = ( t == T ) ? true : false;
		RewireNodes( t, nn, vNodes, bForward );
	}

	return true;
}


bool CBasicRRTStar::Plan()
{
	m_vPathCost.clear();
	m_vPathTime.clear();

	int i;
	double d;
	MSLNode *n,*nn,*n_goal;
	MSLVector nx,u_best;
	list<MSLNode*> pathNodeList;
	MSLVector x(P->StateDim);

	// Make the root node of G
	if (!T)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T = new MSLTree(P->InitialState);
			T->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T = new MSLTree( P->InitialState, this->m_pTopology, this->m_pScaling );
			T->Root()->SetCost( 0 );
		}
#endif

	}

	// Keep track of time
	float t;
	t = used_time();

	i = 0;
	while ( i < m_nIterNum ) 
	{
		x = ChooseState();
		this->m_Samples.push_back( x );

		if( Extend(x,T,nn) ) // 是否RRT向外扩展一步
		{
			if( m_dCurPathCost < P->m_dMinCost )
			{
				m_bSuccess = true;

				CumulativePlanningTime += ((double)used_time(t));
				P->m_dMinCost = m_dCurPathCost;
				// 记录每一次路径更新后的代价及时刻
				m_vPathCost.push_back( m_dCurPathCost );		
				m_vPathTime.push_back( CumulativePlanningTime );

				cout << "Success\n";
				pathNodeList = T->PathToRoot( m_pkGoalNode );
				pathNodeList.reverse();
				RecordSolution( pathNodeList ); // Write to Path and Policy
				// 应该将生成树和路径也存储起来
				// .....
				m_vPaths.push_back( this->Path );
				

				PrintInfo();
				
				cout << "m_nRewireNum: " << m_nRewireNum << endl;

				return m_bSuccess;
				//// 重新开始计时
				//t = used_time();
			}
		}
		i++;
	}

	return m_bSuccess;
}

*/