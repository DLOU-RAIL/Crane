#include "BasicRRTStar.h"
#include "defs.h"

// *********************************************************************
// *********************************************************************
// CLASS:     CBasicRRTStar
// 
// ��Զɽ��ӣ��ڲ����㴦ѡ���ھӼ��ϣ���ͨ���µĽڵ��޸��ھӽڵ������
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

	// Steer����ͳ��
	m_nSteerNum = 0;
	m_dSteerTime = 0;
}

MSLVector CBasicRRTStar::ChooseState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
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
	
	// ��Զɽ��2011.6.6����ʱ��ͳ��
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
			// Ӧ�ý���������·��Ҳ�洢����
			// .....

			cout << "Success\n";
			PrintInfo();

			cout << "m_nRewireNum: " << m_nRewireNum << endl;

			return m_bSuccess;
			break;
		case ExtendType::REACHTARGET:		// ��û�в���
			{
			int a = 10;
			break;
			}
		case ExtendType::REACHNEWSTATE:		// ��û�в���
			{
			int a = 10;
			break;
			}
		case ExtendType::NOEXTEND:			// ��û�в���
			{
			int a = 10;
			break;
			}
		}
		i++;
	}

	return m_bSuccess;
}

// ���������չ
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

	// 1.������״̬
	// 1.1 ѡ������ڵ�
	pBestParent = SelectNode( rTarget, t, true );
	// 1.2 ����״̬
	GenerateNewState( pBestParent, rTarget, vInitTraj, vInitInput, vInitCost );

	// 2.ѡ��kNewState���ڽ��ڵ㼯
	if( vInitTraj.size() <= 1 )		// û�����µĽڵ�
	{
		return ExtendType::NOEXTEND;
	}
	dCost = vInitCost.back();
	kNewState = vInitTraj.back();
	vNodes = SelectNearNodes( kNewState, t, true );

	// 3.��ѡ����ĸ��ڵ㣨�����㣩
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
		// 4.��չ��
		bReachGoal = TreeExtend( pBestParent, vTraj, vInput, vCost, rpNewNode );
		if( rpNewNode && rpNewNode->State() == rTarget )
		{
			eRes = ExtendType::REACHTARGET;
		}
		else
		{
			eRes = ExtendType::REACHNEWSTATE;
		}

		// 5.�ĵ�
		bool bReachGoal2 = false;
		if( iNum > 1 && rpNewNode )
		{
			bReachGoal2 = RewireNodes( rpNewNode, vNodes );
		}

		// 6.������ֵ
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

// �����㷽��������״̬
void CBasicRRTStar::GenerateNewState( const MSLNode *pParent, const MSLVector &rTarget, 
									 std::vector< MSLVector > &rvTraj, 
									 std::vector< MSLVector > &rvInput, 
									 std::vector< double > &rvCost )
{
	std::vector< MSLVector > vTraj;
	std::vector< MSLVector > vInput;
	std::vector< double > vCost;

	// ���ص�vTraj,vInput,vCost����������
	//P->GenerateTraj( pParent->State(), rTarget, vTraj, vInput, vCost );

	MSLVector newTarget = rTarget;
	// ���Ƶ��������ľ���
	const double dD = 15;
	double dDist = P->Metric( pParent->State(), rTarget );
	if( dDist > dD )	// ����һ���ľ���
	{
		newTarget = P->InterpolateState( pParent->State(), rTarget, dD / dDist );
	}
	Steer( pParent->State(), newTarget, vTraj, vInput, vCost );

	if( vTraj.size() > 0 )		// ������һ��������
	{
		rvTraj = vTraj;
		rvInput = vInput;
		rvCost = vCost;
	}
}

// ѡ�����㸸�ڵ�
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
		//if( vNodes[i] == rpParent )		// ԭ���ڵ�
		//{
		//	continue;
		//}

		// ���ص�rvTraj, rvInput, rvCost���������㣬����һ�������
		Steer( vNodes[i]->State(), rNewState, vTraj, vInput, vCost );
		if( rvTraj.back() != rNewState )		// û�в������Ե���
		{
			continue;
		}

		// �������
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

// ���ݾֲ��켣��չ����·��Ŀ��㷵��true
bool CBasicRRTStar::TreeExtend( MSLNode *pParent, const std::vector< MSLVector > &rvTraj, 
							   const std::vector< MSLVector > &rvInput,
							   const std::vector< double > &rvCost, MSLNode *&rpNewNode )
{
	//bool bReachGoal = false;
	//if( rvCost.back() < 0.0001 )	// �����븸�ڵ�һ��
	//{
	//	return bReachGoal;
	//}

	//double dCost = pParent->Cost() + rvCost.back();

	//// �жϾֲ�·��rvTraj�Ƿ�·��Ŀ���
	//int iSuccessIdx = 0;
	//if( IsPassbyGoal( rvTraj, iSuccessIdx ) )
	//{	// ���·������ֻ��չ��Ŀ��㣬�������˴εĸĵ�
	//	dCost = pParent->Cost() + rvCost[iSuccessIdx];
	//	if( dCost < this->m_dCurPathCost )
	//	{
	//		// ��չ��
	//		rpNewNode = T->Extend( pParent, rvTraj[iSuccessIdx], 
	//			rvInput[iSuccessIdx], (iSuccessIdx-1) * PlannerDeltaT );
	//		rpNewNode->SetCost( dCost );

	//		// �ƺ�
	//		m_pkGoalNode = rpNewNode;
	//		this->m_dCurPathCost = rpNewNode->Cost();
	//		bReachGoal = true;

	//		return bReachGoal;
	//	}
	//}
	//else
	//{
	//	// ������������ݶ��ģ�û��̫������
	//	rpNewNode = T->Extend( pParent, rvTraj.back(), rvInput.back(), (rvInput.size() - 1) * PlannerDeltaT );
	//	rpNewNode->SetCost( dCost );
	//	return bReachGoal;
	//}

	bool bReachGoal = false;
	if( rvCost.back() < 0.0001 )	// �����븸�ڵ�һ��
	{
		return bReachGoal;
	}

	bool bForward = true;
	MSLTree *pTree = bForward ? T : T2;

	double dCost;

	// �жϾֲ�·��rvTraj�Ƿ�·��Ŀ���
	int iSuccessIdx = 0;
	int i;
	if( IsPassbyGoal( rvTraj, iSuccessIdx ) )
	{	// ���·������ֻ��չ��Ŀ��㣬�������˴εĸĵ�
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

			// �ƺ�
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

// ���ڽ��ڵ���м�⡢�ĵ���·��Ŀ��㷵��true���������½ڵ�
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
		// ����Ƿ����Ǹ��ӹ�ϵ
		if( rvNodes[i] == pNewNode->Parent() )
		{
			continue;
		}

		// ���ص�rvTraj, rvInput, rvCost���������㣬����һ�������
		Steer( pNewNode->State(), rvNodes[i]->State(), vTraj, vInput, vCost );
		if( vTraj.back() != rvNodes[i]->State() )		// û�в������Ե���
		{
			continue;
		}

		// �������
		// ������Ϊֻ�иĵ��ߵĴ���С��ԭ����95%��ֵ�ã���Ϊ�ĵ���������ڵ�Ĵ���Ҳ��Ҫ���£�
		// �ķ�һ����ʱ��
		double dCost = pNewNode->Cost() + vCost.back();
		if( dCost > 0.95 * rvNodes[i]->Cost() )
		{
			continue;
		}

		// �жϾֲ�·��vTraj�Ƿ�·��Ŀ���
		int iSuccessIdx = 0;
		if( IsPassbyGoal( vTraj, iSuccessIdx ) )
		{	// ���·������ֻ��չ��Ŀ��㣬�������˴εĸĵ�
			dCost = pNewNode->Cost() + vCost[iSuccessIdx];
			if( dCost < this->m_dCurPathCost )
			{
				// ��չ��
				MSLNode * pNode = T->Extend( pNewNode, vTraj[iSuccessIdx], 
					vInput[iSuccessIdx], (iSuccessIdx-1) * PlannerDeltaT );
				pNode->SetCost( dCost );

				// �ƺ�
				m_pkGoalNode = pNode;
				this->m_dCurPathCost = pNode->Cost();
				bReachGoal = true;
				pNewNode = pNode;
				break;

				//bReachGoal = TreeExtend( pNewNode, vTraj, vInput, vCost, pNewNode );
				//break;
			}
			continue;	// �����˴θĵ����ᣬ�����������ڵ���иĵ�
		}

		// ��ǰ�θ��ڵ������ϵ
		rvNodes[i]->Parent()->DetachChild( rvNodes[i] );
		// ���µĸ��ڵ㽨�����ӹ�ϵ
		pNewNode->AddChild( rvNodes[i] );
		rvNodes[i]->SetParent( pNewNode );
		// ������Ϣ
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

		// ����������ڵ�Ĵ���
		if( dDiff < 0.001 )		// �벻����һ��
		{
			continue;
		}
		UpdateSubtreeCost( rvNodes[i], dDiff );
	}

	m_dRewireTime += (double) used_time( time );

	return bReachGoal;
}

// ��x1��x2��������ײ�켣������������
void CBasicRRTStar::Steer( const MSLVector &rState1, const MSLVector &rState2,		// ����
						  std::vector< MSLVector > &rvTraj,			// ���
						  std::vector< MSLVector > &rvInput, 		// ���
						  std::vector< double > &rvCost )			// ���
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
		//n = 0 == n ? 1 : n;		// ����һ��
		//dStep = (double)dDist / n;
		//u = P->InterpolateState( rState1, rState2, 1.0 / (double)n );
		//for( int i = 1; i < n + 1; ++i )	// ����������ӣ�����һ���㿪ʼ
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

		// �ݹ����������
		UpdateSubtreeCost( *it, dDiff );
	}
}

// ���켣�Ƿ�·��Ŀ���
bool CBasicRRTStar::IsPassbyGoal( const std::vector< MSLVector > &rvTraj, int &riSuccessIdx )
{
	int iNum = rvTraj.size();
	for( int i = 1; i < iNum; ++i )		// ��һ���ڵ㲻���ǣ���Ϊ������
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
		fout << "�ҵ�·��" << ",·������" << ",�ܹ滮ʱ��" << ",���ڵ���" << ",��ײ������";
		
		fout << ",��������";
		fout << ",����ʱ��";
		fout << ",����ʱ��ٷֱ�";
		fout << ",ѡ��ڵ����";
		fout << ",ѡ��ڵ�ʱ��";
		fout << ",ѡ��ڵ�ʱ��ٷֱ�";
		fout << ",ѡ���������";
		fout << ",ѡ������ʱ��";
		fout << ",ѡ������ʱ��ٷֱ�";	
		fout << ",������ʱ��";
		fout << ",������ʱ��ٷֱ�";
		fout << ",�ĵ�����";
		fout << ",�ĵ�ʱ��";
		fout << ",�ĵ�ʱ��ٷֱ�";
		fout << ",Steer��������";
		fout << ",Steer����ʱ��";
		fout << ",Steer����ʱ��ٷֱ�";
		fout << ",��ײ������";
		fout << ",��ײ���ʱ��";
		fout << ",��ײ���ʱ��ٷֱ�";
		fout << ",�����������";
		fout << ",�������ʱ��";
		fout << ",�������ʱ��ٷֱ�";

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

// ѡ����õĸ��ڵ�
MSLNode* CBasicRRTStar::SelectBestParent( const MSLVector &x, std::vector< MSLNode* > &vNodes, MSLNode* &pBestParent, double &dMinCost )
{
	//MSLNode* pBestParent = NULL;
	int i;
	int iNum = vNodes.size();
	//vector<std::pair< MSLNode*, double >> vNodeCostPairs( iNum );
	//
	//double dCost = INFINITY;
	//// 1.���㾭��vNodes�ڵ㵽x�Ĵ��ۣ������ݴ˴��۶�vNodes��С�������򣬲���STL�е�algorithm��
	//// 1.1���㵽x�Ĵ��ۣ�������Pairs
	//for( i = 0; i < iNum; ++i )
	//{
	//	dCost = P->Metric( vNodes[i]->State(), x );
	//	vNodeCostPairs[i].first = vNodes[i];
	//	vNodeCostPairs[i].second = dCost;
	//}
	//// 1.2����
	//::sort( vNodeCostPairs.begin(), vNodeCostPairs.end(), Compare );
	//
	//pBestParent = vNodeCostPairs[0].first;
	
	std::vector< MSLVector > vStates;
	std::vector< MSLVector > vInputs;
	MSLVector xnew;
	double dTrajCost;
	for( i = 0; i < iNum; ++i )
	{
		// ����Ƿ�ɴ�
		xnew = P->GetModel()->Steer( vNodes[i]->State(), x, vStates, vInputs, dTrajCost, true );
		if( xnew != x )		// û�в������Ե���
		{
			continue;
		}
		// ��ȡ��������֮ǰ��״̬
		ExtractCollisionFreeStates( vStates );
		int iSize = vStates.size();
		if( iSize <= 0 )		// ��һ���������ϰ�
		{
			continue;
			
		}
		xnew = vStates[iSize-1];
		if( xnew != x )		// ���ϰ�����·��ͨ
		{
			continue;
		}

		// �������
		double dCost = vNodes[i]->Cost() + dTrajCost;
		if( dCost < dMinCost )
		{
			dMinCost = dCost;
			pBestParent = vNodes[i];
		}
	}

	return pBestParent;
}


// �ڽ��ڵ�ĵ�
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
	// 1.���㾭��vNodes�ڵ㵽x�Ĵ��ۣ������ݴ˴��۶�vNodes��С�������򣬲���STL�е�algorithm��
	// 1.1���㵽x�Ĵ��ۣ�������Pairs
	for( i = 0; i < iNum; ++i )
	{
		dCost = vNodes[i]->Cost();
		vNodeCostPairs[i].first = vNodes[i];
		vNodeCostPairs[i].second = dCost;
	}
	//// 1.2����
	//::sort( vNodeCostPairs.begin(), vNodeCostPairs.end(), Compare );

	//iNum = min( 15, iNum );
	for( i = 0; i < iNum; ++i )
	{
		// ����Ƿ�ɸĵ�
		std::vector< MSLVector > vStates;
		std::vector< MSLVector > vInputs;
		xnew = P->GetModel()->Steer( pNewNode->State(), vNodeCostPairs[i].first->State(), vStates, vInputs, dTrajCost, bForward );
		if( xnew != vNodeCostPairs[i].first->State() )		// �ĵ�ʧ��
		{
			continue;
		}
		// ��ȡ��������֮ǰ��״̬
		ExtractCollisionFreeStates( vStates );
		int iSize = vStates.size();
		if( iSize <= 0 )
		{
			continue;
			
		}
		xnew = vStates[iSize-1];
		if( xnew != vNodeCostPairs[i].first->State() )		// �ĵ�ʧ��
		{
			continue;
		}

		// �������
		dCost = pNewNode->Cost() + dTrajCost;
		if( dCost >= 0.95 * vNodeCostPairs[i].first->Cost() )
		{
			continue;
		}

		
		// ������Ϊֻ�иĵ��ߵĴ���С��ԭ����95%��ֵ�ã���Ϊ�ĵ���������ڵ�Ĵ���Ҳ��Ҫ���£�
		// �ķ�һ����ʱ��
		MSLNode *pParentNode = NULL;
		if( dCost < 0.95 * vNodeCostPairs[i].first->Cost() )	
		{
			m_nRewireNum++;
			// ����·�μ�������
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
			
			// ���һ���ڵ㲻���������ɣ�ֻ����¼���
			kInput = vInputs[iTrajLen - 1];
			
			// ��ǰ�θ��ڵ������ϵ
			vNodeCostPairs[i].first->Parent()->DetachChild( vNodeCostPairs[i].first );
			// ���µĸ��ڵ㽨�����ӹ�ϵ
			pParentNode->AddChild( vNodeCostPairs[i].first );
			vNodeCostPairs[i].first->SetParent( pParentNode );
			// ������Ϣ
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

			// ����������ڵ�Ĵ���
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
	// �Ƚ�����������ֵ
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
	// 1.�����淽������һ���½ڵ�
	pParentNode = SelectNode( x, t, forward );

	vStates.clear();
	vInputs.clear();
	xnew = P->GetModel()->Steer( pParentNode->State(), x, vStates, vInputs, dTrajCost, forward );
	if( xnew == pParentNode->State() )		// δ����չ��ȥ
	{
		return false;
	}
	// ��ȡ��������֮ǰ��״̬
	ExtractCollisionFreeStates( vStates );
	if( vStates.size() <= 0 )
	{
		return false;
	}

	// 2.ѡ����õĸ��ڵ�
	xnew = vStates.back();
	vNodes = SelectNearNodes( xnew, t, forward );
	double dMinCost = pParentNode->Cost() + dTrajCost;
	pParentNode = SelectBestParent( xnew, vNodes, pParentNode, dMinCost );
	if( !pParentNode )
	{
		return false;
	}
	

	// 3.���ߺͽڵ���ӵ�����
	MSLVector xtmp = P->GetModel()->Steer( pParentNode->State(), xnew, vStates, vInputs, dTrajCost, forward );
	if( xtmp == pParentNode->State() )		// δ����չ��ȥ
	{
		return false;
	}
	// ��ȡ��������֮ǰ��״̬
	ExtractCollisionFreeStates( vStates );
	if( vStates.size() > 0 && vStates.back() == xnew )
	{
		dNewNodeCost = pParentNode->Cost() + dTrajCost;
		if( dNewNodeCost < P->m_dMinCost )
		{
			kInput = vInputs.front();	// ���裬��������
			nn = t->Extend( pParentNode, xnew, kInput, vStates.size() * PlannerDeltaT );
			nn->SetCost( dNewNodeCost );
			if( GapSatisfied( xnew, P->GoalState ) )
			{
				m_pkGoalNode = nn;
				this->m_dCurPathCost = dNewNodeCost;
			}
		}
			
	    
		//// ��չ������
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

	//// ��չ������
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

	// 4.�ڽ��ڵ�ĵ�
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

		if( Extend(x,T,nn) ) // �Ƿ�RRT������չһ��
		{
			if( m_dCurPathCost < P->m_dMinCost )
			{
				m_bSuccess = true;

				CumulativePlanningTime += ((double)used_time(t));
				P->m_dMinCost = m_dCurPathCost;
				// ��¼ÿһ��·�����º�Ĵ��ۼ�ʱ��
				m_vPathCost.push_back( m_dCurPathCost );		
				m_vPathTime.push_back( CumulativePlanningTime );

				cout << "Success\n";
				pathNodeList = T->PathToRoot( m_pkGoalNode );
				pathNodeList.reverse();
				RecordSolution( pathNodeList ); // Write to Path and Policy
				// Ӧ�ý���������·��Ҳ�洢����
				// .....
				m_vPaths.push_back( this->Path );
				

				PrintInfo();
				
				cout << "m_nRewireNum: " << m_nRewireNum << endl;

				return m_bSuccess;
				//// ���¿�ʼ��ʱ
				//t = used_time();
			}
		}
		i++;
	}

	return m_bSuccess;
}

*/