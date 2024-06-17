#include "RRTConnectPlusPlus.h"
#include "defs.h"

RRTConnectPlusPlus::RRTConnectPlusPlus(Problem *p):RRTDual(p) {
	m_strName = "RRT-Connect++";

	MSLVector nx = p->Integrate( p->InitialState, p->GetInputs().front(), PlannerDeltaT );
	double dOneStepDist = p->Metric( p->InitialState, nx );
	m_dD = dOneStepDist;
	//m_dD = 0.05;

	nFarFromNode = 0;
    nLineNode = 0;

	m_dPoolProb = 0.3;
	m_iPoolSize = 1000;

	ConnectTimeLimit = 100000000;

	m_nExtendBack = 0;

	//m_dMaxCost = p->Metric( p->InitialState, p->GoalState );
	//m_dMinCost = m_dMaxCost * 1.001;
	//m_dFloorQ = 0.5;

	//m_pNewNodeT = m_pNewNodeT2 = NULL;
}

MSLVector RRTConnectPlusPlus::ChooseState() {
	
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	// ��˹����
	MSLVector x1, x2, Xrand;

	Xrand = RandomState();
	//x1 = RandomState();

	//static int nMaxCount = 50;
	//int i = 0;
	//while( i < nMaxCount )
	//{
	//if( this->P->Satisfied( x1 ) )
	//{
	//	x2 = this->NormalState( x1, m_dD );
	//	if( !this->P->Satisfied( x2 ) )
	//	{
	//		Xrand = x1;
	//		break;
	//	}
	//}
	//else
	//{
	//	x2 = this->NormalState( x1, m_dD );
	//	if( this->P->Satisfied( x2 ) )
	//	{
	//		Xrand = x2;
	//		break;
	//	}
	//}
	//i++;
	//}

	//if( nMaxCount == i )
	//{
	//	Xrand = this->P->GoalState;
	//}

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	m_Samples.push_back( Xrand );

	return Xrand;
}


MSLVector RRTConnectPlusPlus::ChooseState( MSLTree *pTree ) 
{
	
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	double rv, prb;
	int n, nid;
	n = nid = 0;
	MSLNode *pNode = NULL;
	MSLVector Xrand;
	double dNearestDist;
	
	R >> rv;
	R >> prb;

	//if( prb > 0.5 )
	//{
	//	Xrand = this->RandomState();
	//}
	//else
	//{
	int nMaxCount = 0;
		if( pTree == T )
		{
			//n = T2->Size();

			//if( n < 500 )
			//{
			//	Xrand = this->RandomState();
			//}
			//else
			//{
			//	nid = (int)(rv * n);
			//	pNode = T2->FindNode( nid );
			//	Xrand = pNode->State();
			//	
			//}


			n = m_kCandidateSamps2.size();
			//n = m_iPoolSize;
			if( n < 20 || prb > m_dPoolProb )
			{
				//Xrand = this->RandomState();
				nMaxCount = 0;
				do
				{
					Xrand = RandomState();
					
#ifdef USE_ANN
					double *pScaling = new double[this->P->StateDim];
					P->GetModel()->GetScaling( Xrand, pScaling );
					pNode = pTree->NearestNode( Xrand, dNearestDist, pScaling );
					delete pScaling;
#else
					pNode = SelectNode( Xrand, pTree, true );
					dNearestDist = P->Metric( Xrand, pNode->State() );
#endif
					nMaxCount++;

				}while( dNearestDist < 5 * m_dD/* && nMaxCount < 100*/ );
			}
			else
			{
				nid = (int)(rv * (n-1));
				Xrand = m_kCandidateSamps2[nid];
			}
			m_Samples.push_back( Xrand );
		}
		else if( pTree == T2 )
		{
			//n = T->Size();

			//if( n < 500 )
			//{
			//	Xrand = this->RandomState();
			//}
			//else
			//{
			//	nid = (int)(rv * n);
			//	pNode = T->FindNode( nid );
			//	Xrand = pNode->State();
			//	
			//}

			n = m_kCandidateSamps.size();
			//n = m_iPoolSize;
			if( n < 2 || prb > m_dPoolProb )
			{
				nMaxCount = 0;
				//Xrand = this->RandomState();
				do
				{
					Xrand = RandomState();

#ifdef USE_ANN
					double *pScaling = new double[this->P->StateDim];
					P->GetModel()->GetScaling( Xrand, pScaling );
					pNode = pTree->NearestNode( Xrand, dNearestDist, pScaling );
					delete pScaling;
#else
					pNode = SelectNode( Xrand, pTree, true );
					dNearestDist = P->Metric( Xrand, pNode->State() );
#endif
					nMaxCount++;

				}while( dNearestDist < 5 * m_dD/* && nMaxCount < 100*/ );
			}
			else
			{
				nid = (int)(rv * (n-1));
				Xrand = m_kCandidateSamps[nid];
			}
			m_Samples2.push_back( Xrand );
		}
	//}
	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return Xrand;
}

MSLVector RRTConnectPlusPlus::ChooseFreeColState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	MSLVector Xrand;

	do
	{
		Xrand = RandomState();
	}while( !P->Satisfied( Xrand ) );

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return Xrand;
}

bool RRTConnectPlusPlus::Plan()
{
#ifdef ANYTIME
	int nClock;
	double dInteralTime;

	nClock = 0;
	dInteralTime = 0.2;
	m_dCurTime = 0.0;
#endif
	

	nFarFromNode = 0;
    nLineNode = 0;
	m_kCandidateSamps.clear();
	m_kCandidateSamps2.clear();
	m_iCandidateSampsIdx = 0;
    m_iCandidateSampsIdx2 = 0;
	//// ��ʼ��������
	//MSLVector kSample;
	//for( int k = 0; k < m_iPoolSize; ++k )
	//{
	//	kSample = ChooseFreeColState();
	//	m_kCandidateSamps.push_back( kSample );
	//
	//	kSample = ChooseFreeColState();
	//	m_kCandidateSamps2.push_back( kSample );
	//}

	// ��ʱʵ��RRTCon��Plan
	int i;
	MSLNode *nn;
	MSLVector nx,u_best;
	list<MSLNode*> path;
	MSLVector Xrand(P->StateDim);

	// Keep track of time
	float t = used_time();

	cout << "P->InitialState: " << P->InitialState << endl;
	cout << "P->GoalState: " << P->GoalState << endl;
	if (!T)
	{
	// Make the root node of G
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T = new MSLTree(P->InitialState);
			T->Root()->SetCost( 0.0 );

#ifdef USE_ANN
		}
		else
		{
			T = new MSLTree( P->InitialState, this->m_pTopology, this->m_pScaling );
			T->Root()->SetCost( 0.0 );
		}
#endif
	}

	if (!T2)
	{
	// Make the root node of G
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree(P->GoalState);
			T2->Root()->SetCost( 0.0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( P->GoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0.0 );
		}
#endif
	}

	//// ��Զɽ��2011.6.5
	//// �ȸ�������·������С����������T�д��۽ϸߵ�����
	//cout << "Before Clear: Tnum = " << T->Size() << "; T2num = " << T2->Size() << "\n";
	//T->ClearByCost( T->Root(), P->m_dMinCost );
	//T2->ClearByCost( T2->Root(), P->m_dMinCost );
	//cout << "After Clear: Tnum = " << T->Size() << "; T2num = " << T2->Size() << "\n";
	//// ----��Զɽ��2011.6.5

	nn = T->Root();
	MSLNode *nn2 = T2->Root();

	bool bConnected = false;
	i = 0;
#ifdef ANYTIME
	while( 1 )
#else
	while((i < NumNodes) && (!bConnected) )
#endif
	{
		if( gbStop )
		{
			break;
		}

		//m_pNewNodeT = m_pNewNodeT2 = NULL;

		Xrand = ChooseState( T );
		if (Connect(Xrand,T,nn)) { 
			//m_pNewNodeT = nn;
			// Update the goal RRT
			//cout << "nn: " << nn->State() << "  nn2: " << nn2->State() << "\n";
			if (Connect(nn->State(),T2,nn2,false)) {
				if (GapSatisfied(nn->State(),nn2->State())) {
					// ��Զɽ��2011.6.5�޸ģ���������·������С���۸���
					if( m_dPathLength >= nn->Cost() + nn2->Cost() )
					{
						// ��Զɽ2011.7.15
						m_bSuccess = true;
						//---- ��Զɽ2011.7.15

						m_dPathLength = nn->Cost() + nn2->Cost();
						if( m_dPathLength < P->m_dMinCost )
						{
							P->m_dMinCost = m_dPathLength;
						}

						//cout << "CONNECTED!!  MSLNodes: " << 
						//	T->Size()+T2->Size() << "\n";
						bConnected = true;

						RecoverSolution(nn,nn2); // Defined in RRTDual
					}
					// ��Զɽ��2011.6.5
				}
			}
		}
		i++;


#ifdef ANYTIME
		double dMaxTime = 60 * 2;	// ����������
		//used_time(t);
		m_dCurTime += ((double)used_time(t));
		if( m_dCurTime >= nClock * dInteralTime )
		{
			m_pkBookKeeper->CollectionInfo();
			nClock++;
			if( m_dCurTime > dMaxTime )
			{
				break;
			}
		}
#endif

		//m_pNewNodeT = m_pNewNodeT2 = NULL;
		Xrand = ChooseState( T2 );
		if ((!bConnected)&&(Connect(Xrand,T2,nn,false))) { 
			//m_pNewNodeT2 = nn;
			// Update the initial RRT
			if (Connect(nn->State(),T,nn2)) { 
				if (GapSatisfied(nn->State(),nn2->State())) {
					// ��Զɽ��2011.6.5�޸ģ���������·������С���۸���
					if( m_dPathLength >= nn->Cost() + nn2->Cost() )
					{

						// ��Զɽ2011.7.15
						m_bSuccess = true;
						//---- ��Զɽ2011.7.15

						m_dPathLength = nn->Cost() + nn2->Cost();
						if( m_dPathLength < P->m_dMinCost )
						{
							P->m_dMinCost = m_dPathLength;
						}

						//cout << "CONNECTED!!  MSLNodes: " << 
						//	T->Size()+T2->Size() << "\n";
						bConnected = true;

						RecoverSolution(nn2,nn);
					}
					// ��Զɽ��2011.6.5
				}
			}
		}
		i++;

#ifdef ANYTIME
		//used_time(t);
		m_dCurTime += ((double)used_time(t));
		if( m_dCurTime >= nClock * dInteralTime )
		{
			m_pkBookKeeper->CollectionInfo();
			nClock++;
			if( m_dCurTime > dMaxTime )
			{
				break;
			}
		}
#endif

	}

	if (!bConnected)
		cout << "Failure to connect after " << 
		T->Size()+T2->Size() << " nodes\n";

	CumulativePlanningTime += ((double)used_time(t));

	PrintInfo();

	cout << "Line Nodes: " << nLineNode << "\nFar From Nodes: " << nFarFromNode << "\n\n";
	cout << "Extend Back Number: " << m_nExtendBack << endl;

	//m_pkBookKeeper->WriteCollectionInfo( "F:\\OpenSource\\Motion Strategy Library\\msl-2.0-win\\data\\car8\\" );

	return bConnected;
}

bool RRTConnectPlusPlus::Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	bool forward)
{
	m_iExtendNum++;

	// ��Զɽ��2011.07.13����ʱ��ͳ��
	float time = used_time();
	MSLNode *n_best;
		MSLVector nx,u_best;
		bool success;

		n_best = SelectNode(x,t,forward);		// ����RRT��t�����нڵ㣬�ҳ��������x���ŵĽڵ�
		u_best = SelectInput(n_best->State(),x,nx,success,forward);		// ��RRT��t�����Žڵ����������õ�����u��ǰ��һ��
		//cout << "u_best: " << u_best << "\n";
		// nx gets next state
		if (success) {   // If a collision-free input was found
			double dCost = n_best->Cost() + P->Metric( n_best->State(), nx );
			if( dCost < m_dPathLength )
			{
				nn = t->Extend(n_best, nx, u_best, PlannerDeltaT);
				nn->SetCost( dCost );
			}
		}

		// ��Զɽ��2011.07.13����ʱ��ͳ��
		m_dExtendTime += ((double)used_time(time));

		return success;
}
/*
MSLNode* RRTConnectPlusPlus::SelectGoodNode( MSLTree* t, bool forward, MSLVector &Sample ) 
{
	MSLVector x;
	MSLNode *pNearNode, *pTargetNode;
	double dQuality;
	double dCosttoGoal;
	double rv;

	pTargetNode = forward ? m_pNewNodeT2 : m_pNewNodeT;
	if( pTargetNode )		// �ڶ���Extend
	{
		Sample = pTargetNode->State();
		pNearNode = SelectNode( Sample, t, forward );
		return pNearNode;
	}

	do
	{
		x = RandomState();
		if( forward )
		{
			this->m_Samples.push_back( x );
		}
		else
		{
			this->m_Samples2.push_back( x );
		}
		
		pNearNode = SelectNode( x, t, forward );
		dCosttoGoal = P->Metric( pNearNode->State(), P->GoalState );
		dQuality = 1.0 - ( pNearNode->Cost() + dCosttoGoal - m_dMinCost )
			/ ( m_dMaxCost - m_dMinCost );
		dQuality = max( dQuality, m_dFloorQ );
		R >> rv;
	}while( rv > dQuality );

	Sample = x;
	return pNearNode;
}


*/


bool RRTConnectPlusPlus::Connect(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	bool forward)
{
	m_iConnectNum++;

	// ��Զɽ��2011.07.13����ʱ��ͳ��
	float Contime = used_time();

	MSLNode *nn_prev,*n_best;
	MSLVector nx,nx_prev,u_best;
	bool success;
	double d,d_prev,clock;
	int steps;
	double dNewCost, dLineCost;
	MSLVector Sample = x;

	n_best = SelectNode(Sample,t,forward);
	//n_best = this->SelectGoodNode( t, forward, Sample );
	u_best = SelectInput(n_best->State(),Sample,nx,success,forward);			// ��ѡ�����Ҫ����ѡ�Է���ѡ���ÿ��ܻ���ԯ����
	steps = 0;
	// nx gets next state
	if (success) {   // If a collision-free input was found

		double dGoalDist;
		if( t == T )
		{
			dGoalDist = P->Metric( nx, P->GoalState );
		}
		else if( t == T2 )
		{
			dGoalDist = P->Metric( nx, P->InitialState );
		}


		dLineCost = P->Metric( n_best->State(), nx );
		d = P->Metric(nx,Sample); 
		d_prev = d;
		nx_prev = nx; // Initialize
		nn = n_best;
		clock = PlannerDeltaT;

		// �������㷽����ȷ�����������ٶ���ǰ��������Ϊ��ȷ��
		// 1������ʱ�������ΪĿ����T2�Ľڵ㣻2������ʱ�������ΪT1�еĽڵ�
		int nStepNode = 5;
		if( Sample == P->GoalState )
		{
			ConnectTimeLimit = INFINITY;
			nStepNode = 10;
		}

		dNewCost = nn->Cost();
		while ((P->Satisfied(nx))&&
			(clock <= ConnectTimeLimit)&&
			(d <= d_prev))
		{
			// ��Զɽ��2011.6.7
			dNewCost += dLineCost;
			if( dNewCost > m_dPathLength )
			{
				dNewCost -= dLineCost;
				break;
			}

			SatisfiedCount++;
			steps++; // Number of steps made in connecting
			nx_prev = nx;
			d_prev = d; nn_prev = nn;
			// Uncomment line below to select best action each time
			//u_best = SelectInput(g.inf(nn),Sample,nx,success,forward); 
			if (Holonomic) { 
				nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
			}
			else { // Nonholonomic
				if (forward)
					nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
				else
					nx = P->Integrate(nx_prev,u_best,-PlannerDeltaT);
			}

			dLineCost = P->Metric( nx_prev, nx );
			d = P->Metric(nx,Sample);
			clock += PlannerDeltaT;

			if( 0 == (int)(clock / PlannerDeltaT) % nStepNode )
			{
				nLineNode++;
				if( dNewCost > m_dPathLength )
				{
					break;
				}

				// ��Զɽ2011.9.20������ع���������½ڵ����丸�ڵ�Ĺ�ϵ
				double dNearestDist;
				double *pScaling = new double[this->P->StateDim];
				MSLNode* pNearestNode = NULL;

#ifdef USE_ANN
				P->GetModel()->GetScaling( nx, pScaling );
				pNearestNode = t->NearestNode( nx, dNearestDist, pScaling );
#else
				pNearestNode = SelectNode( nx, t, forward );
				//dNearestDist = P->Metric( nx, pNearestNode->State() );
#endif
				if( pNearestNode == nn )
					//if( dNearestDist > 10 * m_dD )
				{
					nn = t->Extend(nn, nx, u_best, steps*PlannerDeltaT);
					nn->SetCost( dNewCost );
					steps = 0;


					MSLNode* pNode = NULL;
					if( t == T )
					{
						if( m_iCandidateSampsIdx < m_iPoolSize )
						{
							m_kCandidateSamps.push_back( nx );
						}
						else
						{
							m_kCandidateSamps[m_iCandidateSampsIdx % m_iPoolSize] = nx;
						}
						//m_kCandidateSamps[m_iCandidateSampsIdx % m_iPoolSize] = nx;
						m_iCandidateSampsIdx++;

						//double dDist = this->P->Metric( nx, P->GoalState );
						//if( ( dDist > dGoalDist ) && P->CollisionFreeLine( nx, P->GoalState) )
						//{
						//	nFarFromNode++;
						//	dGoalDist = dDist;
						//	break;
						//}

						//// ��Զɽ2011.9.22ע��
						//pNode = SelectNode(nx,T2,false);
						//if( P->CollisionFreeLine( nx, pNode->State()) )
						//{
						//	nFarFromNode++;
						//	//Connect( pNode->State(), t, nn, true );
						//	break;
						//}
						//// --��Զɽ2011.9.22ע��
					}
					else if( t == T2 )
					{
						if( m_iCandidateSampsIdx2 < m_iPoolSize )
						{
							m_kCandidateSamps2.push_back( nx );
						}
						else
						{
							m_kCandidateSamps2[m_iCandidateSampsIdx2 % m_iPoolSize] = nx;
						}
						//m_kCandidateSamps2[m_iCandidateSampsIdx2 % m_iPoolSize] = nx;
						m_iCandidateSampsIdx2++;

						//double dDist = this->P->Metric( nx, P->InitialState );
						//if( ( dDist > dGoalDist ) && P->CollisionFreeLine( nx, P->InitialState) )
						//{
						//	nFarFromNode++;
						//	dGoalDist = dDist;
						//	break;
						//}

						//// ��Զɽ2011.9.22ע��
						//pNode = SelectNode(nx,T,true);
						//if( P->CollisionFreeLine( nx, pNode->State()) )
						//{
						//	nFarFromNode++;
						//	//Connect( pNode->State(), t, nn, false );
						//	break;
						//}
						//// --��Զɽ2011.9.22ע��
					}
				}
				else if( clock > 2 * this->PlannerDeltaT )
				{
					m_nExtendBack++;
				}
				delete pScaling;
				// --��Զɽ2011.9.20
			}
			// Uncomment the subsequent two lines to
			//   make each intermediate node added
			//nn = g.new_node(nx_prev); // Make a new node
			//g.new_edge(nn_prev,nn,u_best);
		}

		// ��Զɽ��2011.7.13����ʱ��ͳ��
		float time = used_time();

		if( dNewCost < m_dPathLength && steps > 0 )
		{
			nn = t->Extend(nn_prev, nx_prev, u_best, steps*PlannerDeltaT);

			// ��Զɽ��2011.6.7
			nn->SetCost( dNewCost );
		}
		
		m_dTreeOperTime += ((double)used_time(time));
	}

	// ��Զɽ��2011.07.13����ʱ��ͳ��
	m_dConnectTime += ((double)used_time(Contime));
	return success;
}

