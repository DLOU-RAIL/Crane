#include "BiMRRTs.h"
#include "defs.h"


BiMRRTs::BiMRRTs(Problem* p):RRTConnectPlusPlus(p)
{
	NullInput = MSLVector(p->InputDim);
	m_strName = "BiMRRTs";

	m_kIntruders.clear();
	m_kIntruders2.clear();
	m_dCheckIntruderTime = 0;
	m_dAddRootProb = 0.15;
	m_bUseIntruders = true;
	m_bGuass = true;

	m_kCandidateSamps.clear();
	m_kCandidateSamps2.clear();
	m_iCandidateSampsIdx = 0;
    m_iCandidateSampsIdx2 = 0;


	// 可视化树结构用
  bVisual_ = true;
  ros::NodeHandle node_handle; 
  marker_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  markerVertexes_.header.frame_id  = markerEdges_.header.frame_id = markerPath_.header.frame_id = "/base_footprint";
    markerVertexes_.header.stamp = markerEdges_.header.stamp = markerPath_.header.stamp = ros::Time::now();
    markerVertexes_.ns = markerEdges_.ns = markerPath_.ns = "PRM";
    markerVertexes_.action = markerEdges_.action = markerPath_.action = visualization_msgs::Marker::ADD;
    markerVertexes_.pose.orientation.w =  markerEdges_.pose.orientation.w = markerPath_.pose.orientation.w = 1.0;
    markerVertexes_.id = 0;
    markerEdges_.id = 1;
    markerPath_.id = 2;
    markerVertexes_.type = visualization_msgs::Marker::POINTS;
    markerEdges_.type = visualization_msgs::Marker::LINE_LIST;
    markerPath_.type = visualization_msgs::Marker::LINE_STRIP;
    // POINTS markers use x and y scale for width/height respectively
    markerVertexes_.scale.x = 0.6;
    markerVertexes_.scale.y = 0.6;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    markerEdges_.scale.x = 0.3;
    markerPath_.scale.x = 0.8;
    // Points are green
    markerVertexes_.color.g = 1.0f;
    markerVertexes_.color.a = 1.0;
    // Line list is red
    markerEdges_.color.r = 1.0;
    markerEdges_.color.a = 1.0;
    // Line strip is red
    markerPath_.color.b = 1.0;
    markerPath_.color.a = 0.5;
}

BiMRRTs::~BiMRRTs(void)
{
}

void BiMRRTs::Reset()
{
	RRTConnectPlusPlus::Reset();

	m_kIntruders.clear();
	m_kIntruders2.clear();
	m_dCheckIntruderTime = 0;
	NumNodes = 2000;
}

bool BiMRRTs::Plan()
{

	m_bSuccess = false;
	MSLVector kInitState, kGoalState;
	kInitState = SamplingFromInitTaskSpace();
	kGoalState = SamplingFromGoalTaskSpace();

	m_kCandidateSamps.clear();
	m_kCandidateSamps2.clear();
	m_iCandidateSampsIdx = 0;
    m_iCandidateSampsIdx2 = 0;

	markerVertexes_.points.clear();
    markerEdges_.points.clear();

	if (!T)
	{

#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T = new MSLTree( kInitState/*MSLVector( P->StateDim )*/ );
			T->Root()->SetCost( 0.0 );

#ifdef USE_ANN
		}
		else
		{
			T = new MSLTree( kInitState/*MSLVector( P->StateDim )*/, this->m_pTopology, this->m_pScaling );
			T->Root()->SetCost( 0.0 );
		}
#endif
	}

	if (!T2)
	{

#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree( kGoalState/*MSLVector( P->StateDim )*/ );
			T2->Root()->SetCost( 0.0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( kGoalState/*MSLVector( P->StateDim )*/, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0.0 );
		}
#endif
	}
	
	double dRv;
	int j = 0;
	double dExtendIntruderTime = 0;
	float fTime = 0;
	float t = used_time();

	this->m_lstTrees.clear();
	this->m_lstTrees.push_back( T );
	this->m_lstTrees.push_back( T2 );

	while (j < NumNodes)
	{
		this->R >> dRv;
		if (dRv < m_dAddRootProb || T->Root()->Children().size() < 1
			|| T2->Root()->Children().size() < 1 )
		{
			this->AddInitRoot();
			this->AddGoalRoot();

			if(this->m_bSuccess)
			{
				break;
			}
		}
		else
		{
			if( m_bUseIntruders )
			{
				fTime = used_time();
				if( ExtendIntruders() )
				{
					break;
				}
				dExtendIntruderTime += (double)used_time( fTime );
			}

			if( this->ExtendTrees() )
			{
				break;
			}
		}
		j++;
	}

	CumulativePlanningTime += ((double)used_time(t));
	if (!m_bSuccess)
	{
		cout << "Failure to connect after " << T->Size()+T2->Size() << " nodes\n";
	}
	PrintInfo();

	cout << "CheckTime: " << m_dCheckIntruderTime << "s, account for " 
		<< 100 * m_dCheckIntruderTime / CumulativePlanningTime << "% of Planning Time.\n"; 
	cout << "ExtendIntruderTime: " << dExtendIntruderTime << "s, account for " 
		<< 100 * dExtendIntruderTime / CumulativePlanningTime << "% of Planning Time.\n"; 

	return m_bSuccess;
}
/*
bool BiMRRTs::PlanStar()
{
	m_bSuccess = false;
	MSLVector kInitState, kGoalState;
	kInitState = SamplingFromInitTaskSpace();
	kGoalState = SamplingFromGoalTaskSpace();

	m_kCandidateSamps.clear();
	m_kCandidateSamps2.clear();
	m_iCandidateSampsIdx = 0;
    m_iCandidateSampsIdx2 = 0;

	markerVertexes_.points.clear();
    markerEdges_.points.clear();

	if (!T)
	{

#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T = new MSLTree( kInitState);
			T->Root()->SetCost( 0.0 );

#ifdef USE_ANN
		}
		else
		{
			T = new MSLTree( kInitState, this->m_pTopology, this->m_pScaling );
			T->Root()->SetCost( 0.0 );
		}
#endif
	}

	if (!T2)
	{

#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree( kGoalState );
			T2->Root()->SetCost( 0.0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( kGoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0.0 );
		}
#endif
	}
	
	double dRv;
	int j = 0;
	double dExtendIntruderTime = 0;
	float fTime = 0;
	float t = used_time();

	this->m_lstTrees.clear();
	this->m_lstTrees.push_back( T );
	this->m_lstTrees.push_back( T2 );

	while (j < NumNodes)
	{
		if(!this->m_bSuccess)		// 尚未找到路径
		{
			this->R >> dRv;
			if (dRv < m_dAddRootProb || T->Root()->Children().size() < 1
				|| T2->Root()->Children().size() < 1 )                         // 需要改进：控制树根占总树节点不超过1/3
			{
				this->AddInitRoot();
				this->AddGoalRoot();
			}
			else
			{
				if( m_bUseIntruders )
				{
					fTime = used_time();
					ExtendIntruders();
					dExtendIntruderTime += (double)used_time( fTime );
				}
				this->ExtendTrees();
			}
		}
		else             // 已找到一条可行的路径
		{
			OptimizeTrees();
		}


		j++;
	}

	CumulativePlanningTime += ((double)used_time(t));
	if (!m_bSuccess)
	{
		cout << "Failure to connect after " << T->Size()+T2->Size() << " nodes\n";
	}
	PrintInfo();

	cout << "CheckTime: " << m_dCheckIntruderTime << "s, account for " 
		<< 100 * m_dCheckIntruderTime / CumulativePlanningTime << "% of Planning Time.\n"; 
	cout << "ExtendIntruderTime: " << dExtendIntruderTime << "s, account for " 
		<< 100 * dExtendIntruderTime / CumulativePlanningTime << "% of Planning Time.\n"; 

	return m_bSuccess;
}

void BiMRRTs::OptimizeTrees()
{
	MSLVector kSample;
	ExtendStatus es;

	// 1.选择随机采样点
	kSample = ChooseState();

	// 2.向随机采样点生长
	es = ExtendStar(T, kSample, pNewNode, true);
}

// 向采样点扩展
BiMRRTs::ExtendStatus BiMRRTs::ExtendStar( MSLTree *pTree, const MSLVector &kSample, MSLNode *&pNewNode, bool bForward )
{
	MSLNode *pBestParent, *pInitParent; 
	MSLVector kNewState, kBestInput;
	std::vector< MSLNode* > vNearNodes;
    bool bSuccess = false;
	pBestParent = pInitParent = NULL;

	// 1.生成新状态
	// 1.1 选择最近节点
	pBestParent = SelectNode( kSample, pTree, bForward );
	// 1.2 选择最优动作，尝试扩展
	kBestInput = SelectInput(pBestParent->State(), kSample, kNewState, bSuccess, bForward);
	// 1.3 生成树节点
	if (bSuccess) {   // If a collision-free input was found
		// Extend the tree
		pNewNode = pTree->Extend(pBestParent, kNewState, kBestInput, PlannerDeltaT);
		double dCost = pBestParent->Cost() + P->Metric( pBestParent->State(), kNewState );
		pNewNode->SetCost( dCost );
	}
	else     // 扩展失败
	{
		return ExtendStatus::NOEXTEND;
	}

	// 2.选择kNewState的邻近节点集
	vNearNodes = SelectNearNodes( kNewState, pTree, bForward );          // 已实现

	// 3.挑选优秀的父节点（生长点）
	pInitParent = pBestParent;
	SelectBestParent( kNewState, vNearNodes, dCost, pBestParent, pNewNode );     // 找到优秀的父节点，并已修改树枝，确保pNewNode自身已是最优

	// 4.改道
	RewireNodes( pNewNode, vNearNodes );

	// 5.尝试让另一棵树连接到新节点pNewNode
	MSLNode *pOtherNewNode = NULL;
	MSLTree *theOtherTree = pTree == T ? T2 : T;
	if (Connect(pNewNode->State(), theOtherTree, pOtherNewNode, false)) 
	{
		if (GapSatisfied(pNewNode->State(), pOtherNewNode->State())) 
		{
			if( pNewNode->Cost() + pOtherNewNode->Cost() < P->m_dMinCost )
			{
				P->m_dMinCost = pNewNode->Cost() + pOtherNewNode->Cost();
				connected = true;
				m_bSuccess = true;
				RecoverSolution(pNewNode, pOtherNewNode);
			}
		}
	}

	return eRes;
}
*/

void BiMRRTs::DisplayTreeNodeAndEdge(const MSLVector& x1, const MSLVector& x2)
{
	// 添加可视化坐标点
	vertex_.x = x1[0];
    vertex_.y = x1[1];
    vertex_.z = 0.0;
    markerEdges_.points.push_back(vertex_);
    vertex_.x = x2[0];
    vertex_.y = x2[1];
    vertex_.z = 0.0;
	markerEdges_.points.push_back(vertex_);
    markerVertexes_.points.push_back(vertex_);
    if(bVisual_)
    {
      marker_pub_.publish(markerVertexes_);
	  marker_pub_.publish(markerEdges_);
    }
}
bool BiMRRTs::AddInitRoot()
{
	MSLVector x;
	int iMaxIter = 10;
	int i = 0;
	while( 1/*i < iMaxIter*/ )
	{
		x = SamplingFromInitTaskSpace();
		
		if( P->Satisfied( x ) )
		{
			T->Extend( T->Root(), x, NullInput, 1 );
			DisplayTreeNodeAndEdge(T->Root()->State(), x);
			// ���ýڵ�����������
			if( m_iCandidateSampsIdx < m_iPoolSize )
			{
				m_kCandidateSamps.push_back( x );
			}
			else
			{
				m_kCandidateSamps[m_iCandidateSampsIdx % m_iPoolSize] = x;
			}
			m_iCandidateSampsIdx++;

			return true;
		}
		++i;
	}
    
	return false;
}
bool BiMRRTs::AddGoalRoot()
{
	MSLVector x;
	int iMaxIter = 10;
	int i = 0;
	while( 1/*i < iMaxIter*/ )
	{
		x = SamplingFromGoalTaskSpace();
		if( P->Satisfied( x ) )
		{
			T2->Extend( T2->Root(), x, NullInput, 1 );

			DisplayTreeNodeAndEdge(T2->Root()->State(), x);

			// ���ýڵ�����������
			if( m_iCandidateSampsIdx2 < m_iPoolSize )
			{
				m_kCandidateSamps2.push_back( x );
			}
			else
			{
				m_kCandidateSamps2[m_iCandidateSampsIdx2 % m_iPoolSize] = x;
			}
			//m_kCandidateSamps2[m_iCandidateSampsIdx2 % m_iPoolSize] = nx;
			m_iCandidateSampsIdx2++;

			return true;
		}
		i++;
	}
    
	return false;
}

MSLVector BiMRRTs::SamplingFromInitTaskSpace()
{
	return P->SamplingFromPickCLR();
}


MSLVector BiMRRTs::SamplingFromGoalTaskSpace()
{
	return P->SamplingFromPlaceCLR();
}


bool BiMRRTs::ExtendTrees()
{
	bool connected = false;
	int i = 0;
	MSLNode *nn, *nn2;
	MSLVector x;

	nn = T->Root()->Children().front();
	nn2 = T2->Root()->Children().front();
	x = ChooseState();
	this->m_Samples.push_back( x );

	if (Connect(x,T,nn)) 
	{ 
		if (Connect(nn->State(),T2,nn2,false)) 
		{
			if (GapSatisfied(nn->State(),nn2->State())) 
			{
				if( nn->Cost() + nn2->Cost() < P->m_dMinCost )
				{
					P->m_dMinCost = nn->Cost() + nn2->Cost();
					connected = true;
					m_bSuccess = true;
					RecoverSolution(nn,nn2);
					return connected;
				}
			}
		}
	}
	i++;

	x = ChooseState();
	this->m_Samples2.push_back( x );

	if ((!connected)&&(Connect(x,T2,nn2,false)))
	{ 
		if (Connect(nn2->State(),T,nn)) 
		{ 
			if (GapSatisfied(nn2->State(),nn->State())) 
			{
							
				if( nn->Cost() + nn2->Cost() < P->m_dMinCost )
				{
					P->m_dMinCost = nn->Cost() + nn2->Cost();						
					connected = true;
					m_bSuccess = true;		
					RecoverSolution(nn,nn2);
					return connected;
				}
			}
		}
	}
	i++;	
	return connected;

}


bool BiMRRTs::ExtendIntruders()
{
	list<MSLNode*> path;
	bool bSuccess = false;
	MSLNode* pGrowingNode = NULL;
	MSLNode* pNewNode = NULL;
	MSLVector kSample;
	int n = 0;
	int nid;
	double rv;
	R >> rv;

	// 1 ��չT����
	// 1.1 �����ּ���ѡ��������
	n = m_kIntruders.size();
	if( n > 0 )
	{
		nid = (int)(rv * (n-1));
		pGrowingNode = m_kIntruders[nid];
		// 1.2 ���������
		std::vector<double> baseConf(3, 0.0), craneConf(8, 0.0);
		baseConf[0] = pGrowingNode->State()[0];
		baseConf[1] = pGrowingNode->State()[1];
		baseConf[2] = pGrowingNode->State()[2];
		P->m_pPlaceCLR->upperIK(baseConf, craneConf);
		craneConf.erase(craneConf.begin()+5);
		kSample.setFromSTDVector(craneConf);
		// 1.3 ��չ
		//if( Connect( kSample, T, pNewNode ) )

		if( ConnectLine( pGrowingNode, kSample, T, pNewNode ) )
		{
			if( GapSatisfied( pNewNode->State(), kSample ) )
			{
				if( pNewNode->Cost() < P->m_dMinCost )
				{
					P->m_dMinCost = pNewNode->Cost();						
					m_bSuccess = true;		
					path = T->PathToRoot( pNewNode );
					path.reverse();
					//path.pop_front();	

					RecordSolution(path); // Write to Path and Policy

					// ȥ��·�����������
					cout << Path.size() << endl;
					Path.pop_front();
					TimeList.pop_front();
					CostList.pop_front();
					//cout << Path.size() << endl << TimeList.size() << endl << CostList.size() << endl;
					return m_bSuccess;
				}
			}
		}
	}

	// 2 ��չT2����
	// 2.1 �����ּ���ѡ��������
	pNewNode = NULL;
	n = m_kIntruders2.size();
	if( n > 0 )
	{
		nid = (int)(rv * (n-1));
		pGrowingNode = m_kIntruders2[nid];
		// 2.2 ���������
		std::vector<double> baseConf(3, 0.0), craneConf(8, 0.0);
		baseConf[0] = pGrowingNode->State()[0];
		baseConf[1] = pGrowingNode->State()[1];
		baseConf[2] = pGrowingNode->State()[2];
		P->m_pPickCLR->upperIK(baseConf, craneConf);
		craneConf.erase(craneConf.begin()+5);
		kSample.setFromSTDVector(craneConf);
		// 2.3 ��չ
		//if( Connect( kSample, T2, pNewNode, false ) )

		if( ConnectLine( pGrowingNode, kSample, T2, pNewNode, false ) )
		{
			if( GapSatisfied( pNewNode->State(), kSample ) )
			{
				if( pNewNode->Cost() < P->m_dMinCost )
				{
					P->m_dMinCost = pNewNode->Cost();						
					m_bSuccess = true;		
					path = T2->PathToRoot( pNewNode );
					RecordSolution(path); // Write to Path and Policy
					//Path.pop_front();
					// ȥ��·���������յ�
					cout << Path.size() << endl;
					Path.pop_back();
					TimeList.pop_back();
					CostList.pop_back();
					cout << Path.size() << endl << TimeList.size() << endl << CostList.size() << endl;
					return m_bSuccess;
				}
			}
		}
	}

	return bSuccess;
}

void BiMRRTs::RecoverSolution(MSLNode *n1, MSLNode *n2) 
{
	list<MSLNode*> path,path2;

	path = T->PathToRoot(n1);
	path.reverse();

	path2 = T2->PathToRoot(n2); 

	RecordSolution(path,path2); // Write to Path and Policy

	Path.pop_front();
	Path.pop_back();
	TimeList.pop_front();
	TimeList.pop_back();
	CostList.pop_front();
	CostList.pop_back();
}

bool BiMRRTs::Connect(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	bool forward)
{
	m_iConnectNum++;

	// ��Զɽ��2011.07.13����ʱ��ͳ��
	float Contime = used_time();
	float dCheckTime = 0;

	MSLNode *nn_prev,*n_best;
	MSLVector nx,nx_prev,u_best;
	bool success;
	double d,d_prev,clock;
	int steps;
	double dNewCost, dLineCost;
	MSLVector Sample = x;

	n_best = SelectNode(Sample,t,forward);
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
			if( dNewCost > P->m_dMinCost )
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
				if( dNewCost > P->m_dMinCost )
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

					DisplayTreeNodeAndEdge(nn->State(), nx);

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

						dCheckTime = used_time();
						std::vector<double> baseConf(3, 0.0);
						baseConf[0] = nn->State()[0];
						baseConf[1] = nn->State()[1];
						baseConf[2] = nn->State()[2];
						if( P->m_pPlaceCLR->baseInCLR( baseConf ) )
						{
							m_kIntruders.push_back( nn );
						}
						m_dCheckIntruderTime += (double)used_time( dCheckTime );

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
						m_iCandidateSampsIdx2++;

						dCheckTime = used_time();
						std::vector<double> baseConf(3, 0.0);
						baseConf[0] = nn->State()[0];
						baseConf[1] = nn->State()[1];
						baseConf[2] = nn->State()[2];
						if( P->m_pPickCLR->baseInCLR( baseConf ) )
						{
							m_kIntruders2.push_back( nn );
						}
						m_dCheckIntruderTime += (double)used_time( dCheckTime );
					}
				}
				else if( clock > 2 * this->PlannerDeltaT )
				{
					m_nExtendBack++;
				}
				delete pScaling;
				// --��Զɽ2011.9.20
			}
		}

		// ��Զɽ��2011.7.13����ʱ��ͳ��
		float time = used_time();

		if( dNewCost < P->m_dMinCost && steps > 0 )
		{
			nn = t->Extend(nn_prev, nx_prev, u_best, steps*PlannerDeltaT);

			DisplayTreeNodeAndEdge(nn_prev->State(), nx_prev);

			// ��Զɽ��2011.6.7
			nn->SetCost( dNewCost );
		}
		
		m_dTreeOperTime += ((double)used_time(time));
	}

	// ��Զɽ��2011.07.13����ʱ��ͳ��
	m_dConnectTime += ((double)used_time(Contime));
	return success;
}


bool BiMRRTs::ConnectLine( MSLNode* pGrowingNode, const MSLVector &x, MSLTree *t, MSLNode *&nn, bool forward )
{
	bool bExtended = false;
	MSLVector kInput( P->InputDim );
	MSLVector kNewState( P->StateDim );

	kInput = P->GetInputFromStates( pGrowingNode->State(), x );
	kInput = kInput * 0.05;
	MSLVector kStep = P->StateDifference( pGrowingNode->State(), x );
	kStep = kStep * 0.05;
	//cout << pGrowingNode->State() << "\t" << x << endl;
	//cout << "Step: " << kStep << endl;
	double dStepTime = forward ? 1 : -1;
	int i = 0;
	double dNewCost = 0;
	do
	{
		++i;
		//kNewState = P->Integrate( pGrowingNode->State(), kInput, dStepTime );
		kNewState = pGrowingNode->State() + kStep;
		kNewState = P->Round( kNewState );
		//cout << kNewState << endl;

		// �����ϰ����˳�ѭ��
		if( !P->Satisfied( kNewState ) )
		{
			//cout << "COLLISION!!" << endl;
			break;
		}
		// ��������㣬�޸�����״̬Ϊx
		if( this->GapSatisfied( kNewState, x ) )
		{
			kNewState = x;
			//cout << "REACHED!!" << endl;
		}
		nn = t->Extend( pGrowingNode, kNewState, dStepTime * kInput );
		DisplayTreeNodeAndEdge(pGrowingNode->State(), kNewState);
		dNewCost = pGrowingNode->Cost() + P->Metric( pGrowingNode->State(), kNewState );
		nn->SetCost( dNewCost );
		pGrowingNode = nn;
		bExtended = true;
	}while( !this->GapSatisfied( kNewState, x ) );

	//cout << "i = " << i << endl;
	//if( i == 10 )
	//{
	//	i = 0;
	//}

	return bExtended;
}


void BiMRRTs::ConstructTrees()
{
	//kInitState = SamplingFromInitTaskSpace();
	//kGoalState = SamplingFromGoalTaskSpace();

	//T = new MSLTree( kInitState );
	//T2 = new MSLTree( kGoalState );


}
