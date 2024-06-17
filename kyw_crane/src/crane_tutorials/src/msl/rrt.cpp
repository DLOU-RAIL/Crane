//----------------------------------------------------------------------
//               The Motion Strategy Library (MSL)
//----------------------------------------------------------------------
//
// Copyright (c) University of Illinois and Steven M. LaValle.     
// All Rights Reserved.
// 
// Permission to use, copy, and distribute this software and its 
// documentation is hereby granted free of charge, provided that 
// (1) it is not a component of a commercial product, and 
// (2) this notice appears in all copies of the software and
//     related documentation. 
// 
// The University of Illinois and the author make no representations
// about the suitability or fitness of this software for any purpose.  
// It is provided "as is" without express or implied warranty.
//----------------------------------------------------------------------

#include <math.h>
#include <stdio.h>
#include "defs.h"

#include "rrt.h"

double Gamma( double x )
{
    int i,k,m;
    double ga,gr,r,z;

    static double g[] = {
        1.0,
        0.5772156649015329,
       -0.6558780715202538,
       -0.420026350340952e-1,
        0.1665386113822915,
       -0.421977345555443e-1,
       -0.9621971527877e-2,
        0.7218943246663e-2,
       -0.11651675918591e-2,
       -0.2152416741149e-3,
        0.1280502823882e-3,
       -0.201348547807e-4,
       -0.12504934821e-5,
        0.1133027232e-5,
       -0.2056338417e-6,
        0.6116095e-8,
        0.50020075e-8,
       -0.11812746e-8,
        0.1043427e-9,
        0.77823e-11,
       -0.36968e-11,
        0.51e-12,
       -0.206e-13,
       -0.54e-14,
        0.14e-14};

    if (x > 171.0) return 1e308;    // This value is an overflow flag.
    if (x == (int)x) {
        if (x > 0.0) {
            ga = 1.0;               // use factorial
            for (i=2;i<x;i++) {
               ga *= i;
            }
         }
         else
            ga = 1e308;
     }
     else {
        if (fabs(x) > 1.0) {
            z = fabs(x);
            m = (int)z;
            r = 1.0;
            for (k=1;k<=m;k++) {
                r *= (z-k);
            }
            z -= m;
        }
        else
            z = x;
        gr = g[24];
        for (k=23;k>=0;k--) {
            gr = gr*z+g[k];
        }
        ga = 1.0/(gr*z);
        if (fabs(x) > 1.0) {
            ga *= r;
            if (x < 0.0) {
                ga = -PI/(x*ga*sin(PI*x));
            }
        }
    }
    return ga;
}

double Volume( double dR, int nDim )
{
	double dVol = pow( PI, 0.5 * nDim ) * pow( dR, nDim ) / (Gamma( 0.5 * nDim + 1 ));
	return dVol;
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRT base class
// 
// *********************************************************************
// *********************************************************************

RRT::RRT(Problem *problem): IncrementalPlanner(problem) {

	UseANN = true;
	
//#ifdef USE_ANN
//	m_pMAG = m_pMAG2 = NULL;
//	m_pnTopology = NULL;
//	m_pdScaling = NULL;
//#endif

	m_strName = "RRT";
	m_bFirstWrite = true;

	READ_PARAMETER_OR_DEFAULT(ConnectTimeLimit,INFINITY);

	Reset();

}

// // ��Զɽ2011.7.15
// #include <windows.h>
// #include <process.h>
// static string GetAppPath( HMODULE hModule )
// {
// 	char dir[MAX_PATH];
	
// 	::GetModuleFileName( hModule, dir, MAX_PATH );
// 	string strAppDir( dir );
// 	string strAppDir1 = strAppDir;
// 	string::size_type pos = strAppDir1.rfind( "\\" );
// 	strAppDir = strAppDir1.substr( 0, pos );

// 	return strAppDir;
// }

void RRT::Reset() {
	IncrementalPlanner::Reset();

	// ��Զɽ��2011.6.5���
	m_dPathLength = INFINITY;
	// --��Զɽ��2011.6.5���

	// ��Զɽ��2011.6.6
	m_dChooseStateTime = 0;
	m_dSelectNodeTime = 0;
	m_dSelectInputTime = 0;
	m_dTreeOperTime = 0;
	m_dExtendTime = 0;
	m_dConnectTime = 0;
	P->m_dCumulativeCollisionTime = 0;
	P->m_dMetricTime = 0;
	// --��Զɽ��2011.6.6

	// ��Զɽ��2011.7.18
	m_iChooseStateNum = 0;
	m_iSelectNodeNum = 0;
	m_iSelectInputNum = 0;
	m_iExtendNum = 0;
	m_iConnectNum = 0;
	P->m_iCollisionNum = 0;
	P->m_iMetricNum = 0;
	P->m_dMinCost = INFINITY;
	// --��Զɽ��2011.7.18

	m_dInputCost = INFINITY;

	NumNodes = 15000;

	SatisfiedCount = 0;
	GoalDist = P->Metric(P->InitialState,P->GoalState);
	BestState = P->InitialState;

	
	//double dMaxR = 0.5 * P->Metric( P->InitialState, P->GoalState );
	//m_dGamma = 1.5 * 2 * pow( 1.0 + 1.0 / (double)P->StateDim, 1.0 / (double)P->StateDim )
	//	* Volume( dMaxR, P->StateDim ) / Volume( 1, P->StateDim );

	m_dGamma = 100.0;

	//#ifdef USE_ANN
	//	MAG = MultiANN(&G);
	//	MAG2 = MultiANN(&G2);
	//#endif

#ifdef USE_ANN

	m_pTopology = NULL;
	m_pScaling = NULL;
	// ��ȡ״̬ά��
	int nStateDim = P->StateDim;

	std::ifstream fin;

	// ���ԭ��״̬��������Ϣ��������Ϣ
	if( m_pTopology )
	{
		delete m_pTopology;
	}
	if( m_pScaling )
	{
		delete m_pScaling;
	}

	// ��ȡ״̬������Ϣ
	m_pTopology = new int[nStateDim];
	fin.open((FilePath + "topology").c_str());
	//string str = "F:\\OpenSource\\Motion Strategy Library\\msl-2.0-win\\data\\car8clutterobst1\\";
	//fin.open(( str + "topology").c_str());
	if (fin) 
	{
		for (int i = 0; i < nStateDim; i++) 
		{
			fin >> m_pTopology[i];
		}
	}
	else 
	{
		cout << "Error: No Topology file was found\n";
		exit(-1);
	}
	fin.close();

	// ��ȡ״̬����������
	m_pScaling = new double[nStateDim];

	fin.open((FilePath + "scaling").c_str());
	//fin.open((str + "scaling").c_str());
	if (fin) 
	{
		for (int i = 0; i < nStateDim; i++) 
		{
			fin >> m_pScaling[i];
		}
	}
	else 
	{
		cout << "Error: No Scaling file was found\n";
		exit(-1);
	}
	fin.close();
#endif
}




// Return the best new state in nx_best
// success will be false if no action is collision free
// ��RRT��t�����Žڵ����������õ�����u��ǰ��һ��
MSLVector RRT::SelectInput(const MSLVector &x1, const MSLVector &x2, 
	MSLVector &nx_best, bool &success,
	bool forward = true)
{
	m_iSelectInputNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	float time = used_time();

	MSLVector u_best,nx;
	list<MSLVector>::iterator u;
	double d,d_min;
	success = false;
	d_min = (forward) ? P->Metric(x1,x2) : P->Metric(x2,x1);
	list<MSLVector> il = P->GetInputs(x1);


	if (Holonomic) { // Just do interpolation
		u_best = P->InterpolateState(x1,x2,1.0) - x1;
		u_best = u_best.norm(); // Normalize the direction
		nx_best = P->Integrate(x1,u_best,PlannerDeltaT);
		SatisfiedCount++;

		if (P->Satisfied(nx_best))
			success = true;
	}
	else {  // Nonholonomic (the more general case -- look at Inputs)
		// ��Զɽ���
		double d_tmpMin = INFINITY;
		// ��Զɽ��� 
		forall(u,il) {
			if (forward)
				nx = P->Integrate(x1,*u,PlannerDeltaT);
			else
				nx = P->Integrate(x1,*u,-PlannerDeltaT);

			d  = (forward) ? P->Metric(nx,x2): P->Metric(x2,nx);

			SatisfiedCount++;
// ԭ���ģ��о���̫����------------------------------------------
			if ((d < d_min)&&(x1 != nx)) { 
				if (P->Satisfied(nx)) {
					d_min = d; 
					u_best = *u; 
					nx_best = nx; 
					success = true;
				}
			}
// -------------------------------------------ԭ���ģ��о���̫����
// ��Զɽ��ӣ�ѡ������С��u��Ϊ�������
			//if ((x1 != nx)) { 
			//	if (P->Satisfied(nx)) {
			//		if( d_tmpMin > d )
			//		{
			//		d_tmpMin = d; 
			//		u_best = *u; 
			//		nx_best = nx; 
			//		}
			//		success = true;
			//	}
			//}
// ------------------------------------��Զɽ���
		}
	}

	//cout << "u_best: " << u_best << "\n";
	//cout << "nx_best: " << nx_best << "\n";
	//cout << "success: " << success << "\n";
	//success = true;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	m_dSelectInputTime += ((double)used_time(time));

	return u_best;
}



// ����RRT��t�����нڵ㣬�ҳ��������x���ŵĽڵ�
MSLNode* RRT::SelectNode(const MSLVector &x, MSLTree* t,
	bool forward) {

		m_iSelectNodeNum++;

		// ��Զɽ��2011.6.6����ʱ��ͳ��
		float time = used_time();

		double d,d_min;
		MSLNode *n_best;
		list<MSLNode*>::iterator n;

		d_min = INFINITY; d = 0.0;

		//#ifdef USE_ANN
		//		if (!UseANN) {
		//#endif

#ifdef USE_ANN
		if (!UseANN) {
#endif
			list<MSLNode*> nl;
			nl = t->Nodes();
			forall(n,nl) {
				d = (forward) ? P->Metric((*n)->State(),x) : P->Metric(x,(*n)->State());
				if (d < d_min) {
					d_min = d; n_best = (*n); 
				}
			} 
			//#ifdef USE_ANN
			//		}
			//		else {
			//			if (&g == &G)
			//				n_best = MAG.SelectNode(x);
			//			else
			//				n_best = MAG2.SelectNode(x);
			//		}
			//#endif

#ifdef USE_ANN
		}
		else 
		{
			double dBestDist;
			//n_best = t->NearestNode( x, dBestDist );
			double *pScaling = new double[this->P->StateDim];
			this->P->GetModel()->GetScaling( x, pScaling );
			n_best = t->NearestNode( x, dBestDist, pScaling );
			delete pScaling;
		}
#endif
		//cout << "n_best: " << (*n_best) << "\n";

		// ��Զɽ��2011.6.6����ʱ��ͳ��
		m_dSelectNodeTime += ((double)used_time(time));

		return n_best;
}

// ��Զɽ2011.4.29
std::vector< MSLNode* > RRT::SelectNearNodes( const MSLVector &x, MSLTree *t,
	bool forward )
{
	m_iSelectNodeNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	float time = used_time();
	std::vector< MSLNode* > vNodes;

#ifdef USE_ANN
	if( !UseANN )
	{
#endif
	double d,d_rn;
	
	list<MSLNode*>::iterator n;
	int num = t->Size();
	//const double gama = 20.0;
	//const double d_rmax = 30;

	d_rn = m_dGamma * pow( log( (double)(num+1) ) / (double)(num+1), 1.0 / (double)P->StateDim ); 
	d = 0.0;

	list<MSLNode*> nl;
	nl = t->Nodes();
	forall(n,nl) 
	{
		d = (forward) ? P->Metric((*n)->State(),x) : P->Metric(x,(*n)->State());
		if (d < d_rn) 
		{
			vNodes.push_back( *n ); 
		}
	} 

#ifdef USE_ANN
	}
	else
	{
		vector< double > vBestDist;
		vNodes = t->kNearestNodes( x, vBestDist );
	}
#endif

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	m_dSelectNodeTime += ((double)used_time(time));

	return vNodes;
}
  // ------------------��Զɽ2011.4.29


bool RRT::Extend(const MSLVector &x, 
	MSLTree *t,
	MSLNode *&nn, bool forward = true) {

		m_iExtendNum++;

		// ��Զɽ��2011.7.17����ʱ��ͳ��
		float time = used_time();

		MSLNode *n_best;
		MSLVector nx,u_best;
		bool success;

		n_best = SelectNode(x,t,forward);		// ����RRT��t�����нڵ㣬�ҳ��������x���ŵĽڵ�
		u_best = SelectInput(n_best->State(),x,nx,success,forward);		// ��RRT��t�����Žڵ����������õ�����u��ǰ��һ��
		//cout << "u_best: " << u_best << "\n";
		// nx gets next state
		if (success) {   // If a collision-free input was found
			// Extend the tree
			nn = t->Extend(n_best, nx, u_best, PlannerDeltaT);

			double dCost = n_best->Cost() + P->Metric( n_best->State(), nx );
			nn->SetCost( dCost );

			//cout << "n_best: " << n_best << "\n";
			//cout << "New node: " << nn << "\n";
		}


		// ��Զɽ��2011.07.13����ʱ��ͳ��
		m_dExtendTime += ((double)used_time(time));

		return success;
}



// This function essentially iterates Extend until it has to stop
// The same action is used for every iteration
bool RRT::Connect(const MSLVector &x, 
	MSLTree *t,
	MSLNode *&nn, bool forward = true) {

		m_iConnectNum++;

		// ��Զɽ��2011.07.13����ʱ��ͳ��
		float Contime = used_time();

		MSLNode *nn_prev,*n_best;
		MSLVector nx,nx_prev,u_best;
		bool success;
		double d,d_prev,clock;
		int steps;

		n_best = SelectNode(x,t,forward);
		u_best = SelectInput(n_best->State(),x,nx,success,forward);			// ��ѡ�����Ҫ����ѡ�Է���ѡ���ÿ��ܻ���ԯ����
		steps = 0;
		double dCost = n_best->Cost();
		// nx gets next state
		if (success) {   // If a collision-free input was found

			dCost += P->Metric( n_best->State(), nx );

			d = P->Metric(nx,x); d_prev = d;
			nx_prev = nx; // Initialize
			nn = n_best;
			clock = PlannerDeltaT;
			while ((P->Satisfied(nx))&&
				(clock <= ConnectTimeLimit)&&
				(d <= d_prev))
			{	
				SatisfiedCount++;
				steps++; // Number of steps made in connecting
				nx_prev = nx;
				d_prev = d; nn_prev = nn;
				// Uncomment line below to select best action each time
				//u_best = SelectInput(g.inf(nn),x,nx,success,forward); 
				if (Holonomic) { 
					nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
				}
				else { // Nonholonomic
					if (forward)
						nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
					else
						nx = P->Integrate(nx_prev,u_best,-PlannerDeltaT);
				}
				d = P->Metric(nx,x);
				clock += PlannerDeltaT;

				dCost += P->Metric( nx_prev, nx );

				// Uncomment the subsequent two lines to
				//   make each intermediate node added
				//nn = g.new_node(nx_prev); // Make a new node
				//g.new_edge(nn_prev,nn,u_best);
			}

			if( dCost < m_dPathLength )
			{
				nn = t->Extend(n_best, nx_prev, u_best, steps*PlannerDeltaT);
				nn->SetCost( dCost );
			}
		}

		// ��Զɽ��2011.07.13����ʱ��ͳ��
		m_dConnectTime += ((double)used_time(Contime));

		return success;
}



bool RRT::Plan()
{
	int i;
	double d;
	MSLNode *n,*nn,*n_goal;
	MSLVector nx,u_best;
	list<MSLNode*> path;
	MSLVector x(P->StateDim);

	// Keep track of time
	float t = used_time();

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

	nn = T->Root();

	i = 0;
	n = SelectNode(P->GoalState,T);		// ���е�T�п������ϴι�������
	n_goal = n;

	GoalDist = P->Metric(n->State(),P->GoalState);		// GoalDist��Ϊ���嵼��
	while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))) {

		x = ChooseState();
		this->m_Samples.push_back( x );

		//cout<< i << " ������Ϊ��" << Xrand << "\n";
		if (Extend(x,T,nn)) {		// �Ƿ�RRT������չһ��
			d = P->Metric(nn->State(),P->GoalState);
			//cout<< "���еĽڵ�" << nn->State() << "��������㣬����Ϊ��" << d << "\n";
			if (d < GoalDist) {  // Decrease if goal closer
				//cout << "�þ����ԭ�ȵľ���" << GoalDist << " ��\n";		 
				GoalDist = d;
				BestState = nn->State();
				n_goal = nn;

			}
		}
		i++;
	}

	CumulativePlanningTime += ((double)used_time(t));
	

	// Get the solution path
	if (GapSatisfied(n_goal->State(),P->GoalState)) {
		m_bSuccess = true;
		cout << "Success\n";
		path = T->PathToRoot(n_goal);
		path.reverse();
		RecordSolution(path); // Write to Path and Policy

		m_dPathLength = n_goal->Cost();
		if( m_dPathLength < P->m_dMinCost )
		{
			P->m_dMinCost = m_dPathLength;
		}
	}
	else {
		cout << "Failure\n";
		m_bSuccess = false;	
	}
	
	PrintInfo();

	return m_bSuccess;
}



MSLVector RRT::ChooseState() {

	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();
	MSLVector Xrand = RandomState();
	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return Xrand;
}

void RRT::PrintInfo()
{
	cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

	cout << "Planning Time: " << CumulativePlanningTime << "s\n"; 
	cout << "Collision Time: " << P->m_dCumulativeCollisionTime << "s, account for " << 
		P->m_dCumulativeCollisionTime * 100 / CumulativePlanningTime << "% of Planning Time.\n"; 
	cout << "ChooseState Time: " << m_dChooseStateTime << "s, account for " << 
		m_dChooseStateTime * 100 / CumulativePlanningTime << "% of Planning Time.\n"; 
	cout << "Select Input Time: " << m_dSelectInputTime << "s, account for " << 
		m_dSelectInputTime * 100 / CumulativePlanningTime << "% of Planning Time.\n"; 
	cout << "Select Node Time: " << m_dSelectNodeTime << "s, account for " << 
		m_dSelectNodeTime * 100 / CumulativePlanningTime << "% of Planning Time.\n"; 
	cout << "Tree Operation Time: " << m_dTreeOperTime << "s, account for " << 
		m_dTreeOperTime * 100 / CumulativePlanningTime << "% of Planning Time.\n";
	cout << "Extend Time: " << m_dExtendTime << "s, account for " << 
		m_dExtendTime * 100 / CumulativePlanningTime << "% of Planning Time.\n";
	cout << "Connect Time: " << m_dConnectTime << "s, account for " << 
		m_dConnectTime * 100 / CumulativePlanningTime << "% of Planning Time.\n";
	cout << "Metric Time: " << P->m_dMetricTime << "s, account for " << 
		P->m_dMetricTime * 100 / CumulativePlanningTime << "% of Planning Time.\n";
	double dStatTime = P->m_dCumulativeCollisionTime + m_dChooseStateTime + 
		m_dSelectInputTime + m_dSelectNodeTime + m_dTreeOperTime;
	cout << "Statistics Time: " << dStatTime << "s, account for " << 
		dStatTime * 100 / CumulativePlanningTime << "% of Planning Time.\n";
	cout << "Cost of Path: " << P->m_dMinCost/*m_dPathLength*/ << "\n\n";

	//double dCost = INFINITY;
	//if( !CostList.empty() )
	//{
	//	dCost = CostList.back();
	//}
	//cout << "Cost of Path Recorded by CostList: " << dCost << "\n\n";
}

void RRT::Statistics( string path )
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
		fout << ",��չ����";
		fout << ",��չ��ʱ��";
		fout << ",��չ��ʱ��ٷֱ�";
		fout << ",���Ӵ���";
		fout << ",����ʱ��";
		fout << ",����ʱ��ٷֱ�";
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

	fout << "\n" << m_bSuccess << "," << P->m_dMinCost/*m_dPathLength*/ << "," << CumulativePlanningTime 
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
	fout << "," << m_iExtendNum;
	fout << "," << m_dExtendTime;
	fout << "," << m_dExtendTime * 100 / CumulativePlanningTime << "%";
	fout << "," << m_iConnectNum;
	fout << "," << m_dConnectTime;
	fout << "," << m_dConnectTime * 100 / CumulativePlanningTime << "%";
	fout << "," << P->m_iCollisionNum;
	fout << "," << P->m_dCumulativeCollisionTime;
	fout << "," << P->m_dCumulativeCollisionTime * 100 / CumulativePlanningTime << "%";
	fout << "," << P->m_iMetricNum;
	fout << "," << P->m_dMetricTime;
	fout << "," << P->m_dMetricTime * 100 / CumulativePlanningTime << "%";

	
	strFileName = path + this->m_strName +  ".States.csv";

	ofstream fout1( strFileName.c_str(), std::ios::out|std::ios::app );
	int i = 0;
	int nDim =  P->InitialState.dim();
	fout1.seekp( 0, std::ios::end );
	fout1 << "\n";
	fout1 << nDim;
	for( i = 0; i < nDim; ++i )
	{
		fout1 << "," << P->InitialState[i];
	}
	fout1 << "," << nDim;
	for( i = 0; i < nDim; ++i )
	{
		fout1 << "," << P->GoalState[i];
	}
	fout1.close();
	//fout << "," << P->InitialState;
	//fout << "," << P->GoalState;

	fout.close();
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTGoalBias
//
// Occasionally pick the goal state instead of a random sample. 
// *********************************************************************
// *********************************************************************


RRTGoalBias::RRTGoalBias(Problem *p):RRT(p) {

	m_strName = "RRTGoalBias";

	READ_PARAMETER_OR_DEFAULT(GoalProb,0.05);
}



MSLVector RRTGoalBias::ChooseState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	double rv;
	MSLVector Xrand;

	R >> rv;
	if (rv > GoalProb)
	{
		Xrand = RandomState();
	}
	else
	{
		Xrand = P->GoalState;
	}

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return Xrand;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTCon
// 
// Make the regular RRT greedy by attempting to connect all the way
// to the random sample, or go as far as possible before colliding.
// *********************************************************************
// *********************************************************************


// Build an RRT that tries to go as far as it can for each edge
RRTCon::RRTCon(Problem *p):RRTGoalBias(p) {

	m_strName = "RRTCon";

	READ_PARAMETER_OR_DEFAULT(GoalProb,0.0);
}




bool RRTCon::Plan()
{
	int i;
	double d;
	MSLNode *n,*nn,*n_goal;
	MSLVector nx,u_best;
	list<MSLNode*> path;
	MSLVector x(P->StateDim);

	// Keep track of time
	float t = used_time();

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


	i = 0;
	n = SelectNode(P->GoalState,T);
	n_goal = n;

	GoalDist = P->Metric(n->State(),P->GoalState);
	while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))) {

		x = ChooseState();
		this->m_Samples.push_back( x );

		//cout<< "������Ϊ��" << Xrand << "\n";
		if (Connect(x,T,nn)) { 

			d = P->Metric(nn->State(),P->GoalState);
			//cout<< "�����нڵ㵽������㣬����Ϊ��" << d << "\n";
			if (d < GoalDist) {  // Decrease if goal closer
				//cout << "�þ����ԭ�ȵľ���" << GoalDist << " ��\n";
				GoalDist = d;
				BestState = nn->State();
				n_goal = nn;

			}
		}
		i++;
	}

	CumulativePlanningTime += ((double)used_time(t));
	

	PrintInfo();

	// Get the solution path
	if (GapSatisfied(n_goal->State(),P->GoalState)) {
		// ��Զɽ2011.7.15
		m_bSuccess = true;
		//---- ��Զɽ2011.7.15

		m_dPathLength = n_goal->Cost();
		if( m_dPathLength < P->m_dMinCost )
		{
			P->m_dMinCost = m_dPathLength;
		}

		cout << "Successful Path Found\n";
		path = T->PathToRoot(n_goal);
		path.reverse();
		RecordSolution(path); // Write to Path and Policy
	}
	else {
		cout << "Failure to connect after " << 
			T->Size() << " nodes\n";
		cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

		m_bSuccess = false;	
	}
	return m_bSuccess;
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTDual
// 
// The dual-tree planner used in LaValle, Kuffner, ICRA 1999.  Each 
// tree is extended toward a randomly-sampled point.  RRTExtExt is
// generally better.
// *********************************************************************
// *********************************************************************

RRTDual::RRTDual(Problem *p):RRT(p) {

	m_strName = "RRTDual";
}


void RRTDual::RecoverSolution(MSLNode *n1, MSLNode *n2) {
	list<MSLNode*> path,path2;

	path = T->PathToRoot(n1);
	path.reverse();

	path2 = T2->PathToRoot(n2); 

	//cout << "Path: " << path;
	//cout << "Path2: " << path2;

	RecordSolution(path,path2); // Write to Path and Policy
}



bool RRTDual::Plan()
{
	int i;
	MSLVector x,u_best,nx;
	MSLNode *nn,*nn2;
	bool connected;

	connected = false;

	float t = used_time();

	//int total = 0;
	//for (int j=0; j < 100; j++) {

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
	if (!T2)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree(P->GoalState);
			T2->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( P->GoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0 );
		}
#endif
	}

	nn = T->Root();
	nn2 = T2->Root();

	i = 0;
	connected = false;
	while ((i < NumNodes) && (!connected)) {
		
		x = ChooseState();
		this->m_Samples.push_back( x );

		Extend(x,T,nn);
		Extend(x,T2,nn2,false);  // false means reverse-time integrate

		if (GapSatisfied(nn->State(),nn2->State())) {
			// ��Զɽ2011.7.15
			m_bSuccess = true;
			//---- ��Զɽ2011.7.15

			m_dPathLength = nn->Cost() + nn2->Cost();
			if( m_dPathLength < P->m_dMinCost )
			{
				P->m_dMinCost = m_dPathLength;
			}

			cout << "CONNECTED!!  MSLNodes: " << 
				T->Size() + T2->Size() << "\n";
			connected = true;
			RecoverSolution(nn,nn2);
		}

		i++;
	}

	if (!connected)
		cout << "Failure to connect after " << 
		T->Size()+T2->Size() 
		<< " nodes\n";

	//total += i;
	// }
	//cout << "Avg: " << total/100 << "\n";

	CumulativePlanningTime += ((double)used_time(t));
	
	PrintInfo();

	return connected;
}





// *********************************************************************
// *********************************************************************
// CLASS:     RRTExtExt
// 
// A dual-tree planner in which computation is balanced between 
// exploring and connecting trees to each other.  Greedier verions
// of this are RRTExtCon and RRTConCon.
// *********************************************************************
// *********************************************************************

RRTExtExt::RRTExtExt(Problem *p):RRTDual(p) {
	m_strName = "RRTExtExt";
}


bool RRTExtExt::Plan()
{
	int i;
	MSLVector u_best,nx,nx2, x;
	MSLNode *nn,*nn2;
	bool connected;

	connected = false;

	float t = used_time();

	//int total = 0;
	//for (int j=0; j < 100; j++) {

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
	if (!T2)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree(P->GoalState);
			T2->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( P->GoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0 );
		}
#endif
	}

	nn = T->Root();
	nn2 = T2->Root();

	i = 0;
	connected = false;
	while ((i < NumNodes) && (!connected)) {

		x = ChooseState();
		this->m_Samples.push_back( x );

		if (Extend(x,T,nn)) {
			if (Extend(nn->State(),T2,nn2,false)) {
				i++;
				//cout << "nn: " << nn->State() << endl;
				//cout <<" nn2: " << nn2->State() << "\n";
				if (GapSatisfied(nn->State(),nn2->State())) {
					
					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}

					cout << "CONNECTED!!  MSLNodes: " << 
						T->Size() + T2->Size() << "\n";
					connected = true;
					RecoverSolution(nn,nn2);
				}
			}
		}
		i++;

		x = ChooseState();
		this->m_Samples2.push_back( x );

		if ((!connected) && (Extend(x,T2,nn,false))) {
			if (Extend(nn->State(),T,nn2)) {
				i++;
				//cout << "nn: " << nn->State() << endl;
				//cout <<" nn2: " << nn2->State() << "\n";
				if (GapSatisfied(nn2->State(),nn->State())) {

					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}

					cout << "CONNECTED!!  MSLNodes: " << 
						T->Size() + T2->Size();
					connected = true;
					RecoverSolution(nn2,nn);
				}
			}
		}
	}

	i++;

	if (!connected)
		cout << "Failure to connect after " << 
		T->Size() + T2->Size()
		<< " nodes\n";

	//total += i;
	// }
	//cout << "Avg: " << total/100 << "\n";

	CumulativePlanningTime += ((double)used_time(t));
	
	PrintInfo();

	return connected;
}




// *********************************************************************
// *********************************************************************
// CLASS:     RRTGoalZoom
// 
// Bias sampling towards a region that contains the goal.  The 
// region shrinks around the goal as the RRT comes nearer.  This
// planner is based on a class project at Iowa State by Jun Qu in 1999.
// *********************************************************************
// *********************************************************************

RRTGoalZoom::RRTGoalZoom(Problem *p):RRT(p) {

	m_strName = "RRTGoalZoom";

	GoalProb = 0.05;
	ZoomProb = 0.5;
	ZoomFactor = 2.0;
}


// Zoom using a square box
MSLVector RRTGoalZoom::ChooseState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	double rv,r,diff;
	MSLVector zx;
	int i;

	R >> rv;
	diff = 0.0;
	zx = P->LowerState;  // Initialize to anything

	if (rv < GoalProb)
	{
		zx = P->GoalState;
	}
	else 
	{
		if (rv < (ZoomProb+GoalProb)) {
			for (i = 0; i < P->GoalState.dim(); i++) {
				if (fabs(P->GoalState[i] - BestState[i]) > diff)
					diff = fabs(P->GoalState[i] - BestState[i]);
			}
			for (i = 0; i < P->GoalState.dim(); i++) {
				R >> r;
				zx[i] += P->GoalState[i] - diff + 2.0*r*ZoomFactor*diff;
			}
			
		}
		else
		{
			zx = RandomState();
		}
	}
	
	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return zx;
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTPolar
// 
// Instead of random sampling, attempt to gradually bias samples
// toward the goal.
// *********************************************************************
// *********************************************************************

RRTPolar::RRTPolar(Problem *p):RRT(p) {
	m_strName = "RRTPolar";

	// RadiusExp = 1.0/(P->InitialState.dim() - 1);
	RadiusExp = 1.0;
}






// This implementation ignores C-space topology, VERY BAD!
MSLVector RRTPolar::ChooseState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	double r,w;
	MSLVector zx;
	int i,j;
	bool success;

	w = 0.0;
	zx = P->GoalState;  // Initialize to anything

	success = false;

	while (!success) {
		for (i = 0; i < P->GoalState.dim(); i++) {
			// Generate sample from N(0,1)
			zx[i] = 0.0;
			for (j = 0; j < 12; j++) {
				R >> r; zx[i] += r;
			}
			zx[i] -= 6.0;
			w += zx[i]*zx[i];
		}

		w = sqrt(w);

		//cout << "RadiusExp: " << RadiusExp;

		R >> r;  // Radius
		r = pow(r,RadiusExp);
		for (i = 0; i < P->GoalState.dim(); i++) {
			zx[i] = (P->UpperState[i] - P->LowerState[i])*
				sqrt((float)P->GoalState.dim())*r*zx[i]/w + 
				P->GoalState[i];
		}

		// Check if sample is within bounds
		success = true;
		for (i = 0; i < P->GoalState.dim(); i++) {
			if ((zx[i] >= P->UpperState[i])||(zx[i] <= P->LowerState[i]))
				success = false;
		}
	}

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return zx;
}



MSLVector RRTPolar::SelectInput(const MSLVector &x1, const MSLVector &x2, 
	MSLVector &nx_best, bool &success)
{
	MSLVector u_best,nx;
	list<MSLVector>::iterator u;
	double d,dg,dmax,d_min;
	success = false;
	d_min = INFINITY;
	dg  = P->Metric(x1,x2);
	dmax  = P->Metric(P->LowerState,P->UpperState);
	list<MSLVector> il = P->GetInputs(x1);
	forall(u,il) {
		//nx = P->Integrate(x1,u,PlannerDeltaT*sqrt(dg/dmax));  // Slow down near goal
		nx = P->Integrate(x1,*u,PlannerDeltaT);
		d  = P->Metric(nx,x2);
		if ((d < d_min)&&(P->Satisfied(nx))&&(x1 != nx)) 
		{
			d_min = d; 
			u_best = *u; 
			nx_best = nx; 
			success = true;
			//cout << "u_best: " << u_best << "\n");
		}
	}

	

	return u_best;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTHull
// 
// A simple exploration of what happens to the RRT in a large disc.
// Only works for 2DPoint model!!!!!
// *********************************************************************
// *********************************************************************

RRTHull::RRTHull(Problem *p):RRT(p) {
	m_strName = "RRTHull";
	Radius = 100000000.0;
}



MSLVector RRTHull::ChooseState() {
	
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	double theta;
	MSLVector v(2);

	R >> theta;  theta *= 2.0*PI;

	v[0] = Radius*cos(theta);
	v[1] = Radius*sin(theta);

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return v;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTExtCon
// 
// The planner presented in Kuffner, LaValle, 2000.  There are two
// trees, and the attempt to connect them is greedy.
// *********************************************************************
// *********************************************************************

RRTExtCon::RRTExtCon(Problem *p):RRTDual(p) {

	m_strName = "RRTExtCon";
}



bool RRTExtCon::Plan()
{
	int i;
	MSLVector nx,nx_prev, x;
	MSLNode *nn,*nn2;
	bool connected;

	connected = false;

	float t = used_time();

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
	if (!T2)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree(P->GoalState);
			T2->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( P->GoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0 );
		}
#endif
	}

	nn = T->Root();
	nn2 = T2->Root();

	i = 0;
	connected = false;
	while ((i < NumNodes) && (!connected)) {

		x = ChooseState();
		this->m_Samples.push_back( x );

		if (Extend(x,T,nn)) { 
			// Update the goal RRT
			if (Connect(nn->State(),T2,nn2,false)) {
				if (GapSatisfied(nn->State(),nn2->State())) {
					
					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}

					cout << "CONNECTED!!  MSLNodes: " << 
						T->Size() + T2->Size()
						<< "\n";
					connected = true;
					RecoverSolution(nn,nn2);
				}
			}
		}
		i++;

		x = ChooseState();
		this->m_Samples2.push_back( x );

		if ((!connected)&&(Extend(x,T2,nn,false))) { 
			// Update the initial RRT
			if (Connect(nn->State(),T,nn2)) { 
				if (GapSatisfied(nn->State(),nn2->State())) {
					
					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}

					cout << "CONNECTED!!  MSLNodes: " << 
						T->Size() + T2->Size() << "\n";
					connected = true;
					RecoverSolution(nn2,nn);
				}
			}
		}
		i++;
	}

	if (!connected)
		cout << "Failure to connect after " << T->Size() + T2->Size()
		<< " nodes\n";

	CumulativePlanningTime += ((double)used_time(t));
	

	PrintInfo();

	return connected;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTConCon
// 
// One the best dual-tree planners.  Be greedy in growing the trees
// and in trying to connect them to each other.  This is usually the
// fastest planner, but there is often a price for being greedy...
// *********************************************************************
// *********************************************************************

RRTConCon::RRTConCon(Problem *p):RRTDual(p) {

	m_strName = "RRTConCon";

	//ConnectTimeLimit = 5;
}



bool RRTConCon::Plan()
{
	int i;
	MSLVector nx,nx_prev;
	MSLNode *nn,*nn2;
	bool connected;

	connected = false;

#ifdef ANYTIME
	int nClock;
	double dInteralTime;

	nClock = 0;
	dInteralTime = 0.2;
#endif

	// Keep track of time
	float t = used_time();

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
	if (!T2)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree(P->GoalState);
			T2->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( P->GoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0 );
		}
#endif
	}

	nn = T->Root();
	nn2 = T2->Root();

	i = 0;
	connected = false;
	MSLVector x;

#ifdef ANYTIME
	while( 1 )
#else
	while ( (i < NumNodes) && (!connected)) 
#endif
	{
		x = ChooseState();
		this->m_Samples.push_back( x );

		if (Connect(x,T,nn)) { 
			// Update the goal RRT
			//cout << "nn: " << nn->State() << "  nn2: " << nn2->State() << "\n";
			if (Connect(nn->State(),T2,nn2,false)) {
				if (GapSatisfied(nn->State(),nn2->State())) {
					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					if( nn->Cost() + nn2->Cost() < m_dPathLength )
					{
					
					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}
					

					//cout << "CONNECTED!!  MSLNodes: " << 
					//	T->Size()+T2->Size() << "\n";
					connected = true;
					RecoverSolution(nn,nn2); // Defined in RRTDual
					}
				}
			}
		}
		i++;

#ifdef ANYTIME
		double dMaxTime = 60 * 2;	// ����������
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

		x = ChooseState();
		this->m_Samples2.push_back( x );

		if ((!connected)&&(Connect(x,T2,nn,false))) { 
			// Update the initial RRT
			if (Connect(nn->State(),T,nn2)) { 
				if (GapSatisfied(nn->State(),nn2->State())) {
					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					if( nn->Cost() + nn2->Cost() < m_dPathLength )
					{

					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}

					//cout << "CONNECTED!!  MSLNodes: " << 
					//	T->Size()+T2->Size() << "\n";
					connected = true;
					RecoverSolution(nn2,nn);
					}
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

	if (!connected)
		cout << "Failure to connect after " << 
		T->Size()+T2->Size() << " nodes\n";

	CumulativePlanningTime += ((double)used_time(t));
	

	PrintInfo();

	//m_pkBookKeeper->WriteCollectionInfo( "F:\\OpenSource\\Motion Strategy Library\\msl-2.0-win\\data\\car7\\Info\\" );

	return connected;
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTBidirBalanced
// 
// This planner behaves similar to RRTConCon, except that a
// cardinality criteria is introduced to maintain relative balance
// between the number of nodes in each tree.  This planner was the
// first to be able to consistently solve the original well-known
// alpha puzzle motion planning benchmark.
// *********************************************************************
// *********************************************************************

RRTBidirBalanced::RRTBidirBalanced(Problem *p):RRTDual(p) {
	m_strName = "RRTBidirBalanced";
}

bool RRTBidirBalanced::Plan()
{
	// Code for time trials
	float t = used_time();

	// init trees if needed
	MSLNode *nn,*nn2;
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
	if (!T2)
	{
#ifdef USE_ANN
		if( !UseANN )
		{
#endif

			T2 = new MSLTree(P->GoalState);
			T2->Root()->SetCost( 0 );

#ifdef USE_ANN
		}
		else
		{
			T2 = new MSLTree( P->GoalState, this->m_pTopology, this->m_pScaling );
			T2->Root()->SetCost( 0 );
		}
#endif
	}

	// initialize planner
	bool connected = false;
	nn = T->Root();
	nn2 = T2->Root();
	int i = 0;

	// set current active tree and target node
	bool bInitActive = true;

	MSLTree *pActiveTree = T;  
	MSLTree *pOtherTree  = T2;  
	MSLVector target = P->GoalState; 

	while ((i < NumNodes) && (!connected))
	{
		if (Connect(target, pActiveTree, nn, bInitActive)) { 
			if (Connect(nn->State(), pOtherTree, nn2, !bInitActive)) {
				if (GapSatisfied(nn->State(), nn2->State())) {
					// ��Զɽ2011.7.15
					m_bSuccess = true;
					//---- ��Զɽ2011.7.15

					m_dPathLength = nn->Cost() + nn2->Cost();
					if( m_dPathLength < P->m_dMinCost )
					{
						P->m_dMinCost = m_dPathLength;
					}

					cout << "CONNECTED!!  MSLNodes: " << 
						pActiveTree->Size() + pOtherTree->Size() << "\n";
					connected = true;
					if (bInitActive)
						RecoverSolution(nn, nn2); // Defined in RRTDual
					else
						RecoverSolution(nn2, nn);
				}
			}
		}

		// select the active tree and new target
		if (!connected) {
			bInitActive = (T->Size() <= T2->Size());
			if (bInitActive)
			{ pActiveTree = T;  pOtherTree = T2; }
			else
			{ pActiveTree = T2; pOtherTree = T;  }
			target = ChooseState();
		}
		i++;
	}

	if (!connected)
		cout << "Failure to connect after " << 
		T->Size() + T2->Size() << " nodes\n";


	// Code for time trials
	//total += i;
	//  }
	//cout << "Avg: " << total/100.0 << "\n";
	//cout << "Avg time: " << ((double)used_time(t)/100.0) << "\n"; 
	CumulativePlanningTime += ((double)used_time(t));


	PrintInfo();

	return connected;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RandomTree
// 
// Technically not an RRT.  Pick a vertex and input at random.
// *********************************************************************
// *********************************************************************

RandomTree::RandomTree(Problem *p):RRT(p) {
}


MSLNode* RandomTree::SelectNode(const MSLVector &x, MSLTree *t,
	bool forward = true) {
		cout << "WARNING: RESULT NOT CORRECT!!!\n";
		cout << "NEED TO FIX RandomTree::SelectNode\n";

		//return g.choose_node();
		return t->Root();
}


MSLVector RandomTree::SelectInput(const MSLVector &x1, const MSLVector &x2, 
	MSLVector &nx_best, bool &success,
	bool forward = true) {
		double r;
		list<MSLVector> il;
		int k;
		MSLVector u_best;

		il = P->GetInputs(x1);
		R >> r;

		k = (int) (r * il.size());

		cout << "WARNING: RESULT NOT CORRECT!!!\n";
		cout << "NEED TO FIX RandomTree::SelectInput\n";
		//u_best = il.inf(il[k]);
		u_best = il.front();

		nx_best = P->Integrate(x1,u_best,PlannerDeltaT);
		SatisfiedCount++;

		return u_best;

}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTAStar
// 
// ��Զɽ��ӣ��ڲ����㴦ѡ���ھӼ��ϣ���ͨ���µĽڵ��޸��ھӽڵ������
// *********************************************************************
// *********************************************************************
/*
RRTAStar::RRTAStar(Problem *p): RRTDual(p)
{
	m_strName = "RRTAStar";

	GoalProb = 0.2;
	ZoomProb = 0.6;
	ZoomFactor = 3.0;
	m_nRewireNum = 0;

	int i;
	MSLVector min, max;
	min = max = m_ZoomSize = MSLVector(P->StateDim);
	for (i = 0; i < P->GoalState.dim(); i++) 
	{
		if (P->InitialState[i] < P->GoalState[i])
		{
			min[i] = P->InitialState[i];
			max[i] = P->GoalState[i];
		}
		else
		{
			min[i] = P->GoalState[i];
			max[i] = P->InitialState[i];
		}
		m_ZoomSize[i] = fabs( max[i] - min[i] );
	}
	m_ZoomCenter = 0.5 * ( max + min );
}

MSLVector RRTAStar::ChooseState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	float time;
	time = used_time();

	double rv,r;
	MSLVector zx;
	int i;

	//R >> rv;
	//zx = P->LowerState;  // Initialize to anything

	//if (rv < GoalProb)
	//{
	//	zx = P->GoalState;
	//}
	//else 
	//{
	//	if (rv < (ZoomProb+GoalProb)) 
	//	{
	//		for (i = 0; i < P->GoalState.dim(); i++) 
	//		{
	//			R >> r;
	//			zx[i] = m_ZoomCenter[i] + ( -0.5 * m_ZoomSize[i] + r * m_ZoomSize[i] ) * ZoomFactor;
	//		}
	//	}
	//	else
	//	{
	//		zx = RandomState();
	//	}
	//}

	int nMaxCount = 20;
	i = 0;
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
	// ��Զɽ��2011.6.6����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return zx;
}


MSLVector RRTAStar::ChooseState2()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	float time;
	time = used_time();

	double rv,r;
	MSLVector zx;
	int i;

	//R >> rv;
	//zx = P->LowerState;  // Initialize to anything

	//if (rv < GoalProb)
	//{
	//	zx = P->InitialState;
	//}
	//else 
	//{
	//	if (rv < (ZoomProb+GoalProb)) 
	//	{
	//		for (i = 0; i < P->GoalState.dim(); i++) 
	//		{
	//			R >> r;
	//			zx[i] = m_ZoomCenter[i] 
	//				+ ( -0.5 * m_ZoomSize[i] + r * m_ZoomSize[i] ) * ZoomFactor;
	//		}
	//	}
	//	else
	//	{
	//		zx = RandomState();
	//	}
	//}

	int nMaxCount = 20;
	i = 0;
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
		zx = P->InitialState;
	}

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));
	return zx;
}

MSLVector RRTAStar::SelectInput(const MSLVector &x1, 
		const MSLVector &x2, 
		MSLVector &nx_best, 
		bool &success, 
		bool forward)
{
	m_iSelectInputNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	float time, time1, dColTime;
	dColTime = 0;
	time = used_time();

	MSLVector u_best,nx;
	list<MSLVector>::iterator u;
	double d,d_min;
	success = false;
	d_min = (forward) ? P->Metric(x1,x2) : P->Metric(x2,x1);
	list<MSLVector> il = P->GetInputs(x1);

	if (Holonomic) { // Just do interpolation
		u_best = P->InterpolateState(x1,x2,0.1) - x1;
		u_best = u_best.norm(); // Normalize the direction
		nx_best = P->Integrate(x1,u_best,PlannerDeltaT);
		SatisfiedCount++;
		// ��Զɽ��2011.6.6����ʱ��ͳ��
		time1 = used_time();

		if (P->Satisfied(nx_best))
			success = true;

		// ��Զɽ��2011.6.6����ʱ��ͳ��
		dColTime += used_time( time1 );
	}
	else {  // Nonholonomic (the more general case -- look at Inputs)

		double d_tmpMin = INFINITY;

		forall(u,il) {
			if (forward)
				nx = P->Integrate(x1,*u,PlannerDeltaT);
			else
				nx = P->Integrate(x1,*u,-PlannerDeltaT);

			d  = (forward) ? P->Metric(nx,x2): P->Metric(x2,nx);

			SatisfiedCount++;
			if ((d < d_min)&&(x1 != nx)) { 

				// ��Զɽ��2011.6.6����ʱ��ͳ��
				time1 = used_time();

				if (P->Satisfied(nx)) {
					d_min = d; 
					u_best = *u; 
					nx_best = nx; 
					success = true;
				}

				// ��Զɽ��2011.6.6����ʱ��ͳ��
				dColTime += used_time( time1 );
			}
		}
	}

	//cout << "u_best: " << u_best << "\n";
	//cout << "nx_best: " << nx_best << "\n";
	//cout << "success: " << success << "\n";
	//success = true;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	this->m_dSelectInputTime += ((double)used_time(time)) - dColTime;
	return u_best;
}


MSLNode* RRTAStar::SelectNode(const MSLVector &x, MSLTree *t,
	bool forward)
{
	m_iSelectNodeNum++;

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	float time = used_time();

	double d,d_min;
	MSLNode *n_best;
	list<MSLNode*>::iterator n;

	d_min = INFINITY; d = 0.0;

#ifdef USE_ANN
	if (!UseANN) {
#endif
		list<MSLNode*> nl;
		nl = t->Nodes();
		forall(n,nl) {
			d = (forward) ? P->Metric((*n)->State(),x) : P->Metric(x,(*n)->State());
			if (d < d_min) {
				d_min = d; n_best = (*n); 
			}
		} 

#ifdef USE_ANN
	}
	else {
		double dBestDist;
		n_best = t->NearestNode( x, dBestDist );
	}
#endif

	//cout << "n_best: " << (*n_best) << "\n";

	// ��Զɽ��2011.6.6����ʱ��ͳ��
	m_dSelectNodeTime += ((double)used_time(time));

	return n_best;
}

#include <algorithm>
int Compare( pair<MSLNode*,double> a, pair<MSLNode*,double> b) 
{
  return (a.second < b.second);
}

// ѡ����õĸ��ڵ�
MSLNode* RRTAStar::SelectBestParent( const MSLVector &x, std::vector< MSLNode* > &vNodes )
{
	MSLNode* pBestParent = NULL;
	int i;
	int iNum = vNodes.size();
	vector<std::pair< MSLNode*, double > > vNodeCostPairs( iNum );
	
	double dCost = INFINITY;
	// 1.���㾭��vNodes�ڵ㵽x�Ĵ��ۣ������ݴ˴��۶�vNodes��С�������򣬲���STL�е�algorithm��
	// 1.1���㵽x�Ĵ��ۣ�������Pairs
	for( i = 0; i < iNum; ++i )
	{
		dCost = vNodes[i]->Cost() + P->Metric( vNodes[i]->State(), x );
		vNodeCostPairs[i].first = vNodes[i];
		vNodeCostPairs[i].second = dCost;
	}
	// 1.2����
	::sort( vNodeCostPairs.begin(), vNodeCostPairs.end(), Compare );
	
	//// 2.����ڽ��ڵ㵽�����x�Ƿ�ɴ�
	//for( i = 0; i < iNum; ++i )
	//{
	//	if( P->CollisionFreeLine( vNodeCostPairs[i].first->State(), x ) )
	//	{
	//		pBestParent = vNodeCostPairs[i].first;
	//		break;
	//	}
	//}
	pBestParent = vNodeCostPairs[0].first;

	return pBestParent;
}



// �ڽ��ڵ�ĵ�
void RRTAStar::RewireNodes( MSLTree *t, MSLNode* pNewNode, std::vector< MSLNode* > &vNodes, bool bForward )
{
	int iNum = vNodes.size();
	int i;
	double dCost, dTrajCost, dTime;
	MSLVector kInput, xnew;
	MSLNode *pNode;
	for( i = 0; i < iNum; ++i )
	{
		// ����Ƿ�ɸĵ�
		//xnew = Steer( pNewNode->State(), vNodes[i]->State(), kInput, dTime, dTrajCost, bForward );
		std::vector< MSLVector > vStates;
		std::vector< MSLVector > vInputs;
		std::vector< double > vCosts;
		P->GetModel()->Steer( pNewNode->State(), vNodes[i]->State(), vStates, vInputs, vCosts, bForward );
		xnew = vStates.back();
		dTrajCost = vCosts.back();
		if( xnew != vNodes[i]->State() )		// �ĵ�ʧ��
		{
			continue;
		}
		// ��ȡ��������֮ǰ��״̬
		ExtractCollisionFreeStates( vStates );
		//xnew = SteerConnect( pNewNode->State(), vNodes[i]->State(), vStates, vInputs, dTrajCost, bForward );
		int iSize = vStates.size();
		if( iSize <= 0 )
		{
			continue;
			
		}
		xnew = vStates[iSize-1];
		if( xnew != vNodes[i]->State() )		// �ĵ�ʧ��
		{
			continue;
		}

		// �������
		dCost = pNewNode->Cost() + dTrajCost;
		if( dCost >= 0.95 * vNodes[i]->Cost() )
		{
			continue;
		}

		
		// ������Ϊֻ�иĵ��ߵĴ���С��ԭ����95%��ֵ�ã���Ϊ�ĵ���������ڵ�Ĵ���Ҳ��Ҫ���£�
		// �ķ�һ����ʱ��
		MSLNode *pParentNode = NULL;
		if( dCost < 0.95 * vNodes[i]->Cost() )	
		{
			m_nRewireNum++;
			// ����·�μ�������
			double dNewNodeCost;
			int iTrajLen = vStates.size();
			pParentNode = pNewNode;
			for( int j = 0; j < iTrajLen - 1; ++j )
			{
				xnew = vStates[j];
				kInput = vInputs[j];
				dNewNodeCost = pParentNode->Cost() + P->Metric( pParentNode->State(), xnew );
				if( dNewNodeCost < P->m_dMinCost )
				{	
					pNode = t->Extend( pParentNode, xnew, kInput, PlannerDeltaT );
					pNode->SetCost( dNewNodeCost );
					pParentNode = pNode;
				}
			}
			// ���һ���ڵ㲻���������ɣ�ֻ����¼���
			kInput = vInputs[iTrajLen - 1];
			// ��ǰ�θ��ڵ������ϵ
			vNodes[i]->Parent()->DetachChild( vNodes[i] );
			// ���µĸ��ڵ㽨�����ӹ�ϵ
			pParentNode->AddChild( vNodes[i] );
			vNodes[i]->SetParent( pParentNode );
			// ������Ϣ
			vNodes[i]->SetInputandTime( kInput, PlannerDeltaT );
			vNodes[i]->SetCost( dCost );
			double dDiff = vNodes[i]->Cost() - dCost;

			// ����������ڵ�Ĵ���
			UpdateSubtreeCost( vNodes[i], dDiff );
		}
	}
}

void RRTAStar::UpdateSubtreeCost( MSLNode* pNode, double dDiff )
{
	list<MSLNode*> children = pNode->Children();
	list<MSLNode*>::iterator it;
	double dCost;
	for( it = children.begin(); it != children.end(); it++ )
	{
		dCost = (*it)->Cost() - dDiff;
		(*it)->SetCost( dCost );

		// �ݹ����������
		UpdateSubtreeCost( *it, dDiff );
	}
}


// ��x1��x2�������������һ����״̬�����õ����뼰����ʱ��
MSLVector RRTAStar::Steer( const MSLVector &x1, const MSLVector &x2, 
						  MSLVector &u, double &time, double &dTrajCost, bool bForward )
{
	time = dTrajCost = 0;
	double dTmpTime = 0;
	double dTmpTrajCost = 0;

	MSLVector nx, prex, newx;
	list<MSLVector>::iterator it;
	double d,d_min;
	double dDisttox2, dPreDisttox2;
	d_min = dDisttox2 = (bForward) ? P->Metric(x1,x2) : P->Metric(x2,x1);
	list<MSLVector> il = P->GetInputs(x1);

	newx = x1;
	if (Holonomic) 
	{ // Just do interpolation
		u = P->InterpolateState(x1,x2,0.1) - x1;
		prex = x1;
		double dDetaT = (bForward) ? PlannerDeltaT : - PlannerDeltaT;
		nx = P->Integrate(prex,u,dDetaT);
		while( P->Satisfied(nx) )
		{
			dTrajCost += P->Metric( prex, nx );
			time += dDetaT;
			if( GapSatisfied( nx, x2 ) )
			{
				return x2;
			}
			else
			{
				prex = nx;
				nx = P->Integrate(prex,u,dDetaT);
			}
		}
		
		// δ����x2
		return prex;
	}
	else 
	{  // Nonholonomic (the more general case -- look at Inputs)
		forall(it,il) 
		{
			dTmpTime = 0;
			dTmpTrajCost = 0;
			prex = x1;
			if (bForward)
			{
				nx = P->Integrate(prex,*it,PlannerDeltaT);
				dPreDisttox2 = dDisttox2;
				dDisttox2 = P->Metric( nx, x2 );
				while( P->Satisfied(nx) && dDisttox2 <= dPreDisttox2 )
				{
					dTmpTrajCost += P->Metric( prex, nx );
					dTmpTime += PlannerDeltaT;
					if( GapSatisfied( nx, x2 ) )
					{	
						u = *it;
						time = dTmpTime;
						dTrajCost = dTmpTrajCost;
						return x2;
					}
					else
					{
						prex = nx;
						nx = P->Integrate(prex,*it,PlannerDeltaT);
						dPreDisttox2 = dDisttox2;
						dDisttox2 = P->Metric( nx, x2 );
					}
				}
			}
			else
			{
				nx = P->Integrate(prex,*it,PlannerDeltaT);
				dPreDisttox2 = dDisttox2;
				dDisttox2 = P->Metric( nx, x2 );
				while( P->Satisfied(nx) && dDisttox2 <= dPreDisttox2 )
				{
					dTmpTrajCost += P->Metric( prex, nx );
					dTmpTime -= PlannerDeltaT;
					if( GapSatisfied( nx, x2 ) )
					{	
						u = *it;
						time = dTmpTime;
						dTrajCost = dTmpTrajCost;
						return x2;
					}
					else
					{
						prex = nx;
						nx = P->Integrate(prex,*it,-PlannerDeltaT);
						dPreDisttox2 = dDisttox2;
						dDisttox2 = P->Metric( nx, x2 );
					}
				}	
			}

			// δ����x2(�����ϰ�)
			d  = (bForward) ? P->Metric(prex,x2): P->Metric(x2,prex);
			if ((d < d_min)) 
			{ 
				d_min = d; 
				newx = prex;
				u = *it;
				time = dTmpTime;
				dTrajCost = dTmpTrajCost;
			}
		}

		return newx;
	}
}



MSLVector RRTAStar::SteerConnect( const MSLVector &x1, const MSLVector &x2, 
				std::vector< MSLVector > &vStates, std::vector< MSLVector > &vInputs,
				double &dTrajCost, bool bForward )
{
	MSLVector nx, nx_prev;
	double d,d_prev;
	bool success;

	// ��ʼ������ֵ
	MSLVector u = SelectInput(x1,x2,nx,success,bForward);
	dTrajCost = 0;
	nx_prev = x1;

	if (success) 
	{   // If a collision-free input was found
		d_prev = P->Metric(x1,x2);
		d = P->Metric(nx,x2); 

		while ((P->Satisfied(nx))&&	(d <= d_prev))
		{	
			dTrajCost += P->Metric( nx_prev, nx );

			// ��״̬���������������
			vStates.push_back( nx );
			vInputs.push_back( u );

			if( GapSatisfied( nx, x2 ) )
			{
				vStates[vStates.size()-1] = x2;
				return x2;
			}

			nx_prev = nx;
			d_prev = d;

			if (Holonomic) { 
				nx = P->Integrate(nx_prev,u,PlannerDeltaT);
			}
			else { // Nonholonomic
				if (bForward)
					nx = P->Integrate(nx_prev,u,PlannerDeltaT);
				else
					nx = P->Integrate(nx_prev,u,-PlannerDeltaT);
			}
			d = P->Metric(nx,x2);
		}
	}

	// �����ϰ���δ����x2
	return nx_prev;
}

void RRTAStar::ExtractCollisionFreeStates( std::vector< MSLVector > &vStates )
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
bool RRTAStar::Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	bool forward)
{
	m_iExtendNum++;

	std::vector< MSLNode* > vNodes;
	MSLNode* pParentNode = NULL;
	double dNewNodeCost = 0;
	MSLVector kInput;
	MSLVector xnew;
	double dTime = 0;
	double dTrajCost;

	// 1.�����ڽ��㼯��
	vNodes = SelectNearNodes( x, t, forward );

	// 2.ѡ����õĸ��ڵ�
	if( vNodes.size() <= 1 )
	{
		pParentNode = SelectNode(x,t,forward);
		//if( !P->CollisionFreeLine( pParentNode->State(), x ))
		//{
		//	m_dExtendTime += ((double)used_time(time));
		//	return false;
		//}
	}
	else
	{
		pParentNode = SelectBestParent( x, vNodes );
		if( !pParentNode )
		{
			return false;
		}
	}

	// 3.���ߺͽڵ���ӵ�����
	//xnew = Steer( pParentNode->State(), x, kInput, dTime, dTrajCost, forward );
	std::vector< MSLVector > vStates;
	std::vector< MSLVector > vInputs;
	std::vector< double > vCosts;
	P->GetModel()->Steer( pParentNode->State(), x, vStates, vInputs, vCosts, forward );
	xnew = vStates.back();
	dTrajCost = vCosts.back();
	//xnew = SteerConnect( pParentNode->State(), x, vStates, vInputs, dTrajCost, forward );
	if( xnew == pParentNode->State() )		// δ����չ��ȥ
	{
		return false;
	}

	// ��ȡ��������֮ǰ��״̬
	ExtractCollisionFreeStates( vStates );

	// ��չ������
	int iTrajLen = vStates.size();
	for( int i = 0; i < iTrajLen; ++i )
	{
		xnew = vStates[i];
		kInput = vInputs[i];
		dNewNodeCost = pParentNode->Cost() + P->Metric( pParentNode->State(), xnew );
		if( dNewNodeCost < P->m_dMinCost )
		{	
			nn = t->Extend( pParentNode, xnew, kInput, PlannerDeltaT );
			nn->SetCost( dNewNodeCost );
			pParentNode = nn;
		}
	}

	//// 4.�ڽ��ڵ�ĵ�
	//if( vNodes.size() > 0 )
	//{
	//	bool bForward = ( t == T ) ? true : false;
	//	RewireNodes( t, nn, vNodes, bForward );
	//}

	return true;
}


//! Iterated Extend
bool RRTAStar::Connect(const MSLVector &x, MSLTree *t, MSLNode *&nn,
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

	n_best = SelectNode(x,t,forward);
	u_best = SelectInput(n_best->State(),x,nx,success,forward);			// ��ѡ�����Ҫ����ѡ�Է���ѡ���ÿ��ܻ���ԯ����
	steps = 0;
	// nx gets next state
	if (success) {   // If a collision-free input was found
		dLineCost = P->Metric( n_best->State(), nx );
		d = P->Metric(nx,x); 
		d_prev = d;
		nx_prev = nx; // Initialize
		nn = n_best;
		clock = PlannerDeltaT;

		// �������㷽����ȷ�����������ٶ���ǰ��������Ϊ��ȷ��
		// 1������ʱ�������ΪĿ����T2�Ľڵ㣻2������ʱ�������ΪT1�еĽڵ�
		int nStepNode = 5;
		if( x == P->GoalState )
		{
			ConnectTimeLimit = INFINITY;
			nStepNode = 10;
		}

		dNewCost = n_best->Cost();
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
			//u_best = SelectInput(g.inf(nn),x,nx,success,forward); 
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
			d = P->Metric(nx,x);
			clock += PlannerDeltaT;

			if( 0 == (int)(clock / PlannerDeltaT) % nStepNode )
			{
				nn = t->Extend(nn, nx, u_best, steps*PlannerDeltaT);
				nn->SetCost( dNewCost );
				steps = 0;
			}
			// Uncomment the subsequent two lines to
			//   make each intermediate node added
			//nn = g.new_node(nx_prev); // Make a new node
			//g.new_edge(nn_prev,nn,u_best);
		}

		// ��Զɽ��2011.7.13����ʱ��ͳ��
		float time = used_time();

		nn = t->Extend(nn_prev, nx_prev, u_best, steps*PlannerDeltaT);

		// ��Զɽ��2011.6.7
		nn->SetCost( dNewCost );
		
		m_dTreeOperTime += ((double)used_time(time));
	}

	// ��Զɽ��2011.07.13����ʱ��ͳ��
	m_dConnectTime += ((double)used_time(Contime));
	return success;
}


bool RRTAStar::Plan()
{

	// ��ʱʵ��RRTCon��Plan
	int i;
	MSLNode *nn;
	MSLVector nx,u_best;
	list<MSLNode*> path;
	MSLVector Xrand(P->StateDim);

	// Keep track of time
	float t = used_time();

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

	// ��Զɽ��2011.6.5
	// �ȸ�������·������С����������T�д��۽ϸߵ�����
	T->ClearByCost( T->Root(), P->m_dMinCost );
	T2->ClearByCost( T2->Root(), P->m_dMinCost );
	// ----��Զɽ��2011.6.5

	nn = T->Root();
	MSLNode *nn2 = T2->Root();

	bool bConnected = false;
	MSLVector x;
	i = 0;
	while( (i < NumNodes) && (!bConnected) )
	{
		x = ChooseState();
		this->m_Samples.push_back( x );
		
		if (Extend(x,T,nn)) { 
			// Update the goal RRT
			//cout << "nn: " << nn->State() << "  nn2: " << nn2->State() << "\n";
			if (Extend(nn->State(),T2,nn2,false)) {
				if (GapSatisfied(nn->State(),nn2->State())) {
					// ��Զɽ��2011.6.5�޸ģ���������·������С���۸���
					if( m_dPathLength >= nn->Cost() + nn2->Cost() )
					{
						m_dPathLength = nn->Cost() + nn2->Cost();
						if( m_dPathLength < P->m_dMinCost )
						{
							P->m_dMinCost = m_dPathLength;
						}
						// ��Զɽ2011.7.15
						m_bSuccess = true;
						//---- ��Զɽ2011.7.15

						cout << "CONNECTED!!  MSLNodes: " << 
							T->Size()+T2->Size() << "\n";
						bConnected = true;

						RecoverSolution(nn,nn2); // Defined in RRTDual
					}
					// ��Զɽ��2011.6.5
				}
			}
		}
		i++;

		x = ChooseState2();
		this->m_Samples2.push_back( x );

		if ((!bConnected)&&(Extend(x,T2,nn2,false))) { 
			// Update the initial RRT
			if (Extend(nn2->State(),T,nn)) { 
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

						cout << "CONNECTED!!  MSLNodes: " << 
							T->Size()+T2->Size() << "\n";
						bConnected = true;

						RecoverSolution(nn,nn2);
					}
					// ��Զɽ��2011.6.5
				}
			}
		}
		i++;

	}

	if (!bConnected)
		cout << "Failure to connect after " << 
		T->Size()+T2->Size() << " nodes\n";

	CumulativePlanningTime += ((double)used_time(t));
	

	PrintInfo();

	cout << "Rewire Number: " << m_nRewireNum << endl;

	return bConnected;
}
*/


// *********************************************************************
// *********************************************************************
// CLASS:     RRTAnytime
// 
// ��Զɽ���
// *********************************************************************
// *********************************************************************
RRTAnytime::RRTAnytime(Problem *p): RRT( p )
{
	m_strName = "RRTAnytime";

	m_dGoalProb = 0.1;
	m_dDistBias = 1.0;
	m_dCostBias = 0.0;
	m_dCostSol = INFINITY;
}

void RRTAnytime::ReinitializeRRT()
{
	Reset();

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


MSLVector RRTAnytime::ChooseState()
{
	m_iChooseStateNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	double dProb;
	MSLVector x_sample;
	int nAttempts;
	const int nAttemptsMax = 20;

	R >> dProb;

	if (dProb < m_dGoalProb)
	{
		x_sample = P->GoalState;
	}
	else 
	{
		x_sample = RandomState();
		nAttempts = 0;
		while( P->Metric( P->InitialState, x_sample) + P->Metric( x_sample, P->GoalState ) > m_dCostSol )
		{
			x_sample = RandomState();
			nAttempts++;
			if( nAttempts > nAttemptsMax )
			{
				x_sample = MSLVector( 0 );		// ʧ�ܣ�����ά��Ϊ0������
			}
		}
	}

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dChooseStateTime += ((double)used_time(time));

	return x_sample;
}


MSLVector RRTAnytime::SelectInput(const MSLVector &x1, 
		const MSLVector &x2, 
		MSLVector &nx_best, 
		bool &success, 
		bool forward)
{

	m_iSelectInputNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	MSLVector u_best,nx;
	list<MSLVector>::iterator u;
	double d,d_min;
	success = false;
	d_min = (forward) ? P->Metric(x1,x2) : P->Metric(x2,x1);
	list<MSLVector> il = P->GetInputs(x1);

	if (Holonomic) { // Just do interpolation
		u_best = P->InterpolateState(x1,x2,0.1) - x1;
		u_best = u_best.norm(); // Normalize the direction
		nx_best = P->Integrate(x1,u_best,PlannerDeltaT);
		SatisfiedCount++;
		if (P->Satisfied(nx_best))
			success = true;
	}
	else {  // Nonholonomic (the more general case -- look at Inputs)

		double d_tmpMin = INFINITY;

		forall(u,il) {
			if (forward)
				nx = P->Integrate(x1,*u,PlannerDeltaT);
			else
				nx = P->Integrate(x1,*u,-PlannerDeltaT);

			d  = (forward) ? P->Metric(nx, x2): P->Metric(x2, nx);

			SatisfiedCount++;
			if ((d < d_min)&&(x1 != nx)) { 
				if (P->Satisfied(nx)) {
					d_min = d; 
					u_best = *u; 
					nx_best = nx; 
					success = true;
				}
			}
		}
	}


	// ��Զɽ��2011.7.17����ʱ��ͳ��
	this->m_dSelectInputTime += ((double)used_time(time));

	return u_best;
}


// ����RRT��t�����нڵ㣬�ҳ����������xΪ���ģ�rnΪ�뾶�����ڵĽڵ�
std::vector< MSLNode* > RRTAnytime::SelectNearNodes( const MSLVector &x, MSLTree *t,
	bool forward )
{
	m_iSelectNodeNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	const int nNearMax = 50;
	const double dIndefinite = INFINITY;
	int nCount = 0;
	int i;
	double dCost2Goal,d_rn;
	std::vector< MSLNode* > vNodes;
	MSLNode* aNodes[ nNearMax ];
	double aCosts[ nNearMax ];

	list<MSLNode*>::iterator n;
	MSLVector x_near;

	int num = t->Size();
	const double gama = 50.0;
	const double d_rmax = 100.0;
	double d;

	for( i = 0; i < nNearMax; ++i )
	{
		aNodes[i] = NULL;
		aCosts[i] = dIndefinite;
	}
	
	//d_rn = min( pow( gama * log( 1.0 + (double)num ) / (double)num, 1.0 / (double)P->StateDim ), d_rmax ); 
	d_rn = 100.0;

	list<MSLNode*> nl;
	nl = t->Nodes();
	forall(n,nl) 
	{
		x_near = (*n)->State();
		d = (forward) ? P->Metric(x_near,x) : P->Metric(x,x_near);
		if (d < d_rn) 
		{
			dCost2Goal = m_dDistBias * P->Metric( x_near, x ) + m_dCostBias * P->Metric( x, P->GoalState );
			// �������򣬽�*n������������aNodes��
			for( i = nCount; i > 0; --i )
			{
				if( dCost2Goal < aCosts[i-1] )
				{
					aCosts[i] = aCosts[i-1];
					aNodes[i] = aNodes[i-1];
				}
				else
				{
					break;
				}
			}
			aCosts[i] = dCost2Goal;
			aNodes[i] = *n;
			nCount++;
			if( nCount >= nNearMax )
			{
				break;
			}
		}	
	} 

	// ��aNodes�����еĽڵ�ת�浽����vNodes�У���󷵻�
	vNodes.clear();
	for( i = 0; i < nCount; ++i )
	{
		vNodes.push_back( aNodes[i] );
	}

	return vNodes;
}


bool RRTAnytime::Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
		bool forward)
{
	m_iExtendNum++;

	// ��Զɽ��2011.7.17����ʱ��ͳ��
	float time = used_time();

	//std::vector< MSLNode* > vNodes;
	//MSLNode* pNode = NULL;
	//MSLVector x_near, x_new, u_best;
	//int nNodes = 0;
	//double dCost2Goal, dCost, dCost2New;
	//bool bSuccess = false;

	//vNodes = SelectNearNodes( x, t, true );		// vNodes�Ѵ�С��������
	//nNodes = vNodes.size();
	//for( int i = 0; i < nNodes; ++i )
	//{
	//	pNode = vNodes[i];
	//	x_near = pNode->State();
	//	
	//	u_best = this->SelectInput( x_near, x, x_new, bSuccess, true );
	//	if( bSuccess )
	//	{
	//		dCost2New = pNode->Cost() + P->Metric( x_near, x_new );
	//		dCost =  dCost2New + P->Metric( x_new, P->GoalState );
	//		if( dCost <= m_dCostSol )
	//		{
	//			nn = t->Extend( pNode, x_new, u_best, PlannerDeltaT);
	//			nn->SetCost( dCost2New );
	//			return true;
	//		}
	//	}
	//}

	//// ʧ�ܣ�nn�������һ�ε��ھӽڵ�
	//nn = pNode;
	//return false;

	return RRT::Extend( x, t, nn, forward );
}


double RRTAnytime::GrowRRT( MSLNode* &n_new )
{
	MSLVector x_new, x_sample;
	int i = 0;
	bool bSuccess = false;
	const double dTimeMax = 60;		// ÿ�����������10s��

	n_new = T->Root();
	x_new = T->Root()->State();

	float dTimeCur = used_time();
	float dTime = 0.0;

	while ((!GapSatisfied(x_new, P->GoalState))) 
	{
		x_sample = ChooseState();
		if( 0 != x_sample.dim() )
		{
			bSuccess = Extend( x_sample, T, n_new, true );
		}
		dTime += used_time( dTimeCur );
		if( dTime > dTimeMax )
		{
			cout<< "GrowRRT������" << dTime << "s����ʱ��" <<endl;
			n_new = NULL;
			return 0.0;
		}
	}

	return n_new->Cost();
}

bool RRTAnytime::Plan()
{
	const int K = 1;
	const double dDetaD = 0.1;
	const double dDetaC = 0.1;
	const double dEslon = 0.15;
	float t = used_time();

	MSLNode *n_goal;
	list<MSLNode*> path;

	for( int i = 0; i < K; ++i )
	{
		ReinitializeRRT();
		double dCost = GrowRRT( n_goal );
		if( 0 != dCost )
		{
			m_dCostSol = ( 1.0 - dEslon ) * dCost;
			m_dDistBias -= dDetaD;
			if( m_dDistBias < 0 )
			{
				m_dDistBias = 0.0;
			}

			m_dCostBias += dDetaC;
			if( m_dCostBias > 1.0 )
			{
				m_dCostBias = 1.0;
			}
		}
	}

	CumulativePlanningTime += ((double)used_time(t));
	

	PrintInfo();

	// Get the solution path
	if (n_goal && GapSatisfied(n_goal->State(),P->GoalState)) 
	{
		// ��Զɽ2011.7.15
		m_bSuccess = true;
		//---- ��Զɽ2011.7.15

		P->m_dMinCost = n_goal->Cost();

		cout << "Successful Path Found\n";
		path = T->PathToRoot(n_goal);
		path.reverse();
		RecordSolution(path); // Write to Path and Policy
	}
	else 
	{
		cout << "Failure to connect after " << 
			T->Size() << " nodes\n";
		cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

		m_bSuccess = false;	
	}

	return m_bSuccess;
}


