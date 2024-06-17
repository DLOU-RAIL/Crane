//-------------------------------------------------------------------------
//                  The Motion Strategy Library (MSL)
//-------------------------------------------------------------------------
//
// Copyright (c) 2003 University of Illinois and Steven M. LaValle
// All rights reserved.
//
// Developed by:                Motion Strategy Laboratory
//                              University of Illinois
//                              http://msl.cs.uiuc.edu/msl/
//
// Versions of the Motion Strategy Library from 1999-2001 were developed
// in the Department of Computer Science, Iowa State University.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Motion Strategy Laboratory, University
//       of Illinois, nor the names of its contributors may be used to 
//       endorse or promote products derived from this Software without 
//       specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------



#include <math.h>
#include <stdio.h>

#include "planner.h"
#include "defs.h"



// *********************************************************************
// *********************************************************************
// CLASS:     Planner base class
// 
// *********************************************************************
// *********************************************************************

Planner::Planner(Problem *problem):Solver(problem) {
  T = NULL;
  T2 = NULL;
  Roadmap = NULL;

  m_strName = "Planner";
  m_bFirstWrite = true;
  // --��Զɽ2011.7.14

  // ��Զɽ2011.7.29
  m_pkLocalPlanner = new CLocalPlanner( this );
  // --��Զɽ

  Reset();
}


Planner::~Planner() {
  Reset();
}

void Planner::Reset() {
  int i;

  NumNodes = 3000;
  std::ifstream fin;

  READ_PARAMETER_OR_DEFAULT(PlannerDeltaT,1.0);

  GapError = MSLVector(P->StateDim);
  for (i = 0; i < P->StateDim; i++)
    GapError[i] = 1.0;
  READ_OPTIONAL_PARAMETER(GapError);

  Path.clear();
  TimeList.clear();
  Policy1.clear();
  TimeList1.clear();
  Policy2.clear();
  TimeList2.clear();

  fin.open((FilePath+"Holonomic").c_str());
  Holonomic = fin ? true : false; // Nonholonomic by default
  fin.close();

  CumulativePlanningTime = 0.0;
  CumulativeConstructTime = 0.0;

  if (T)
  {
	  delete T;
	  T = NULL;
  }
  if (T2)
  {
	  delete T2;
	  T2 = NULL;
  }

  if (Roadmap)
    delete Roadmap;
  Roadmap = NULL;

  // ��Զɽ2011.7.15
  m_bSuccess = false;
  m_dCurTime = 0.0;

  m_dPoolProb = 0.2;
  m_dPathLength = INFINITY;
}


MSLVector Planner::RandomState() {
  int i;
  double r;
  MSLVector rx;

  rx = P->LowerState;
  for (i = 0; i < P->StateDim; i++) {
      R >> r; 
      rx[i] += r * (P->UpperState[i] - P->LowerState[i]);
    }

  return rx;
}



MSLVector Planner::NormalState(MSLVector mean, double sd = 0.5) {
  int i,j;
  double r;
  MSLVector rx;
  bool success = false;

  rx = mean;
  for (i = 0; i < P->StateDim; i++) {
    success = false;
    while (!success) {
      rx[i] = 0.0;
      for (j = 0; j < 12; j++) {  // Increase 12 here and below for more accuracy
	R >> r; rx[i] += r;
      }
      rx[i] = (rx[i] - 12/2)*sd*(P->UpperState[i]-P->LowerState[i])+mean[i];
      if ((rx[i] <= P->UpperState[i])&&(rx[i] >= P->LowerState[i]))
	success = true;
    }
  }

  return rx;
}



bool Planner::GapSatisfied(const MSLVector &x1, const MSLVector &x2) {
  MSLVector x;
  int i;

  x = P->StateDifference(x1,x2);
  for (i = 0; i < P->StateDim; i++) {
    if (fabs(x[i]) > GapError[i])
      return false;
  }

  return true;
}

//  // ��Զɽ����(2011.4.18)
// void Planner::Step()
// {
// 	static double ptime = 0;
// 	MSLVector state;
// 	state = P->Step();
// 	Path.push_back( state );

// 	MSLVector u = MSLVector(5);
// 	u[0] = 0.1;
// 	u[1] = u[2] = u[3] = u[4] = 0.0;
// 	Policy1.push_back( u );
// 	TimeList1.push_back(PlannerDeltaT);
// 	ptime += PlannerDeltaT;

// 	TimeList.push_back(ptime);
// }
//   // ---��Զɽ����(2011.4.18)


void Planner::Statistics( string path )
{
	string strFileName = path + this->m_strName + ".Statistics.csv";

	ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );

	if( m_bFirstWrite )
	{
		fout << "�ҵ�·��" << ",·������" << ",�ܹ滮ʱ��" << ",���ڵ���" << ",��ײ������";
		m_bFirstWrite = false;
	}

	fout.seekp( 0, std::ios::end );
	int nCount = T->Size();
	if( T2 )
	{
		nCount += T2->Size();
	}
	fout << "\n" << m_bSuccess << "," << P->m_dMinCost << "," << CumulativePlanningTime << "," << nCount << "," << P->m_iCollisionNum;
	fout.close();
}

void Planner::WriteResult( string path )
{
	string strFileName = path + this->m_strName + ".Result.csv";

	ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );

	static bool bFirstWrite = true;
	if( bFirstWrite )
	{
		fout << "�Ƿ�ɹ�" << ",�滮ʱ��" << ",�滮��·����" << ",�Ż�ʱ��" << ",�Ż���·����" << ",�滮���Ż���ʱ��";
		bFirstWrite = false;
	}

	fout.seekp( 0, std::ios::end );
	double dCost = INFINITY;
	if( !CostList.empty() )
	{
		dCost = CostList.back();
	}
	fout << "\n" << m_bSuccess << "," << CumulativePlanningTime << "," << P->m_dMinCost 
		<< "," << m_dOptimizeTime << "," << dCost << "," << CumulativePlanningTime + m_dOptimizeTime;
	fout.close();

	m_pkLocalPlanner->WriteDetail( path );
}

void Planner::SmoothenPath( int nIters )
{
	m_dOptimizeTime = 0;
	cout << "Before smoothen, cost is: " << CostList.back() << endl;
	float t = ::used_time();
	m_pkLocalPlanner->Smoothen( nIters );
	float fSmoothenTime = used_time( t );
	cout << "Smoothen Time: " << fSmoothenTime << endl;
	cout << "After smoothen, cost is: " << CostList.back() << endl;
	m_dOptimizeTime = fSmoothenTime;
}


void CLocalPlanner::Smoothen( int nIters )
{
	m_fLocalPlnTime = 0;
	m_dSelInputTime = 0;
	m_dMetricTime = 0;
	m_dIntegrateTime = 0;
	m_dGapSatisfiedTime = 0;
	m_dSatisfiedTime = 0;
	m_iTotalChangeNum = 0;
	m_iLocalPlanFailure = 0;
	m_vLocalPlanInfo.clear();

	double dRv1, dRv2;
	int nPathLen;
	int iIdx1, iIdx2;
	MSLVector x1, x2;

	list<MSLVector> path;
	list<MSLVector>::iterator pi, pi1, pi2;
	path = m_pkGlobalPlanner->Path;
	list<double> timelist;
	list<double>::iterator ti, ti1, ti2;
	timelist = m_pkGlobalPlanner->TimeList;
	list<double>::iterator ci, ci1, ci2;

	//// ����·������
	//double dPathCost1 = 0;
	//MSLVector xxx1 = m_pkGlobalPlanner->Path.front();
	//forall( pi, m_pkGlobalPlanner->Path )
	//{
	//	dPathCost1 += m_pkGlobalPlanner->P->Metric( xxx1, *pi );
	//	xxx1 = *pi;
	//}
	//cout << "Before Smoothen, the cost calculated by path is:" << dPathCost1 << endl;


	//ofstream outfile( "f:\\data\\SmoothenInfo.csv" );

	m_nNoNeedPlanCount = 0;
	m_nCostHigherNum = 0;
	float t, tt;
	m_fLocalPlnTime = m_fUpdateTime = 0;
	for( int i = 0; i < nIters; ++i )
	{
		m_kLastCostList.clear();

		// ���ȡ·���е�����
		m_pkGlobalPlanner->R >> dRv1;
		m_pkGlobalPlanner->R >> dRv2;

		if( dRv1 > dRv2 )
		{
			double dTmp = dRv1;
			dRv1 = dRv2;
			dRv2 = dTmp;
		}

		if( dRv2 - dRv1 > 0.6 )
		{
			dRv2 = dRv1 + 0.6;
		}

		nPathLen = m_pkGlobalPlanner->Path.size() - 1;
		iIdx1 = (int)(nPathLen * dRv1);
		iIdx2 = (int)(nPathLen * dRv2);
		while( (iIdx2 - iIdx1 < 2) && (iIdx2 < nPathLen - 1) )
		{
			dRv2 += (dRv2 - dRv1) * 0.1 ;
			dRv2 = min( dRv2, 1.0 );
			iIdx2 = (int)(nPathLen * dRv2);
		}

		m_kCurItem.dStartPos = dRv1;
		m_kCurItem.dTargetPos = dRv2;

		int idx = 0;
		pi = pi1 = pi2 = m_pkGlobalPlanner->Path.begin();
		ti = ti1 = ti2 = m_pkGlobalPlanner->TimeList.begin();
		ci = ci1 = ci2 = m_pkGlobalPlanner->CostList.begin();
		double dPrevCost = 0;
		for( idx = 0; idx <= nPathLen; ++idx )
		{
			if( idx <= iIdx1 )
			{
				pi1 = pi;
				ti1 = ti;
				ci1 = ci;
				x1 = *pi;
				m_dTime = *ti;
				m_dCost = *ci;	
			}

			if( idx <= iIdx2 )
			{
				pi2 = pi;
				ti2 = ti;
				ci2 = ci;
				x2 = *pi;
				m_dTimeInterval = *ti2 - *ti1;
				dPrevCost = *ci;
			}
			else
			{
				m_kLastCostList.push_back( *ci - dPrevCost );
			}
			pi++;
			ti++;
			ci++;
		}   // ���ˣ�x1��x2��pi1��pi2��ti1��ti2���ѻ��

		double dD = m_pkGlobalPlanner->P->Metric( x1, x2 );
		if( *ci2 - *ci1 < 1.1 * dD )
		{
			m_nNoNeedPlanCount++;
			continue;
		}

		// ���ɾֲ�·��
		t = ::used_time();
		//bool bUpdate = IntervalPlan( x1, x2 );
		bool bUpdate = IntervalPlanConLike( x1, x2 );
		m_fLocalPlnTime += used_time( t );

		// ����ȫ��·��
		tt = ::used_time();
		if( bUpdate )
		{

			//// ����޸�ǰ����Ϣ
			//if( outfile )
			//{
			//	outfile << "�� " <<  i << " �θ���ǰ��" << endl;
			//	forall( ti, m_pkGlobalPlanner->TimeList )
			//	{

			//		outfile << *ti << ",";
			//	}
			//	outfile << endl;

			//	forall( ci, m_pkGlobalPlanner->CostList )
			//	{
			//		outfile << *ci << ",";
			//	}
			//	outfile << endl;

			//	outfile << iIdx1 << "," << iIdx2 << ",,,,";
			//	forall( ci, m_kLastCostList )
			//	{
			//		outfile << *ci << ",";
			//	}
			//	outfile << endl;
			//}

			// �¹滮��Ĵ������ԭ���۴󣬱���ԭ������
			if( (m_kSubCostList.back() - m_kSubCostList.front()) > *ci2 - *ci1 )
			{
				m_nCostHigherNum++;
				continue;
			}

			m_iLocalPlanFailure++;
			// ����·��
			m_pkGlobalPlanner->Path.erase( pi1, ++pi2 );
			m_pkGlobalPlanner->TimeList.erase( ti1, ++ti2 );

			pi = pi1 = pi2 = m_pkGlobalPlanner->Path.begin();
			ti = ti1 = ti2 = m_pkGlobalPlanner->TimeList.begin();
			ci = ci1 = ci2 = m_pkGlobalPlanner->CostList.begin();
			for( idx = 0; idx < iIdx1; ++idx )
			{

				pi1++;
				ti1++;
				ci1++;

				pi2++;
				ti2++;
				ci2++;
				pi++;
			}   

			m_pkGlobalPlanner->Path.splice( pi1, m_kSubPath );
			// ����ʱ��
			m_pkGlobalPlanner->TimeList.splice( ti1, m_kSubTimeList );
			// ���´���
			m_pkGlobalPlanner->CostList.erase( ci1, m_pkGlobalPlanner->CostList.end() );
			m_pkGlobalPlanner->CostList.merge( m_kSubCostList );
			dPrevCost =  m_pkGlobalPlanner->CostList.back();
			ci2 = m_kLastCostList.begin();
			double dCost = 0;
			forall( ci2, m_kLastCostList )
			{
				dCost = dPrevCost + (*ci2);
				m_pkGlobalPlanner->CostList.push_back( dCost );
			}


			//// ����޸ĺ����Ϣ
			//if( outfile )
			//{
			//	forall( ti, m_pkGlobalPlanner->TimeList )
			//	{

			//		outfile << *ti << ",";
			//	}
			//	outfile << endl;

			//	forall( ci, m_pkGlobalPlanner->CostList )
			//	{
			//		outfile << *ci << ",";
			//	}
			//	outfile << endl;
			//}

		}
		m_fUpdateTime += used_time( t );
	}

	//// ����·������
	//double dPathCost = 0;
	//MSLVector xx1 = m_pkGlobalPlanner->Path.front();
	//forall( pi, m_pkGlobalPlanner->Path )
	//{
	//	dPathCost += m_pkGlobalPlanner->P->Metric( xx1, *pi );
	//	xx1 = *pi;
	//}

	m_iLocalPlanFailure = nIters - m_iLocalPlanFailure;
	cout << "Local Plan Time: " << m_fLocalPlnTime << endl;
	cout << "Update Time: " << m_fUpdateTime << endl;
	cout << "Select Input Time: " << m_dSelInputTime << endl;
	cout << "Metric Time: " << m_dMetricTime << endl;
	cout << "Integrate Time: " << m_dIntegrateTime << endl;
	cout << "GapSatisfied Time: " << m_dGapSatisfiedTime << endl;
	cout << "Satisfied Time: " << m_dSatisfiedTime << endl;
	cout << "Change Input Count: " << m_iTotalChangeNum << endl;
	cout << "Local Plan Failure Count: " << m_iLocalPlanFailure << endl;
	//cout << "After Smoothen, the cost calculated by path is:" << dPathCost << endl;
	cout << "No Need Local Plan Count: " << m_nNoNeedPlanCount << endl;
	cout << "Cost Higher Count: " << m_nCostHigherNum << endl;

	//WriteInfo( "f:\\LocalInfo.csv" );
}

bool CLocalPlanner::IntervalPlan( const MSLVector &x1, const MSLVector &x2 )
{
	bool bShortest = false;
	// �ж�x1,x2�Ƿ���ԭ·����ͬһֱ����

	if( bShortest )	// x1,x2�Ѿ���ԭ·���е�ͬһֱ���ϣ��������ţ�����Ҫ�ٹ滮
	{
		return false;
	}
	else
	{
		if( m_pkGlobalPlanner->P->CollisionFreeLine( x1, x2 ) )
		{
			// �����һ�εľֲ�·����Ϣ
			m_kSubPath.clear();
			m_kSubTimeList.clear();

			//MSLVector kNewX = m_pkGlobalPlanner->P->Integrate( m_pkGlobalPlanner->P->InitialState, m_pkGlobalPlanner->P->GetInputs().front(), m_pkGlobalPlanner->PlannerDeltaT );
			//const double dDetaQ = 5 * m_pkGlobalPlanner->P->Metric( m_pkGlobalPlanner->P->InitialState, kNewX );
			const double dDetaQ = 0.1;
			MSLVector x;
			double dStep = 0;
			double dA = 0;
			double dDist = m_pkGlobalPlanner->P->Metric( x1, x2 );
			int n = dDist / dDetaQ;
			dStep = (double)dDist / n;
			double dDeltaT = m_dTimeInterval / n;
			for( int i = 0; i < n; ++i )
			{
				dA = ( i * dStep ) / dDist;
				x = m_pkGlobalPlanner->P->InterpolateState( x1, x2, dA );
				m_kSubPath.push_back( x );
				m_dTime += dDeltaT;
				m_kSubTimeList.push_back( m_dTime );
				m_dCost += dStep;
				m_kSubCostList.push_back( m_dCost );
			}
			return true;
		}
		else		// ��������ϰ��������¹滮
		{
			return false;
		}
	}
}


MSLVector CLocalPlanner::SelectInput(const MSLVector &x1, const MSLVector &x2, 
	MSLVector &nx_best, bool &success)
{
	float t = used_time();

	MSLVector u_best,nx;
	list<MSLVector>::iterator u;
	double d,d_min;
	success = false;
	d_min = m_pkGlobalPlanner->P->Metric(x1,x2);
	list<MSLVector> il = m_pkGlobalPlanner->P->GetInputs(x1);

	double d_tmpMin = INFINITY;
	// ��Զɽ��� 
	forall(u,il) {
		nx = m_pkGlobalPlanner->P->Integrate(x1,*u,m_pkGlobalPlanner->PlannerDeltaT);

		d  = m_pkGlobalPlanner->P->Metric(nx,x2);

		if ((d < d_min)&&(x1 != nx)) { 
			if (m_pkGlobalPlanner->P->Satisfied(nx)) {
				d_min = d; 
				u_best = *u; 
				nx_best = nx; 
				success = true;
			}
		}
	}

	m_dSelInputTime += used_time( t );
	return u_best;
}

bool CLocalPlanner::IntervalPlanConLike( const MSLVector &x1, const MSLVector &x2 )
{
	// �����һ�εľֲ�·����Ϣ
	m_kSubPath.clear();
	m_kSubTimeList.clear();
	m_kSubCostList.clear();

	MSLVector nx,nx_prev,u_best;
	bool success;
	double d,d_prev, dDist;
	int iChange = 0;

	// 1.ѡ������
	u_best = SelectInput( x1, x2, nx, success );

	if (success) 
	{   // If a collision-free input was found
		dDist = m_pkGlobalPlanner->P->Metric( x1, nx );
		d = m_pkGlobalPlanner->P->Metric(nx,x2);
		d_prev = d;
		nx_prev = nx; // Initialize

		// �Ѿֲ������Ϣ��������
		m_kSubPath.push_back( x1 );
		m_kSubTimeList.push_back( m_dTime );
		m_kSubCostList.push_back( m_dCost );

		// 2.��������
		success = false;
		const int nChangeInputCount = 5;
		bool bSatisfied = true;
		while( bSatisfied && (iChange < nChangeInputCount) )
		{
			if( d > d_prev )
			{
				iChange++;
				u_best = SelectInput( nx_prev, x2, nx, success );
				if( success )
				{
					success = false;
					d_prev = d = m_pkGlobalPlanner->P->Metric(nx,x2);
				}
			}
			else
			{
				float dGapT = used_time();
				if( m_pkGlobalPlanner->GapSatisfied( nx, x2 ) )
				{
					success = true;
					break;
				}
				m_dGapSatisfiedTime += used_time( dGapT );

				m_kSubPath.push_back( nx );
				m_dTime += m_pkGlobalPlanner->PlannerDeltaT;
				m_kSubTimeList.push_back( m_dTime );
				m_dCost += dDist;
				m_kSubCostList.push_back( m_dCost );

				nx_prev = nx;
				d_prev = d; 
				
				float dIntegrateT = used_time();
				nx = m_pkGlobalPlanner->P->Integrate(nx_prev,u_best,m_pkGlobalPlanner->PlannerDeltaT);
				m_dIntegrateTime += used_time( dIntegrateT );

				float dMetricT = used_time();
				dDist = m_pkGlobalPlanner->P->Metric( nx_prev, nx );
				d = m_pkGlobalPlanner->P->Metric(nx,x2);
				m_dMetricTime += used_time( dMetricT );
			}

			float dSatisfiedT = used_time();
			bSatisfied = m_pkGlobalPlanner->P->Satisfied( nx );
			m_dSatisfiedTime += used_time( dSatisfiedT );
		}

		m_kSubPath.push_back( x2 );
		m_dTime += m_pkGlobalPlanner->PlannerDeltaT;
		m_kSubTimeList.push_back( m_dTime );
		m_dCost += dDist;
		m_kSubCostList.push_back( m_dCost );

		m_iTotalChangeNum += iChange;
	}

	m_kCurItem.bSuccess = success;
	m_kCurItem.nChangeInput = iChange;
	m_vLocalPlanInfo.push_back( m_kCurItem );

	return success;
}

void CLocalPlanner::WriteInfo( string strFileName )
{
	ofstream outfile( strFileName.c_str() );
	if( outfile )
	{
		int iSize = m_vLocalPlanInfo.size();
		outfile << iSize << endl;
		outfile << m_iLocalPlanFailure << endl;
		
		for( int i = 0; i < iSize; ++i )
		{
			outfile << m_vLocalPlanInfo[i].bSuccess << ","
				<< m_vLocalPlanInfo[i].dStartPos << "," 
				<< m_vLocalPlanInfo[i].dTargetPos << ","
				<< m_vLocalPlanInfo[i].nChangeInput << endl;
		}
	}
}

void CLocalPlanner::WriteDetail( string path )
{
	string strFileName = path + m_pkGlobalPlanner->m_strName + ".OptimizeDetail.csv";

	ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
	static bool bFirstWrite = true;
	if( bFirstWrite )
	{
		fout << "���ֲ��滮ʱ��" 
			<< ",ѡ������ʱ��"  
			<< ",�������ʱ��" 
			<< ",����ʱ��" 
			<< ",�����յ��ж�ʱ��"
			<< ",�½ڵ��Ƿ������ж�ʱ��" 
			<< ",��������ʱ��"
			<< ",�ı��������" 
			<< ",ʧ�ܴ���" 
			<< ",��ת����" 
			<< ",��滮����";
		bFirstWrite = false;
	}

	fout.seekp( 0, std::ios::end );

	fout << m_fLocalPlnTime 
			<< "," << m_dSelInputTime
			<< "," << m_dMetricTime
			<< "," << m_dIntegrateTime
			<< "," << m_dGapSatisfiedTime
			<< "," << m_dSatisfiedTime
			<< "," << m_fUpdateTime
			<< "," << m_iTotalChangeNum
			<< "," << m_iLocalPlanFailure
			<< "," << m_nNoNeedPlanCount
			<< "," << m_nCostHigherNum;
	fout.close();
}

// *********************************************************************
// *********************************************************************
// CLASS:     IncrementalPlanner base class
// 
// *********************************************************************
// *********************************************************************

IncrementalPlanner::IncrementalPlanner(Problem *problem):Planner(problem) {
}

void IncrementalPlanner::Construct() {
  cout << "  Incremental planners do not use Construct.\n";
  cout << "  Try Plan.\n";
}


void IncrementalPlanner::RecordSolution(const list<MSLNode*> &glist, 
					const list<MSLNode*> &g2list)
{
  list<MSLNode*>::const_iterator n,nfirst,nlast;
  double ptime;

  Path.clear();
  TimeList.clear();
  CostList.clear();
  Policy1.clear();
  TimeList1.clear();
  Policy2.clear();
  TimeList2.clear();

  ptime = 0.0; 
  nfirst = glist.begin();

  forall(n,glist) {
    Path.push_back((*n)->State());
	// ��Զɽ2011.8.1
	CostList.push_back( (*n)->Cost() );
	// ��Զɽ
    if (n != nfirst) {
      Policy1.push_back((*n)->Input());
      TimeList1.push_back(PlannerDeltaT);
    }
    ptime += (*n)->Time();
    TimeList.push_back(ptime);
  }

  // The GapState is always comes from last node in glist
  GapState1 = Path.back();

  if(g2list.size() == 0)
    GapState2 = P->GoalState;
  else
    GapState2 = (*(g2list.begin()))->State();

  if (g2list.size() == 0) {
    // Push the goal state onto the end (jumps the gap)
    Path.push_back(P->GoalState);
  }
  else { // Using two graphs
    ptime += PlannerDeltaT; // Add a time step for the gap
    nlast = g2list.end();
	// ��Զɽ2011.8.1
	double dPathCost1 = glist.back()->Cost();
	double dPathCost2 = g2list.front()->Cost();
    forall(n,g2list) {
      Path.push_back( (*n)->State() );
	  // ��Զɽ2011.8.1
	  CostList.push_back( dPathCost1 + (dPathCost2 - (*n)->Cost()) );
	  // ��Զɽ
      if (n != nlast) {
	Policy2.push_back((*n)->Input());
        TimeList2.push_back(PlannerDeltaT);
      }
      TimeList.push_back(ptime);
      ptime += (*n)->Time();
    }
  }

  //cout << "Path: " << Path << "\n";
  //cout << "Policy: " << Policy << "\n";
  //cout << "TimeList: " << TimeList << "\n";
}



void IncrementalPlanner::RecordSolution(const list<MSLNode*> &glist)
{
  list<MSLNode*> emptylist;
  emptylist.clear(); // Make sure it is clear

  RecordSolution(glist,emptylist);
}
void IncrementalPlanner::RecordSolutionRvrs(const list<MSLNode*> &glist)
{
	list<MSLNode*> emptylist;
	emptylist.clear(); // Make sure it is clear

	RecordSolution(emptylist,glist);
}


void IncrementalPlanner::WriteGraphs(ofstream &fout)
{
  if (T)
    fout << *T << "\n\n\n";
  if (T2)
    fout << *T2;
}



void IncrementalPlanner::ReadGraphs(ifstream &fin)
{
  if (T)
    delete T;
  if (T2)
    delete T2;

  T = new MSLTree();
  T2 = new MSLTree();

  fin >> *T;
  //cout << "T \n" << *T << endl;
  fin >> *T2;
}




// *********************************************************************
// *********************************************************************
// CLASS:     RoadmapPlanner base class
// 
// *********************************************************************
// *********************************************************************

RoadmapPlanner::RoadmapPlanner(Problem *problem):Planner(problem) {
}




void RoadmapPlanner::WriteGraphs(ofstream &fout)
{
  fout << *Roadmap << "\n\n\n";
}



void RoadmapPlanner::ReadGraphs(ifstream &fin)
{
  if (Roadmap)
    delete Roadmap;

  Roadmap = new MSLGraph();
  fin >> *Roadmap;
}





