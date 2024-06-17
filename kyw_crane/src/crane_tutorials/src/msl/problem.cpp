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



//#include <fstream.h>
#include <math.h>

#include "problem.h"
#include "defs.h"
#include <moveit/robot_model/joint_model.h>

void Problem::Init(std::string strGroupName)
{
	// ��Զɽ��2011.6.5���
	m_dMinCost = INFINITY;
	m_dCumulativeCollisionTime = 0;
	m_dMetricTime = 0;
	m_iCollisionNum = 0;
	// --��Զɽ��2011.6.5���

  ModelDeltaT = 1.0;
  // 设置规划组及相关配置工作
  m_strGroupName = strGroupName;
  whole_group_ = new moveit::planning_interface::MoveGroupInterface("whole");
  rs_ = whole_group_->getCurrentState();
  joint_model_group_ = rs_->getJointModelGroup(m_strGroupName);

  // 获得当前活动的场景，为碰撞检测做好准备
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef;
  monitor_ptr_udef.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));   
  monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");  
  planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);  
  ps->getCurrentStateNonConst().update();  
  scene_ = ps->diff();  
  scene_->decoupleParent();

  StateDim = joint_model_group_->getVariableCount()- joint_model_group_->getMimicJointModels().size();
  if(m_strGroupName == "upper")
  	InputDim = StateDim;
  else
  	InputDim = StateDim - 1;

  LowerState = UpperState = MSLVector(StateDim);
  const robot_state::JointModelGroup* base_jmg = rs_->getJointModelGroup(m_strGroupName);
  std::vector<const robot_model::JointModel::Bounds*> limits = base_jmg->getActiveJointModelsBounds();
cout << limits.size() << endl;
//LowerState[0] = LowerState[1] = -100;
//UpperState[0] = UpperState[1] = 100;
  for(int i = 0; i < limits.size(); i++)
  {
    LowerState[i] = limits[i]->at(0).min_position_;
	UpperState[i] = limits[i]->at(0).max_position_;
  }
  cout << "LowerState: " << LowerState << endl;
  cout << "UpperState: " << UpperState << endl;

  //MaxDeviates = G->MaxDeviates;
}

// Constructor
Problem::Problem(std::string strGroupName, CraneLocationRegions* pPickCLR, CraneLocationRegions* pPlaceCLR) {
  Init(strGroupName);

  m_pPickCLR = pPickCLR;
  m_pPlaceCLR = pPlaceCLR;

  bool bValidState = false;
  int i, nMaxAttempts;
  i = 0;
  nMaxAttempts = 1000;
  while(!bValidState && i < nMaxAttempts )
  {
	InitialState = SamplingFromPickCLR();
	if( Satisfied( InitialState ) )
		bValidState = true;
	i++;
  }
  if(bValidState)
  {
  	cout << "GET AN VALID INITIALSTATE." << endl;
	//cout << InitialState << endl;
  }
  else
    cout << "NO VALID INITIALSTATE." << endl;

  bValidState = false;
  i = 0;
  while(!bValidState && i < nMaxAttempts )
  {
	GoalState = SamplingFromPlaceCLR();
	if( Satisfied( GoalState ) )
		bValidState = true;
	i++;
  }
  if(bValidState)
  {
  	cout << "GET AN VALID GOALSTATE." << endl;
	//cout << GoalState << endl;
  }
  else
    cout << "NO VALID GOALSTATE." << endl;
}

Problem::Problem(std::string strGroupName, MSLVector init, MSLVector goal) {
  Init(strGroupName);

  InitialState = init;
  GoalState = goal;
}


list<MSLVector> Problem::GetInputs(const MSLVector &x) {
  list<MSLVector> Inputs;
  Inputs.clear();
  MSLVector u;
  int nActions = 9;
  double linearVel = 0.5;
  double maxAngularVel = 0.4;

  if(StateDim == 3)
  {
	int nActions = 9;
	double linearVel = 0.2;
	double maxAngularVel = 0.5;
	for(int i = 0; i < nActions; i++)
	{
		u = MSLVector(InputDim);
		u[0] = linearVel;
		u[1] = -maxAngularVel + (maxAngularVel/(nActions-1)) * i;
		Inputs.push_back(u);
	}
  }
  else //if(m_strGroupName == "upper" || m_strGroupName == "whole")
  {
	for(int i = 0; i < InputDim; i++)
	{
		u = MSLVector(InputDim);
		u[i] = 0.1;
		Inputs.push_back(u);
		u = MSLVector(InputDim);
		u[i] = -0.1;
		Inputs.push_back(u);
	}
  }

  return Inputs;
}

list<MSLVector> Problem::GetInputs() {
  MSLVector x(StateDim);

  return GetInputs(x);
}


MSLVector Problem::InterpolateState(const MSLVector &x1, const MSLVector &x2, 
				 const double &a) {
  MSLVector v;

	v = (1.0-a)*x1 + a*x2;

	for( int i = 0; i < StateDim; ++i )
	{
		// 是否为下车转角、转台回转角、臂架仰角或吊钩旋转角，他们的下标是【2,3,4,6】
		if( i == 2 || i == 3 || i == 4 || i == 6 )
		{
			if (fabs(x2[i] - x1[i]) > PI) 
			{
				if (x1[i] > x2[i])
				{
					v[i] = (1.0-a)*x1[i] + a*(x2[i]+2.0*PI);
				}
				else
				{
					v[i] = (1.0-a)*(x1[i]+2.0*PI) + a*x2[i];
				}
			}

			if (v[i] >= PI)
			{
				v[i] -= 2.0*PI;
			}
			else if (v[i] < -PI)
			{
				v[i] += 2.0*PI;
			}
		}
	}

	return v;
}


// Default metric: use the metric from the model
double Problem::Metric(const MSLVector &x1, const MSLVector &x2) {
	// ��Զɽ��2011.6.6
	// ��¼��ײ��������ѵ�ʱ��

	m_iMetricNum++;

	float t = used_time();
	double dDist = 0;
	for(int i = 0; i < StateDim; i++)
		dDist += (x2[i]-x1[i]) * (x2[i]-x1[i]);
	dDist = sqrt(dDist);
	m_dMetricTime += ((double)used_time(t));
	return dDist;
}

MSLVector Problem::StateDifference(const MSLVector &x1, 
				const MSLVector &x2) {
  MSLVector v;

	v = x2 - x1;

	for( int i = 0; i < this->StateDim; ++i )
	{
		// 是否为下车转角、转台回转角、臂架仰角或吊钩旋转角，他们的下标是【2,3,4,6】
		if( i == 2 || i == 3 || i == 4 || i == 6 )
		{
			if (v[i] >= PI)
			{
				v[i] -= 2.0*PI;
			}
			else if (v[i] < -PI)
			{
				v[i] += 2.0*PI;
			}
		}
	}

	return v;
}

bool Problem::Satisfied(const MSLVector &x) {
	float t = used_time();
	m_iCollisionNum++;

	//robot_state::RobotState rs(*whole_group_->getCurrentState());
	std::vector<double> craneConf(8, 0.0);
	if(StateDim == 3)	// 只规划下车
	{
		craneConf[0] = x[0];
		craneConf[1] = x[1];
		craneConf[2] = x[2];
		craneConf[3] = 0.0;
		craneConf[4] = 1.45;
		craneConf[5] = -1.45;
		craneConf[6] = 50;
		craneConf[7] = 0.0;
	}
	else if(StateDim == 7)
	{
		for(int i = 0; i < StateDim; i++)
			craneConf[i] = x[i];
	}
	else
	{
		cout << "Not planning for crawler crane!" << endl;
		return false;
	}
    rs_->setJointGroupPositions(joint_model_group_, craneConf);
	scene_->setCurrentState(*rs_);
    robot_state::RobotState& current_state = scene_->getCurrentStateNonConst();
	bool bSatisfied = scene_->isStateValid(current_state, "whole");
	m_dCumulativeCollisionTime += ((double)used_time(t));

	//cout << "Satisfied Test: " << m_iCollisionNum << endl;

	return bSatisfied;
}

bool Problem::CollisionFree(const MSLVector &x) {
	float t = used_time();
	m_iCollisionNum++;

	std::vector<double> craneConf(8, 0.0);
	if(StateDim == 3)	// 只规划下车
	{
		craneConf[0] = x[0];
		craneConf[1] = x[1];
		craneConf[2] = x[2];
		craneConf[3] = 0.0;
		craneConf[4] = 1.45;
		craneConf[5] = -1.45;
		craneConf[6] = 50;
		craneConf[7] = 0.0;
	}
	else if(StateDim == 7)
	{
		for(int i = 0; i < StateDim; i++)
			craneConf[i] = x[i];
	}
	else
	{
		cout << "Not planning for crawler crane!" << endl;
		return false;
	}
    rs_->setJointGroupPositions(joint_model_group_, craneConf);
	bool bCollision = scene_->isStateColliding(*rs_, "whole");
	m_dCumulativeCollisionTime += ((double)used_time(t));

	//cout << "Satisfied Test: " << m_iCollisionNum << endl;

	return !bCollision;
}

MSLVector Problem::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
	MSLVector dx(StateDim);
	//double L = 5;		// 前后驱动轮距离暂定为5m

	// nx = (x, z, alpha, beta, gama, H)
	dx[0] = u[0] * cos( x[2] );			// xdot = v * cos(alpha)
	dx[1] = -u[0] * sin( x[2] );		// zdot = -v * sin(alpha)
	dx[2] = u[1]/*u[0] * tan( u[1] ) / L*/;		// alpha = v * tan(u[1]) / L
	for(int i = 2; i < InputDim; i++)
		dx[i+1] = u[2];
	
	return dx;
}

MSLVector Problem::Integrate(const MSLVector &x, const MSLVector &u, 
			  const double &deltat) {

  //cout << "x: " << x << "\n" << "u: " << u << endl;
  //! Integrate xdot using Euler integration
  int s,i,k;
  double c;
  MSLVector nx;

  s = (deltat > 0) ? 1 : -1;

  c = s*deltat/ModelDeltaT;  // Number of iterations (as a double)
  k = (int) c;

  nx = x;
  for (i = 0; i < k; i++) {
    nx += s * ModelDeltaT * StateTransitionEquation(nx,u);
  }

  // Integrate the last step for the remaining time
  nx += s * (c - k) * ModelDeltaT * StateTransitionEquation(nx,u);

  return nx;
} 


//! ��Զɽ2011.4.30
bool Problem::CollisionFreeLine( const MSLVector &x1, const MSLVector &x2 )
{
	const double dDetaQ = 0.1;
	MSLVector x, q;
	double dStep = 0;
	double dA = 0;
	double dDist = Metric( x1, x2 );
	int n = dDist / dDetaQ;
	dStep = (double)dDist / n;
	bool bCollision = true;
	robot_state::RobotState rs(*whole_group_->getCurrentState());     // 耗时操作
	for( int i = 0; i < n; ++i )
	{
		dA = ( i * dStep ) / dDist;
		x = InterpolateState( x1, x2, dA );
		if( !CollisionFree(x) )
			return false;
	}
	return true;
}


MSLVector Problem::ConfigurationDifference(const MSLVector &q1, const MSLVector &q2) {
  return StateDifference(q1,q2);
}


MSLVector Problem::GetInputFromStates( const MSLVector &x1, const MSLVector &x2 )
{
	MSLVector u(InputDim);
	MSLVector detax(StateDim);

	detax = StateDifference( x1, x2 );

	// u = (v, deta_alpha, deta_beta, deta_gama, deta_H)
	// v = ( (x2-x1)^2 + (z2-z1)^2 )^0.5
	u[0] = sqrt( pow( detax[0], 2 ) + pow( detax[1], 2 ) );
	// deta_alpha = atan( -(z2-z1) / (x2-x1) )
	u[1] = atan2( -detax[1],detax[0] ) - x1[2];
	if (u[1] >= PI)
	{
		u[1] -= 2.0*PI;
	}
	else if (u[1] < -PI)
	{
		u[1] += 2.0*PI;
	}

	for( int i = 2; i < InputDim; ++i )
	{

		u[i] = detax[i+1];
	}

	return u;
}


MSLVector Problem::Round( MSLVector kState )
{
	MSLVector v = kState;
	for( int i = 0; i < this->StateDim; ++i )
	{
		// 是否为下车转角、转台回转角、臂架仰角或吊钩旋转角，他们的下标是【2,3,4,6】
		if( i == 2 || i == 3 || i == 4 || i == 6 )
		{
			if (v[i] >= PI)
			{
				v[i] -= 2.0*PI;
			}
			else if (v[i] < -PI)
			{
				v[i] += 2.0*PI;
			}
		}
	}
	return v;
}

MSLVector Problem::SamplingFromPickCLR()
{
	std::vector<double> craneConf;
	MSLVector craneState;
	m_pPickCLR->directSampling(craneConf);
	craneConf.erase(craneConf.begin()+5);          // 去除mimic joint的值
	craneState.setFromSTDVector(craneConf);
	return craneState;
}

MSLVector Problem::SamplingFromPlaceCLR()
{
	std::vector<double> craneConf;
	MSLVector craneState;
	m_pPlaceCLR->directSampling(craneConf);
	craneConf.erase(craneConf.begin()+5);          // 去除mimic joint的值
	craneState.setFromSTDVector(craneConf);
	return craneState;
}
