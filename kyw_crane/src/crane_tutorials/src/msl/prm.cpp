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

#include "prm.h"
#include "defs.h"


// These are used for quasi-random sampling
static int Primes[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,
		       61,67,71,73,79,83,89,97,101,103,107,109,113,127,
		       131,137,139,149,151,157,163,167,173};


// *********************************************************************
// *********************************************************************
// CLASS:     PRM base class
// 
// *********************************************************************
// *********************************************************************

PRM::PRM(Problem *problem): RoadmapPlanner(problem) {
  MaxEdgesPerVertex = 200;
  MaxNeighbors = 200;
  StepSize = 0.6;  // 根据不同问题来设置

  READ_PARAMETER_OR_DEFAULT(Radius,10.0);

  SatisfiedCount = 0;
  QuasiRandom = true;

  //! Choose Hammersley (which is better) or Halton sequence for quasi-random points
  //QuasiRandomSukharev = false;
  QuasiRandomHammersley = false;
  SukharevDispersion = 0.0; // The Sukharev sampler will compute it

  //PRMs are for holonomic problems!
  if (!Holonomic)
    cout << "WARNING: Differential constraints will be ignored.\n";

  // 可视化prm用
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




list<MSLVertex*> PRM::NeighboringVertices(const MSLVector &x) {
  double d;
  MSLVertex n;
  list<MSLVertex*> best_list,all_vertices;
  list<MSLVertex*>::iterator vi;
  int k;

  all_vertices = Roadmap->Vertices();

  best_list.clear();
  k = 0;
  forall(vi,all_vertices) {
    d = P->Metric((*vi)->State(),x);
    if ((d < Radius)&&(k < MaxNeighbors)) {
      best_list.push_back(*vi);
      k++;  // Count the number of attempted connections
    }
  } 
  
  //cout << "Number of Neighbors: " << best_list.size() << "\n";

  return best_list;
}




// This function essentially iterates Extend until it has to stop

bool PRM::Connect(const MSLVector &x1, const MSLVector &x2, MSLVector &u_best) {
  double a;
  int k;
  MSLVector x;

  k = (int) (P->Metric(x1,x2) / StepSize);

  //cout << "metric: " << P->Metric(x1,x2) << 
  //  "    stepsize: " << StepSize << "\n";
  //cout << "Connect steps: " << k << "\n";

  if (k == 0)
    return true;

  for (a = 1.0/(k+1); a < 1.0; a += 1.0/(k+1) ) {
    //cout << "In Connect: " ;
    x = P->InterpolateState(x1,x2,a);
    SatisfiedCount++;
    if (!P->Satisfied(x)) {
      //cout << "Returning FALSE in Connect\n";
      return false;
    }
  }

  return true;
}



void PRM::Construct(int nAddNodes)
{
  int i,k;
  MSLVector u_best,nx;
  MSLVertex *nn;
  list<MSLVertex*> nhbrs;
  list<MSLVertex*>::iterator ni;
  
  float t = used_time();

  if (!Roadmap)
  {
    Roadmap = new MSLGraph();
    markerVertexes_.points.clear();
    markerEdges_.points.clear();
  }

  i = 0;
  while (i < nAddNodes) {
    nx = ChooseState(i,nAddNodes,P->StateDim);
    // cout << "i = " << i << endl;
    // cout << "N in Map is: " << Roadmap->NumVertices() << endl;
    SatisfiedCount++;
    i++;
    while (!P->Satisfied(nx)) {
      nx = ChooseState(i,nAddNodes,P->StateDim);  
      SatisfiedCount++;
      i++;
      cout << "i = " << i << endl;
      // Keep trying until a good sample is found
    }
    nhbrs = NeighboringVertices(nx);
    nn = Roadmap->AddVertex(nx);
    // 添加可视化坐标点
    vertex_.x = nn->State()[0];
    vertex_.y = nn->State()[1];
    vertex_.z = 0.0;
    markerVertexes_.points.push_back(vertex_);
    if(bVisual_)
    {
      marker_pub_.publish(markerVertexes_);
    }

    ds.makeSet(nn);

    k = 0;
    forall(ni,nhbrs) {
      if (ds.findSet(nn) == ds.findSet((*ni)))
      {
        //cout << "Already in same connected component, abortingconnection attempt.\n";
        continue;
      }
      if (Connect((*ni)->State(),nx,u_best)) {
        Roadmap->AddEdge(nn,*ni,u_best,1.0);
        Roadmap->AddEdge(*ni,nn,-1.0*u_best,1.0);
        // 添加可视化边
        vertex_.x = nn->State()[0];
        vertex_.y = nn->State()[1];
        vertex_.z = 0.0;
        markerEdges_.points.push_back(vertex_);
        vertex_.x = (*ni)->State()[0];
        vertex_.y = (*ni)->State()[1];
        vertex_.z = 0.0;
        markerEdges_.points.push_back(vertex_);
        if(bVisual_)
        {
          marker_pub_.publish(markerEdges_);
        }

        k++;
        ds.setUnion(nn, (*ni));
      }
      //else { cout << "bad edge = must be collision\n";}
      if (k > MaxEdgesPerVertex)
	      break;
    }

    if (Roadmap->NumVertices() % 1000 == 0)
      cout << Roadmap->NumVertices() << " vertices in the PRM.\n";
  }

  cout << "PRM Vertices: " << Roadmap->NumVertices() << 
    "  Edges: " << Roadmap->NumEdges() <<
    "  ColDet: " << SatisfiedCount;
  //cout << "  Number of components: " << ds.getNumSets();
  //cout <<  "  Components: " << c;
  cout << "\n";

  CumulativeConstructTime += ((double)used_time(t));
  cout << "Construct Time: " << CumulativeConstructTime << "s\n"; 

}



bool PRM::Plan()
{    
  list<MSLVertex*> nlist, ilist, glist;
  list<MSLEdge*> elist;
  MSLVertex *n,*ni,*nn,*n_best;
  MSLVector u_best;
  bool success = false;
  list<MSLVertex*> vpath;
  list<MSLVertex*>::iterator vi;
  list<MSLEdge*>::iterator ei;
  priority_queue<MSLVertex*,vector<MSLVertex*>,MSLVertexGreater> Q;
  double cost,mincost,time;

  float t = used_time();

  if (!Roadmap) {
    cout << "Empty roadmap.  Run Construct before Plan.\n";
    return false;
  }

  // Set the step size
  StepSize = P->Metric(P->InitialState,P->Integrate(P->InitialState,
 	     P->GetInputs(P->InitialState).front(),PlannerDeltaT));

  // Connect to the initial state
  nlist = NeighboringVertices(P->InitialState);

  if (nlist.size() == 0) {
    cout << "No neighboring vertices to the Initial State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n"; 
    return false;
  }

  vi = nlist.begin();
  map<MSLVertex*, bool> imap, gmap;
  //find all vertices that neighbor the initial state, represent unique 
  //connected components, and can be connected to the initial state
  forall(vi, nlist)
    {
      //cout << "connect trial...\n";
      if ((imap.find(ds.findSet((*vi))) == imap.end()) &&
          (Connect(P->InitialState,(*vi)->State(),u_best)))
        {
          imap[ds.findSet((*vi))] = true;
          ilist.push_back((*vi));
        }
    }

  if (ilist.size() == 0) {
    cout << "Failure to connect to Initial State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n"; 
    return false;
  }

  // Connect to the goal state
  nlist = NeighboringVertices(P->GoalState);

  if (nlist.size() == 0) {
    cout << "No neighboring vertices to the Goal State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n"; 
    return false;
  }

  //find all vertices that neighbor the initial state, represent unique 
  //connected components, and can be connected to the initial state
  forall(vi, nlist){
    //cout << "connect trial...\n";
    if ((imap.find(ds.findSet((*vi))) != imap.end()) && 
        (gmap.find(ds.findSet((*vi))) == gmap.end()) &&
        (Connect(P->GoalState,(*vi)->State(),u_best)))
      {
        gmap[ds.findSet((*vi))] = true;
        glist.push_back((*vi));
      }
  }
  
  if (glist.size() == 0) {
    cout << "Failure to connect to Goal State or no path in roadmap\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n"; 
    return false;
  }

  // Initialize for DP search (the original PRM used A^*)
  nlist = Roadmap->Vertices();
  forall(vi, nlist)
  {
	  (*vi)->Unmark();
  }

  forall(vi, ilist) //initialize all vertices which can connect to i.s.
  {
	  (*vi)->SetCost(1.0);
	  (*vi)->Mark();
	  Q.push((*vi));

  }

  // Loop until Q is empty or goal is found
  success = false;
  while ((!success)&&(!Q.empty())) {
	  // Remove smallest element
	  n = Q.top();
	  cost = n->Cost();
	  Q.pop();

	  // Expand its unexplored neighbors
	  elist = n->Edges();
	  forall(ei,elist) {
		  nn = (*ei)->Target();
		  if (!nn->IsMarked()) { // If not yet visited
			  nn->Mark();
			  nn->SetCost(cost+(*ei)->Cost());
			  Q.push(nn);
		  }
	  }
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n"; 

  //now, take as the endpoint of the path in the graph the vertex
  //  of glist whose cost is least

  mincost = INFINITY;
  forall(vi, glist) {
    if ((*vi)->Cost() < mincost) {
      mincost = (*vi)->Cost();
      n = (*vi);
    }
  }
  //figure out which vertex in ilist it came from
  forall(vi, ilist) {
    if (ds.findSet((*vi)) == ds.findSet(n)) {
      ni = (*vi);
      break;
    }
  }
  // Get the path
  while (n != ni) {
    mincost = INFINITY;
    vpath.push_front(n);
    // Pick neighboring vertex with lowest cost
    elist = n->Edges();
    forall(ei,elist) {
      nn = (*ei)->Target();
      if (nn->Cost() < mincost) {
        n_best = nn; 
        mincost = nn->Cost();
      }
    }
    n = n_best;
  }

  // Make the solution
  Path.clear();
  TimeList.clear();
  time = 0.0;
  markerPath_.points.clear();

  Path.push_back(P->InitialState);
  TimeList.push_back(time);
  time += 1.0;

  vertex_.x = P->InitialState[0];
  vertex_.y = P->InitialState[1];
  vertex_.z = 0.0;
  markerPath_.points.push_back(vertex_);

  forall(vi,vpath) {
    Path.push_back((*vi)->State());
    TimeList.push_back(time);
    time += 1.0;

    vertex_.x = (*vi)->State()[0];
    vertex_.y = (*vi)->State()[1];
    vertex_.z = 0.0;
    markerPath_.points.push_back(vertex_);
  }

  Path.push_back(P->GoalState);
  TimeList.push_back(time);

  vertex_.x = P->GoalState[0];
  vertex_.y = P->GoalState[1];
  vertex_.z = 0.0;
  markerPath_.points.push_back(vertex_);

  if(bVisual_)
  {
    marker_pub_.publish(markerPath_);
  }

  cout << "  Success\n";

  return true;
}


MSLVector PRM::ChooseState(int i, int maxnum, int dim) {
  if (QuasiRandom) {
    if (QuasiRandomSukharev)
      return QuasiRandomStateSukharev(i,maxnum,dim);
    if (QuasiRandomHammersley)
      return QuasiRandomStateHammersley(i,maxnum,dim);
    else
      return QuasiRandomStateHalton(i,dim);
  }
  else
  {
    return RandomState();
  }
}




MSLVector PRM::QuasiRandomStateHammersley(int i, int maxnum, int dim) {
  int j,k,r,ppow;
  MSLVector qrx;

  if (dim > 30) {
    cout << "ERROR: Dimension too high for quasi-random samples\n";
    exit(-1);
  }

  qrx = MSLVector(dim);

  k = i;
  
  qrx[0] = ((double) k / maxnum) * (P->UpperState[0] - P->LowerState[0]);
  qrx[0] += P->LowerState[0];
  for (j = 1; j < dim; j++) {
    qrx[j] = 0.0;
    ppow = Primes[j-1];
    k = i;
    while (k != 0) {
      r = k % Primes[j-1];
      k = (int) (k / Primes[j-1]);
      qrx[j] += (double) r / ppow;
      ppow *= Primes[j-1];
    }
    qrx[j] *= (P->UpperState[j] - P->LowerState[j]);
    qrx[j] += P->LowerState[j];
  }

  return qrx;
}



MSLVector PRM::QuasiRandomStateSukharev(int i, int maxnum, int dim) {
  int j,index,remainder;
  MSLVector qrx;
  double ppax;

  if (SukharevDispersion == 0.0) // Needs to be computed
    {
      cout << maxnum << "    dim = " << dim << endl;
      cout << "Computing Sukharev grid spacing\n";
      ppax = pow((float)maxnum,(float)1.0/dim);
      SukharevPointsPerAxis = (int) ppax;
      cout << "  Points per Axis: " << SukharevPointsPerAxis << "\n";
      SukharevDispersion = 0.5/((double) SukharevPointsPerAxis);
      cout << "  Normalized Dispersion: " << SukharevDispersion << "\n";
    }

  qrx = MSLVector(dim);

  remainder = i;
  for (j = 0; j < dim; j++) {
    // Compute jth index into grid
    index = (int) (remainder / pow( (double) SukharevPointsPerAxis, dim-j-1));
    remainder = (int) (remainder - index*pow( (double) SukharevPointsPerAxis, dim-j-1));
    // Compute jth coordinate
    qrx[j] = P->LowerState[j];
    qrx[j] += (index+0.5) * (P->UpperState[j] - 
			     P->LowerState[j]) / SukharevPointsPerAxis;
  }

  return qrx;
}



MSLVector PRM::QuasiRandomStateHalton(int i, int dim) {
  int j,k,r,ppow;
  MSLVector qrx;

  if (dim > 30) {
    cout << "ERROR: Dimension too high for quasi-random samples\n";
    exit(-1);
  }

  qrx = MSLVector(dim);
  k = i;

  for (j = 0; j < dim; j++) {
    qrx[j] = 0.0;
    ppow = Primes[j];
    k = i;
    while (k != 0) {
      r = k % Primes[j];
      k = (int) (k / Primes[j]);
      qrx[j] += (double) r / ppow;
      ppow *= Primes[j];
    }
    qrx[j] *= (P->UpperState[j] - P->LowerState[j]);
    qrx[j] += P->LowerState[j];
  }

  return qrx;
}
