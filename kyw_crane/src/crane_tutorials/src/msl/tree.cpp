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




#include "tree.h"
#include "defs.h"

// *********************************************************************
// *********************************************************************
// CLASS:     MSLNode class
// 
// *********************************************************************
// *********************************************************************

ostream& operator<<(ostream& out, const MSLNode& n)
{
  out << n.id;
  if (n.parent) 
    out << " " << n.parent->id;
  else
    out << " -1";
  out << " " << n.state << " " << n.input;
  out << "\n";

  return out;
}


ostream& operator<<(ostream& out, const list<MSLNode*>& L)
{
  list<MSLNode*>::iterator x; 
  list<MSLNode*> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++) 
    out << " " << **x;
  return out;
}




MSLNode::MSLNode() { 
  // ��Զɽ��2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- ��Զɽ��2011.6.30
}


MSLNode::MSLNode(void* pninfo) { 
  info = pninfo;

  // ��Զɽ��2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- ��Զɽ��2011.6.30
}


MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u) {
  state = x;
  input = u;
  parent = pn;
  time = 1.0; // Make up a default
  cost = 0.0;

  // ��Զɽ��2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- ��Զɽ��2011.6.30
}


MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u, double t) {
  state = x;
  input = u;
  parent = pn;
  time = t;
  cost = 0.0;

  // ��Զɽ��2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- ��Զɽ��2011.6.30
}

MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u, double t, void* pninfo) {
  state = x;
  input = u;
  parent = pn;
  time = t;
  cost = 0.0;

  info = pninfo;

  // ��Զɽ��2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- ��Զɽ��2011.6.30
}


// *********************************************************************
// *********************************************************************
// CLASS:     MSLTree class
// 
// *********************************************************************
// *********************************************************************


ostream& operator<< (ostream& os, const MSLTree& T) {
  list<MSLNode*>::iterator x;
  list<MSLNode*> vl;
  vl = T.nodes;

#ifdef USE_ANN
  
  os << T.m_pMAG->dimension << "\n";
  os << T.m_pMAG->NumNeighbors << "\n";
  for( int i = 0; i < T.m_pMAG->dimension; ++i )
  {
  os << T.m_pMAG->topology[i] << "\n";
  os << T.m_pMAG->scaling[i] << "\n";
  }
#endif

  os << T.size << "\n";
  for (x = vl.begin(); x != vl.end(); x++) 
    os << **x;
  return os;
}



istream& operator>> (istream& is, MSLTree & T) {
  int i,nid,pid,tsize;
  MSLVector x,u;
  MSLNode *pnode,*n; // Parent node

  T.Clear();

#ifdef USE_ANN
  int nDim, nMaxNeighbors;
  int* pTopology;
  double *pScaling;
  
  is >> nDim;
  is >> nMaxNeighbors;
  pTopology = new int[nDim];
  pScaling = new double[nDim];
  for( int i = 0; i < nDim; ++i )
  {
	  is >> pTopology[i];
	  is >> pScaling[i];
  }

  T.m_pMAG = new MultiANN( nDim, nMaxNeighbors, pTopology, pScaling );

  delete pTopology;
  delete pScaling;
#endif

  is >> tsize;
  cout << "Loading a tree that has " << tsize << " nodes\n";
  for (i = 0; i < tsize; i++) {
    is >> nid >> pid >> x >> u;
    pnode = T.FindNode(pid);
    if (pnode) {
      n = T.Extend(pnode,x,u);
      n->SetID(nid);
    }
    else
      T.MakeRoot(x);
  }

  return is; 
}


MSLTree::MSLTree() { 
	root = NULL;
	size = 0;

#ifdef USE_ANN
	m_pMAG = NULL;
	
#endif
}


MSLTree::MSLTree(const MSLVector &x) { 
  MSLVector u;

  root = new MSLNode(NULL,x,u,0.0);
  root->id = 0;
  nodes.push_back(root);
  size = 1;

#ifdef USE_ANN
	m_pMAG = NULL;
#endif
}


MSLTree::MSLTree(const MSLVector &x, void* nodeinfo) { 
  MSLVector u;

  root = new MSLNode(NULL,x,u,0.0,nodeinfo);
  root->id = 0;
  nodes.push_back(root);
  size = 1;

#ifdef USE_ANN
	m_pMAG = NULL;
#endif
}

#ifdef USE_ANN
MSLTree::MSLTree(const MSLVector &x, int *pTopology, double *pScaling )
{
	int nDim = x.dim();
	const int nMaxNeighbors = 8;
	m_pMAG = new MultiANN( nDim, nMaxNeighbors, pTopology, pScaling );

	MSLVector u;

	root = new MSLNode(NULL,x,u,0.0);
	root->id = 0;
	nodes.push_back(root);
	size = 1;

	AddPoint2MultiANN( x );
}


void MSLTree::AddPoint2MultiANN( const MSLVector &x )
{
	ANNpoint pt = annAllocPt( x.dim() );
	for( int i = 0; i < x.dim(); ++i )
	{
		pt[i] = x[i];
	}
	m_pMAG->AddPoint( pt, pt );
	annDeallocPt( pt );
}

// ��Զɽ��2011.6.5
void MSLTree::RemovePointFromMultiANN( const MSLVector &x )
{	
	ANNpoint pt = annAllocPt( x.dim() );
	for( int i = 0; i < x.dim(); ++i )
	{
		pt[i] = x[i];
	}
	m_pMAG->RemovePoint( pt );
	annDeallocPt( pt );
}
// ---��Զɽ��2011.6.5

MSLNode* MSLTree::NearestNode( const MSLVector &x, double &dBestDist )
{
	int iIdxNearest = 0;
	double dDistance = 0;
	int nDim = x.dim();
	ANNpoint ANNpt;
	ANNpt = new double[nDim];
	for( int i = 0; i < nDim; ++i )
	{
		ANNpt[i] = x[i];
	}
	m_pMAG->NearestNeighbor( ANNpt, iIdxNearest, dDistance );
	dBestDist = dDistance;
	return FindNode( iIdxNearest );
}

MSLNode* MSLTree::NearestNode( const MSLVector &x, double &dBestDist, double *pScaling )
{
	for (int i = 0; i < m_pMAG->dimension; i++) 
	{
		m_pMAG->scaling[i] = pScaling[i];
	}

	return NearestNode( x, dBestDist );
}

vector< MSLNode* > MSLTree::kNearestNodes( const MSLVector &x, vector< double > &vBestDist )
{
	int nDim = x.dim();
	ANNpoint ANNpt;				// ��ѯ��
	ANNpoint ANNbestDist;		// �����ھ����ѯ��ľ���ֵ
	int *pANNbestIdx;			// �����ھӵ�����
	void **ppANNbestList;		// �����ھӵ�ָ��
	vector<MSLNode*> vNodes;	// ����������ֵ
	MSLNode* pNearNode = NULL;	// ��ʱ����

	// ��ѯ��
	ANNpt = new double[nDim];
	for( int i = 0; i < nDim; ++i )
	{
		ANNpt[i] = x[i];
	}
	// Ϊ���ؽ�������ڴ�ռ�
	ANNbestDist = new double[m_pMAG->NumNeighbors];
	pANNbestIdx = new int[m_pMAG->NumNeighbors];
	ppANNbestList = (void**)new double*[m_pMAG->NumNeighbors];

	m_pMAG->NearestNeighbor( ANNpt, ANNbestDist, pANNbestIdx, ppANNbestList );
	// ��m_pMAG->NearestNeighbor�У�ANNbestDist����ʼ��Ϊ�����(1.0e40)��pANNbestIdx����ʼ��Ϊ0.
	// ���ص�ANNbestDist, pANNbestIdx�Ѿ����ݾ����С��������
	const double dBigDouble = 1.0e30;
	for( int i = 0; i < m_pMAG->NumNeighbors; ++i )
	{
		if( ANNbestDist[i] > dBigDouble )		// �Ѿ�����β���������ھ�����С���������
		{
			break;
		}
		else
		{
			pNearNode = FindNode( pANNbestIdx[i] );
			vNodes.push_back( pNearNode );
			vBestDist.push_back( ANNbestDist[i] );
		}
	}
	delete ANNbestDist;
	delete pANNbestIdx;
	delete ppANNbestList;

	return vNodes;
}
#endif

// ��Զɽ��2011.6.5
void MSLTree::ClearByCost( MSLNode* pTree, double dCost )
{
	if( pTree->Cost() > dCost )		// ����������ɾ������
	{
		RemoveSubTree( pTree );
	}
	else			// ������ɾ����������������������
	{
		list<MSLNode*> children = pTree->Children();
		list<MSLNode*>::iterator it;
		for( it = children.begin(); it != children.end(); it++ )
		{
			ClearByCost( *it, dCost );
		}
	}
}

void MSLTree::RemoveSubTree( MSLNode* pSubTree )
{
	list<MSLNode*> children = pSubTree->Children();
	list<MSLNode*>::iterator it;
	for( it = children.begin(); it != children.end(); it++ )
	{
		RemoveSubTree( *it );
	}

	// ɾ�����ڵ�
	// �븸�ڵ������ϵ
	MSLNode* pParent = pSubTree->Parent();
	if( pParent )
	{
		pParent->DetachChild( pSubTree );
	}
	// ��nodes�б���ɾ���ڵ�
	nodes.remove( pSubTree );
	// ɾ��MPNN�ж�Ӧ�ĵ�
#ifdef USE_ANN
	RemovePointFromMultiANN( pSubTree->State() );
#endif
	// ɾ�����ڵ�
	delete pSubTree;
}
// ------��Զɽ��2011.6.5

MSLTree::~MSLTree() {
  Clear();
}


void MSLTree::MakeRoot(const MSLVector &x) {
  MSLVector u;
  
  if (!root) {
    root = new MSLNode(NULL,x,u,0.0);
    root->id = 0;
    nodes.push_back(root);
  }
  else
    cout << "Root already made.  MakeRoot has no effect.\n";
  size = 1;
}

#include <stdio.h>
#include <stdlib.h>
MSLNode* MSLTree::Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u) {
  MSLNode *nn;

  nn = new MSLNode(parent, x, u);
  nn->id = size;
  nodes.push_back(nn);
  size++;
  
  parent->AddChild( nn );
  
#ifdef USE_ANN
  AddPoint2MultiANN( x );
#endif

  return nn;
}



MSLNode* MSLTree::Extend(MSLNode *parent, const MSLVector &x, 
			 const MSLVector &u,
			 double time) {
  MSLNode *nn;

  nn = new MSLNode(parent, x, u, time);
  nn->id = size;
  nodes.push_back(nn);
  size++;
    
  parent->AddChild( nn );

#ifdef USE_ANN
  AddPoint2MultiANN( x );
#endif

  return nn;
}


MSLNode* MSLTree::Extend(MSLNode *parent, const MSLVector &x, 
			 const MSLVector &u,
			 double time, void* pninfo) {
  MSLNode *nn;

  nn = new MSLNode(parent, x, u, time, pninfo);
  nn->id = size;
  nodes.push_back(nn);
  size++;
  
  parent->AddChild( nn );

#ifdef USE_ANN
  AddPoint2MultiANN( x );
#endif

  return nn;
}

void MSLTree::RemoveEdge( MSLNode *parent, MSLNode *child )
{
	parent->DetachChild( child );
}

MSLNode* MSLTree::FindNode(int nid) {
  list<MSLNode*>::iterator ni;

  for(ni = nodes.begin(); ni != nodes.end(); ni++) {
    if ((*ni)->id == nid)
      return *ni;
  }

  return NULL; // Indicates failure
}



list<MSLNode*> MSLTree::PathToRoot(MSLNode *n) {
  list<MSLNode*> nl;
  MSLNode *ni;
  
  ni = n;
  while (ni != root) {
    nl.push_back(ni);
    ni = ni->Parent();
  }

  nl.push_back(root);
 
  return nl;
}


void MSLTree::Clear() {
  list<MSLNode*>::iterator n; 
  for (n = nodes.begin(); n != nodes.end(); n++)
    delete *n;
  nodes.clear();
  root = NULL;

#ifdef USE_ANN
  if( m_pMAG )
  {
	  delete m_pMAG;
  }
#endif
}

bool MSLTree::Contain( MSLNode* pSubTree, MSLNode* pNode )
{
	//if( pSubTree == pNode )
	//	return true;

	//list<MSLNode*> children = pSubTree->Children();
	//list<MSLNode*>::iterator it;
	//for( it = children.begin(); it != children.end(); it++ )
	//{	
	//	Contain( *it, pNode, iCount );
	//}

	//return false;

	bool bFound = false;
	list<MSLNode*>::iterator it;

	forall( it, nodes )
	{
		if( *it == pNode )
		{
			bFound = true;
			break;
		}
	}
	return bFound;
}

void MSLTree::ResetSubTreeCost(MSLNode * node)
{
	double dCost, dParentCost;
	dParentCost = node->Parent() ? node->Parent()->Cost() : 0.0;
	dCost = dParentCost + node->EdgeCost();
	node->SetCost(dCost);

	list<MSLNode*> children = node->Children();
	list<MSLNode*>::iterator it;
	for( it = children.begin(); it != children.end(); it++ )
	{	
		ResetSubTreeCost( *it );
	}
}

void MSLTree::Addnode(MSLNode *node)
{
	nodes.push_back(node);
}
void MSLTree::SizeAddOne()
{
	size++;
}


void MSLTree::Merge( MSLNode* pNode1, MSLTree* pTree2, MSLNode* pNode2 )
{
	// 1.��pTree2���нڵ���ӵ�nodes����Ӧ��m_pMAG��
	list<MSLNode*> lstNodes;
	list<MSLNode*>::iterator it;
	lstNodes = pTree2->Nodes();
	forall( it, lstNodes )
	{
		(*it)->SetID( size );
		nodes.push_back( *it );
#ifdef USE_ANN
		AddPoint2MultiANN( (*it)->State() );
#endif
		size++;
	}

	// 2.��pTree2�ҽӵ�����
	// 2.1 ����
	MSLNode *pNode, *pNewParent, *pOldParent;
	double dNewEdgeCost, dOldEdgeCost;
	pNode = pNode2;
	pNewParent = pNode1;
	pOldParent = pNode2->Parent();
	dNewEdgeCost = 0.0;	// �ٶ�pNode1��pNode2�㹻��
	dOldEdgeCost = pNode2->EdgeCost();
	while( pNode )
	{
		// �ȴӾ���ժ����
		if( pOldParent )
		{
			pOldParent->DetachChild( pNode );
		}
		// �ҵ�������
		pNewParent->AddChild( pNode );
		// ���±ߵĴ���
		pNode->SetEdgeCost( dNewEdgeCost );

		// ��һ��
		pNewParent = pNode;
		pNode = pOldParent;
		pOldParent = pOldParent ? pOldParent->Parent() : NULL;
		dNewEdgeCost = dOldEdgeCost;
		dOldEdgeCost = pNode ? pNode->EdgeCost() : 0.0;
	}

	// 2.2 ������������
	ResetSubTreeCost( pNode2 );

	// ���pTree2
	pTree2->Nodes().clear();
	pTree2->root = NULL;

#ifdef USE_ANN
	if( pTree2->m_pMAG )
	{
		delete pTree2->m_pMAG;
	}
#endif
}