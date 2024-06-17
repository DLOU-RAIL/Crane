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
  // 林远山于2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- 林远山于2011.6.30
}


MSLNode::MSLNode(void* pninfo) { 
  info = pninfo;

  // 林远山于2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- 林远山于2011.6.30
}


MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u) {
  state = x;
  input = u;
  parent = pn;
  time = 1.0; // Make up a default
  cost = 0.0;

  // 林远山于2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- 林远山于2011.6.30
}


MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u, double t) {
  state = x;
  input = u;
  parent = pn;
  time = t;
  cost = 0.0;

  // 林远山于2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- 林远山于2011.6.30
}

MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u, double t, void* pninfo) {
  state = x;
  input = u;
  parent = pn;
  time = t;
  cost = 0.0;

  info = pninfo;

  // 林远山于2011.6.30
  m_dRadius = INFINITY;
  m_dEdgeCost = 0;
  children.clear();
  //--- 林远山于2011.6.30
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

// 林远山于2011.6.5
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
// ---林远山于2011.6.5

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
	ANNpoint ANNpt;				// 查询点
	ANNpoint ANNbestDist;		// 最优邻居与查询点的距离值
	int *pANNbestIdx;			// 最优邻居的索引
	void **ppANNbestList;		// 最优邻居的指针
	vector<MSLNode*> vNodes;	// 本函数返回值
	MSLNode* pNearNode = NULL;	// 临时变量

	// 查询点
	ANNpt = new double[nDim];
	for( int i = 0; i < nDim; ++i )
	{
		ANNpt[i] = x[i];
	}
	// 为返回结果分配内存空间
	ANNbestDist = new double[m_pMAG->NumNeighbors];
	pANNbestIdx = new int[m_pMAG->NumNeighbors];
	ppANNbestList = (void**)new double*[m_pMAG->NumNeighbors];

	m_pMAG->NearestNeighbor( ANNpt, ANNbestDist, pANNbestIdx, ppANNbestList );
	// 在m_pMAG->NearestNeighbor中，ANNbestDist被初始化为无穷大(1.0e40)；pANNbestIdx被初始化为0.
	// 返回的ANNbestDist, pANNbestIdx已经根据距离从小到达排序
	const double dBigDouble = 1.0e30;
	for( int i = 0; i < m_pMAG->NumNeighbors; ++i )
	{
		if( ANNbestDist[i] > dBigDouble )		// 已经到达尾部，最优邻居数量小于最大数量
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

// 林远山于2011.6.5
void MSLTree::ClearByCost( MSLNode* pTree, double dCost )
{
	if( pTree->Cost() > dCost )		// 满足条件则删除子树
	{
		RemoveSubTree( pTree );
	}
	else			// 不满足删除条件，则向子树深入检测
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

	// 删除本节点
	// 与父节点脱离关系
	MSLNode* pParent = pSubTree->Parent();
	if( pParent )
	{
		pParent->DetachChild( pSubTree );
	}
	// 从nodes列表中删除节点
	nodes.remove( pSubTree );
	// 删除MPNN中对应的点
#ifdef USE_ANN
	RemovePointFromMultiANN( pSubTree->State() );
#endif
	// 删除本节点
	delete pSubTree;
}
// ------林远山于2011.6.5

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
	// 1.把pTree2所有节点添加到nodes及对应的m_pMAG中
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

	// 2.将pTree2挂接到此树
	// 2.1 逆序
	MSLNode *pNode, *pNewParent, *pOldParent;
	double dNewEdgeCost, dOldEdgeCost;
	pNode = pNode2;
	pNewParent = pNode1;
	pOldParent = pNode2->Parent();
	dNewEdgeCost = 0.0;	// 假定pNode1和pNode2足够近
	dOldEdgeCost = pNode2->EdgeCost();
	while( pNode )
	{
		// 先从旧树摘下来
		if( pOldParent )
		{
			pOldParent->DetachChild( pNode );
		}
		// 挂到新树上
		pNewParent->AddChild( pNode );
		// 更新边的代价
		pNode->SetEdgeCost( dNewEdgeCost );

		// 下一步
		pNewParent = pNode;
		pNode = pOldParent;
		pOldParent = pOldParent ? pOldParent->Parent() : NULL;
		dNewEdgeCost = dOldEdgeCost;
		dOldEdgeCost = pNode ? pNode->EdgeCost() : 0.0;
	}

	// 2.2 更新子树代价
	ResetSubTreeCost( pNode2 );

	// 清除pTree2
	pTree2->Nodes().clear();
	pTree2->root = NULL;

#ifdef USE_ANN
	if( pTree2->m_pMAG )
	{
		delete pTree2->m_pMAG;
	}
#endif
}