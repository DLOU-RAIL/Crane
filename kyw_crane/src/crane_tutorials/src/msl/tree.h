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


#ifndef MSL_TREE_H
#define MSL_TREE_H

#include <list>
#include <string>
using namespace std;


#include "vector.h"
#include "mslio.h"

#ifdef USE_ANN
#include "ANN.h"			// ANN declarations
#include "nn.h"
#include "multiann.h"
#endif

class MSLTree;

class MSLNode {
private:
	MSLVector state;
	MSLVector input;
	MSLNode* parent;
	list<MSLNode*> children;
	double time;
	double cost;
	double edgecost;//边的代价 子节点相对于父节点的代价（金辉加）
	 int NodeSign;//姿态点所对应的离散点序号（金辉加）
	int id;

	void* info;

public:

	// 林远山于2011.6.30
	double m_dRadius;
	double m_dEdgeCost;
	//--- 林远山于2011.6.30

	//! The state to which this node corresponds
	MSLVector State() const {return state; };

	//! The input vector that leads to this state from the parent
	inline MSLVector Input() const {return input; };

	inline MSLNode* Parent() {return parent; };
	inline list<MSLNode*> const Children() {return children; };

	//! The time required to reach this node from the parent
	inline double Time() const {return time; };

	//! A cost value, useful in some algorithms
	inline double Cost() const {return cost; };
	inline double EdgeCost() const {return edgecost; }; //金辉加
	inline int GetNodeSign() const {return NodeSign; }; //金辉加
	//! A cost value, useful in some algorithms
	inline void SetCost(const double &x) {cost = x; };

	//! A cost value, useful in some algorithms（金辉加）
	inline void SetEdgeCost(const double &x) { edgecost = x; };

	//! Change the node ID
	inline void SetID(const int &i) {id = i; };

	//! Get the node ID
	inline int ID() const {return id; };

	//! Get the information 
	void* GetInfo() {return info; };

	//! Set the information
	void SetInfo(void* in) {info = in; };

	//! Clear the memory for the information
	//void ClearInfo() {if (!info) delete (MSLNodeInfo *) info; };
	//NOTE: Above incorrect design - shouldn't type the void* here

	MSLNode();
	MSLNode(void* pninfo);
	MSLNode(MSLNode* pn, const MSLVector &x, const MSLVector &u);
	MSLNode(MSLNode* pn, const MSLVector &x, const MSLVector &u, double t);
	MSLNode(MSLNode* pn, const MSLVector &x, const MSLVector &u, double t, void* pninfo);
	~MSLNode() { children.clear(); /*ClearInfo();*/ };

	inline void AddChild(MSLNode *cn) { children.push_back(cn);cn->SetParent( this ); }

	// 林远山添加（2011.4.25）
	void DetachChild( MSLNode *child )
	{
		list<MSLNode*>::iterator it;
		for( it = children.begin(); it != children.end(); it++ )
		{
			if( child == *it )
			{
				children.erase( it );
				child->SetParent( NULL );
				break;
			}
		}
	}

	void SetParent( MSLNode *newParent )
	{
		parent = newParent;
	}

	void SetInputandTime( MSLVector kInput, double dTime )
	{
		this->input = kInput;
		this->time = dTime;
	}
	// --------------林远山添加（2011.4.25）

	//friend istream& operator>> (istream& is, MSLNode& n);
	friend ostream& operator<< (ostream& os, const MSLNode& n);
	//friend istream& operator>> (istream& is, list<MSLNode*> & nl);
	friend ostream& operator<< (ostream& os, const list<MSLNode*> & nl);

	friend class MSLTree;
};


//! This is a comparison object to be used for STL-based sorting
class MSLNodeLess {
public:
	bool operator() (MSLNode* p, MSLNode* q) const {
		return p->Cost() < q->Cost();
	}
};


//! This is a comparison object to be used for STL-based sorting
class MSLNodeGreater {
public:
	bool operator() (MSLNode* p, MSLNode* q) const {
		return p->Cost() > q->Cost();
	}
};


class MSLTree {
private:
	list<MSLNode*> nodes;
	MSLNode* root;
	int size;


public:
	bool Contain( MSLNode* pSubTree, MSLNode* pNode );

	// 林远山2011.4.28。使用MPNN
#ifdef USE_ANN
	MultiANN *m_pMAG;
	void AddPoint2MultiANN( const MSLVector &x );
	MSLTree(const MSLVector &x, int *pTopology, double *pScaling ); // Argument is state of root node
	MSLNode* NearestNode( const MSLVector &x, double &dBestDist );
	MSLNode* NearestNode( const MSLVector &x, double &dBestDist, double *pScaling );
	vector< MSLNode* > kNearestNodes( const MSLVector &x, vector< double > &vBestDist );
#endif
	// ----------------林远山2011.04.28

	// 林远山于2011.6.5
	// 删除树中代价大于dCost的所有节点
	void ClearByCost( MSLNode* pTree, double dCost );
	// 删除以pSubTree为树根的子树
	void RemoveSubTree( MSLNode* pSubTree );

#ifdef USE_ANN
	// 从MPNN中删除点x
	void RemovePointFromMultiANN( const MSLVector &x );
#endif
	// ---林远山于2011.6.5

	MSLTree();
	MSLTree(const MSLVector &x); // Argument is state of root node
	MSLTree(const MSLVector &x, void* nodeinfo);
	~MSLTree();

	void MakeRoot(const MSLVector &x);
	MSLNode* Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u);
	MSLNode* Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u, 
		double time);
	MSLNode* Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u, 
		double time, void* pninfo);

	void Addnode(MSLNode *node); //金辉加 加入节点
	void SizeAddOne();//金辉加  size+1
	// 林远山添加（2011.4.25）
	void RemoveEdge( MSLNode *parent, MSLNode *child );
	void Merge( MSLNode* pNode1, MSLTree* pTree2, MSLNode* pNode2 );

	list<MSLNode*> PathToRoot(MSLNode *n);
	MSLNode* FindNode(int nid);
	inline list<MSLNode*> Nodes() const { return nodes; };
	inline MSLNode* Root() {return root; };
	inline int Size() {return size;}

	void Clear();

	friend istream& operator>> (istream& is, MSLTree& n);
	friend ostream& operator<< (ostream& os, const MSLTree& n);
	 void ResetSubTreeCost(MSLNode * node);//金辉加 重设以node为树根的子树的代价
};

#endif

