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
	double edgecost;//�ߵĴ��� �ӽڵ�����ڸ��ڵ�Ĵ��ۣ���Լӣ�
	 int NodeSign;//��̬������Ӧ����ɢ����ţ���Լӣ�
	int id;

	void* info;

public:

	// ��Զɽ��2011.6.30
	double m_dRadius;
	double m_dEdgeCost;
	//--- ��Զɽ��2011.6.30

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
	inline double EdgeCost() const {return edgecost; }; //��Լ�
	inline int GetNodeSign() const {return NodeSign; }; //��Լ�
	//! A cost value, useful in some algorithms
	inline void SetCost(const double &x) {cost = x; };

	//! A cost value, useful in some algorithms����Լӣ�
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

	// ��Զɽ��ӣ�2011.4.25��
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
	// --------------��Զɽ��ӣ�2011.4.25��

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

	// ��Զɽ2011.4.28��ʹ��MPNN
#ifdef USE_ANN
	MultiANN *m_pMAG;
	void AddPoint2MultiANN( const MSLVector &x );
	MSLTree(const MSLVector &x, int *pTopology, double *pScaling ); // Argument is state of root node
	MSLNode* NearestNode( const MSLVector &x, double &dBestDist );
	MSLNode* NearestNode( const MSLVector &x, double &dBestDist, double *pScaling );
	vector< MSLNode* > kNearestNodes( const MSLVector &x, vector< double > &vBestDist );
#endif
	// ----------------��Զɽ2011.04.28

	// ��Զɽ��2011.6.5
	// ɾ�����д��۴���dCost�����нڵ�
	void ClearByCost( MSLNode* pTree, double dCost );
	// ɾ����pSubTreeΪ����������
	void RemoveSubTree( MSLNode* pSubTree );

#ifdef USE_ANN
	// ��MPNN��ɾ����x
	void RemovePointFromMultiANN( const MSLVector &x );
#endif
	// ---��Զɽ��2011.6.5

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

	void Addnode(MSLNode *node); //��Լ� ����ڵ�
	void SizeAddOne();//��Լ�  size+1
	// ��Զɽ��ӣ�2011.4.25��
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
	 void ResetSubTreeCost(MSLNode * node);//��Լ� ������nodeΪ�����������Ĵ���
};

#endif

