#pragma once
#include "rrt.h"

class RRTConnectPlusPlus: public RRTDual {
 public:
  RRTConnectPlusPlus(Problem *p);
  virtual ~RRTConnectPlusPlus() {};

  int m_nExtendBack;
  double m_dD;
  int nFarFromNode;
  int nLineNode;

  // T树的采样池，装T树中能成为潜在采样点的节点
  vector< MSLVector > m_kCandidateSamps;
  // T2树的采样池，装T2树中能成为潜在采样点的节点
  vector< MSLVector > m_kCandidateSamps2;
  int m_iPoolSize;
  int m_iCandidateSampsIdx;
  int m_iCandidateSampsIdx2;

  //! Pick a state using some sampling technique
  virtual MSLVector ChooseState();
  MSLVector ChooseState( MSLTree *pTree );
  MSLVector ChooseFreeColState();

  //! The greediest of the dual-tree planners.  
  //! Very fast for holonomic planning.
  virtual bool Plan();
  //! Incrementally extend the RRT
  virtual bool Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	  bool forward = true);

  //! Iterated Extend
  virtual bool Connect(const MSLVector &x, MSLTree *t, MSLNode *&nn,
	  bool forward = true);
  //// 林远山2011.9.28，采用hRRT选择节点策略
  //double m_dMaxCost;
  //double m_dMinCost;
  //double m_dFloorQ;
  //MSLNode *m_pNewNodeT, *m_pNewNodeT2;
  //MSLNode* SelectGoodNode( MSLTree* t, bool forward, MSLVector &Sample );
};