
#ifndef MSL_PROBLEM_H
#define MSL_PROBLEM_H

//#include <list.h>
//#include <string>

#include "util.h"
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> 
#include "vector.h"
#include "../crane_location_regions.h"

//! An interface class that provides the primary input to a planner.

/*! This interface class contains protected instances of Geom and
Model.  Wrappers to methods from Geom provide collision detection and
distance computation.  Wrappers to methods from Model provide
incremental simulation of a kinematic or dynamical system.  It is
expected a planner can get all (or nearly all) of the information it
needs from Problem.
*/

class Problem {

 public:
  CraneLocationRegions* m_pPickCLR;
  CraneLocationRegions* m_pPlaceCLR;

	 // ��Զɽ��2011.6.5���?
	 // �ѵõ���·���е���С����
	 double m_dMinCost;
	 double m_dCumulativeCollisionTime;
	 double m_dMetricTime;
	 //--��Զɽ��2011.6.5���?

	 // ��Զɽ��2011.7.18
	 int m_iCollisionNum;
	 int m_iMetricNum;
	 // --��Զɽ��2011.7.18

  moveit::planning_interface::MoveGroupInterface* whole_group_;
  const robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr rs_;
  planning_scene::PlanningScenePtr scene_;
  std::string m_strGroupName;

  //! The directory in which all files for a problem will be stored
  string FilePath;

  //! The time interval to use for numerical integration (affects accuracy)
  double ModelDeltaT;

  //! The dimenson of the state space
  int StateDim;

  //! The dimension of the input space
  int InputDim;


  //! MSLVector of minimum values for each state variable
  MSLVector LowerState;

  //! MSLVector of maximum values for each state variable
  MSLVector UpperState;

  //! The starting state for a planner
  MSLVector InitialState;

  //! The goal state for a planner
  MSLVector GoalState;

  //! Problem must be given any instance of Geom and any instance of
  //! Model from each of their class hierarchies
  void Init(std::string strGroupName);
  Problem(std::string strGroupName, CraneLocationRegions* pPickCLR, CraneLocationRegions* pPlaceCLR);
  Problem(std::string strGroupName, MSLVector init, MSLVector goal);

  //! Empty destructor
  virtual ~Problem() {};

  //! Return a list of possible inputs, which may depend on state
  virtual list<MSLVector> GetInputs(const MSLVector &x);

  //! Return a list of possible inputs
  virtual list<MSLVector> GetInputs();

  //! The state transition equation, or equations of motion, xdot=f(x,u)
  MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);

  //! Perform integration from state x, using input u, over time deltat
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, 
			   const double &deltat);

  //! Linearly interpolate two states while respecting topology.
  /*! If a=0, then x1 is returned; if a=1, then x2 is returned.  All
      intermediate values of $a \in [0,1]$ yield intermediate states.
      This method is defined by Model.
  */
  virtual MSLVector InterpolateState(const MSLVector &x1, const MSLVector &x2, 
				   const double &a);  // Depends on topology

  //! A distance metric defined in Model.
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);

  //! Compute a MSLVector based on x2-x1.  In R^n, the states are simply
  //! subtracted to make the MSLVector.  This method exists to make things
  //! work correctly for other state-space topologies.
  virtual MSLVector StateDifference(const MSLVector &x1, const MSLVector &x2); 

  //! This takes the logical AND of CollisionFree from Geom, and Satisfied from
  //!  Model.
  virtual bool Satisfied(const MSLVector &x);

  bool CollisionFree(const MSLVector &x);

  //! ��Զɽ2011.4.30
  virtual bool CollisionFreeLine( const MSLVector &x1, const MSLVector &x2 );

  //! Compute a MSLVector based on q2-q1.  In R^n, the configurations are simply
  //! subtracted to make the MSLVector.  This method exists to make things
  //! work correctly for other configuration-space topologies.
  virtual MSLVector ConfigurationDifference(const MSLVector &q1, const MSLVector &q2); 

  MSLVector GetInputFromStates( const MSLVector &x1, const MSLVector &x2 );
  MSLVector Round( MSLVector kState );
  MSLVector SamplingFromPickCLR();
  MSLVector SamplingFromPlaceCLR();
};

#endif


