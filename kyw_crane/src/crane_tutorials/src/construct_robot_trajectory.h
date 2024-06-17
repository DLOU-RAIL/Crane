#include "msl/vector.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

class ConstructRobotTrajectory
{
public:
    robot_trajectory::RobotTrajectoryPtr robot_traj_;

    robot_model::RobotModelConstPtr  kmodel_;
    const robot_model::JointModelGroup* jmg_;
    std::string  groupName_;
    robot_state::RobotStatePtr rs_;

    ConstructRobotTrajectory(const robot_model::RobotModelConstPtr& kmodel, const std::string& group );
    ~ConstructRobotTrajectory(){};
    void convertPath(const std::list<MSLVector>& path, robot_trajectory::RobotTrajectory& traj) const;
};


ConstructRobotTrajectory::ConstructRobotTrajectory(const robot_model::RobotModelConstPtr& kmodel, const std::string& groupName )
{
    kmodel_ = kmodel;
    groupName_ = groupName;
    jmg_ = kmodel_->getJointModelGroup( groupName_ );
    rs_.reset(new robot_state::RobotState(kmodel_));
    robot_traj_.reset(new robot_trajectory::RobotTrajectory(kmodel_, groupName_));
}

void ConstructRobotTrajectory::convertPath(const std::list<MSLVector>& path, robot_trajectory::RobotTrajectory& traj) const
{
    std::vector<double> state(8, 0.0);
    std::list<MSLVector>::const_iterator iter;
    MSLVector s;
    for(iter = path.begin(); iter != path.end() ;iter++)
    {
        s = *iter;
        s.toSTDVector(state);
        state.insert(state.begin()+5, -state[4]);
        rs_->setJointGroupPositions(jmg_, state);
        for(int i = 0; i < 8; i++)
        {
            cout << "  " << state[i];
        }
        cout << endl;
        traj.addSuffixWayPoint(*rs_, 0.0);
        robot_traj_->addSuffixWayPoint(*rs_, 0.0);
    }
    
}