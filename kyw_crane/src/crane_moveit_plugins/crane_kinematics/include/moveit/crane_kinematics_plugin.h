#ifndef MOVEIT_CRANE_KINEMATICS_PLUGIN_H
#define MOVEIT_CRANE_KINEMATICS_PLUGIN_H

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
//#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>


// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace crane_kinematics
{
/**
 * @brief Specific implementation of kinematics.
 */
  const double PI = 3.1415926;

  class CraneKinematicsPlugin : public kinematics::KinematicsBase
  {
    public:

    /**
     *  @brief Default constructor
     */
    CraneKinematicsPlugin();

    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool getPositionFK(const std::vector<std::string> &link_names,
                               const std::vector<double> &joint_angles,
                               std::vector<geometry_msgs::Pose> &poses) const;

    virtual bool initialize(const std::string &robot_description,
                            const std::string &group_name,
                            const std::string &base_name,
                            const std::string &tip_name,
                            double search_discretization);

    /**
     * @brief  Return all the joint names in the order they are used internally
     */
    const std::vector<std::string>& getJointNames() const;

    /**
     * @brief  Return all the link names in the order they are represented internally
     */
    const std::vector<std::string>& getLinkNames() const;

  protected:

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param check_consistency Set to true if consistency check needs to be performed
   * @param redundancy The index of the redundant joint
   * @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   * @return True if a valid solution was found, false otherwise
   */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const std::vector<double> &consistency_limits,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    // 起重机逆向运动学伪解析求解
    bool solveIK( const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &ik_out,
        const kinematics::KinematicsQueryOptions &options) const;

    /**
   * @brief Set a set of redundant joints for the kinematics solver to use.
   * This can fail, depending on the IK solver and choice of redundant joints!
   * @param redundant_joint_indices The set of redundant joint indices (corresponding to
   * the list of joints you get from getJointNames()).
   * @return False if any of the input joint indices are invalid (exceed number of
   * joints)
   */
  virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices);

  /**
   * @brief Set a set of redundant joints for the kinematics solver to use.
   * This function is just a convenience function that calls the previous definition of setRedundantJoints()
   * @param redundant_joint_names The set of redundant joint names.
   * @return False if any of the input joint indices are invalid (exceed number of
   * joints)
   */
  bool setRedundantJoints(const std::vector<std::string> &redundant_joint_names);

  /**
   * \brief Check if this solver supports a given JointModelGroup.
   *
   * Override this function to check if your kinematics solver
   * implementation supports the given group.
   *
   * The default implementation just returns jmg->isChain(), since
   * solvers written before this function was added all supported only
   * chain groups.
   *
   * \param jmg the planning group being proposed to be solved by this IK solver
   * \param error_text_out If this pointer is non-null and the group is
   *          not supported, this is filled with a description of why it's not
   *          supported.
   * \return True if the group is supported, false if not.
   */
  virtual bool supportsGroup(const moveit::core::JointModelGroup *jmg,
                                   std::string* error_text_out = NULL) const;


  private:
    bool active_; /** Internal variable that indicates whether solvers are configured and ready */

    unsigned int dimension_; /** Dimension of the group */

    mutable random_numbers::RandomNumberGenerator random_number_generator_;

    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;
    robot_model::JointModelGroup* joint_model_group_;
    
    // 起重机相关常量
    double boomJointOffsetX_;  // 臂架铰点距回转中心距离
    double boomJointOffsetZ_;  // 臂架铰点离地高度
    double boomLength_;  // 臂架长
    double ropeJointOffsetZ_;  // 起升滑轮组距离臂架中心线距离，中心线以下为负值
    double hookHeight_;  // 吊钩总高（吊钩块+吊钩）

    // 起重机作业时的相关变量
    double liftedObjectWeight_;  // 被吊物重量
    double workingRadiusMin_;  // 最小作业半径
    double workingRadiusMax_;  // 最大作业半径
  };
}

#endif
