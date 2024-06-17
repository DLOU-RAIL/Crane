#include <moveit/crane_kinematics_plugin.h>
#include <class_loader/class_loader.h>

//#include <tf/transform_datatypes.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

//#include <Eigen/geometry>
#include <eigen_conversions/eigen_msg.h>

#include <pluginlib/class_list_macros.h>

//register CraneKinematics as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(crane_kinematics::CraneKinematicsPlugin, kinematics::KinematicsBase);

namespace crane_kinematics
{
    void printVectorD( const std::string &header, std::vector< double > vec )
    {
        ROS_INFO( "   %s", header.c_str());
        for( size_t i = 0; i < vec.size(); ++i )
        {
            ROS_INFO( "     %.4f",vec[i] );
        }
    }

    double boundAngle( double angle )
    {
        //std::assert( angle >= -0.0001 && angle <= 2 * PI + 0.0001 );
        while( angle > PI )
            angle -= 2 * PI;
        while( angle < -PI )
            angle += 2 * PI;
        return angle;
    }

    CraneKinematicsPlugin::CraneKinematicsPlugin():active_(false) {}


    bool CraneKinematicsPlugin::initialize(const std::string &robot_description,
        const std::string& group_name,
        const std::string& base_frame,
        const std::string& tip_frame,
        double search_discretization)
    {
        // 初始化起重机相关常量及工作时的变量，这些变量实际上应该从参数服务器获取
        boomJointOffsetX_ = 1.5;
        boomJointOffsetZ_ = 2.59;
        boomLength_ = 56.0;
        ropeJointOffsetZ_ = -1.66;
        hookHeight_ = 3.0;
        liftedObjectWeight_ = 50.0;
        workingRadiusMin_ = 10.0;  // 实际上应该根据重物的重量及起重机的起重性能求得
        workingRadiusMax_ = 28.0;  // 实际上应该根据重物的重量及起重机的起重性能求得

        setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

        ros::NodeHandle private_handle("~");
        rdf_loader::RDFLoader rdf_loader(robot_description_);
        const std::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
        const std::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

        if (!urdf_model || !srdf)
        {
            ROS_ERROR_NAMED("crane","URDF and SRDF must be loaded for KDL kinematics solver to work.");
            return false;
        }

        robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

        robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        if (!joint_model_group)
        return false;
  
        dimension_ = 8;   // 包含其中的mimic joint，即起升绳竖直那个关节

        if(!joint_model_group->hasLinkModel(getTipFrame()))
        {
            ROS_ERROR_NAMED("crane","Could not find tip name in joint group '%s'", group_name.c_str());
            return false;
        }

        // Setup the joint state groups that we need
        robot_state_.reset(new robot_state::RobotState(robot_model_));

        // Store things for when the set of redundant joints may change
        joint_model_group_ = joint_model_group;

        // 暂时强行将world_joint设置为冗余关节
        std::vector< unsigned int > red_joints(1);
        red_joints[0] = 0;
        setRedundantJoints( red_joints );

        active_ = true;
        ROS_DEBUG_NAMED("Crane","KDL solver initialized");
        return true;
    }


    bool CraneKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        std::vector<double> consistency_limits;

        return searchPositionIK(ik_pose,
            ik_seed_state,
            default_timeout_,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
    }

    bool CraneKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        std::vector<double> consistency_limits;

        return searchPositionIK(ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
    }

    bool CraneKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        return searchPositionIK(ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
    }

    bool CraneKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        std::vector<double> consistency_limits;
        return searchPositionIK(ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
    }

    bool CraneKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        return searchPositionIK(ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
    }


    // 进一步改进建议：
    // 1）根据当前状态或ik_seed_state控制起重机下车的位置，让下车尽可能少动或动的距离短，比如直接取ik_seed_state的下车位姿作为解的值（若可能）；
    // 2）根据consistency_limits控制起重机状态变化的连续性，在末端位姿轨迹给定的路径规划中很有用；
    // 3）利用kinematics::KinematicsQueryOptions控制求解过程，是否锁定冗余自由度（可考虑利用冗余自由度辅助求解），是否返回近似解。
    bool CraneKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const std::vector<double> &consistency_limits,
        const kinematics::KinematicsQueryOptions &options) const
    {
          
        ros::WallTime n1 = ros::WallTime::now();
        if(!active_)
        {
            ROS_ERROR_NAMED("crane","kinematics not active");
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        if(ik_seed_state.size() != dimension_)
        {
            ROS_ERROR_STREAM_NAMED("crane","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
        {
            ROS_ERROR_STREAM_NAMED("crane","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        // 显示IK请求的信息
        ROS_DEBUG_STREAM_NAMED("crane","searchPositionIK2: Position request pose is " <<
            ik_pose.position.x << " " <<
            ik_pose.position.y << " " <<
            ik_pose.position.z << " " <<
            ik_pose.orientation.x << " " <<
            ik_pose.orientation.y << " " <<
            ik_pose.orientation.z << " " <<
            ik_pose.orientation.w);
  
        // 开始求解IK
        solution.resize(dimension_);
        std::vector< double > ik_out;
        bool isValid;
        //printVectorD( std::string("IN SOLVER, SEED_STATE:"), ik_seed_state );
        isValid = solveIK( ik_pose, ik_seed_state, ik_out, options );
        //printVectorD( std::string("IN SOLVER, IK_SOLUTION_STATE:"), ik_out );
        if( !isValid )
        {
            error_code.val = error_code.NO_IK_SOLUTION;
            ROS_DEBUG_NAMED( "crane", "Can not find IK!" );
            return false;
        }

        ROS_DEBUG_NAMED("crane","Found IK solution");
        for(unsigned int j=0; j < dimension_; j++)
        {
            solution[j] = ik_out[j];
            //std::cout << solution[j] << std::endl;
        }

        // 对此IK解进行进一步校验，比如用OMPL的状态空间StateValidityChecker检测
        if(!solution_callback.empty())
        solution_callback(ik_pose,solution,error_code);
        else
        error_code.val = error_code.SUCCESS;

        return true;
    }

    bool CraneKinematicsPlugin::solveIK( const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &ik_out,
        const kinematics::KinematicsQueryOptions &options) const
    {
        ik_out.resize( dimension_ );
        double r, theta, alpha;
        if( options.lock_redundant_joints && 0 == redundant_joint_indices_[0])
        {
            r = sqrt( pow((ik_seed_state[0] - ik_pose.position.x),2) + pow((ik_seed_state[1] - ik_pose.position.y),2) );
            theta = atan2(ik_seed_state[1] - ik_pose.position.y, ik_seed_state[0] - ik_pose.position.x);
            alpha = ik_seed_state[2];
            
            ik_out[0] = ik_seed_state[0];
            ik_out[1] = ik_seed_state[1];
            ik_out[2] = ik_seed_state[2];
        }
        else
        {
            r = random_number_generator_.uniformReal( workingRadiusMin_, workingRadiusMax_ );
            theta = random_number_generator_.uniformReal( -PI, PI );
            alpha = random_number_generator_.uniformReal( -PI, PI );

            // 在此应该校验起重机位置是否在规划的范围内，若不在则重新采样计算新的位置
            ik_out[0] = ik_pose.position.x + r * cos( theta );  // 起重机位置x
            ik_out[1] = ik_pose.position.y + r * sin( theta );  // 起重机位置y
            ik_out[2] = alpha;  // 起重机下车方向角
        }
  
        // 求解转台回转角
        Eigen::Vector3d vecZ( 0, 0, 1 );
        Eigen::Vector3d vecBottom( cos(alpha), sin(alpha), 0 );
        Eigen::Vector3d vecBoom( -cos(theta), -sin(theta), 0 );
        double beta = acos( vecBottom.dot( vecBoom ) / sqrt( vecBottom.dot( vecBottom ) * vecBoom.dot( vecBoom ) ) );
        double sign = vecZ.dot( vecBottom.cross( vecBoom ) );
        sign = sign < 0.0 ? -1 : 1;
        ik_out[3] = sign * beta;
        ik_out[3] = boundAngle( ik_out[3] );

        // 求臂架仰角: gama = acos( (r-Bx) / sqrt( L*L + d*d ) )
        ik_out[4] = acos( ( r - boomJointOffsetX_ ) / sqrt( boomLength_ * boomLength_ + ropeJointOffsetZ_ * ropeJointOffsetZ_ ) )
                      - atan( ropeJointOffsetZ_ / boomLength_ );

        // 确保起升绳竖直
        ik_out[5] = -ik_out[4];

        // 求起升绳长: h = Bz + sqrt( (L*L + d*d) - (r-Bx)*(r-Bx) ) - y_L - h_hook
        ik_out[6] = boomJointOffsetZ_ + sqrt( ( boomLength_*boomLength_ + ropeJointOffsetZ_*ropeJointOffsetZ_ ) - ( r-boomJointOffsetX_ ) * ( r-boomJointOffsetX_ ) )
                      - ik_pose.position.z - hookHeight_;

        // 求吊钩旋转角
        ik_out[7] = ik_pose.orientation.z - ( alpha + beta );
        ik_out[7] = boundAngle( ik_out[7] );

        return true;
    }


    bool CraneKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
        const std::vector<double> &joint_angles,
        std::vector<geometry_msgs::Pose> &poses) const
    {
  
        ros::WallTime n1 = ros::WallTime::now();
        if(!active_)
        {
            ROS_ERROR_NAMED("crane","kinematics not active");
            return false;
        }
        poses.resize(link_names.size());
        if(joint_angles.size() != dimension_)
        {
            ROS_ERROR_NAMED("crane","Joint angles vector must have size: %d",dimension_);
            return false;
        }

        robot_state_->setJointGroupPositions( joint_model_group_, joint_angles );
        robot_state_->update( true );
        std::size_t count = link_names.size();
        for( std::size_t i = 0; i < count; ++i )
        {
            Eigen::Affine3d trans = robot_state_->getGlobalLinkTransform( link_names[i] );
            tf::poseEigenToMsg( trans, poses[i] );
        }

        return true;
    }

    const std::vector<std::string>& CraneKinematicsPlugin::getJointNames() const
    {
        return joint_model_group_->getJointModelNames();
    }

    const std::vector<std::string>& CraneKinematicsPlugin::getLinkNames() const
    {
        return joint_model_group_->getLinkModelNames();
    }

    bool CraneKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices) 
    {
        for(std::size_t i = 0; i < redundant_joint_indices.size(); ++i)
        {
            if(redundant_joint_indices[i] >= getJointNames().size())
            {
                return false;
            }
        }
        redundant_joint_indices_ = redundant_joint_indices;
        return true;
    }

    bool CraneKinematicsPlugin::setRedundantJoints(const std::vector<std::string> &redundant_joint_names) 
    {
        const std::vector<std::string> &jnames = getJointNames();
        std::vector<unsigned int> redundant_joint_indices;
        for (std::size_t i = 0 ; i < redundant_joint_names.size() ; ++i)
        for (std::size_t j = 0 ; j < jnames.size() ; ++j)
        if (jnames[j] == redundant_joint_names[i])
        {
            redundant_joint_indices.push_back(j);
            break;
        }
        return redundant_joint_indices.size() == redundant_joint_names.size() ? setRedundantJoints(redundant_joint_indices) : false;
    }

    bool CraneKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
        std::string* error_text_out) const
    {
        // // Default implementation for legacy solvers:
        // if (!jmg->isChain())
        // {
        //     if(error_text_out)
        //     {
        //         *error_text_out = "This plugin only supports joint groups which are chains";
        //     }
        //     return false;
        // }

        return true;
    }

} // namespace
