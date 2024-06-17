#include <random_numbers/random_numbers.h>
#include <ros/ros.h> 
//const double M_PI = 3.1415926;

class CraneLocationRegions
{
public:
  struct LiftedObjectPose
  {
    double dX, dY, dZ;       // 被吊物的位置
    double dFai;             // 被吊物的水平方向角
  };

  struct AnnularSector
  {
    double dMinR;             // 扇环内径
    double dMaxR;             // 扇环外径
    double dMinTheta;         // 扇环起始角
    double dMaxTheta;         // 扇环终止角
    double dMinAlpha;         // 小车方向起始角
    double dMaxAlpha;         // 小车方向终止角
  };

  LiftedObjectPose liftObjPose_;
  std::vector< AnnularSector > annularSectors_;
  
  // 起重机相关常量
  double boomJointOffsetX_;  // 臂架铰点距回转中心距离
  double boomJointOffsetZ_;  // 臂架铰点离地高度
  double boomLength_;        // 臂架长
  double ropeJointOffsetZ_;  // 起升滑轮组距离臂架中心线距离，中心线以下为负值
  double hookHeight_;        // 吊钩总高（吊钩块+吊钩）
  double workingRadiusMin_;       // 最小工作半径——[应由起重性能表和被吊物重量确定]
  double workingRadiusMax_;       // 最大工作半径——[应由起重性能表和被吊物重量确定]


  mutable random_numbers::RandomNumberGenerator rng_;

  CraneLocationRegions()
  {
    liftObjPose_.dX = liftObjPose_.dY = liftObjPose_.dZ = liftObjPose_.dFai = 0.0;

      // 初始化起重机相关常量及工作时的变量，这些变量实际上应该从参数服务器获取
      boomJointOffsetX_ = 1.5;
      boomJointOffsetZ_ = 2.59;
      boomLength_ = 56.0;
      ropeJointOffsetZ_ = -1.66;
      hookHeight_ = 3.0;
      workingRadiusMin_ = 8.0;  // 实际上应该根据重物的重量及起重机的起重性能求得
      workingRadiusMax_ = 48.0;  // 实际上应该根据重物的重量及起重机的起重性能求得
  }

  CraneLocationRegions(double dX, double dY, double dZ, double dFai, 
                       double dMinR = 8, double dMaxR = 48, double dMinTheta = -M_PI, double dMaxTheta = M_PI,
                       double dMinAlpha = -M_PI, double dMaxAlpha = M_PI)
  {
    liftObjPose_.dX = dX;
    liftObjPose_.dY = dY;
    liftObjPose_.dZ = dZ;
    liftObjPose_.dFai = dFai;
    AnnularSector as = {dMinR, dMaxR, dMinTheta, dMaxTheta, dMinAlpha, dMaxAlpha};
    annularSectors_.push_back(as);

      // 初始化起重机相关常量及工作时的变量，这些变量实际上应该从参数服务器获取
      boomJointOffsetX_ = 1.5;
      boomJointOffsetZ_ = 2.59;
      boomLength_ = 56.0;
      ropeJointOffsetZ_ = -1.66;
      hookHeight_ = 3.0;
      workingRadiusMin_ = dMinR;  // 实际上应该根据重物的重量及起重机的起重性能求得
      workingRadiusMax_ = dMaxR;  // 实际上应该根据重物的重量及起重机的起重性能求得
  }

  CraneLocationRegions(const geometry_msgs::Pose& liftObjPose,          // 只用到其中四个自由度x,y,z,orientation.z 
                       double dMinR = 8, double dMaxR = 48, double dMinTheta = -M_PI, double dMaxTheta = M_PI,
                       double dMinAlpha = -M_PI, double dMaxAlpha = M_PI)
  {
    liftObjPose_.dX = liftObjPose.position.x;
    liftObjPose_.dY = liftObjPose.position.y;
    liftObjPose_.dZ = liftObjPose.position.z;
    liftObjPose_.dFai = liftObjPose.orientation.z;
    AnnularSector as = {dMinR, dMaxR, dMinTheta, dMaxTheta, dMinAlpha, dMaxAlpha};
    annularSectors_.push_back(as);

      // 初始化起重机相关常量及工作时的变量，这些变量实际上应该从参数服务器获取
      boomJointOffsetX_ = 1.5;
      boomJointOffsetZ_ = 2.59;
      boomLength_ = 56.0;
      ropeJointOffsetZ_ = -1.66;
      hookHeight_ = 3.0;
      workingRadiusMin_ = dMinR;  // 实际上应该根据重物的重量及起重机的起重性能求得
      workingRadiusMax_ = dMaxR;  // 实际上应该根据重物的重量及起重机的起重性能求得
  }

  ~CraneLocationRegions()
  {

  }

  void initCraneParams()
  {
    // 初始化起重机相关常量及工作时的变量，这些变量实际上应该从参数服务器获取
    boomJointOffsetX_ = 1.5;
    boomJointOffsetZ_ = 2.59;
    boomLength_ = 56.0;
    ropeJointOffsetZ_ = -1.66;
    hookHeight_ = 3.0;
    workingRadiusMin_ = 8.0;  // 实际上应该根据重物的重量及起重机的起重性能求得
    workingRadiusMax_ = 48.0;  // 实际上应该根据重物的重量及起重机的起重性能求得
  }

  void setLiftedObjectPose(double dX, double dY, double dZ, double dFai)
  {
    liftObjPose_.dX = dX;
    liftObjPose_.dY = dY;
    liftObjPose_.dZ = dZ;
    liftObjPose_.dFai = dFai;
  }

  int addAnnularSector(double dMinR, double dMaxR, double dMinTheta, double dMaxTheta, double dMinAlpha, double dMaxAlpha)
  {
    AnnularSector as = {dMinR, dMaxR, dMinTheta, dMaxTheta, dMinAlpha, dMaxAlpha};
    annularSectors_.push_back(as);
    return annularSectors_.size();
  }

  int selectAnnularSector()
  {
    std::size_t size = annularSectors_.size();
    std::size_t i;
    std::vector<double> sectorAreaRatios(size,0.0);
    double sumArea = 0.0;
    for( i = 0; i < size; ++i )
    {   
        sectorAreaRatios[i] = 0.5 * ( annularSectors_[i].dMaxTheta - annularSectors_[i].dMinTheta ) * 
               ( annularSectors_[i].dMaxR * annularSectors_[i].dMaxR - annularSectors_[i].dMinR * annularSectors_[i].dMinR );
        sumArea += sectorAreaRatios[i];
        //ROS_INFO( "sumArea = %.4f", sumArea );
    }

    // 面积为0的退化扇环处理

    // 归一化
    for( i = 0; i < size; ++i )
    {
        sectorAreaRatios[i] /= sumArea;
        //ROS_INFO( "%d: Ratios = %.4f", i, sectorAreaRatios_[i] );
    }


    double p = rng_.uniformReal( 0.0, 1.0 );
    int sel = 0;
    double sum = sectorAreaRatios[0];
    while( p > sum )
    {
        sel++;
        sum += sectorAreaRatios[sel];
    }

    return sel;
  }

  double boundAngle( double angle )
  {
    //std::assert( angle >= -0.0001 && angle <= 2 * M_PI + 0.0001 );
    while( angle > M_PI )
      angle -= 2 * M_PI;
    while( angle < -M_PI )
      angle += 2 * M_PI;
    return angle;
  }

  bool directSampling(std::vector<double>& craneConf)
  {
    // 从站位扇环列表中选出一个扇环
    int which_sector = selectAnnularSector();
    // 从此扇环中随机获取下车的坐标和方向
    double r = rng_.uniformReal( annularSectors_[which_sector].dMinR, annularSectors_[which_sector].dMaxR );
    double theta = rng_.uniformReal( annularSectors_[which_sector].dMinTheta, annularSectors_[which_sector].dMaxTheta );
    double alpha = rng_.uniformReal( annularSectors_[which_sector].dMinAlpha, annularSectors_[which_sector].dMaxAlpha );
    craneConf.resize(8);        // 包含mimic joint
    craneConf[0] = liftObjPose_.dX + r * cos( theta );  // 起重机位置x
    craneConf[1] = liftObjPose_.dY + r * sin( theta );  // 起重机位置y
    craneConf[2] = alpha;  // 起重机下车方向角

    //ROS_INFO("which_sector = %d, ThetaBound = [%f, %f], Theta = %f", which_sector, annularSectors_[which_sector].dMinTheta, annularSectors_[which_sector].dMaxTheta, theta);

    // 求解转台回转角
    Eigen::Vector3d vecZ( 0, 0, 1 );
    Eigen::Vector3d vecBottom( cos(alpha), sin(alpha), 0 );
    Eigen::Vector3d vecBoom( -cos(theta), -sin(theta), 0 );
    double beta = acos( vecBottom.dot( vecBoom ) / sqrt( vecBottom.dot( vecBottom ) * vecBoom.dot( vecBoom ) ) );
    double sign = vecZ.dot( vecBottom.cross( vecBoom ) );
    sign = sign < 0.0 ? -1 : 1;
    craneConf[3] = sign * beta;
    craneConf[3] = boundAngle( craneConf[3] );

    // 求臂架仰角: gama = acos( (r-Bx) / sqrt( L*L + d*d ) )
    craneConf[4] = acos( ( r - boomJointOffsetX_ ) / sqrt( boomLength_ * boomLength_ + ropeJointOffsetZ_ * ropeJointOffsetZ_ ) )
                      - atan( ropeJointOffsetZ_ / boomLength_ );

    // 确保起升绳竖直
    craneConf[5] = -craneConf[4];

    // 求起升绳长: h = Bz + sqrt( (L*L + d*d) - (r-Bx)*(r-Bx) ) - y_L - h_hook
    craneConf[6] = boomJointOffsetZ_ + sqrt( ( boomLength_*boomLength_ + ropeJointOffsetZ_*ropeJointOffsetZ_ ) - ( r-boomJointOffsetX_ ) * ( r-boomJointOffsetX_ ) )
                      - liftObjPose_.dZ - hookHeight_;

    // 求吊钩旋转角
    craneConf[7] = liftObjPose_.dFai - ( alpha + beta );
    craneConf[7] = boundAngle( craneConf[7] ); 
   
    // for(int i = 0; i < craneConf.size(); i++)
    // {
    //   ROS_INFO("joint %d: %f", i, craneConf[i]);
    // }
    return true;
  }


  void samplingBaseConfig(std::vector<double>& baseconf)
  {
    // 从站位扇环列表中选出一个扇环
    int which_sector = selectAnnularSector();
    // 从此扇环中随机获取下车的坐标和方向
    double r = rng_.uniformReal( annularSectors_[which_sector].dMinR, annularSectors_[which_sector].dMaxR );
    double theta = rng_.uniformReal( annularSectors_[which_sector].dMinTheta, annularSectors_[which_sector].dMaxTheta );
    double alpha = rng_.uniformReal( annularSectors_[which_sector].dMinAlpha, annularSectors_[which_sector].dMaxAlpha );
    baseconf.resize(3);       
    baseconf[0] = liftObjPose_.dX + r * cos( theta );  // 起重机位置x
    baseconf[1] = liftObjPose_.dY + r * sin( theta );  // 起重机位置y
    baseconf[2] = alpha;  // 起重机下车方向角
  }

  bool upperIK(const std::vector<double>& baseConf, std::vector<double>& craneConf)
  {
    craneConf.resize(8);
    // 设置下车位形
    craneConf[0] = baseConf[0];
    craneConf[1] = baseConf[1];
    craneConf[2] = baseConf[2];

    double r = sqrt( pow(liftObjPose_.dX - baseConf[0], 2) + pow(liftObjPose_.dY - baseConf[1], 2) );

    // 求解转台回转角
    //double theta = atan2(baseConf[0] - liftObjPose_.dX, baseConf[1] - liftObjPose_.dY);   // 范围是[-PI, PI]
    Eigen::Vector3d vecZ( 0, 0, 1 );
    Eigen::Vector3d vecBottom( cos(baseConf[2]), sin(baseConf[2]), 0 );
    //Eigen::Vector3d vecBoom( -cos(theta), -sin(theta), 0 );
    Eigen::Vector3d vecBoom( liftObjPose_.dX - baseConf[0], liftObjPose_.dY - baseConf[1], 0 );
    double beta = acos( vecBottom.dot( vecBoom ) / sqrt( vecBottom.dot( vecBottom ) * vecBoom.dot( vecBoom ) ) );
    double sign = vecZ.dot( vecBottom.cross( vecBoom ) );
    sign = sign < 0.0 ? -1 : 1;
    craneConf[3] = sign * beta;
    craneConf[3] = boundAngle( craneConf[3] );

    // 求臂架仰角: gama = acos( (r-Bx) / sqrt( L*L + d*d ) )
    craneConf[4] = acos( ( r - boomJointOffsetX_ ) / sqrt( boomLength_ * boomLength_ + ropeJointOffsetZ_ * ropeJointOffsetZ_ ) )
                      - atan( ropeJointOffsetZ_ / boomLength_ );
    //ROS_INFO("boomLength_ = %f, boomJointOffsetX_ = %f, ropeJointOffsetZ_= %f, gama = %f", boomLength_, boomJointOffsetX_,  ropeJointOffsetZ_, craneConf[4]);

    // 确保起升绳竖直
    craneConf[5] = -craneConf[4];

    // 求起升绳长: h = Bz + sqrt( (L*L + d*d) - (r-Bx)*(r-Bx) ) - y_L - h_hook
    craneConf[6] = boomJointOffsetZ_ + sqrt( ( boomLength_*boomLength_ + ropeJointOffsetZ_*ropeJointOffsetZ_ ) - ( r-boomJointOffsetX_ ) * ( r-boomJointOffsetX_ ) )
                      - liftObjPose_.dZ - hookHeight_;

    // 求吊钩旋转角
    craneConf[7] = liftObjPose_.dFai - ( baseConf[2] + beta );
    craneConf[7] = boundAngle( craneConf[7] ); 
   
    // for(int i = 0; i < craneConf.size(); i++)
    // {
    //   ROS_INFO("joint %d: %f", i, craneConf[i]);
    // }
    return true;
  }

  bool baseInCLR(std::vector<double> baseConf)
  {
    double dR, dTheta, dAlpha;
    dR = sqrt(pow(baseConf[0] - liftObjPose_.dX, 2) + pow(baseConf[1] - liftObjPose_.dY, 2));
    dTheta = atan2(baseConf[0] - liftObjPose_.dX, baseConf[1] - liftObjPose_.dY);   // 范围是[-PI, PI]
    dAlpha = baseConf[2];

    int n = annularSectors_.size();
    for(int i = 0; i < n; i++)
    {
      if( (annularSectors_[i].dMinTheta <= dTheta && dTheta <= annularSectors_[i].dMaxTheta) && 
          (annularSectors_[i].dMinR <= dR && dR <= annularSectors_[i].dMaxR) &&
          (annularSectors_[i].dMinAlpha <= dAlpha && dAlpha <= annularSectors_[i].dMaxAlpha) )
      {
        return true;
      }
    }

    return false;
  }

    static bool getCLRsFromParamServer(const std::string &key, std::vector<CraneLocationRegions> &clrs)
    {
        ros::NodeHandle nh;
        CraneLocationRegions clr;
        XmlRpc::XmlRpcValue list;
        if (!nh.getParam(key, list))
        {
            ROS_DEBUG("No CraneLocationRegions specification found.");
            return false;
        }

        if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Extra joints specification is not an array. Ignoring.");
            return false;
        }

        for(std::size_t i = 0; i < list.size(); ++i) {
            XmlRpc::XmlRpcValue &elem = list[i];

            if (elem.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << elem.getType() <<
                                                                                           "'. Ignoring.");
                continue;
            }

            // 获取CLR的被吊物位姿信息
            if (!elem.hasMember("lift_obj_pose"))
            {
                ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
                continue;
            }

            XmlRpc::XmlRpcValue &lift_obj_pose = elem["lift_obj_pose"];
            clr.liftObjPose_.dX = static_cast<double>(lift_obj_pose["x"]);
            clr.liftObjPose_.dY = static_cast<double>(lift_obj_pose["y"]);
            clr.liftObjPose_.dZ = static_cast<double>(lift_obj_pose["z"]);
            clr.liftObjPose_.dFai = static_cast<double>(lift_obj_pose["fai"]);

            // 获取CLR的扇环列表
            if (!elem.hasMember("annular_sectors"))
            {
                ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
                continue;
            }
            XmlRpc::XmlRpcValue &annular_sectors = elem["annular_sectors"];
            if (annular_sectors.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("Extra joints specification is not an array. Ignoring.");
                continue;
            }
            CraneLocationRegions::AnnularSector as;
            for(std::size_t j = 0; j < annular_sectors.size(); ++j) {
                XmlRpc::XmlRpcValue &sec = annular_sectors[j];
                ROS_DEBUG("%s",sec.toXml().c_str());
                as.dMinR = static_cast<double>(sec["min_R"]);
                as.dMaxR = static_cast<double>(sec["max_R"]);
                as.dMinTheta = static_cast<double>(sec["min_theta"]);
                as.dMaxTheta = static_cast<double>(sec["min_theta"]);
                as.dMinAlpha = static_cast<double>(sec["min_alpha"]);
                as.dMaxAlpha = static_cast<double>(sec["min_alpha"]);
                clr.annularSectors_.push_back(as);
            }

            clrs.push_back(clr);
        }

        return true;
    }
};


