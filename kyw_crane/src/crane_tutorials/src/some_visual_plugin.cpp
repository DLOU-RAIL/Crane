#include "some_visual_plugin.h"
#include <tf/transform_listener.h>

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    SomeVisualPlugin::SomeVisualPlugin(): 
      line(NULL)
    {
       std::cout << "KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK" << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    SomeVisualPlugin::~SomeVisualPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void SomeVisualPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      this->visual_ = _parent;

      std::cout << "KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK" << std::endl;

      this->visual_namespace_ = "visual/";

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      //this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
      //this->force_sub_ = this->rosnode_->subscribe("/some_force", 1000, &SomeVisualPlugin::VisualizeForceOnLink, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&SomeVisualPlugin::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void SomeVisualPlugin::UpdateChild()
    {
        VisualizeForceOnLink();
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void SomeVisualPlugin::VisualizeForceOnLink()
    {
        std::cout << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH" << std::endl;

        tf::TransformListener listener;
   /*     tf::StampedTransform transform1;
        tf::StampedTransform transform2;
        try{
          listener.waitForTransform("/base_footprint", "/rope_link", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("/base_footprint", "/rope_link",  
                                  ros::Time(0), transform1);
          listener.waitForTransform("/base_footprint", "/hook_block_link", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("/base_footprint", "/hook_block_link",  
                                  ros::Time(0), transform2);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        
        this->visual_->DeleteDynamicLine(this->line);
      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      double x, y, z;
      x = transform1.getOrigin().x();
      y = transform1.getOrigin().y();
      z = transform1.getOrigin().z();
      math::Vector3 pt0(x, y, z);
      x = transform2.getOrigin().x();
      y = transform2.getOrigin().y();
      z = transform2.getOrigin().z();
      math::Vector3 pt1(x, y, z);
      // Add two points to a connecting line strip from link_pose to endpoint
      this->line->AddPoint( pt0 );
      this->line->AddPoint( pt1 );*/

      this->visual_->DeleteDynamicLine(this->line);
      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      static double z = 0.5;
      this->line->AddPoint(math::Vector3(0.0, 0, 0));
      this->line->AddPoint(math::Vector3(0.0, 0, z));

      // set the Material of the line, in this case to purple
      this->line->setMaterial("Gazebo/Purple");
      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
      z += 0.01;
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(SomeVisualPlugin)
  }
}