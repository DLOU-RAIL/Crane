#include <gazebo_visual_plugin.h>
#include <time.h> 

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    SomeVisualPlugin::SomeVisualPlugin(): 
      line(NULL)
    {

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

      this->visual_namespace_ = "visual/";

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
      this->force_sub_ = this->rosnode_->subscribe("/some_force", 1000, &SomeVisualPlugin::VisualizeForceOnLink, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&SomeVisualPlugin::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void SomeVisualPlugin::UpdateChild()
    {
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void SomeVisualPlugin::VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
    {
      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      //TODO: Get the current link position
      //link_pose = CurrentLinkPose();
      //TODO: Get the current end position
      //endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

      // Add two points to a connecting line strip from link_pose to endpoint
      srand( (unsigned)time( NULL ) );
      double x = rand()%10;
      double y = rand()%10;
      
      this->line->AddPoint(math::Vector3(x, y, 0 ));
      this->line->AddPoint(math::Vector3(x, y, 10));
      // set the Material of the line, in this case to purple
      this->line->setMaterial("Gazebo/Purple");
      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(SomeVisualPlugin)
  }
}