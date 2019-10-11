#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/rendering/Visual.hh>

namespace gazebo
{
class HideVisualPlugin : public VisualPlugin
{
public:
  HideVisualPlugin() : VisualPlugin()
  {
  }
  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
  {
    ROS_INFO("Hide plugin init.");
    _visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  }
};
GZ_REGISTER_VISUAL_PLUGIN(HideVisualPlugin)
}
