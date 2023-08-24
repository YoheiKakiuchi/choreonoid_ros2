#include "ROS2Plugin.hpp"
#include "BodyROS2Item.h"
#include "WorldROS2Item.h"
#include "ScenePublisherItem.h"
#include "deprecated/BodyPublisherItem.h"
#include <cnoid/MessageView>


using namespace std;
using namespace cnoid;

ROS2Plugin::ROS2Plugin()
    : Plugin("ROS2")
    , Node("choreonoid_ros2", rclcpp::NodeOptions())
{
    require("Body");
}
bool ROS2Plugin::initialize()
{
    WorldROS2Item::initializeClass(this);
    BodyROS2Item::initializeClass(this);
    ScenePublisherItem::initializeClass(this);
    BodyPublisherItem::initializeClass(this);

    //
    //
    return true;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROS2Plugin)
