/**
   \note Alghough multiple instances of WorldROSItem can be created,
   only one /clock topic can exist in a system. The current implementation
   does not consider this limitation and the clock value may be inconsistent
   when multiple WorldROSItem publish it simultaneously. The implementation
   should be improved to avoid this problem.
*/

#include "ScenePublisherItem.h"
#include "gettext.h"
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace cnoid;

namespace cnoid {

class ScenePublisherItem::Impl : public rclcpp::Node
{
public:
    ScenePublisherItem *self;

    //    unique_ptr<ros::NodeHandle> rosNode;
    //rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher;
    double publishingRate;

    Impl(ScenePublisherItem *self);
    Impl(ScenePublisherItem *self, const Impl &org);
    ~Impl();

    void initialize();
};

}  // namespace cnoid

void ScenePublisherItem::initializeClass(ExtensionManager *ext)
{
    ext->itemManager().registerClass<ScenePublisherItem>(N_("ScenePublisherItem"));
    ext->itemManager().addCreationPanel<ScenePublisherItem>();
}

ScenePublisherItem::ScenePublisherItem()
{
    impl = new Impl(this);
}

ScenePublisherItem::Impl::Impl(ScenePublisherItem *self)
    : rclcpp::Node("scene_publisher", rclcpp::NodeOptions())
    , self(self)
{
    publishingRate=30;
    initialize();
}

ScenePublisherItem::ScenePublisherItem(const ScenePublisherItem &org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}

ScenePublisherItem::Impl::Impl(ScenePublisherItem *self, const Impl &org)
    : rclcpp::Node("scene_publisher", rclcpp::NodeOptions())
    , self(self)
{
    publishingRate = org.publishingRate;
    initialize();
}

void ScenePublisherItem::Impl::initialize()
{
    //
}

ScenePublisherItem::~ScenePublisherItem()
{
    delete impl;
}

ScenePublisherItem::Impl::~Impl()
{
}

Item *ScenePublisherItem::doDuplicate() const
{
    return new ScenePublisherItem(*this);
}

void ScenePublisherItem::doPutProperties(PutPropertyFunction &putProperty)
{
    putProperty.decimals(2).min(0.0)(_("image publishing rate"),
                                     impl->publishingRate);
}

bool ScenePublisherItem::store(Archive &archive)
{
    archive.write("scene_publishing_rate", impl->publishingRate);
    return true;
}

bool ScenePublisherItem::restore(const Archive &archive)
{
    archive.read("scene_publishing_rate", impl->publishingRate);
    return true;
}
