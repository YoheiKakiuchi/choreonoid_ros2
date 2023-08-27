/**

*/

#include "ScenePublisherItem.h"
#include "gettext.h"
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Timer>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace cnoid {

class ScenePublisherItem::Impl : public rclcpp::Node
{
public:
    ScenePublisherItem *self;

    double publishingRate;

    Impl(ScenePublisherItem *self);
    Impl(ScenePublisherItem *self, const Impl &org);
    ~Impl();

    void initialize();

    Timer tm;
    image_transport::Publisher pub;
    void publishImage() {
        QImage im = SceneView::instance()->sceneWidget()->getImage();

        sensor_msgs::msg::Image msg;
        msg.height = im.height();
        msg.width = im.width();
        msg.encoding = "bgra8";
        msg.step = msg.width * 4;
        msg.data.resize(msg.height * msg.width * 4);
        for (int i = 0; i < msg.height * msg.width * 4; i++) {
            msg.data[i] = im.bits()[i];
        }
        pub.publish(msg);
    }
};

}  // namespace cnoid

void ScenePublisherItem::initializeClass(ExtensionManager *ext)
{
    ext->itemManager().registerClass<ScenePublisherItem>(N_("ScenePublisherItem"));
    ext->itemManager().addCreationPanel<ScenePublisherItem>();
}

ScenePublisherItem::ScenePublisherItem()
{
    std::cerr << "ScenePublisherItem : created" << std::endl;
    impl = new Impl(this);
}

ScenePublisherItem::Impl::Impl(ScenePublisherItem *self)
    : rclcpp::Node("scene_publisher", rclcpp::NodeOptions())
    , self(self),  tm()
{
    std::cerr << "Created Impl" << std::endl;
    publishingRate=30;
    initialize();
    pub = image_transport::create_publisher(this, "scene/image");
    tm.sigTimeout().connect(
        [this]() { this->publishImage(); });
    tm.start(33);
}

ScenePublisherItem::ScenePublisherItem(const ScenePublisherItem &org)
    : Item(org)
{
    std::cerr << "ScenePublisherItem : copied" << std::endl;
    impl = new Impl(this, *org.impl);
}

ScenePublisherItem::Impl::Impl(ScenePublisherItem *self, const Impl &org)
    : rclcpp::Node("scene_publisher", rclcpp::NodeOptions())
    , self(self)
{
    std::cerr << "Copied Impl" << std::endl;
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
