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
#include <cnoid/SceneCameras>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <cmath>

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
    image_transport::CameraPublisher pub_cam;
    rclcpp::Clock ros_clock;

    int counter;
    void publishImage() {
        std::vector<SceneView *> view_instances = SceneView::instances();
        if (view_instances.size() > 2) {
            //std::cerr << "sceneWidget: 0/" << view_instances.at(1)->name();
            //std::cerr << ", 1/" << view_instances.at(2)->name() << std::endl;
            {
                Vector3 eye(3, 0, 0);
                Vector3 direction(-1, 0, 0);
                Vector3 up(0, std::sin(M_PI/1000*counter), std::cos(M_PI/1000*counter));
                view_instances.at(1)->sceneWidget()->setScreenSize(1280, 720);
                view_instances.at(1)->sceneWidget()->setCameraPosition(eye, direction, up);
                //view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
            }
            {
                Vector3 eye(0, 0, 3);
                Vector3 direction(0, 0, -1);
                Vector3 up(std::sin(M_PI/1000*counter), std::cos(M_PI/1000*counter), 0);
                view_instances.at(2)->sceneWidget()->setScreenSize(1280, 720);
                view_instances.at(2)->sceneWidget()->setCameraPosition(eye, direction, up);
                //view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
            }
            // notify only :OK
            // render only :OK
            view_instances.at(1)->sceneWidget()->renderScene(true);//
            view_instances.at(2)->sceneWidget()->renderScene(true);//
            publishCamera(view_instances.at(1)->sceneWidget(),
                          view_instances.at(2)->sceneWidget());
            counter++;
            return;
        }
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
    void publishCamera(SceneWidget *left, SceneWidget *right) {
        QImage im_l =  left->getImage();
        QImage im_r = right->getImage();

        if(im_l.height() != im_r.height()) {
            return;
        }
        if(im_l.width() != im_r.width()) {
            return;
        }
        // std::cerr << "size: " << im_l.width() << "x" << im_l.height() << std::endl;
        sensor_msgs::msg::Image msg;
        msg.width  = im_l.width();
        msg.height = im_l.height() * 2;
        msg.encoding = "bgra8";
        msg.step = msg.width * 4;
        msg.data.resize(msg.height * msg.width * 4);
//// [TODO] memcpy
        int j;
        for (int i = 0; i < msg.height * msg.width * 2; i++, j++) {
            msg.data[j] = im_l.bits()[i];
        }
        for (int i = 0; i < msg.height * msg.width * 2; i++, j++) {
            msg.data[j] = im_r.bits()[i];
        }
////
        rclcpp::Time now = ros_clock.now();
        msg.header.stamp = now;
        msg.header.frame_id = "cnoid";
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp = now;
        info.header.frame_id = "cnoid";
        // [TODO] add camera_info
        pub_cam.publish(msg, info);
        //pub.publish(msg);
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
    , self(self),  tm(), ros_clock(RCL_ROS_TIME)
{
    counter = 0;
    std::cerr << "Created Impl" << std::endl;
    publishingRate=30;

    initialize();
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
    // render scene
    // subscribe
    //
    pub = image_transport::create_publisher(this, "scene/image");
    pub_cam = image_transport::create_camera_publisher(this, "scene_camera/image");

    tm.sigTimeout().connect(
        [this]() { this->publishImage(); });
    //tm.start(33);
    tm.start(67);
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
