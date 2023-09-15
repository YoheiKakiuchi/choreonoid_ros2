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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
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
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Clock ros_clock;
    Eigen::Isometry3d hmd_pos;
    double fov_;

    int counter;
    void runCycle() {
        std::string toFrame = "world";
        //std::string fromFrame = "world_vive";
        std::string fromFrame = "hmd";
        try {
            geometry_msgs::msg::TransformStamped tmp = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
            //tmp.header.frame_id;
            //tmp.header.stamp;
            //tmp.header.child_frame_id;
            hmd_pos = tf2::transformToEigen(tmp);
            Vector3 p(hmd_pos.translation());
            RCLCPP_INFO(this->get_logger(), "cam_pos(p) %f %f %f",p.x(), p.y(), p.z());
            Quaternion q(hmd_pos.linear());
            RCLCPP_INFO(this->get_logger(), "cam_pos(q) %f %f %f %f", q.x(), q.y(), q.z(), q.w());
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrame.c_str(), fromFrame.c_str(), ex.what());
            //return;
        }
        publishImage();
    }
    void publishImage() {
        std::vector<SceneView *> view_instances = SceneView::instances();
        if (view_instances.size() > 2) {
            //std::cerr << "sceneWidget: 0/" << view_instances.at(1)->name();
            //std::cerr << ", 1/" << view_instances.at(2)->name() << std::endl;
            {// left scene
                view_instances.at(1)->sceneWidget()->setScreenSize(1280, 720);
                view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
                //view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
                AngleAxis q( 0.1, Vector3::UnitY());
                Eigen::Translation3d tr(-0.1, 0, 0);
                Isometry3 cam = tr * q;
                Isometry3 cur = hmd_pos * cam;
                {
                    Vector3 v(cur.translation());
                    Quaternion q(cur.linear());
                    std::cerr << "lpos: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
                    std::cerr << "l  q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
                }
                view_instances.at(1)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
            }
            {// right scene
                view_instances.at(2)->sceneWidget()->setScreenSize(1280, 720);
                view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
                //view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
                AngleAxis q(-0.1, Vector3::UnitY());
                Eigen::Translation3d tr( 0.1, 0, 0);
                Isometry3 cam = tr * q;
                Isometry3 cur = hmd_pos * cam;
                {
                    Vector3 v(cur.translation());
                    Quaternion q(cur.linear());
                    std::cerr << "rpos: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
                    std::cerr << "r  q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
                }
                view_instances.at(2)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
            }
            view_instances.at(0)->sceneWidget()->setScreenSize(1280, 720);
            view_instances.at(0)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
            view_instances.at(0)->sceneWidget()->builtinCameraTransform()->setPosition(hmd_pos);
            // notify only :OK
            // render only :OK
            view_instances.at(0)->sceneWidget()->renderScene(true);//
            view_instances.at(1)->sceneWidget()->renderScene(true);//
            view_instances.at(2)->sceneWidget()->renderScene(true);//
            publishCamera(view_instances.at(1)->sceneWidget(),
                          view_instances.at(2)->sceneWidget());
            counter++;
            return;
        }
        ////
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
#if 0
        unsigned char *bufr = im_r.bits();
        unsigned char *bufl = im_l.bits();
        for(int y = 0; y < im_r.height(); y++) {
            for(int x = 0; x < im_r.width(); x++) {
                if (x > im_r.width()/2) continue;
                int pos = 4*(y * im_r.width() + x);
                bufr[pos] = 255;
                bufr[pos+1] = 255;
                bufr[pos+2] = 255;
                bufr[pos+3] = 255;
                bufl[pos] = 255;
                bufl[pos+1] = 255;
                bufl[pos+2] = 255;
                bufl[pos+3] = 255;
            }
        }
#endif
//// [TODO] memcpy
        {
            int j = 0;
            for (int i = 0; i < msg.height * msg.width * 2; i++, j++) {
                msg.data[j] = im_l.bits()[i];
            }
            for (int i = 0; i < msg.height * msg.width * 2; i++, j++) {
                msg.data[j] = im_r.bits()[i];
            }
        }
////
        rclcpp::Time now = ros_clock.now();
        msg.header.stamp = now;
        msg.header.frame_id = "cnoid";
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp = now;
        info.header.frame_id = "cnoid";
        // [TODO] add camera_info
        //const double minLength = std::min(msg.width, msg.height/2);
        const double focalLength = msg.width / 2.0 / tan(0.5*fov_);
        //const double principalPointX = (info.width - 1.0) / 2.0;
        //const double principalPointY = (info.height - 1.0) / 2.0;
        info.k[0] = focalLength;
        info.k[4] = focalLength;
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
    fov_ = 110 * M_PI / 180;
    pub = image_transport::create_publisher(this, "scene/image");
    pub_cam = image_transport::create_camera_publisher(this, "scene_camera/images");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    hmd_pos = Eigen::Isometry3d::Identity();

    tm.sigTimeout().connect( [this]() { this->runCycle(); });
    //tm.start(33);
    //tm.start(67);
    tm.start(120);
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
