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
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp> // for compress
#include <iostream>
#include <cmath>
#include <cstring>

#include <cnoid/SceneRenderer>

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
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compress_pub;

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
            //RCLCPP_INFO(this->get_logger(), "cam_pos(p) %f %f %f",p.x(), p.y(), p.z());
            Quaternion q(hmd_pos.linear());
            //RCLCPP_INFO(this->get_logger(), "cam_pos(q) %f %f %f %f", q.x(), q.y(), q.z(), q.w());
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
                //view_instances.at(1)->sceneWidget()->setScreenSize(1280, 1280);
                view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
                //view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
                AngleAxis q( 0.0, Vector3::UnitY());
                Eigen::Translation3d tr(-0.0341, 0, 0);
                Isometry3 cam = tr * q;
                Isometry3 cur = hmd_pos * cam;
                {
                    Vector3 v(cur.translation());
                    Quaternion q(cur.linear());
                    //std::cerr << "lpos: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
                    //std::cerr << "l  q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
                }
                view_instances.at(1)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
            }
            {// right scene
                //view_instances.at(2)->sceneWidget()->setScreenSize(1280, 1280);
                view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
                //view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
                AngleAxis q( 0.0, Vector3::UnitY());
                Eigen::Translation3d tr(0.0341, 0, 0);
                Isometry3 cam = tr * q;
                Isometry3 cur = hmd_pos * cam;
                {
                    Vector3 v(cur.translation());
                    Quaternion q(cur.linear());
                    //std::cerr << "rpos: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
                    //std::cerr << "r  q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
                }
                view_instances.at(2)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
            }
            //
            //view_instances.at(0)->sceneWidget()->setScreenSize(1280, 720);
            //view_instances.at(0)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
            //view_instances.at(0)->sceneWidget()->builtinCameraTransform()->setPosition(hmd_pos);
            //view_instances.at(0)->sceneWidget()->renderScene(true);//
            // TODO:: render HMD, controllers
            view_instances.at(1)->sceneWidget()->renderScene(true);//
            view_instances.at(2)->sceneWidget()->renderScene(true);//
#if 0 // Raw Image
            publishCamera(view_instances.at(1)->sceneWidget(),
                          view_instances.at(2)->sceneWidget());
#endif
#if 1 // Compressed Image
            publishRawCamera(view_instances.at(1)->sceneWidget(),
                             view_instances.at(2)->sceneWidget());
#endif
            counter++;
            return;
        }
#if 0
        ////
        QImage im = SceneView::instance()->sceneWidget()->getImage();

        sensor_msgs::msg::Image msg;
        msg.height = im.height();
        msg.width =  im.width();
        msg.encoding = "bgra8";
        msg.step = msg.width * 4;
        msg.data.resize(msg.height * msg.width * 4);
        for (int i = 0; i < msg.height * msg.width * 4; i++) {
            msg.data[i] = im.bits()[i];
        }
        pub.publish(msg);
#endif
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
        im_r.convertTo(QImage::Format_RGB888);
        im_l.convertTo(QImage::Format_RGB888);
        //std::cerr << "fmt: " << im_l.format() << std::endl;
        //std::cerr << "size: " << im_l.width() << "x" << im_l.height() << std::endl;
        //std::cerr << "bbl: " << im_l.bytesPerLine() << std::endl;
        //int nW=1140;
        //int nH=1148;
        int nW=1700;
        int nH=1708;
        //
        cv::Mat new_cv_l;
        cv::Mat new_cv_r;
        {
            cv::Mat cv_l(im_l.height(), im_l.width(), CV_8UC3, im_l.bits());
            cv::Mat cv_r(im_r.height(), im_r.width(), CV_8UC3, im_r.bits());
            //rows = height
            //cols = width
            cv::Rect roiL(   0, 46, nW, nH);
            cv::Rect roiR( 140, 46, nW, nH);
            cv::Mat(cv_l, roiL).copyTo(new_cv_l);
            cv::Mat(cv_r, roiR).copyTo(new_cv_r);
        }
        sensor_msgs::msg::Image msg;
        msg.width  = new_cv_l.cols;
        msg.height = new_cv_l.rows + new_cv_r.rows;
        msg.encoding = "rgb8";
        msg.step = msg.width * 3;
        msg.data.resize(msg.height * msg.width * 3);
        std::memcpy(msg.data.data(), new_cv_l.ptr(), msg.step * new_cv_l.rows);
        std::memcpy(msg.data.data()+(msg.step * new_cv_l.rows), new_cv_r.ptr(), msg.step * new_cv_r.rows);

        //
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
        //pub_cam.publish(msg, info);
        //pub.publish(msg);
        {
            std::shared_ptr<sensor_msgs::msg::CompressedImage> compressed(new sensor_msgs::msg::CompressedImage());
            compressed->header = msg.header;
            compressed->format = msg.encoding;
            std::vector<int> params;
            params.reserve(2);
            params.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
            params.emplace_back(9);
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, compressed);
            cv::imencode(".png", cv_ptr->image, compressed->data, params);
            compress_pub->publish(*compressed);
        }
    }
    void publishRawCamera(SceneWidget *left, SceneWidget *right) {
        QImage im_l =  left->getImage();
        QImage im_r = right->getImage();

        if(im_l.height() != im_r.height()) {
            return;
        }
        if(im_l.width() != im_r.width()) {
            return;
        }
        im_r.convertTo(QImage::Format_RGB888);
        im_l.convertTo(QImage::Format_RGB888);
        int nW=1140;
        int nH=1148;
        //
        cv::Mat new_cv_l;
        cv::Mat new_cv_r;
        {
            cv::Mat cv_l(im_l.height(), im_l.width(), CV_8UC3, im_l.bits());
            cv::Mat cv_r(im_r.height(), im_r.width(), CV_8UC3, im_r.bits());
            cv::Rect roiL(   0, 46, nW, nH);
            cv::Rect roiR( 140, 46, nW, nH);
            cv::Mat(cv_l, roiL).copyTo(new_cv_l);
            cv::Mat(cv_r, roiR).copyTo(new_cv_r);
        }
        sensor_msgs::msg::Image msg;
        msg.width  = new_cv_l.cols;
        msg.height = new_cv_l.rows + new_cv_r.rows;
        msg.encoding = "rgb8";
        msg.step = msg.width * 3;
        msg.data.resize(msg.height * msg.width * 3);
        std::memcpy(msg.data.data(), new_cv_l.ptr(), msg.step * new_cv_l.rows);
        std::memcpy(msg.data.data()+(msg.step * new_cv_l.rows), new_cv_r.ptr(), msg.step * new_cv_r.rows);

        //
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
    publishingRate = 30.0;

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
    fov_ = 1.5504;
    //pub = image_transport::create_publisher(this, "scene/image");
    // raw
    pub_cam = image_transport::create_camera_publisher(this, "scene_camera/images");
    // compressed
    compress_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("scene_camera/compressed_images", 1);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    hmd_pos = Eigen::Isometry3d::Identity();

    std::vector<SceneView *> view_instances = SceneView::instances();
    if (view_instances.size() > 2) {
      view_instances.at(1)->sceneWidget()->setScreenSize(1900, 1900);
      view_instances.at(2)->sceneWidget()->setScreenSize(1900, 1900);
    }
    tm.sigTimeout().connect( [this]() { this->runCycle(); });

    int interval_ms = 1000/publishingRate;
    tm.start(interval_ms);
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
