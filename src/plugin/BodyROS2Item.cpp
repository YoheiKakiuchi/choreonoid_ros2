#include "BodyROS2Item.h"
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/Link>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Sensor>
// #include <ros/console.h>
#include "gettext.h"
#include <fmt/format.h>
#include <geometry_msgs/msg/point32.hpp>

// #include <boost/bind.hpp>
// #include <boost/function.hpp>

using namespace cnoid;
using fmt::format;
using std::placeholders::_1;
using std::placeholders::_2;

void BodyROS2Item::initializeClass(ExtensionManager *ext) {
  std::cout << "initialize BodyROS2Item" << std::endl;
  ext->itemManager().registerClass<BodyROS2Item>(N_("BodyROS2Item"));
  ext->itemManager().addCreationPanel<BodyROS2Item>();
}

BodyROS2Item::BodyROS2Item()
    : rclcpp::Node("choreonoid_body_ros2", rclcpp::NodeOptions()),
      os(MessageView::instance()->cout()) {
  std::cout << "constructing BodyROS2Item class..." << std::endl;
  io = nullptr;
  joint_state_update_rate_ = 100.0;
}

BodyROS2Item::BodyROS2Item(const BodyROS2Item &org)
    : ControllerItem(org), rclcpp::Node("choreonoid_" + org.bodyName,
                                        rclcpp::NodeOptions()),
      os(MessageView::instance()->cout()) {
  std::cout << "constructing BodyROS2Item class..." << std::endl;
  io = nullptr;
  joint_state_update_rate_ = 100.0;
}

BodyROS2Item::~BodyROS2Item() { stop(); }

Item *BodyROS2Item::doDuplicate() const { return new BodyROS2Item(*this); }

bool BodyROS2Item::store(Archive &archive) {
  archive.write("body_ros_version", 0);
  archive.write("joint_state_update_rate", joint_state_update_rate_);

  return true;
}

bool BodyROS2Item::restore(const Archive &archive) {
  if (!archive.read("joint_state_update_rate", joint_state_update_rate_)) {
    archive.read("jointStateUpdateRate", joint_state_update_rate_); // old
  }
  return true;
}

void BodyROS2Item::doPutProperties(PutPropertyFunction &putProperty) {
  putProperty.decimals(2).min(0.0)("Update rate", joint_state_update_rate_,
                                   changeProperty(joint_state_update_rate_));
}

bool BodyROS2Item::initialize(ControllerIO *io) {
  std::cout << "initialize BodyROS2Item" << std::endl;
  if (!io->body()) {
    MessageView::instance()->putln(
        format(_("BodyROS2Item \"{0}\" is invalid because it is not assigned "
                 "to a body."),
               displayName()),
        MessageView::WARNING);
    std::cout << "initialize failed" << std::endl;
    return false;
  }

  this->io = io;
  simulationBody = io->body();
  timeStep_ = io->worldTimeStep();
  controlTime_ = io->currentTime();
  std::cout << "initialize success" << std::endl;
  return true;
}

bool BodyROS2Item::start() {
  std::cout << "BodyROS2Item start" << std::endl;
  // buffer of preserve currently state of joints.
  joint_state_.header.stamp = getStampMsgFromSec(controlTime_);
  joint_state_.name.resize(body()->numAllJoints());
  joint_state_.position.resize(body()->numAllJoints());
  joint_state_.velocity.resize(body()->numAllJoints());
  joint_state_.effort.resize(body()->numAllJoints());

  // preserve initial state of joints.
  for (size_t i = 0; i < body()->numAllJoints(); i++) {
    Link *joint = body()->joint(i);

    joint_state_.name[i] = joint->name();
    joint_state_.position[i] = joint->q();
    joint_state_.velocity[i] = joint->dq();
    joint_state_.effort[i] = joint->u();
  }

  std::string name = simulationBody->name();
  std::replace(name.begin(), name.end(), '-', '_');
  createSensors(simulationBody);

  joint_state_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);
  joint_state_update_period_ = 1.0 / joint_state_update_rate_;
  joint_state_last_update_ = io->currentTime();
  //    ROS_DEBUG("Joint state update rate %f", joint_state_update_rate_);
  RCLCPP_DEBUG(get_logger(), "Joint state update rate %f",
               joint_state_update_rate_);
  std::cout << "BodyROS2Item start finish" << std::endl;
  return true;
}

void BodyROS2Item::createSensors(BodyPtr body) {
  using SetBoolCallback =
      std::function<void(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
                         std::shared_ptr<std_srvs::srv::SetBool::Response>)>;
  DeviceList<> devices = body->devices();

  forceSensors_.assign(devices.extract<ForceSensor>());
  gyroSensors_.assign(devices.extract<RateGyroSensor>());
  accelSensors_.assign(devices.extract<AccelerationSensor>());
  visionSensors_.assign(devices.extract<Camera>());
  rangeVisionSensors_.assign(devices.extract<RangeCamera>());
  rangeSensors_.assign(devices.extract<RangeSensor>());

  for (size_t i = 0; i < visionSensors_.size(); ++i) {
    if (Camera *sensor = visionSensors_[i]) {
      RangeCamera *camera = dynamic_cast<RangeCamera *>(sensor);
      if (camera) {
        rangeVisionSensors_.push_back(camera);
      }
    }
  }

  force_sensor_publishers_.resize(forceSensors_.size());
  force_sensor_switch_servers_.clear();
  force_sensor_switch_servers_.reserve(forceSensors_.size());
  for (size_t i = 0; i < forceSensors_.size(); ++i) {
    if (ForceSensor *sensor = forceSensors_[i]) {
      std::string name = sensor->name();
      std::replace(name.begin(), name.end(), '-', '_');
      force_sensor_publishers_[i] =
          create_publisher<geometry_msgs::msg::WrenchStamped>(name, 1);
      sensor->sigStateChanged().connect(
          std::bind(&BodyROS2Item::updateForceSensor, this, sensor,
                    force_sensor_publishers_[i]));
      SetBoolCallback requestCallback =
          std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
      force_sensor_switch_servers_.push_back(
          create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                 requestCallback));
      RCLCPP_INFO(get_logger(), "Create force sensor %s",
                  sensor->name().c_str());
    }
  }
  rate_gyro_sensor_publishers_.resize(gyroSensors_.size());
  rate_gyro_sensor_switch_servers_.clear();
  rate_gyro_sensor_switch_servers_.reserve(gyroSensors_.size());
  for (size_t i = 0; i < gyroSensors_.size(); ++i) {
    if (RateGyroSensor *sensor = gyroSensors_[i]) {
      std::string name = sensor->name();
      std::replace(name.begin(), name.end(), '-', '_');
      rate_gyro_sensor_publishers_[i] =
          create_publisher<sensor_msgs::msg::Imu>(name, 1);
      sensor->sigStateChanged().connect(
          std::bind(&BodyROS2Item::updateRateGyroSensor, this, sensor,
                    rate_gyro_sensor_publishers_[i]));
      SetBoolCallback requestCallback =
          std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
      rate_gyro_sensor_switch_servers_.push_back(
          create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                 requestCallback));
      RCLCPP_INFO(get_logger(), "Create gyro sensor %s",
                  sensor->name().c_str());
    }
  }
  accel_sensor_publishers_.resize(accelSensors_.size());
  accel_sensor_switch_servers_.clear();
  accel_sensor_switch_servers_.reserve(accelSensors_.size());
  for (size_t i = 0; i < accelSensors_.size(); ++i) {
    if (AccelerationSensor *sensor = accelSensors_[i]) {
      std::string name = sensor->name();
      std::replace(name.begin(), name.end(), '-', '_');
      accel_sensor_publishers_[i] =
          create_publisher<sensor_msgs::msg::Imu>(name, 1);
      sensor->sigStateChanged().connect(
          std::bind(&BodyROS2Item::updateAccelSensor, this, sensor,
                    accel_sensor_publishers_[i]));
      SetBoolCallback requestCallback =
          std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
      accel_sensor_switch_servers_.push_back(
          create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                 requestCallback));
      RCLCPP_INFO(get_logger(), "Create accel sensor %s",
                  sensor->name().c_str());
    }
  }

  std::cout << "Start creating vision sensors" << std::endl;

  vision_sensor_publishers_.resize(visionSensors_.size());
  vision_sensor_switch_servers_.clear();
  vision_sensor_switch_servers_.reserve(visionSensors_.size());

  for (size_t i = 0; i < visionSensors_.size(); ++i) {
    if (Camera *sensor = visionSensors_[i]) {
      std::string name = sensor->name();
      std::cout << "-- camera name : " << name << " --" << std::endl;
      std::replace(name.begin(), name.end(), '-', '_');
      auto raw_publisher = image_transport::create_camera_publisher(this, name + "/image_raw");
      std::shared_ptr<image_transport::CameraPublisher> cam_publisher(&raw_publisher, [](image_transport::CameraPublisher*){});

      std::cout << "create vision sensor publisher" << std::endl;
      vision_sensor_publishers_[i] = cam_publisher;
      sensor->sigStateChanged().connect(
          std::bind(&BodyROS2Item::updateVisionSensor, this, sensor,
                    vision_sensor_publishers_[i]));
      SetBoolCallback requestCallback =
          std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
      vision_sensor_switch_servers_.push_back(
          create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                 requestCallback));
      RCLCPP_INFO(get_logger(), "Create RGB camera %s (%f Hz)",
                  sensor->name().c_str(), sensor->frameRate());
    }
  }

  range_vision_sensor_publishers_.resize(rangeVisionSensors_.size());
  range_vision_sensor_switch_servers_.clear();
  range_vision_sensor_switch_servers_.reserve(rangeVisionSensors_.size());
  for (size_t i = 0; i < rangeVisionSensors_.size(); ++i) {
    if (RangeCamera *sensor = rangeVisionSensors_[i]) {
      std::string name = sensor->name();
      std::replace(name.begin(), name.end(), '-', '_');
      range_vision_sensor_publishers_[i] =
          create_publisher<sensor_msgs::msg::PointCloud2>(name + "/point_cloud",
                                                          1);
      sensor->sigStateChanged().connect(
          std::bind(&BodyROS2Item::updateRangeVisionSensor, this, sensor,
                    range_vision_sensor_publishers_[i]));
      // adds a server only for the camera whose type is COLOR_DEPTH or
      // POINT_CLOUD. Without this exception, a new service server may be a
      // duplicate of one added to 'vision_sensor_switch_servers_'.
      if (sensor->imageType() == Camera::NO_IMAGE) {
        SetBoolCallback requestCallback =
            std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
        range_vision_sensor_switch_servers_.push_back(
            create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                   requestCallback));
        RCLCPP_INFO(get_logger(), "Create depth camera %s (%f Hz)",
                    sensor->name().c_str(), sensor->frameRate());
      } else {
        RCLCPP_INFO(get_logger(), "Create RGBD camera %s (%f Hz)",
                    sensor->name().c_str(), sensor->frameRate());
      }
    }
  }
  range_sensor_publishers_.resize(rangeSensors_.size());
  range_sensor_switch_servers_.clear();
  range_sensor_switch_servers_.reserve(rangeSensors_.size());
  range_sensor_pc_publishers_.resize(rangeSensors_.size());
  range_sensor_pc_switch_servers_.clear();
  range_sensor_pc_switch_servers_.reserve(rangeSensors_.size());
  for (size_t i = 0; i < rangeSensors_.size(); ++i) {
    if (RangeSensor *sensor = rangeSensors_[i]) {
      if (sensor->numPitchSamples() > 1) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        range_sensor_pc_publishers_[i] =
            create_publisher<sensor_msgs::msg::PointCloud>(
                name + "/point_cloud", 1);
        sensor->sigStateChanged().connect(
            std::bind(&BodyROS2Item::update3DRangeSensor, this, sensor,
                      range_sensor_pc_publishers_[i]));
        SetBoolCallback requestCallback =
            std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
        range_sensor_pc_switch_servers_.push_back(
            create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                   requestCallback));
        RCLCPP_DEBUG(get_logger(), "Create 3d range sensor %s (%f Hz)",
                     sensor->name().c_str(), sensor->scanRate());
      } else {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        range_sensor_publishers_[i] =
            create_publisher<sensor_msgs::msg::LaserScan>(name + "/scan", 1);
        sensor->sigStateChanged().connect(
            std::bind(&BodyROS2Item::updateRangeSensor, this, sensor,
                      range_sensor_publishers_[i]));
        SetBoolCallback requestCallback =
            std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
        range_sensor_switch_servers_.push_back(
            create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                   requestCallback));
        RCLCPP_DEBUG(get_logger(), "Create 2d range sensor %s (%f Hz)",
                     sensor->name().c_str(), sensor->scanRate());
      }
    }
  }
}

bool BodyROS2Item::control() {
  controlTime_ = io->currentTime();
  double updateSince = controlTime_ - joint_state_last_update_;

  if (updateSince > joint_state_update_period_) {
    // publish current joint states
    joint_state_.header.stamp = getStampMsgFromSec(controlTime_);

    for (int i = 0; i < body()->numAllJoints(); i++) {
      Link *joint = body()->joint(i);

      joint_state_.position[i] = joint->q();
      joint_state_.velocity[i] = joint->dq();
      joint_state_.effort[i] = joint->u();
    }

    joint_state_publisher_->publish(joint_state_);
    joint_state_last_update_ += joint_state_update_period_;
  }

  return true;
}

void BodyROS2Item::updateForceSensor(
    ForceSensor *sensor,
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher) {
  if (!sensor->on()) {
    return;
  }
  geometry_msgs::msg::WrenchStamped force;
  force.header.stamp = getStampMsgFromSec(io->currentTime());
  force.header.frame_id = sensor->name();
  force.wrench.force.x = sensor->F()[0] / 1000.0;
  force.wrench.force.y = sensor->F()[1] / 1000.0;
  force.wrench.force.z = sensor->F()[2] / 1000.0;
  force.wrench.torque.x = sensor->F()[3] / 1000.0;
  force.wrench.torque.y = sensor->F()[4] / 1000.0;
  force.wrench.torque.z = sensor->F()[5] / 1000.0;
  std::cout << "publish force sensor" << std::endl;
  publisher->publish(force);
}

void BodyROS2Item::updateRateGyroSensor(
    RateGyroSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher) {
  if (!sensor->on()) {
    return;
  }
  sensor_msgs::msg::Imu gyro;
  gyro.header.stamp = getStampMsgFromSec(io->currentTime());
  gyro.header.frame_id = sensor->name();
  gyro.angular_velocity.x = sensor->w()[0];
  gyro.angular_velocity.y = sensor->w()[1];
  gyro.angular_velocity.z = sensor->w()[2];
  publisher->publish(gyro);
}

void BodyROS2Item::updateAccelSensor(
    AccelerationSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher) {
  if (!sensor->on()) {
    return;
  }
  sensor_msgs::msg::Imu accel;
  accel.header.stamp = getStampMsgFromSec(io->currentTime());
  accel.header.frame_id = sensor->name();
  accel.linear_acceleration.x = sensor->dv()[0] / 10.0;
  accel.linear_acceleration.y = sensor->dv()[1] / 10.0;
  accel.linear_acceleration.z = sensor->dv()[2] / 10.0;
  publisher->publish(accel);
}

void BodyROS2Item::updateVisionSensor(
    Camera *sensor,
    std::shared_ptr<image_transport::CameraPublisher> publisher) {
  if (!sensor->on()) {
    std::cout << "not on" << std::endl;
    return;
  }
  sensor_msgs::msg::Image vision;
  vision.header.stamp = getStampMsgFromSec(io->currentTime());
  vision.header.frame_id = sensor->name();
  vision.height = sensor->image().height();
  vision.width = sensor->image().width();
  if (sensor->image().numComponents() == 3)
    vision.encoding = "rgb8";
  else if (sensor->image().numComponents() == 1)
    vision.encoding = "mono8";
  else {
    RCLCPP_WARN(get_logger(), "unsupported image component number: %i",
                sensor->image().numComponents());
  }
  vision.is_bigendian = 0;
  vision.step = sensor->image().width() * sensor->image().numComponents();
  vision.data.resize(vision.step * vision.height);
  std::memcpy(&(vision.data[0]), &(sensor->image().pixels()[0]),
              vision.step * vision.height);
  // TODO
  sensor_msgs::msg::CameraInfo camera_info;
  std::cout << "publish image sensor" << std::endl;
  publisher->publish(vision, camera_info);
}

void BodyROS2Item::updateRangeVisionSensor(
    RangeCamera *sensor,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) {
  if (!sensor->on()) {
    std::cout << "not on" << std::endl;
    return;
  }
  sensor_msgs::msg::PointCloud2 range;
  range.header.stamp = getStampMsgFromSec(io->currentTime());
  range.header.frame_id = sensor->name();
  range.width = sensor->resolutionX();
  range.height = sensor->resolutionY();
  range.is_bigendian = false;
  range.is_dense = true;
  range.row_step = range.point_step * range.width;
  if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
    range.fields.resize(6);
    range.fields[3].name = "rgb";
    range.fields[3].offset = 12;
    range.fields[3].count = 1;
    range.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    /*
      range.fields[3].name = "r";
      range.fields[3].offset = 12;
      range.fields[3].datatype = sensor_msgs::PointField::UINT8;
      range.fields[3].count = 1;
      range.fields[4].name = "g";
      range.fields[4].offset = 13;
      range.fields[4].datatype = sensor_msgs::PointField::UINT8;
      range.fields[4].count = 1;
      range.fields[5].name = "b";
      range.fields[5].offset = 14;
      range.fields[5].datatype = sensor_msgs::PointField::UINT8;
      range.fields[5].count = 1;
    */
    range.point_step = 16;
  } else {
    range.fields.resize(3);
    range.point_step = 12;
  }
  range.fields[0].name = "x";
  range.fields[0].offset = 0;
  range.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  range.fields[0].count = 4;
  range.fields[1].name = "y";
  range.fields[1].offset = 4;
  range.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  range.fields[1].count = 4;
  range.fields[2].name = "z";
  range.fields[2].offset = 8;
  range.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  range.fields[2].count = 4;
  const std::vector<Vector3f> &points = sensor->constPoints();
  const unsigned char *pixels = sensor->constImage().pixels();
  range.data.resize(points.size() * range.point_step);
  unsigned char *dst = (unsigned char *)&(range.data[0]);
  for (size_t j = 0; j < points.size(); ++j) {
    float x = points[j].x();
    float y = -points[j].y();
    float z = -points[j].z();
    std::memcpy(&dst[0], &x, 4);
    std::memcpy(&dst[4], &y, 4);
    std::memcpy(&dst[8], &z, 4);
    if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
      dst[14] = *pixels++;
      dst[13] = *pixels++;
      dst[12] = *pixels++;
      dst[15] = 0;
    }
    dst += range.point_step;
  }
  std::cout << "publish range image sensor" << std::endl;
  publisher->publish(range);
}

void BodyROS2Item::updateRangeSensor(
    RangeSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher) {
  if (!sensor->on()) {
    std::cout << "not on" << std::endl;
    return;
  }
  sensor_msgs::msg::LaserScan range;
  range.header.stamp = getStampMsgFromSec(io->currentTime());
  range.header.frame_id = sensor->name();
  range.range_max = sensor->maxDistance();
  range.range_min = sensor->minDistance();
  if (sensor->yawRange() == 0.0) {
    range.angle_max = sensor->pitchRange() / 2.0;
    range.angle_min = -sensor->pitchRange() / 2.0;
    range.angle_increment = sensor->pitchStep();
  } else {
    range.angle_max = sensor->yawRange() / 2.0;
    range.angle_min = -sensor->yawRange() / 2.0;
    range.angle_increment = sensor->yawStep();
  }
  range.ranges.resize(sensor->rangeData().size());
  // range.intensities.resize(sensor->rangeData().size());
  //  for (size_t j = 0; j < sensor->rangeData().size(); ++j) {
  for (size_t j = 0; j < sensor->numYawSamples(); ++j) {
    range.ranges[j] = sensor->rangeData()[j];
    // range.intensities[j] = -900000;
  }
  std::cout << "publish range sensor" << std::endl;
  publisher->publish(range);
}

void BodyROS2Item::update3DRangeSensor(
    RangeSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher) {
  if (!sensor->on()) {
    std::cout << "not on" << std::endl;
    return;
  }
  sensor_msgs::msg::PointCloud range;
  // Header Info
  range.header.stamp = getStampMsgFromSec(io->currentTime());
  range.header.frame_id = sensor->name();

  // Calculate Point Cloud data
  const int numPitchSamples = sensor->numPitchSamples();
  const double pitchStep = sensor->pitchStep();
  const int numYawSamples = sensor->numYawSamples();
  const double yawStep = sensor->yawStep();

  for (int pitch = 0; pitch < numPitchSamples; ++pitch) {
    const double pitchAngle = pitch * pitchStep - sensor->pitchRange() / 2.0;
    const double cosPitchAngle = cos(pitchAngle);
    const int srctop = pitch * numYawSamples;

    for (int yaw = 0; yaw < numYawSamples; ++yaw) {
      const double distance = sensor->rangeData()[srctop + yaw];
      if (distance <= sensor->maxDistance()) {
        double yawAngle = yaw * yawStep - sensor->yawRange() / 2.0;
        geometry_msgs::msg::Point32 point;
        point.x = distance * cosPitchAngle * sin(-yawAngle);
        point.y = distance * sin(pitchAngle);
        point.z = -distance * cosPitchAngle * cos(-yawAngle);
        range.points.push_back(point);
      }
    }
  }

  std::cout << "publish 3d range sensor" << std::endl;
  publisher->publish(range);
}

void BodyROS2Item::input() {}

void BodyROS2Item::output() {}

// void BodyROS2Item::stop_publish()
//{
//     size_t i;
//
//     for (i = 0; i < force_sensor_publishers_.size(); i++) {
//         force_sensor_publishers_[i].shutdown();
//     }
//
//     for (i = 0; i < rate_gyro_sensor_publishers_.size(); i++) {
//         rate_gyro_sensor_publishers_[i].shutdown();
//     }
//
//     for (i = 0; i < accel_sensor_publishers_.size(); i++) {
//         accel_sensor_publishers_[i].shutdown();
//     }
//
//     for (i = 0; i < vision_sensor_publishers_.size(); i++) {
//         vision_sensor_publishers_[i].shutdown();
//     }
//
//     for (i = 0; i < range_vision_sensor_publishers_.size(); i++) {
//         range_vision_sensor_publishers_[i].shutdown();
//     }
//
//     for (i = 0; i < range_sensor_publishers_.size(); i++) {
//         range_sensor_publishers_[i].shutdown();
//     }
//
//     for (i = 0; i < range_sensor_pc_publishers_.size(); i++) {
//         range_sensor_pc_publishers_[i].shutdown();
//     }
//
//     return;
// }

void BodyROS2Item::switchDevice(
    std_srvs::srv::SetBool::Request::ConstSharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response, Device *sensor) {
  std::cout << "switchDevice : " << sensor->name() << std::endl;
  sensor->on(request->data);
  response->success = (request->data == sensor->on());
}

void BodyROS2Item::stop() {
  if (rclcpp::ok()) {

    rclcpp::shutdown();
  }

  return;
}

builtin_interfaces::msg::Time BodyROS2Item::getStampMsgFromSec(double sec) {
  builtin_interfaces::msg::Time msg;
  msg.sec = int(sec);
  msg.nanosec = (sec - int(sec)) * 1000000000;
  return msg;
}