/*
Copyright 2007-2023. Algoryx Simulation AB.

All AGX source code, intellectual property, documentation, sample code,
tutorials, scene files and technical white papers, are copyrighted, proprietary
and confidential material of Algoryx Simulation AB. You may not download, read,
store, distribute, publish, copy or otherwise disseminate, use or expose this
material unless having a written signed agreement with Algoryx Simulation AB, or having been
advised so by Algoryx Simulation AB for a time limited evaluation, or having purchased a
valid commercial license from Algoryx Simulation AB.

Algoryx Simulation AB disclaims all responsibilities for loss or damage caused
from using this software, unless otherwise stated in written agreements with
Algoryx Simulation AB.
*/

#pragma once

// AGX Networking Toolbox includes.
#include "agx-nt-ros2/agx-nt-ros2_export.h"

// Standard library includes.
#include <stdint.h>
#include <string>
#include <vector>

#ifdef _MSC_VER
# pragma warning(push)
# pragma warning(disable: 4251) // Disable warnings about members needing dll interface.
#endif


namespace agxROS2
{
  namespace agxMsgs
  {
    struct AGXNTROS2_EXPORT Any
    {
      std::vector<uint8_t> data;
    };

    struct AGXNTROS2_EXPORT AnySequence
    {
      std::vector<Any> data;
    };
  }

  namespace builtinInterfaces
  {
    struct AGXNTROS2_EXPORT Time
    {
      int32_t sec {0};
      uint32_t nanosec {0};
    };

    struct AGXNTROS2_EXPORT Duration
    {
      int32_t sec {0};
      uint32_t nanosec {0};
    };
  }

  namespace rosgraphMsgs
  {
    struct AGXNTROS2_EXPORT Clock
    {
      builtinInterfaces::Time clock;
    };
  }

  namespace stdMsgs
  {
    struct AGXNTROS2_EXPORT MultiArrayDimension
    {
      std::string label;
      uint32_t size {0};
      uint32_t stride {0};
    };

    struct AGXNTROS2_EXPORT MultiArrayLayout
    {
      std::vector<MultiArrayDimension> dim;
      uint32_t data_offset {0};
    };

    struct AGXNTROS2_EXPORT Bool
    {
      bool data = false;
    };

    struct AGXNTROS2_EXPORT Byte
    {
      uint8_t data {0};
    };

    struct AGXNTROS2_EXPORT ByteMultiArray
    {
      MultiArrayLayout layout;
      std::vector<uint8_t> data;
    };

    struct AGXNTROS2_EXPORT Char
    {
      uint8_t data {0};
    };

    struct AGXNTROS2_EXPORT ColorRGBA
    {
      float r {0.f};
      float g {0.f};
      float b {0.f};
      float a {0.f};
    };

    struct AGXNTROS2_EXPORT Empty
    {
      uint8_t structure_needs_at_least_one_member {0};
    };

    struct AGXNTROS2_EXPORT Float32
    {
      float data {0.f};
    };

    struct AGXNTROS2_EXPORT Float32MultiArray
    {
      MultiArrayLayout layout;
      std::vector<float> data;
    };

    struct AGXNTROS2_EXPORT Float64
    {
      double data {0.0};
    };

    struct AGXNTROS2_EXPORT Float64MultiArray
    {
      MultiArrayLayout layout;
      std::vector<double> data;
    };

    struct AGXNTROS2_EXPORT Int16
    {
      int16_t data {0};
    };

    struct AGXNTROS2_EXPORT Int16MultiArray
    {
      MultiArrayLayout layout;
      std::vector<int16_t> data;
    };

    struct AGXNTROS2_EXPORT Int32
    {
      int32_t data {0};
    };

    struct AGXNTROS2_EXPORT Int32MultiArray
    {
      MultiArrayLayout layout;
      std::vector<int32_t> data;
    };

    struct AGXNTROS2_EXPORT Int64
    {
      int64_t data {0};
    };

    struct AGXNTROS2_EXPORT Int64MultiArray
    {
      MultiArrayLayout layout;
      std::vector<int64_t> data;
    };

    struct AGXNTROS2_EXPORT Int8
    {
      int8_t data {0};
    };

    struct AGXNTROS2_EXPORT Int8MultiArray
    {
      MultiArrayLayout layout;
      std::vector<int8_t> data;
    };

    struct AGXNTROS2_EXPORT String
    {
      std::string data;
    };

    struct AGXNTROS2_EXPORT UInt16
    {
      uint16_t data {0};
    };

    struct AGXNTROS2_EXPORT UInt16MultiArray
    {
      MultiArrayLayout layout;
      std::vector<uint16_t> data;
    };

    struct AGXNTROS2_EXPORT UInt32
    {
      uint32_t data {0};
    };

    struct AGXNTROS2_EXPORT UInt32MultiArray
    {
      MultiArrayLayout layout;
      std::vector<uint32_t> data;
    };

    struct AGXNTROS2_EXPORT UInt64
    {
      uint64_t data {0};
    };

    struct AGXNTROS2_EXPORT UInt64MultiArray
    {
      MultiArrayLayout layout;
      std::vector<uint64_t> data;
    };

    struct AGXNTROS2_EXPORT UInt8
    {
      uint8_t data {0};
    };

    struct AGXNTROS2_EXPORT UInt8MultiArray
    {
      MultiArrayLayout layout;
      std::vector<uint8_t> data;
    };

    struct AGXNTROS2_EXPORT Header
    {
      builtinInterfaces::Time stamp;
      std::string frame_id;
    };
  }

  namespace geometryMsgs
  {
    struct Vector3
    {
      double x {0.0};
      double y {0.0};
      double z {0.0};
    };

    struct Quaternion
    {
      double x {0.0};
      double y {0.0};
      double z {0.0};
      double w {0.0};
    };

    struct Accel
    {
      Vector3 linear;
      Vector3 angular;
    };

    struct AccelStamped
    {
      stdMsgs::Header header;
      Accel accel;
    };

    struct AccelWithCovariance
    {
      Accel accel;
      double covariance[36];
    };

    struct AccelWithCovarianceStamped
    {
      stdMsgs::Header header;
      AccelWithCovariance accel;
    };

    struct Inertia
    {
      double m {0.0};
      Vector3 com;
      double ixx {0.0};
      double ixy {0.0};
      double ixz {0.0};
      double iyy {0.0};
      double iyz {0.0};
      double izz {0.0};
    };

    struct InertiaStamped
    {
      stdMsgs::Header header;
      Inertia inertia;
    };

    struct Point
    {
      double x {0.0};
      double y {0.0};
      double z {0.0};
    };

    struct Point32
    {
      float x {0.f};
      float y {0.f};
      float z {0.f};
    };

    struct PointStamped
    {
      stdMsgs::Header header;
      Point point;
    };

    struct Polygon
    {
      std::vector<Point32> points;
    };

    struct PolygonStamped
    {
      stdMsgs::Header header;
      Polygon polygon;
    };

    struct Pose
    {
      Point position;
      Quaternion orientation;
    };

    struct Pose2D
    {
      double x {0.0};
      double y {0.0};
      double theta {0.0};
    };

    struct PoseArray
    {
      stdMsgs::Header header;
      std::vector<Pose> poses;
    };

    struct PoseStamped
    {
      stdMsgs::Header header;
      Pose pose;
    };

    struct PoseWithCovariance
    {
      Pose pose;
      double covariance[36];
    };

    struct PoseWithCovarianceStamped
    {
      stdMsgs::Header header;
      PoseWithCovariance pose;
    };

    struct QuaternionStamped
    {
      stdMsgs::Header header;
      Quaternion quaternion;
    };

    struct Transform
    {
      Vector3 translation;
      Quaternion rotation;
    };

    struct TransformStamped
    {
      stdMsgs::Header header;
      std::string child_frame_id;
      Transform transform;
    };

    struct Twist
    {
      Vector3 linear;
      Vector3 angular;
    };

    struct TwistStamped
    {
      stdMsgs::Header header;
      Twist twist;
    };

    struct TwistWithCovariance
    {
      Twist twist;
      double covariance[36];
    };

    struct TwistWithCovarianceStamped
    {
      stdMsgs::Header header;
      TwistWithCovariance twist;
    };

    struct Vector3Stamped
    {
      stdMsgs::Header header;
      Vector3 vector;
    };

    struct Wrench
    {
      Vector3 force;
      Vector3 torque;
    };

    struct WrenchStamped
    {
      stdMsgs::Header header;
      Wrench wrench;
    };
  }

  namespace sensorMsgs
  {
    struct BatteryState
    {
      stdMsgs::Header header;
      float voltage {0.f};
      float temperature {0.f};
      float current {0.f};
      float charge {0.f};
      float capacity {0.f};
      float design_capacity {0.f};
      float percentage {0.f};
      uint8_t power_supply_status {0};
      uint8_t power_supply_health {0};
      uint8_t power_supply_technology {0};
      bool present = false;
      std::vector<float> cell_voltage;
      std::vector<float> cell_temperature;
      std::string location;
      std::string serial_number;
    };

    struct ChannelFloat32
    {
      std::string name;
      std::vector<float> values;
    };

    struct CompressedImage
    {
      stdMsgs::Header header;
      std::string format;
      std::vector<uint8_t> data;
    };

    struct FluidPressure
    {
      stdMsgs::Header header;
      double fluid_pressure {0.0};
      double variance {0.0};
    };

    struct Illuminance
    {
      stdMsgs::Header header;
      double illuminance {0.0};
      double variance {0.0};
    };

    struct Image
    {
      stdMsgs::Header header;
      uint32_t height {0};
      uint32_t width {0};
      std::string encoding;
      uint8_t is_bigendian {0};
      uint32_t step {0};
      std::vector<uint8_t> data;
    };

    struct Imu
    {
      stdMsgs::Header header;
      geometryMsgs::Quaternion orientation;
      double orientation_covariance[9];
      geometryMsgs::Vector3 angular_velocity;
      double angular_velocity_covariance[9];
      geometryMsgs::Vector3 linear_acceleration;
      double linear_acceleration_covariance[9];
    };

    struct JointState
    {
      stdMsgs::Header header;
      std::vector<std::string> name;
      std::vector<double> position;
      std::vector<double> velocity;
      std::vector<double> effort;
    };

    struct Joy
    {
      stdMsgs::Header header;
      std::vector<float> axes;
      std::vector<int32_t> buttons;
    };

    struct JoyFeedback
    {
      uint8_t type {0};
      uint8_t id {0};
      float intensity {0.f};
    };

    struct JoyFeedbackArray
    {
      std::vector<JoyFeedback> array;
    };

    struct LaserEcho
    {
      std::vector<float> echoes;
    };

    struct LaserScan
    {
      stdMsgs::Header header;
      float angle_min {0.f};
      float angle_max {0.f};
      float angle_increment {0.f};
      float time_increment {0.f};
      float scan_time {0.f};
      float range_min {0.f};
      float range_max {0.f};
      std::vector<float> ranges;
      std::vector<float> intensities;
    };

    struct MagneticField
    {
      stdMsgs::Header header;
      geometryMsgs::Vector3 magnetic_field;
      double magnetic_field_covariance[9];
    };

    struct MultiDOFJointState
    {
      stdMsgs::Header header;
      std::vector<std::string> joint_names;
      std::vector<geometryMsgs::Transform> transforms;
      std::vector<geometryMsgs::Twist> twist;
      std::vector<geometryMsgs::Wrench> wrench;
    };

    struct MultiEchoLaserScan
    {
      stdMsgs::Header header;
      float angle_min {0.f};
      float angle_max {0.f};
      float angle_increment {0.f};
      float time_increment {0.f};
      float scan_time {0.f};
      float range_min {0.f};
      float range_max {0.f};
      std::vector<LaserEcho> ranges;
      std::vector<LaserEcho> intensities;
    };

    struct NavSatStatus
    {
      int8_t status {0};
      uint16_t service {0};
    };

    struct NavSatFix
    {
      stdMsgs::Header header;
      NavSatStatus status;
      double latitude {0.0};
      double longitude {0.0};
      double altitude {0.0};
      double position_covariance[9];
      uint8_t position_covariance_type {0};
    };

    struct PointCloud
    {
      stdMsgs::Header header;
      std::vector<geometryMsgs::Point32> points;
      std::vector<ChannelFloat32> channels;
    };

    struct PointField
    {
      std::string name;
      uint32_t offset {0};
      uint8_t datatype {0};
      uint32_t count {0};
    };

    struct PointCloud2
    {
      stdMsgs::Header header;
      uint32_t height {0};
      uint32_t width {0};
      std::vector<PointField> fields;
      bool is_bigendian = false;
      uint32_t point_step  {0};
      uint32_t row_step {0};
      std::vector<uint8_t> data;
      bool is_dense = false;
    };

    struct Range
    {
      stdMsgs::Header header;
      uint8_t radiation_type {0};
      float field_of_view {0.f};
      float min_range {0.f};
      float max_range {0.f};
      float range {0.f};
    };

    struct RegionOfInterest
    {
      uint32_t x_offset {0};
      uint32_t y_offset {0};
      uint32_t height {0};
      uint32_t width {0};
      bool do_rectify = false;
    };

    struct CameraInfo
    {
      stdMsgs::Header header;
      uint32_t height {0};
      uint32_t width {0};
      std::string distortion_model;
      std::vector<double> d;
      double k[9];
      double r[9];
      double p[12];
      uint32_t binning_x {0};
      uint32_t binning_y {0};
      RegionOfInterest roi;
    };

    struct RelativeHumidity
    {
      stdMsgs::Header header;
      double relative_humidity {0.0};
      double variance {0.0};
    };

    struct Temperature
    {
      stdMsgs::Header header;
      double temperature {0.0};
      double variance {0.0};
    };

    struct TimeReference
    {
      stdMsgs::Header header;
      builtinInterfaces::Time time_ref;
      std::string source;
    };
  }
}

#ifdef _MSC_VER
# pragma warning(pop)
#endif
