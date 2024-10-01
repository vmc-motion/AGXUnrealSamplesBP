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
#include "agx-nt-ros2/MessageTypes.h"
#include "agx-nt-ros2/Qos.h"

// Standard library includes.
#include <memory>
#include <string>


namespace agxROS2
{
  struct DdsPublisherTypes;
}

#ifdef _MSC_VER
# pragma warning(push)
# pragma warning(disable: 4251) // Disable warnings about members needing dll interface.
#endif

namespace agxROS2
{
  /**
  Publisher class that enables sending ROS2 messages.
  */
  template <typename TMsg>
  class AGXNTROS2_EXPORT Publisher
  {
  public:
    /**
    Constructor for the Publisher.
    \param topic - the topic on which the Subscriber will receive messages.
    \param qos - Quality of Service (QOS) specifies behaviours such as message lifetime among other things.
    \param inDomainId - determines which DDS domain the Subscriber uses. Default for ROS2 is 0.
    */
    Publisher(const std::string& topic, const QOS& qos = QOS(), int8_t inDomainId = 0);
    ~Publisher();

    /**
    Sends a message.
    \param msg - the message to send.
    */
    void sendMessage(const TMsg& msg);

  private:
    void init(const std::string& topic, const QOS& qos);

    std::unique_ptr<DdsPublisherTypes> ddsTypes;
    std::string typeName;
    int8_t domainId{ 0 };
  };
}


//
// Predefined publisher types.
//

namespace agxROS2
{
  namespace agxMsgs
  {
    using PublisherAny = Publisher<Any>;
    using PublisherAnySequence = Publisher<AnySequence>;
  }

  namespace builtinInterfaces
  {
    using PublisherTime = Publisher<Time>;
    using PublisherDuration = Publisher<Duration>;
  }

  namespace rosgraphMsgs
  {
    using PublisherClock = Publisher<Clock>;
  }

  namespace stdMsgs
  {
    using PublisherBool = Publisher<Bool>;
    using PublisherByte = Publisher<Byte>;
    using PublisherByteMultiArray = Publisher<ByteMultiArray>;
    using PublisherChar = Publisher<Char>;
    using PublisherColorRGBA = Publisher<ColorRGBA>;
    using PublisherEmpty = Publisher<Empty>;
    using PublisherFloat32 = Publisher<Float32>;
    using PublisherFloat32MultiArray = Publisher<Float32MultiArray>;
    using PublisherFloat64 = Publisher<Float64>;
    using PublisherFloat64MultiArray = Publisher<Float64MultiArray>;
    using PublisherInt16 = Publisher<Int16>;
    using PublisherInt16MultiArray = Publisher<Int16MultiArray>;
    using PublisherInt32 = Publisher<Int32>;
    using PublisherInt32MultiArray = Publisher<Int32MultiArray>;
    using PublisherInt64 = Publisher<Int64>;
    using PublisherInt64MultiArray = Publisher<Int64MultiArray>;
    using PublisherInt8 = Publisher<Int8>;
    using PublisherInt8MultiArray = Publisher<Int8MultiArray>;
    using PublisherString = Publisher<String>;
    using PublisherUInt16 = Publisher<UInt16>;
    using PublisherUInt16MultiArray = Publisher<UInt16MultiArray>;
    using PublisherUInt32 = Publisher<UInt32>;
    using PublisherUInt32MultiArray = Publisher<UInt32MultiArray>;
    using PublisherUInt64 = Publisher<UInt64>;
    using PublisherUInt64MultiArray = Publisher<UInt64MultiArray>;
    using PublisherUInt8 = Publisher<UInt8>;
    using PublisherUInt8MultiArray = Publisher<UInt8MultiArray>;
    using PublisherHeader = Publisher<Header>;
  }

  namespace geometryMsgs
  {
    using PublisherVector3 = Publisher<Vector3>;
    using PublisherQuaternion = Publisher<Quaternion>;
    using PublisherAccel = Publisher<Accel>;
    using PublisherAccelStamped = Publisher<AccelStamped>;
    using PublisherAccelWithCovariance = Publisher<AccelWithCovariance>;
    using PublisherAccelWithCovarianceStamped = Publisher<AccelWithCovarianceStamped>;
    using PublisherInertia = Publisher<Inertia>;
    using PublisherInertiaStamped = Publisher<InertiaStamped>;
    using PublisherPoint = Publisher<Point>;
    using PublisherPoint32 = Publisher<Point32>;
    using PublisherPointStamped = Publisher<PointStamped>;
    using PublisherPolygon = Publisher<Polygon>;
    using PublisherPolygonStamped = Publisher<PolygonStamped>;
    using PublisherPose = Publisher<Pose>;
    using PublisherPose2D = Publisher<Pose2D>;
    using PublisherPoseArray = Publisher<PoseArray>;
    using PublisherPoseStamped = Publisher<PoseStamped>;
    using PublisherPoseWithCovariance = Publisher<PoseWithCovariance>;
    using PublisherPoseWithCovarianceStamped = Publisher<PoseWithCovarianceStamped>;
    using PublisherQuaternionStamped = Publisher<QuaternionStamped>;
    using PublisherTransform = Publisher<Transform>;
    using PublisherTransformStamped = Publisher<TransformStamped>;
    using PublisherTwist = Publisher<Twist>;
    using PublisherTwistStamped = Publisher<TwistStamped>;
    using PublisherTwistWithCovariance = Publisher<TwistWithCovariance>;
    using PublisherTwistWithCovarianceStamped = Publisher<TwistWithCovarianceStamped>;
    using PublisherVector3Stamped = Publisher<Vector3Stamped>;
    using PublisherWrench = Publisher<Wrench>;
    using PublisherWrenchStamped = Publisher<WrenchStamped>;
  }

  namespace sensorMsgs
  {
    using PublisherBatteryState = Publisher<BatteryState>;
    using PublisherChannelFloat32 = Publisher<ChannelFloat32>;
    using PublisherCompressedImage = Publisher<CompressedImage>;
    using PublisherFluidPressure = Publisher<FluidPressure>;
    using PublisherIlluminance = Publisher<Illuminance>;
    using PublisherImage = Publisher<Image>;
    using PublisherImu = Publisher<Imu>;
    using PublisherJointState = Publisher<JointState>;
    using PublisherJoy = Publisher<Joy>;
    using PublisherJoyFeedback = Publisher<JoyFeedback>;
    using PublisherJoyFeedbackArray = Publisher<JoyFeedbackArray>;
    using PublisherLaserEcho = Publisher<LaserEcho>;
    using PublisherLaserScan = Publisher<LaserScan>;
    using PublisherMagneticField = Publisher<MagneticField>;
    using PublisherMultiDOFJointState = Publisher<MultiDOFJointState>;
    using PublisherMultiEchoLaserScan = Publisher<MultiEchoLaserScan>;
    using PublisherNavSatStatus = Publisher<NavSatStatus>;
    using PublisherNavSatFix = Publisher<NavSatFix>;
    using PublisherPointCloud = Publisher<PointCloud>;
    using PublisherPointField = Publisher<PointField>;
    using PublisherPointCloud2 = Publisher<PointCloud2>;
    using PublisherRange = Publisher<Range>;
    using PublisherRegionOfInterest = Publisher<RegionOfInterest>;
    using PublisherCameraInfo = Publisher<CameraInfo>;
    using PublisherRelativeHumidity = Publisher<RelativeHumidity>;
    using PublisherTemperature = Publisher<Temperature>;
    using PublisherTimeReference = Publisher<TimeReference>;
  }
}

#ifdef _MSC_VER
# pragma warning(pop)
#endif
