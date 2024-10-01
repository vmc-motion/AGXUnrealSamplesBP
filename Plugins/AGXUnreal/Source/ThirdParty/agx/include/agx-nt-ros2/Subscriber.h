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
  struct DdsSubscriberTypes;
}

#ifdef _MSC_VER
# pragma warning(push)
# pragma warning(disable: 4251) // Disable warnings about members needing dll interface.
#endif

namespace agxROS2
{
  /**
  Subscriber class that enables receiving ROS2 messages.
  */
  template <typename TMsg>
  class AGXNTROS2_EXPORT Subscriber
  {
  public:
    /**
    Constructor for the Subscriber.
    \param topic - the topic on which the Subscriber will receive messages.
    \param qos - Quality of Service (QOS) specifies behaviours such as message lifetime among other things.
    \param inDomainId - determines which DDS domain the Subscriber uses. Default for ROS2 is 0.
    */
    Subscriber(const std::string& topic, const QOS& qos = QOS(), int8_t inDomainId = 0);
    ~Subscriber();

    /**
    Try to receive a message.
    This is a non-blocking function, i.e. it returns immediately if no new messages are avaiable.
    \param outMsg - out parameter containing the received message if a receive was successful.
    \return - true if a message was received, false otherwise.
    */
    bool receiveMessage(TMsg& outMsg) const;

  private:
    void init(const std::string& topic, const QOS& qos);
    std::unique_ptr<DdsSubscriberTypes> ddsTypes;
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
    using SubscriberAny = Subscriber<Any>;
    using SubscriberAnySequence = Subscriber<AnySequence>;
  }

  namespace builtinInterfaces
  {
    using SubscriberTime = Subscriber<Time>;
    using SubscriberDuration = Subscriber<Duration>;
  }

  namespace rosgraphMsgs
  {
    using SubscriberClock = Subscriber<Clock>;
  }

  namespace stdMsgs
  {
    using SubscriberBool = Subscriber<Bool>;
    using SubscriberByte = Subscriber<Byte>;
    using SubscriberByteMultiArray = Subscriber<ByteMultiArray>;
    using SubscriberChar = Subscriber<Char>;
    using SubscriberColorRGBA = Subscriber<ColorRGBA>;
    using SubscriberEmpty = Subscriber<Empty>;
    using SubscriberFloat32 = Subscriber<Float32>;
    using SubscriberFloat32MultiArray = Subscriber<Float32MultiArray>;
    using SubscriberFloat64 = Subscriber<Float64>;
    using SubscriberFloat64MultiArray = Subscriber<Float64MultiArray>;
    using SubscriberInt16 = Subscriber<Int16>;
    using SubscriberInt16MultiArray = Subscriber<Int16MultiArray>;
    using SubscriberInt32 = Subscriber<Int32>;
    using SubscriberInt32MultiArray = Subscriber<Int32MultiArray>;
    using SubscriberInt64 = Subscriber<Int64>;
    using SubscriberInt64MultiArray = Subscriber<Int64MultiArray>;
    using SubscriberInt8 = Subscriber<Int8>;
    using SubscriberInt8MultiArray = Subscriber<Int8MultiArray>;
    using SubscriberString = Subscriber<String>;
    using SubscriberUInt16 = Subscriber<UInt16>;
    using SubscriberUInt16MultiArray = Subscriber<UInt16MultiArray>;
    using SubscriberUInt32 = Subscriber<UInt32>;
    using SubscriberUInt32MultiArray = Subscriber<UInt32MultiArray>;
    using SubscriberUInt64 = Subscriber<UInt64>;
    using SubscriberUInt64MultiArray = Subscriber<UInt64MultiArray>;
    using SubscriberUInt8 = Subscriber<UInt8>;
    using SubscriberUInt8MultiArray = Subscriber<UInt8MultiArray>;
    using SubscriberHeader = Subscriber<Header>;
  }

  namespace geometryMsgs
  {
    using SubscriberVector3 = Subscriber<Vector3>;
    using SubscriberQuaternion = Subscriber<Quaternion>;
    using SubscriberAccel = Subscriber<Accel>;
    using SubscriberAccelStamped = Subscriber<AccelStamped>;
    using SubscriberAccelWithCovariance = Subscriber<AccelWithCovariance>;
    using SubscriberAccelWithCovarianceStamped = Subscriber<AccelWithCovarianceStamped>;
    using SubscriberInertia = Subscriber<Inertia>;
    using SubscriberInertiaStamped = Subscriber<InertiaStamped>;
    using SubscriberPoint = Subscriber<Point>;
    using SubscriberPoint32 = Subscriber<Point32>;
    using SubscriberPointStamped = Subscriber<PointStamped>;
    using SubscriberPolygon = Subscriber<Polygon>;
    using SubscriberPolygonStamped = Subscriber<PolygonStamped>;
    using SubscriberPose = Subscriber<Pose>;
    using SubscriberPose2D = Subscriber<Pose2D>;
    using SubscriberPoseArray = Subscriber<PoseArray>;
    using SubscriberPoseStamped = Subscriber<PoseStamped>;
    using SubscriberPoseWithCovariance = Subscriber<PoseWithCovariance>;
    using SubscriberPoseWithCovarianceStamped = Subscriber<PoseWithCovarianceStamped>;
    using SubscriberQuaternionStamped = Subscriber<QuaternionStamped>;
    using SubscriberTransform = Subscriber<Transform>;
    using SubscriberTransformStamped = Subscriber<TransformStamped>;
    using SubscriberTwist = Subscriber<Twist>;
    using SubscriberTwistStamped = Subscriber<TwistStamped>;
    using SubscriberTwistWithCovariance = Subscriber<TwistWithCovariance>;
    using SubscriberTwistWithCovarianceStamped = Subscriber<TwistWithCovarianceStamped>;
    using SubscriberVector3Stamped = Subscriber<Vector3Stamped>;
    using SubscriberWrench = Subscriber<Wrench>;
    using SubscriberWrenchStamped = Subscriber<WrenchStamped>;
  }

  namespace sensorMsgs
  {
    using SubscriberBatteryState = Subscriber<BatteryState>;
    using SubscriberChannelFloat32 = Subscriber<ChannelFloat32>;
    using SubscriberCompressedImage = Subscriber<CompressedImage>;
    using SubscriberFluidPressure = Subscriber<FluidPressure>;
    using SubscriberIlluminance = Subscriber<Illuminance>;
    using SubscriberImage = Subscriber<Image>;
    using SubscriberImu = Subscriber<Imu>;
    using SubscriberJointState = Subscriber<JointState>;
    using SubscriberJoy = Subscriber<Joy>;
    using SubscriberJoyFeedback = Subscriber<JoyFeedback>;
    using SubscriberJoyFeedbackArray = Subscriber<JoyFeedbackArray>;
    using SubscriberLaserEcho = Subscriber<LaserEcho>;
    using SubscriberLaserScan = Subscriber<LaserScan>;
    using SubscriberMagneticField = Subscriber<MagneticField>;
    using SubscriberMultiDOFJointState = Subscriber<MultiDOFJointState>;
    using SubscriberMultiEchoLaserScan = Subscriber<MultiEchoLaserScan>;
    using SubscriberNavSatStatus = Subscriber<NavSatStatus>;
    using SubscriberNavSatFix = Subscriber<NavSatFix>;
    using SubscriberPointCloud = Subscriber<PointCloud>;
    using SubscriberPointField = Subscriber<PointField>;
    using SubscriberPointCloud2 = Subscriber<PointCloud2>;
    using SubscriberRange = Subscriber<Range>;
    using SubscriberRegionOfInterest = Subscriber<RegionOfInterest>;
    using SubscriberCameraInfo = Subscriber<CameraInfo>;
    using SubscriberRelativeHumidity = Subscriber<RelativeHumidity>;
    using SubscriberTemperature = Subscriber<Temperature>;
    using SubscriberTimeReference = Subscriber<TimeReference>;
  }
}

#ifdef _MSC_VER
# pragma warning(pop)
#endif
