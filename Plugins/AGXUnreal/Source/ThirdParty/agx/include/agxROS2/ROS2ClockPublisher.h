/*
Copyright 2007-2024. Algoryx Simulation AB.

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
#include <agxROS2/export.h>
#include <agxROS2/Publisher.h>
#include <agxROS2/Qos.h>

#include <agxSDK/StepEventListener.h>

namespace agxROS2
{
  class AGXROS2_EXPORT ROS2ClockPublisher : public agxSDK::StepEventListener
  {
    public:
      /**
      Constructor creates the ROS2ClockPublisher StepEventListener. When added to the simulation it will publish the current
      simulation time on the /clock topic in each pre-step
      */
      ROS2ClockPublisher();

      /**
      Publish the current simulation time as a agxROS2::rosgraphMsgs::Clock message
      */
      virtual void pre(const agx::TimeStamp& time) override;

    private:
      agxROS2::rosgraphMsgs::PublisherClock m_pub;
      agxROS2::rosgraphMsgs::Clock m_msg;
  };
}
