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

namespace agxROS2
{
  enum class QOS_RELIABILITY
  {
    DEFAULT,
    BEST_EFFORT,
    RELIABLE
  };

  enum class QOS_DURABILITY
  {
    DEFAULT,
    VOLATILE,
    TRANSIENT_LOCAL,
    TRANSIENT,
    PERSISTENT
  };

  enum class QOS_HISTORY
  {
    DEFAULT,
    KEEP_LAST_HISTORY_QOS,
    KEEP_ALL_HISTORY_QOS
  };

  /**
  Struct containing Quality of Service (QOS) settings.
  Reliability, durability, history and historyDepth are supported.
  By default the QOS settings are the same as in ROS2, i.e.
  RELIABILITY "reliable", DURABILITY "volatile", HISTORY "keep last" and history depth 10.
  */
  struct AGXNTROS2_EXPORT QOS
  {
    QOS_RELIABILITY reliability{QOS_RELIABILITY::DEFAULT};
    QOS_DURABILITY durability{QOS_DURABILITY::DEFAULT};
    QOS_HISTORY history{QOS_HISTORY::DEFAULT};
    int32_t historyDepth{10}; // ROS2 default.
  };

  /**
  Create QOS matching ROS2 Default QOS.
  \return - a QOS matching ROS2 Default QOS.
  */
  AGXNTROS2_EXPORT QOS createDefaultQOS();

  /**
  Create QOS matching ROS2 ClockQoS.
  \return - a QOS matching ROS2 ClockQoS.
  */
  AGXNTROS2_EXPORT QOS createClockQOS();

  /**
  Create QOS matching ROS2 SensorDataQoS.
  \return - a QOS matching ROS2 SensorDataQoS.
  */
  AGXNTROS2_EXPORT QOS createSensorDataQOS();

  /**
  Create QOS matching ROS2 ParametersQoS.
  \return - a QOS matching ROS2 ParametersQoS.
  */
  AGXNTROS2_EXPORT QOS createParametersQOS();

  /**
  Create QOS matching ROS2 ServicesQoS.
  \return - a QOS matching ROS2 ServicesQoS.
  */
  AGXNTROS2_EXPORT QOS createServicesQOS();

  /**
  Create QOS matching ROS2 ParameterEventsQoS.
  \return - a QOS matching ROS2 ParameterEventsQoS.
  */
  AGXNTROS2_EXPORT QOS createParameterEventsQOS();

  /**
  Create QOS matching ROS2 RosoutQoS.
  \return - a QOS matching ROS2 RosoutQoS.
  */
  AGXNTROS2_EXPORT QOS createRosoutQOS();
}
