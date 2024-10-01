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

// Standard libary includes.
#include <cstdint>
#include <vector>
#include <string>

#ifdef _MSC_VER
# pragma warning(push)
# pragma warning(disable: 4251) // Disable warnings about members needing dll interface.
#endif

namespace agxROS2
{
  namespace agxMsgs
  {
    struct Any;
  }
}

namespace agxROS2
{
  /**
  AnyMessageParser is a helper class for deserializing agxMsgs::Any back to the original custom
  data type that was created using the AnyMessageBuilder.
  */
  class AGXNTROS2_EXPORT AnyMessageParser
  {
  public:

    /** Must be called once prior to parsing a single complete message. */
    void beginParse();

    int8_t readInt8(const agxROS2::agxMsgs::Any& message);
    uint8_t readUInt8(const agxROS2::agxMsgs::Any& message);
    int16_t readInt16(const agxROS2::agxMsgs::Any& message);
    uint16_t readUInt16(const agxROS2::agxMsgs::Any& message);
    int32_t readInt32(const agxROS2::agxMsgs::Any& message);
    uint32_t readUInt32(const agxROS2::agxMsgs::Any& message);
    int64_t readInt64(const agxROS2::agxMsgs::Any& message);
    uint64_t readUInt64(const agxROS2::agxMsgs::Any& message);
    float readFloat32(const agxROS2::agxMsgs::Any& message);
    double readDouble64(const agxROS2::agxMsgs::Any& message);
    std::string readString(const agxROS2::agxMsgs::Any& message);
    bool readBool(const agxROS2::agxMsgs::Any& message);

    std::vector<int8_t> readInt8Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<uint8_t> readUInt8Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<int16_t> readInt16Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<uint16_t> readUInt16Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<int32_t> readInt32Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<uint32_t> readUInt32Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<int64_t> readInt64Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<uint64_t> readUInt64Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<float> readFloat32Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<double> readDouble64Sequence(const agxROS2::agxMsgs::Any& message);
    std::vector<std::string> readStringSequence(const agxROS2::agxMsgs::Any& message);
    std::vector<bool> readBoolSequence(const agxROS2::agxMsgs::Any& message);

  private:
    size_t index{ 0 };
  };
}


#ifdef _MSC_VER
# pragma warning(pop)
#endif
