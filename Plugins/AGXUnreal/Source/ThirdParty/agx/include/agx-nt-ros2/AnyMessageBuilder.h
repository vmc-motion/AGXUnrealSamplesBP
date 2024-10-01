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
#include <cstdint>
#include <memory>
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
    struct Any;
  }
}


namespace agxROS2
{
  /**
  AnyMessageBuilder is a helper class for serializing custom data structures to an agxMsgs::Any
  message type that can then be sent via ROS2 and deserialized using the AnyMessageParser.
  */
  class AGXNTROS2_EXPORT AnyMessageBuilder
  {
  public:
    AnyMessageBuilder();
    AnyMessageBuilder(const AnyMessageBuilder& other) = delete;
    AnyMessageBuilder(AnyMessageBuilder&& other) noexcept;
    AnyMessageBuilder operator=(const AnyMessageBuilder& other) = delete;
    AnyMessageBuilder& operator=(AnyMessageBuilder&& other) noexcept;
    ~AnyMessageBuilder();

    AnyMessageBuilder& beginMessage();

    AnyMessageBuilder& writeInt8(int8_t d);
    AnyMessageBuilder& writeUInt8(uint8_t d);
    AnyMessageBuilder& writeInt16(int16_t d);
    AnyMessageBuilder& writeUInt16(uint16_t d);
    AnyMessageBuilder& writeInt32(int32_t d);
    AnyMessageBuilder& writeUInt32(uint32_t d);
    AnyMessageBuilder& writeInt64(int64_t d);
    AnyMessageBuilder& writeUInt64(uint64_t d);
    AnyMessageBuilder& writeFloat32(float d);
    AnyMessageBuilder& writeDouble64(double d);
    AnyMessageBuilder& writeString(const std::string& d);
    AnyMessageBuilder& writeBool(bool d);

    AnyMessageBuilder& writeInt8Sequence(const std::vector<int8_t>& d);
    AnyMessageBuilder& writeUInt8Sequence(const std::vector<uint8_t>& d);
    AnyMessageBuilder& writeInt16Sequence(const std::vector<int16_t>& d);
    AnyMessageBuilder& writeUInt16Sequence(const std::vector<uint16_t>& d);
    AnyMessageBuilder& writeInt32Sequence(const std::vector<int32_t>& d);
    AnyMessageBuilder& writeUInt32Sequence(const std::vector<uint32_t>& d);
    AnyMessageBuilder& writeInt64Sequence(const std::vector<int64_t>& d);
    AnyMessageBuilder& writeUInt64Sequence(const std::vector<uint64_t>& d);
    AnyMessageBuilder& writeFloat32Sequence(const std::vector<float>& d);
    AnyMessageBuilder& writeDouble64Sequence(const std::vector<double>& d);
    AnyMessageBuilder& writeStringSequence(const std::vector<std::string>& d);
    AnyMessageBuilder& writeBoolSequence(const std::vector<bool>& d);

    const agxROS2::agxMsgs::Any& getMessage() const;

    void reserveBytes(size_t bytes);

  private:
    std::unique_ptr<agxROS2::agxMsgs::Any> message;
  };
}

#ifdef _MSC_VER
# pragma warning(pop)
#endif
