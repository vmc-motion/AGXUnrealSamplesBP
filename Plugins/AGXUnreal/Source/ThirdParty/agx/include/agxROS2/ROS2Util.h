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

#include "agxROS2/export.h"

namespace agxROS2
{
    /**
    Free the memory held by the given container by swapping it with a
    default-constructed instance.

    This is required when an application is using a different system allocator
    than the AGX Dynamics shared library and a container is passed by-value
    from AGX Dynamics to the application. When the container goes out of scope
    the application's system allocator will try to deallocate the container's
    buffer, a buffer that was allocated with AGX Dynamics' system allocator.
    This will either cause memory errors or a crash. By first calling
    freeContainerMemory the buffer will be freed inside AGX Dynamics, with
    AGX Dynamics' system allocator, and the application's system allocator will
    not need to do anything.
    */
    template<typename ContainerT>
    void freeContainerMemory(ContainerT& container);
}
