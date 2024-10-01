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

#ifdef _WIN32
#if defined(_MSC_VER)
#  if defined(AGXNTROS2_LIBRARY)
#    define AGXNTROS2_EXPORT   __declspec(dllexport)
#  else
#    define AGXNTROS2_EXPORT   __declspec(dllimport)
#  endif
#else
#  define AGXNTROS2_EXPORT
#endif
#else
   // Non Win32
  #if __GNUC__ >= 4
    #define AGXNTROS2_EXPORT __attribute__ ((visibility("default")))
  #else
    #define AGXNTROS2_EXPORT
  #endif
#endif
