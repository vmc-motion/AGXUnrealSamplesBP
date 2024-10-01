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

/*
This source code has been taken and modified by Algoryx Simulation AB
from the source and under the license given below.
*/

#ifndef AGXCOLLIDE_CONVEX_DECOMPOSITION_EXPORT_H
#define AGXCOLLIDE_CONVEX_DECOMPOSITION_EXPORT_H

#include <agx/config.h>
#ifdef _WIN32

#define CONVEX_LIBRARY_STATIC
#if AGX_DYNAMIC() &&  defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
#  if defined( CONVEX_LIBRARY_STATIC )
#    define CONVEX_EXPORT
#  elif defined( CONVEX_LIBRARY )
#    define CONVEX_EXPORT   __declspec(dllexport)
#  else
#    define CONVEX_EXPORT   __declspec(dllimport)
#  endif
#else
#  define CONVEX_EXPORT
#endif

#else
  #define CONVEX_EXPORT
#endif

#endif // AGXCOLLIDE_CONVEX_DECOMPOSITION_EXPORT_H
