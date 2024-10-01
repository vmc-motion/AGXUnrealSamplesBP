/*
Copyright 2007-2024. Algoryx Simulation AB.

All AGX source code, intellectual property, documentation, sample code,
tutorials, scene files and technical white papers, are copyrighted, proprietary
and confidential material of Algoryx Simulation AB. You may not download, read,
store, distribute, publish, copy or otherwise disseminate, use or expose this
material unless having a written signed agreement with Algoryx Simulation AB, or
having been advised so by Algoryx Simulation AB for a time limited evaluation,
or having purchased a valid commercial license from Algoryx Simulation AB.

Algoryx Simulation AB disclaims all responsibilities for loss or damage caused
from using this software, unless otherwise stated in written agreements with
Algoryx Simulation AB.
*/


/////////////////////////////////////////////////////////////////////
// AUTOMATICALLY GENERATED, DO NOT EDIT! (except inline functions) //
/////////////////////////////////////////////////////////////////////

#ifndef AGXFN_MATH_GEOMETRY_SPHERE_CALCULATEVOLUME_H
#define AGXFN_MATH_GEOMETRY_SPHERE_CALCULATEVOLUME_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>


namespace agx { namespace Math { namespace Geometry { namespace Sphere { } } } }

namespace agxFn
{
  namespace Math
  {
    namespace Geometry
    {
      namespace Sphere
      {
        /**
        Function: Math.Geometry.Sphere.CalculateVolume
        Implementation: (default)

        \param volume 
        \param radius 
        */
        AGX_FORCE_INLINE void CalculateVolume
        (
          /* Parameter list automatically generated, do not edit */
          agx::Real& volume,
          const agx::Real& radius
        )
        {
          // 4 pi r^3 / 3
          volume = radius * radius * radius * (agx::Real(4) * agx::PI / agx::Real(3));
        }

        /// Generated return-value wrapper. DO NOT EDIT!
        AGX_FORCE_INLINE agx::Real CalculateVolume
        (
          const agx::Real& radius
        )
        {
          agx::Real volume;
          agxFn::Math::Geometry::Sphere::CalculateVolume(volume, radius);
          return volume;
        }
        


      }
    }
  }
}

#endif
