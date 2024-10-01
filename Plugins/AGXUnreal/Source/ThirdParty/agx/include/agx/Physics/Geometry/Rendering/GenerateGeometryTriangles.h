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

#ifndef AGXFN_PHYSICS_GEOMETRY_RENDERING_GENERATEGEOMETRYTRIANGLES_H
#define AGXFN_PHYSICS_GEOMETRY_RENDERING_GENERATEGEOMETRYTRIANGLES_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/Vec4.h>
#include <agx/Matrix4x4.h>
#include <agx/Physics/Geometry/TriangleEntity.h>
#include <agx/Vec3.h>
#include <agx/GlobalResult.h>


namespace agx { namespace Physics { namespace Geometry { namespace Rendering { } } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Geometry
    {
      namespace Rendering
      {
        /**
        Function: Physics.Geometry.Rendering.GenerateGeometryTriangles
        Implementation: (default)

        \param job The range job specifying what part of the data set to process
        \param transformBuffer 
        \param shapeBuffer 
        \param color 
        \param cameraProjectionViewMatrix 
        \param triangle 
        */
        void GenerateGeometryTriangles
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          const agxData::Array< agx::AffineMatrix4x4 >& transformBuffer,
          agxData::Array< agx::Physics::Geometry::ShapePtr >& shapeBuffer,
          const agx::Vec4& color,
          const agx::Matrix4x4& cameraProjectionViewMatrix,
          agx::Physics::Geometry::TriangleData& triangle
        );


      }
    }
  }
}

#endif
