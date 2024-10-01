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

#ifndef AGXFN_PHYSICS_GEOMETRY_RENDERING_GENERATERENDERBOXESFROMCOLLISIONBOXES_H
#define AGXFN_PHYSICS_GEOMETRY_RENDERING_GENERATERENDERBOXESFROMCOLLISIONBOXES_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Vec3.h>
#include <agxGL/RenderBox.h>


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
        Function: Physics.Geometry.Rendering.GenerateRenderBoxesFromCollisionBoxes
        Implementation: (default)

        \param job The range job specifying what part of the data set to process
        \param transforms 
        \param halfExtentsBuffer 
        \param vertexBuffer 
        \param normalBuffer 
        */
        void GenerateRenderBoxesFromCollisionBoxes
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          const agxData::Array< agx::AffineMatrix4x4 >& transforms,
          const agxData::Array< agx::Vec3 >& halfExtentsBuffer,
          agxData::Array< agxGL::RenderBoxVertices >& vertexBuffer,
          agxData::Array< agxGL::RenderBoxNormals >& normalBuffer
        );


      }
    }
  }
}

#endif
