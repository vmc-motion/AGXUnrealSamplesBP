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

#ifndef AGXFN_FOUNDATION_RENDERING_GENERATEBOXESFROMBOXES_H
#define AGXFN_FOUNDATION_RENDERING_GENERATEBOXESFROMBOXES_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>
#include <agx/AffineMatrix4x4.h>
#include <agxGL/RenderBox.h>
#include <agx/Line.h>


namespace agx { namespace Foundation { namespace Rendering { } } }

namespace agxFn
{
  namespace Foundation
  {
    namespace Rendering
    {
      /**
      Function: Foundation.Rendering.GenerateBoxesFromBoxes
      Implementation: RenderBox

      \param job The range job specifying what part of the data set to process
      \param halfExtents 
      \param transforms 
      \param renderBox_vertices 
      \param renderBox_normals 
      \param renderBox_outlines 
      */
      void GenerateBoxesFromBoxes__RenderBox
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Vec3 >& halfExtents,
        const agxData::Array< agx::AffineMatrix4x4 >& transforms,
        agxData::Array< agxGL::RenderBoxVertices >& renderBox_vertices,
        agxData::Array< agxGL::RenderBoxNormals >& renderBox_normals,
        agxData::Array< agxGL::RenderBoxOutlines >& renderBox_outlines
      );


      /**
      Function: Foundation.Rendering.GenerateBoxesFromBoxes
      Implementation: Line

      \param job The range job specifying what part of the data set to process
      \param halfExtents 
      \param transforms 
      \param line 
      */
      void GenerateBoxesFromBoxes__Line
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Vec3 >& halfExtents,
        const agxData::Array< agx::AffineMatrix4x4 >& transforms,
        agxData::Array< agx::Line32 >& line
      );


    }
  }
}

#endif
