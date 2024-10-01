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

#ifndef AGXFN_PHYSICS_PARTITIONER_RENDERING_ASSIGNISLANDVERTEXCOLORS_H
#define AGXFN_PHYSICS_PARTITIONER_RENDERING_ASSIGNISLANDVERTEXCOLORS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec4.h>


namespace agx { namespace Physics { namespace Partitioner { namespace Rendering { } } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Partitioner
    {
      namespace Rendering
      {
        /**
        Function: Physics.Partitioner.Rendering.AssignIslandVertexColors
        Implementation: (default)

        \param job The range job specifying what part of the data set to process
        \param graphNodeColors 
        \param graphNode_island 
        \param solveIsland_color 
        */
        void AssignIslandVertexColors
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agxData::Array< agx::Vec4 >& graphNodeColors,
          const agxData::Array< agx::UInt32 >& graphNode_island,
          const agxData::Array< agx::Vec4 >& solveIsland_color
        );


      }
    }
  }
}

#endif
