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

#ifndef AGXFN_FOUNDATION_RENDERING_GENERATELINES_H
#define AGXFN_FOUNDATION_RENDERING_GENERATELINES_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>
#include <agx/Line.h>


namespace agx { namespace Foundation { namespace Rendering { } } }

namespace agxFn
{
  namespace Foundation
  {
    namespace Rendering
    {
      /**
      Function: Foundation.Rendering.GenerateLines
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param positions 
      \param offsets 
      \param result 
      \param scale 
      */
      void GenerateLines
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Vec3 >& positions,
        const agxData::Array< agx::Vec3f >& offsets,
        agxData::Array< agx::Line >& result,
        const agx::Real& scale
      );


    }
  }
}

#endif
