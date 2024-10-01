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

#ifndef AGXFN_MATH_SUM_H
#define AGXFN_MATH_SUM_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>


namespace agx { namespace Math { } }

namespace agxFn
{
  namespace Math
  {
    /**
    Function: Math.Sum
    Implementation: (default)

    \param result 
    \param source 
    */
    void Sum
    (
      /* Parameter list automatically generated, do not edit */
      agx::Real& result,
      const agxData::Array< agx::Real >& source
    );

    /// Generated return-value wrapper. DO NOT EDIT!
    AGX_FORCE_INLINE agx::Real Sum
    (
      const agxData::Array< agx::Real >& source
    )
    {
      agx::Real result;
      agxFn::Math::Sum(result, source);
      return result;
    }
    


  }
}

#endif
