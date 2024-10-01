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

#ifndef AGXFN_MATH_MADD_H
#define AGXFN_MATH_MADD_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>


namespace agx { namespace Math { } }

namespace agxFn
{
  namespace Math
  {
    /**
    Function: Math.Madd
    Implementation: (default)

    \param job The range job specifying what part of the data set to process
    \param result 
    \param base 
    \param offset 
    \param scale 
    */
    void Madd
    (
      /* Parameter list automatically generated, do not edit */
      const agx::RangeJob& job,
      agxData::Array< agx::Vec3 >& result,
      const agxData::Array< agx::Vec3 >& base,
      const agx::Vec3& offset,
      const agxData::Array< agx::Real >& scale
    );


    /**
    Function: Math.Madd
    Implementation: Real1_Real1

    \param job The range job specifying what part of the data set to process
    \param result 
    \param base 
    \param offset 
    \param scale 
    */
    void Madd__Real1_Real1
    (
      /* Parameter list automatically generated, do not edit */
      const agx::RangeJob& job,
      agxData::Array< agx::Vec3 >& result,
      const agxData::Array< agx::Vec3 >& base,
      const agx::Vec3& offset,
      const agx::Real& scale
    );


  }
}

#endif
