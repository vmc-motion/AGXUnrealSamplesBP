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

#ifndef AGXFN_MATH_BLAS_COPY_H
#define AGXFN_MATH_BLAS_COPY_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>


namespace agx { namespace Math { namespace BLAS { } } }

namespace agxFn
{
  namespace Math
  {
    namespace BLAS
    {
      /**
      Function: Math.BLAS.COPY
      Implementation: REALn_REALn

      \param job The range job specifying what part of the data set to process
      \param source 
      \param target 
      */
      void COPY__REALn_REALn
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Real >& source,
        agxData::Array< agx::Real >& target
      );


      /**
      Function: Math.BLAS.COPY
      Implementation: REAL3n_REAL3n

      \param job The range job specifying what part of the data set to process
      \param source 
      \param target 
      */
      void COPY__REAL3n_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Vec3 >& source,
        agxData::Array< agx::Vec3 >& target
      );


    }
  }
}

#endif
