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

#ifndef AGXFN_MATH_BLAS_SCAL_H
#define AGXFN_MATH_BLAS_SCAL_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>
#include <agx/Matrix3x3.h>


namespace agx { namespace Math { namespace BLAS { } } }

namespace agxFn
{
  namespace Math
  {
    namespace BLAS
    {
      /**
      Function: Math.BLAS.SCAL
      Implementation: REAL_REALn

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param buffer 
      */
      void SCAL__REAL_REALn
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Real& factor,
        agxData::Array< agx::Real >& buffer
      );


      /**
      Function: Math.BLAS.SCAL
      Implementation: REAL_REAL3n

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param buffer 
      */
      void SCAL__REAL_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Real& factor,
        agxData::Array< agx::Vec3 >& buffer
      );


      /**
      Function: Math.BLAS.SCAL
      Implementation: REALn_REAL3n

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param buffer 
      */
      void SCAL__REALn_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Real >& factor,
        agxData::Array< agx::Vec3 >& buffer
      );


      /**
      Function: Math.BLAS.SCAL
      Implementation: mat3x3n_REAL3n

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param buffer 
      */
      void SCAL__mat3x3n_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Matrix3x3 >& factor,
        agxData::Array< agx::Vec3 >& buffer
      );


    }
  }
}

#endif
