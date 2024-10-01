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

#ifndef AGXFN_MATH_BLAS_AXPY_H
#define AGXFN_MATH_BLAS_AXPY_H

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
      Function: Math.BLAS.AXPY
      Implementation: REALn__REAL_REALn

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param source 
      \param target 
      */
      void AXPY__REALn__REAL_REALn
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Real& factor,
        const agxData::Array< agx::Real >& source,
        agxData::Array< agx::Real >& target
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REAL3n_REAL_REAL3n

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param source 
      \param target 
      */
      void AXPY__REAL3n_REAL_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Real& factor,
        const agxData::Array< agx::Vec3 >& source,
        agxData::Array< agx::Vec3 >& target
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REAL3n_REAL3_REAL3n

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param source 
      \param target 
      */
      void AXPY__REAL3n_REAL3_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Vec3& factor,
        const agxData::Array< agx::Vec3 >& source,
        agxData::Array< agx::Vec3 >& target
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REAL3n_REAL_REAL3

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param source 
      \param target 
      */
      void AXPY__REAL3n_REAL_REAL3
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Real >& factor,
        const agxData::Array< agx::Vec3 >& source,
        agxData::Array< agx::Vec3 >& target
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REALn_REAL_REALn_offset

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param source 
      \param sourceOffset 
      \param target 
      \param targetOffset 
      */
      void AXPY__REALn_REAL_REALn_offset
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Real& factor,
        const agxData::Array< agx::Real >& source,
        const agx::UInt& sourceOffset,
        agxData::Array< agx::Real >& target,
        const agx::UInt& targetOffset
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REAL3n_REAL_REAL3n_offset

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param source 
      \param sourceOffset 
      \param target 
      \param targetOffset 
      */
      void AXPY__REAL3n_REAL_REAL3n_offset
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agx::Real& factor,
        const agxData::Array< agx::Vec3 >& source,
        const agx::UInt& sourceOffset,
        agxData::Array< agx::Vec3 >& target,
        const agx::UInt& targetOffset
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REAL3n_REALn_REAL3n_offset

      \param job The range job specifying what part of the data set to process
      \param factor 
      \param factorOffset 
      \param source 
      \param sourceOffset 
      \param target 
      \param targetOffset 
      */
      void AXPY__REAL3n_REALn_REAL3n_offset
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::Real >& factor,
        const agx::UInt& factorOffset,
        const agxData::Array< agx::Vec3 >& source,
        const agx::UInt& sourceOffset,
        agxData::Array< agx::Vec3 >& target,
        const agx::UInt& targetOffset
      );


      /**
      Function: Math.BLAS.AXPY
      Implementation: REAL3n_3x3n_REAL3n

      \param job The range job specifying what part of the data set to process
      \param target 
      \param factor 
      \param source 
      */
      void AXPY__REAL3n_3x3n_REAL3n
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agx::Vec3 >& target,
        const agxData::Array< agx::Matrix3x3 >& factor,
        const agxData::Array< agx::Vec3 >& source
      );


    }
  }
}

#endif
