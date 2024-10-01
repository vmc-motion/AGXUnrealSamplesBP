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

#ifndef AGXFN_PHYSICS_RIGIDBODY_SOLVER_CALCULATEGMINV_H
#define AGXFN_PHYSICS_RIGIDBODY_SOLVER_CALCULATEGMINV_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/ContactConstraintEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/IndexRange.h>
#include <agx/Physics/ConstraintRowEntity.h>
#include <agx/Range.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/SpinMutex.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/Vec3.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Range6.h>
#include <agx/Jacobian.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>


namespace agx { namespace Physics { namespace RigidBody { namespace Solver { } } } }

namespace agxFn
{
  namespace Physics
  {
    namespace RigidBody
    {
      namespace Solver
      {
        /**
        Function: Physics.RigidBody.Solver.CalculateGMInv
        Implementation: Contact

        \param job The range job specifying what part of the data set to process
        \param contactConstraint 
        \param contactConstraintRow 
        \param rigidBody 
        \param contactConstraintJacobian 
        \param contactConstraintGMInv 
        */
        void CalculateGMInv__Contact
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::ContactConstraintData& contactConstraint,
          agx::Physics::ConstraintRowData& contactConstraintRow,
          agx::Physics::RigidBodyData& rigidBody,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintJacobian,
          agxData::Array< agx::Jacobian6DOFElement >& contactConstraintGMInv
        );


        /**
        Function: Physics.RigidBody.Solver.CalculateGMInv
        Implementation: BinaryConstraint

        \param job The range job specifying what part of the data set to process
        \param binaryConstraint 
        \param binaryConstraintRow 
        \param rigidBody 
        \param binaryConstraintJacobian 
        \param binaryConstraintGMInv 
        */
        void CalculateGMInv__BinaryConstraint
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::BinaryConstraintData& binaryConstraint,
          agx::Physics::ConstraintRowData& binaryConstraintRow,
          agx::Physics::RigidBodyData& rigidBody,
          const agxData::Array< agx::Jacobian6DOFElement >& binaryConstraintJacobian,
          agxData::Array< agx::Jacobian6DOFElement >& binaryConstraintGMInv
        );


        /**
        Function: Physics.RigidBody.Solver.CalculateGMInv
        Implementation: ManyBodyConstraint

        \param job The range job specifying what part of the data set to process
        \param manyBodyConstraint 
        \param manyBodyConstraintRow 
        \param rigidBody 
        \param manyBodyConstraintJacobian 
        \param manyBodyConstraintGMInv 
        */
        void CalculateGMInv__ManyBodyConstraint
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
          agx::Physics::ConstraintRowData& manyBodyConstraintRow,
          agx::Physics::RigidBodyData& rigidBody,
          const agxData::Array< agx::Jacobian6DOFElement >& manyBodyConstraintJacobian,
          agxData::Array< agx::Jacobian6DOFElement >& manyBodyConstraintGMInv
        );


      }
    }
  }
}

#endif
