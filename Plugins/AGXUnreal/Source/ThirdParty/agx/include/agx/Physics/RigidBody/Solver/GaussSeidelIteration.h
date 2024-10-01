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

#ifndef AGXFN_PHYSICS_RIGIDBODY_SOLVER_GAUSSSEIDELITERATION_H
#define AGXFN_PHYSICS_RIGIDBODY_SOLVER_GAUSSSEIDELITERATION_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/SpinMutex.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/Vec3.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Range6.h>
#include <agx/Physics/ContactConstraintEntity.h>
#include <agx/IndexRange.h>
#include <agx/Jacobian.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/ConstraintRowEntity.h>
#include <agx/Range.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>
#include <agx/Solver.h>


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
        Function: Physics.RigidBody.Solver.GaussSeidelIteration
        Implementation: ContactConstraint_IterativeProjectedConeFriction

        \param rigidBody 
        \param contactConstraint 
        \param contactConstraintJacobian 
        \param contactConstraintGMInv 
        \param contactMaterial 
        \param contactConstraintRow 
        */
        void GaussSeidelIteration__ContactConstraint_IterativeProjectedConeFriction
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::RigidBodyData& rigidBody,
          agx::Physics::ContactConstraintData& contactConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintJacobian,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintGMInv,
          agx::Physics::ContactMaterialData& contactMaterial,
          agx::Physics::ConstraintRowData& contactConstraintRow
        );


        /**
        Function: Physics.RigidBody.Solver.GaussSeidelIteration
        Implementation: ContactConstraint_BoxFrictionModel

        \param rigidBody 
        \param contactConstraint 
        \param contactConstraintJacobian 
        \param contactConstraintGMInv 
        \param contactMaterial 
        \param contactConstraintRow 
        */
        void GaussSeidelIteration__ContactConstraint_BoxFrictionModel
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::RigidBodyData& rigidBody,
          agx::Physics::ContactConstraintData& contactConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintJacobian,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintGMInv,
          agx::Physics::ContactMaterialData& contactMaterial,
          agx::Physics::ConstraintRowData& contactConstraintRow
        );


        /**
        Function: Physics.RigidBody.Solver.GaussSeidelIteration
        Implementation: ContactConstraint_ScaleBoxFrictionModel

        \param rigidBody 
        \param contactConstraint 
        \param contactConstraintJacobian 
        \param contactConstraintGMInv 
        \param contactMaterial 
        \param contactConstraintRow 
        */
        void GaussSeidelIteration__ContactConstraint_ScaleBoxFrictionModel
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::RigidBodyData& rigidBody,
          agx::Physics::ContactConstraintData& contactConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintJacobian,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintGMInv,
          agx::Physics::ContactMaterialData& contactMaterial,
          agx::Physics::ConstraintRowData& contactConstraintRow
        );


        /**
        Function: Physics.RigidBody.Solver.GaussSeidelIteration
        Implementation: BinaryConstraint

        \param binaryConstraint 
        \param binaryConstraintJacobian 
        \param binaryConstraintGMInv 
        \param binaryConstraintRow 
        \param rigidBody 
        */
        void GaussSeidelIteration__BinaryConstraint
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::BinaryConstraintData& binaryConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& binaryConstraintJacobian,
          const agxData::Array< agx::Jacobian6DOFElement >& binaryConstraintGMInv,
          agx::Physics::ConstraintRowData& binaryConstraintRow,
          agx::Physics::RigidBodyData& rigidBody
        );


        /**
        Function: Physics.RigidBody.Solver.GaussSeidelIteration
        Implementation: ManyBodyConstraint

        \param manyBodyConstraint 
        \param manyBodyConstraintJacobian 
        \param manyBodyConstraintGMInv 
        \param manyBodyConstraintRow 
        \param rigidBody 
        */
        void GaussSeidelIteration__ManyBodyConstraint
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& manyBodyConstraintJacobian,
          const agxData::Array< agx::Jacobian6DOFElement >& manyBodyConstraintGMInv,
          agx::Physics::ConstraintRowData& manyBodyConstraintRow,
          agx::Physics::RigidBodyData& rigidBody
        );


      }
    }
  }
}

#endif
