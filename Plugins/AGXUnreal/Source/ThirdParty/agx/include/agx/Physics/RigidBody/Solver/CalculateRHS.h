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

#ifndef AGXFN_PHYSICS_RIGIDBODY_SOLVER_CALCULATERHS_H
#define AGXFN_PHYSICS_RIGIDBODY_SOLVER_CALCULATERHS_H

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
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>
#include <agx/Physics/StrongInteractionEntity.h>
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
        Function: Physics.RigidBody.Solver.CalculateRHS
        Implementation: Contact

        \param contactConstraint 
        \param contactConstraintRow 
        \param rigidBody 
        \param contactConstraintJacobian 
        \param contactMaterial 
        \param clock_timeStep 
        \param useComplexImpactStage 
        */
        void CalculateRHS__Contact
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::ContactConstraintData& contactConstraint,
          agx::Physics::ConstraintRowData& contactConstraintRow,
          agx::Physics::RigidBodyData& rigidBody,
          const agxData::Array< agx::Jacobian6DOFElement >& contactConstraintJacobian,
          agx::Physics::ContactMaterialData& contactMaterial,
          const agx::Real& clock_timeStep,
          const agx::Bool& useComplexImpactStage
        );


        /**
        Function: Physics.RigidBody.Solver.CalculateRHS
        Implementation: BinaryConstraint

        \param binaryConstraint 
        \param binaryConstraintJacobian 
        \param binaryConstraintRow 
        \param rigidBody 
        \param clock_timeStep 
        \param useComplexImpactStage 
        */
        void CalculateRHS__BinaryConstraint
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::BinaryConstraintData& binaryConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& binaryConstraintJacobian,
          agx::Physics::ConstraintRowData& binaryConstraintRow,
          agx::Physics::RigidBodyData& rigidBody,
          const agx::Real& clock_timeStep,
          const agx::Bool& useComplexImpactStage
        );


        /**
        Function: Physics.RigidBody.Solver.CalculateRHS
        Implementation: ManyBodyConstraint

        \param manyBodyConstraint 
        \param manyBodyConstraintJacobian 
        \param manyBodyConstraintRow 
        \param rigidBody 
        \param clock_timeStep 
        \param useComplexImpactStage 
        */
        void CalculateRHS__ManyBodyConstraint
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
          const agxData::Array< agx::Jacobian6DOFElement >& manyBodyConstraintJacobian,
          agx::Physics::ConstraintRowData& manyBodyConstraintRow,
          agx::Physics::RigidBodyData& rigidBody,
          const agx::Real& clock_timeStep,
          const agx::Bool& useComplexImpactStage
        );


        /**
        Function: Physics.RigidBody.Solver.CalculateRHS
        Implementation: Direct

        \param solver 
        \param binaryConstraint 
        \param binaryConstraintRow 
        \param manyBodyConstraint 
        \param manyBodyConstraintRow 
        \param contactConstraint 
        \param contactConstraintRow 
        \param strongInteraction 
        \param rigidBody 
        \param clock_timeStep 
        \param useComplexImpactStage 
        */
        void CalculateRHS__Direct
        (
          /* Parameter list automatically generated, do not edit */
          agx::SolveContext context,
          agx::Solver* solver,
          agx::Physics::BinaryConstraintData& binaryConstraint,
          agx::Physics::ConstraintRowData& binaryConstraintRow,
          agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
          agx::Physics::ConstraintRowData& manyBodyConstraintRow,
          agx::Physics::ContactConstraintData& contactConstraint,
          agx::Physics::ConstraintRowData& contactConstraintRow,
          agx::Physics::StrongInteractionData& strongInteraction,
          agx::Physics::RigidBodyData& rigidBody,
          const agx::Real& clock_timeStep,
          const agx::Bool& useComplexImpactStage
        );


      }
    }
  }
}

#endif
