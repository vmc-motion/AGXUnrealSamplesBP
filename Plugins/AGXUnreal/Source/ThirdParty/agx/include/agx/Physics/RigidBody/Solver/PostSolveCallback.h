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

#ifndef AGXFN_PHYSICS_RIGIDBODY_SOLVER_POSTSOLVECALLBACK_H
#define AGXFN_PHYSICS_RIGIDBODY_SOLVER_POSTSOLVECALLBACK_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ConstraintRowEntity.h>
#include <agx/Range.h>
#include <agx/Jacobian.h>
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
        Function: Physics.RigidBody.Solver.PostSolveCallback
        Implementation: BinaryConstraint

        \param job The range job specifying what part of the data set to process
        \param binaryConstraint 
        \param binaryConstraintRow 
        \param binaryConstraintJacobian 
        \param clock_timeStep 
        */
        void PostSolveCallback__BinaryConstraint
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::BinaryConstraintData& binaryConstraint,
          agx::Physics::ConstraintRowData& binaryConstraintRow,
          const agxData::Array< agx::Jacobian6DOFElement >& binaryConstraintJacobian,
          const agx::Real& clock_timeStep
        );


        /**
        Function: Physics.RigidBody.Solver.PostSolveCallback
        Implementation: ManyBodyConstraint

        \param job The range job specifying what part of the data set to process
        \param manyBodyConstraint 
        \param manyBodyConstraintRow 
        \param manyBodyConstraintJacobian 
        \param clock_timeStep 
        */
        void PostSolveCallback__ManyBodyConstraint
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
          agx::Physics::ConstraintRowData& manyBodyConstraintRow,
          agxData::Array< agx::Jacobian6DOFElement >& manyBodyConstraintJacobian,
          const agx::Real& clock_timeStep
        );


      }
    }
  }
}

#endif
