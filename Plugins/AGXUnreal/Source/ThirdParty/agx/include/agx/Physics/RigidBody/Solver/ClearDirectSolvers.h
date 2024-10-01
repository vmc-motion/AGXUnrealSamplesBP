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

#ifndef AGXFN_PHYSICS_RIGIDBODY_SOLVER_CLEARDIRECTSOLVERS_H
#define AGXFN_PHYSICS_RIGIDBODY_SOLVER_CLEARDIRECTSOLVERS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/SolveIslandEntity.h>
#include <agx/Solver.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>
#include <agx/Physics/ContactConstraintEntity.h>
#include <agx/IndexRange.h>
#include <agxSDK/Simulation.h>


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
        Function: Physics.RigidBody.Solver.ClearDirectSolvers
        Implementation: (default)

        \param solveIsland_instance 
        \param solver 
        \param binaryConstraint 
        \param manyBodyConstraint 
        \param contactConstraint 
        \param simulation 
        */
        void ClearDirectSolvers
        (
          /* Parameter list automatically generated, do not edit */
          agxData::Array< agx::Physics::SolveIslandPtr >& solveIsland_instance,
          agx::Solver* solver,
          agx::Physics::BinaryConstraintData& binaryConstraint,
          agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
          agx::Physics::ContactConstraintData& contactConstraint,
          agxSDK::Simulation* simulation
        );


      }
    }
  }
}

#endif
