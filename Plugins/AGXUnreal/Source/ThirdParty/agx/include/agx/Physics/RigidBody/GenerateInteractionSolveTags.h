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

#ifndef AGXFN_PHYSICS_RIGIDBODY_GENERATEINTERACTIONSOLVETAGS_H
#define AGXFN_PHYSICS_RIGIDBODY_GENERATEINTERACTIONSOLVETAGS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/GeometryContactEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/ContactPointEntity.h>
#include <agx/Physics/BroadPhasePairEntity.h>
#include <agxSDK/Simulation.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>
#include <agx/Physics/StrongInteractionEntity.h>


namespace agx { namespace Physics { namespace RigidBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace RigidBody
    {
      /**
      Function: Physics.RigidBody.GenerateInteractionSolveTags
      Implementation: GeometryContact

      \param job The range job specifying what part of the data set to process
      \param geometryContact 
      \param forceIterative 
      \param simulation 
      */
      void GenerateInteractionSolveTags__GeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::GeometryContactData& geometryContact,
        const agx::Bool& forceIterative,
        agxSDK::Simulation* simulation
      );


      /**
      Function: Physics.RigidBody.GenerateInteractionSolveTags
      Implementation: BinaryConstraint

      \param job The range job specifying what part of the data set to process
      \param binaryConstraint 
      \param forceIterative 
      \param simulation 
      */
      void GenerateInteractionSolveTags__BinaryConstraint
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::BinaryConstraintData& binaryConstraint,
        const agx::Bool& forceIterative,
        agxSDK::Simulation* simulation
      );


      /**
      Function: Physics.RigidBody.GenerateInteractionSolveTags
      Implementation: ManyBodyConstraint

      \param job The range job specifying what part of the data set to process
      \param manyBodyConstraint 
      \param forceIterative 
      \param simulation 
      */
      void GenerateInteractionSolveTags__ManyBodyConstraint
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
        const agx::Bool& forceIterative,
        agxSDK::Simulation* simulation
      );


      /**
      Function: Physics.RigidBody.GenerateInteractionSolveTags
      Implementation: StrongInteraction

      \param job The range job specifying what part of the data set to process
      \param strongInteraction 
      \param simulation 
      */
      void GenerateInteractionSolveTags__StrongInteraction
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::StrongInteractionData& strongInteraction,
        agxSDK::Simulation* simulation
      );


    }
  }
}

#endif
