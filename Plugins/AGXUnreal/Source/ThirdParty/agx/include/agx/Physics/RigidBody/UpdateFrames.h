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

#ifndef AGXFN_PHYSICS_RIGIDBODY_UPDATEFRAMES_H
#define AGXFN_PHYSICS_RIGIDBODY_UPDATEFRAMES_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/RigidBodyState.h>


namespace agx { namespace Physics { namespace RigidBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace RigidBody
    {
      /**
      Function: Physics.RigidBody.UpdateFrames
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param rigidBody_instance 
      \param rigidBody_state 
      */
      void UpdateFrames
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agx::Physics::RigidBodyPtr >& rigidBody_instance,
        const agxData::Array< agx::RigidBodyState >& rigidBody_state
      );


      /**
      Function: Physics.RigidBody.UpdateFrames
      Implementation: SSE

      \param job The range job specifying what part of the data set to process
      \param rigidBody_instance 
      \param rigidBody_state 
      */
      void UpdateFrames__SSE
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agx::Physics::RigidBodyPtr >& rigidBody_instance,
        const agxData::Array< agx::RigidBodyState >& rigidBody_state
      );


    }
  }
}

#endif
