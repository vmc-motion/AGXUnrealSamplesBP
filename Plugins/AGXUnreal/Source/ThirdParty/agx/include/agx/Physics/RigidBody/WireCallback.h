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

#ifndef AGXFN_PHYSICS_RIGIDBODY_WIRECALLBACK_H
#define AGXFN_PHYSICS_RIGIDBODY_WIRECALLBACK_H

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

namespace agxWire { class Wire; }

namespace agx { namespace Physics { namespace RigidBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace RigidBody
    {
      /**
      Function: Physics.RigidBody.WireCallback
      Implementation: preCollide

      \param job The range job specifying what part of the data set to process
      \param wireData 
      */
      void WireCallback__preCollide
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agxWire::Wire* >& wireData
      );


      /**
      Function: Physics.RigidBody.WireCallback
      Implementation: pre

      \param job The range job specifying what part of the data set to process
      \param wireData 
      */
      void WireCallback__pre
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agxWire::Wire* >& wireData
      );


      /**
      Function: Physics.RigidBody.WireCallback
      Implementation: post

      \param job The range job specifying what part of the data set to process
      \param wireData 
      */
      void WireCallback__post
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agxWire::Wire* >& wireData
      );


      /**
      Function: Physics.RigidBody.WireCallback
      Implementation: collide

      \param job The range job specifying what part of the data set to process
      \param wireData 
      \param geometryContact 
      */
      void WireCallback__collide
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agxWire::Wire* >& wireData,
        agx::Physics::GeometryContactData& geometryContact
      );


      /**
      Function: Physics.RigidBody.WireCallback
      Implementation: collide2

      \param job The range job specifying what part of the data set to process
      \param geometryContact 
      \param wireData 
      */
      void WireCallback__collide2
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::GeometryContactData& geometryContact,
        agxData::Array< agxWire::Wire* >& wireData
      );


      /**
      Function: Physics.RigidBody.WireCallback
      Implementation: collectSplitInformation

      \param job The range job specifying what part of the data set to process
      \param wireData 
      \param graphNode 
      */
      void WireCallback__collectSplitInformation
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agxWire::Wire* >& wireData,
        agx::Physics::GraphNodeData& graphNode
      );


    }
  }
}

#endif
