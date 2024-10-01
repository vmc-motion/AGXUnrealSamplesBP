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

#ifndef AGXFN_PHYSICS_GRANULARBODY_STORECONTACTLAMBDAS32_H
#define AGXFN_PHYSICS_GRANULARBODY_STORECONTACTLAMBDAS32_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/ParticlePairContactEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Vec3.h>
#include <agx/Physics/GranularBody/ContactConstraint32Entity.h>
#include <agx/Jacobian.h>
#include <agx/Physics/Particle/ContactConstraintRow32Entity.h>
#include <agx/Physics/GranularBodyEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Uuid.h>
#include <agx/Vec4.h>
#include <agx/Quat.h>
#include <agx/Physics/GranularBody/CachedGranularContactEntity.h>
#include <agx/Physics/GranularBody/CachedContact.h>
#include <agx/QuadraticProbingHashTable.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      typedef agx::QuadraticProbingHashTable<agx::Physics::GranularBody::GranularGranularPair, agx::Physics::GranularBody::CachedGranularContactPtr, agx::HashFn< agx::Physics::GranularBody::GranularGranularPair >, agxData::BufferProxyAllocator> ParticleParticleContactTable;
      /**
      Function: Physics.GranularBody.StoreContactLambdas32
      Implementation: GranularGranularContact

      \param job The range job specifying what part of the data set to process
      \param contact 
      \param constraint 
      \param constraintRow 
      \param particle 
      \param cachedGranularGranularContacts 
      \param granularGranularContactTable 
      \param contactMode 
      */
      void StoreContactLambdas32__GranularGranularContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ParticlePairContactData& contact,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        agx::Physics::GranularBodyData& particle,
        agx::Physics::GranularBody::CachedGranularContactData& cachedGranularGranularContacts,
        ParticleParticleContactTable& granularGranularContactTable,
        const agx::UInt& contactMode
      );


    }
  }
}

#endif
