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

#ifndef AGXFN_PHYSICS_GRANULARBODY_UPDATEGRANULARCONTACTTABLES_H
#define AGXFN_PHYSICS_GRANULARBODY_UPDATEGRANULARCONTACTTABLES_H

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
#include <agx/Physics/GranularBodyEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Uuid.h>
#include <agx/Vec4.h>
#include <agx/Quat.h>
#include <agx/Physics/GranularBody/CachedGranularContactEntity.h>
#include <agx/Physics/GranularBody/CachedContact.h>
#include <agx/GlobalResult.h>
#include <agx/QuadraticProbingHashTable.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/RigidBodyEntity.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      typedef agx::QuadraticProbingHashTable<agx::Physics::GranularBody::GranularGranularPair, agx::Physics::GranularBody::CachedGranularContactPtr, agx::HashFn< agx::Physics::GranularBody::GranularGranularPair >, agxData::BufferProxyAllocator> ParticleParticleContactTable;
      typedef agx::QuadraticProbingHashTable<agx::Physics::GranularBody::GranularGranularPair, agx::Physics::GranularBody::CachedGranularContactPtr, agx::HashFn< agx::Physics::GranularBody::GranularGranularPair >, agxData::BufferProxyAllocator> ParticleParticleContactTable;
      typedef agx::QuadraticProbingHashTable<agx::Physics::GranularBody::GranularGeometryPair, agx::Physics::GranularBody::CachedGranularContactPtr, agx::HashFn< agx::Physics::GranularBody::GranularGeometryPair >, agxData::BufferProxyAllocator> ParticleGeometryContactTable;
      typedef agx::QuadraticProbingHashTable<agx::Physics::GranularBody::GranularGeometryPair, agx::Physics::GranularBody::CachedGranularContactPtr, agx::HashFn< agx::Physics::GranularBody::GranularGeometryPair >, agxData::BufferProxyAllocator> ParticleGeometryContactTable;
      /**
      Function: Physics.GranularBody.UpdateGranularContactTables
      Implementation: GranularGranularContact_pass1

      \param job The range job specifying what part of the data set to process
      \param contact 
      \param cachedContactId 
      \param particle 
      \param newGranularGranularContacts 
      \param granularGranularContactTable 
      */
      void UpdateGranularContactTables__GranularGranularContact_pass1
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ParticlePairContactData& contact,
        agxData::Array< agx::UInt32 >& cachedContactId,
        agx::Physics::GranularBodyData& particle,
        agxData::Array< agx::UInt32 >& newGranularGranularContacts,
        ParticleParticleContactTable& granularGranularContactTable
      );


      /**
      Function: Physics.GranularBody.UpdateGranularContactTables
      Implementation: GranularGranularContact_pass2

      \param contact 
      \param cachedContactId 
      \param particle 
      \param cachedGranularGranularContacts 
      \param newGranularGranularContacts 
      \param granularGranularContactTable 
      */
      void UpdateGranularContactTables__GranularGranularContact_pass2
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::ParticlePairContactData& contact,
        agxData::Array< agx::UInt32 >& cachedContactId,
        agx::Physics::GranularBodyData& particle,
        agx::Physics::GranularBody::CachedGranularContactData& cachedGranularGranularContacts,
        const agxData::Array< agx::UInt32 >& newGranularGranularContacts,
        ParticleParticleContactTable& granularGranularContactTable
      );


      /**
      Function: Physics.GranularBody.UpdateGranularContactTables
      Implementation: GranularGeometryContact_pass1

      \param job The range job specifying what part of the data set to process
      \param contact 
      \param cachedContactId 
      \param particle 
      \param geometry 
      \param newGranularGeometryContacts 
      \param granularGeometryContactTable 
      */
      void UpdateGranularContactTables__GranularGeometryContact_pass1
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ParticleGeometryContactData& contact,
        agxData::Array< agx::UInt32 >& cachedContactId,
        agx::Physics::GranularBodyData& particle,
        agx::Physics::GeometryData& geometry,
        agxData::Array< agx::UInt32 >& newGranularGeometryContacts,
        ParticleGeometryContactTable& granularGeometryContactTable
      );


      /**
      Function: Physics.GranularBody.UpdateGranularContactTables
      Implementation: GranularGeometryContact_pass2

      \param contact 
      \param cachedContactId 
      \param particle 
      \param geometry 
      \param cachedGranularGeometryContacts 
      \param newGranularGeometryContacts 
      \param granularGeometryContactTable 
      */
      void UpdateGranularContactTables__GranularGeometryContact_pass2
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::ParticleGeometryContactData& contact,
        agxData::Array< agx::UInt32 >& cachedContactId,
        agx::Physics::GranularBodyData& particle,
        agx::Physics::GeometryData& geometry,
        agx::Physics::GranularBody::CachedGranularContactData& cachedGranularGeometryContacts,
        const agxData::Array< agx::UInt32 >& newGranularGeometryContacts,
        ParticleGeometryContactTable& granularGeometryContactTable
      );


    }
  }
}

#endif
