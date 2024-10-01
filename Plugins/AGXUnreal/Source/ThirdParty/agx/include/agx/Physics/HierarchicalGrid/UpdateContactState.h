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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_UPDATECONTACTSTATE_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_UPDATECONTACTSTATE_H

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
#include <agx/Physics/WarmStartingDataEntity.h>
#include <agxCollide/Geometry.h>
#include <agx/QuadraticProbingHashTable.h>
#include <agx/AtomicValue.h>
#include <agx/Physics/GeometryPairEntity.h>
#include <agx/SpinMutex.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/Vec3.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Range6.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Physics/MaterialEntity.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      typedef agx::QuadraticProbingHashTable<agxData::GeometryPair, agx::Physics::GeometryContactPtr, agx::HashFn< agxData::GeometryPair >, agxData::BufferProxyAllocator> ContactTable;
      typedef agx::QuadraticProbingHashTable<agxData::GeometryPair, agx::Physics::GeometryContactPtr, agx::HashFn< agxData::GeometryPair >, agxData::BufferProxyAllocator> ContactTable;
      /**
      Function: Physics.HierarchicalGrid.UpdateContactState
      Implementation: Pass1

      \param job The range job specifying what part of the data set to process
      \param geometryContact 
      \param broadPhasePair 
      \param contactTable 
      \param numPersistentContacts 
      */
      void UpdateContactState__Pass1
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::BroadPhasePairData& broadPhasePair,
        ContactTable& contactTable,
        agx::AtomicValue& numPersistentContacts
      );


      /**
      Function: Physics.HierarchicalGrid.UpdateContactState
      Implementation: Pass2

      \param geometryContact 
      \param separationPair 
      \param separationPair_globalOrder 
      \param rigidBody 
      \param geometry 
      \param contactTable 
      \param numPersistentContacts 
      */
      void UpdateContactState__Pass2
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::GeometryPairData& separationPair,
        agxData::Array< agx::UInt32 >& separationPair_globalOrder,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::GeometryData& geometry,
        ContactTable& contactTable,
        agx::AtomicValue& numPersistentContacts
      );


    }
  }
}

#endif
