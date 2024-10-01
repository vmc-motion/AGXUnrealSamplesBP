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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_CREATEMISSINGCONTACTZONES_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_CREATEMISSINGCONTACTZONES_H

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
#include <agx/Physics/ParticleEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Uuid.h>
#include <agx/Vec4.h>
#include <agx/Physics/HierarchicalGrid/CellEntity.h>
#include <agx/IndexRange.h>
#include <agx/Physics/HierarchicalGrid/ContactZoneEntity.h>
#include <agx/AtomicValue.h>
#include <agx/Physics/SolveGroupEntity.h>
#include <agx/Physics/HierarchicalGrid/GridTierEntity.h>
#include <agx/Physics/HierarchicalGrid/ContactZoneDependencyEntity.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/GeometryContactEntity.h>
#include <agx/Physics/ContactPointEntity.h>
#include <agx/Physics/BroadPhasePairEntity.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.CreateMissingContactZones
      Implementation: ParticlePairContact

      \param missingParticlePairContactZones 
      \param particlePairContact 
      \param particle 
      \param cell 
      \param contactZone 
      \param contactZoneDependency 
      \param gridTier 
      \param iterationStartZone 
      \param iterationEndZone 
      \param contactZoneAccumulationLevel 
      */
      void CreateMissingContactZones__ParticlePairContact
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt32 >& missingParticlePairContactZones,
        agx::Physics::ParticlePairContactData& particlePairContact,
        agx::Physics::ParticleData& particle,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::ContactZoneData& contactZone,
        agx::Physics::HierarchicalGrid::ContactZoneDependencyData& contactZoneDependency,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agx::Physics::HierarchicalGrid::ContactZonePtr& iterationStartZone,
        agx::Physics::HierarchicalGrid::ContactZonePtr& iterationEndZone,
        const agx::UInt& contactZoneAccumulationLevel
      );


      /**
      Function: Physics.HierarchicalGrid.CreateMissingContactZones
      Implementation: ParticleGeometryContact

      \param missingParticleGeometryContactZones 
      \param particleGeometryContact 
      \param particle 
      \param geometry 
      \param cell 
      \param contactZone 
      \param contactZoneDependency 
      \param gridTier 
      \param iterationStartZone 
      \param iterationEndZone 
      \param contactZoneAccumulationLevel 
      */
      void CreateMissingContactZones__ParticleGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt32 >& missingParticleGeometryContactZones,
        agx::Physics::ParticleGeometryContactData& particleGeometryContact,
        agx::Physics::ParticleData& particle,
        agx::Physics::GeometryData& geometry,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::ContactZoneData& contactZone,
        agx::Physics::HierarchicalGrid::ContactZoneDependencyData& contactZoneDependency,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agx::Physics::HierarchicalGrid::ContactZonePtr& iterationStartZone,
        agx::Physics::HierarchicalGrid::ContactZonePtr& iterationEndZone,
        const agx::UInt& contactZoneAccumulationLevel
      );


      /**
      Function: Physics.HierarchicalGrid.CreateMissingContactZones
      Implementation: GeometryGeometryContact

      \param missingGeometryGeometryContactZones 
      \param geometryContact 
      \param cell 
      \param contactZone 
      \param contactZoneDependency 
      \param gridTier 
      \param iterationStartZone 
      \param iterationEndZone 
      \param contactZoneAccumulationLevel 
      */
      void CreateMissingContactZones__GeometryGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt32 >& missingGeometryGeometryContactZones,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::ContactZoneData& contactZone,
        agx::Physics::HierarchicalGrid::ContactZoneDependencyData& contactZoneDependency,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agx::Physics::HierarchicalGrid::ContactZonePtr& iterationStartZone,
        agx::Physics::HierarchicalGrid::ContactZonePtr& iterationEndZone,
        const agx::UInt& contactZoneAccumulationLevel
      );


    }
  }
}

#endif
