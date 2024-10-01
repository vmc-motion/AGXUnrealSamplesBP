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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_CALCULATECONTACTZONEASSIGNMENTS_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_CALCULATECONTACTZONEASSIGNMENTS_H

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
#include <agx/Physics/HierarchicalGrid/GridTierEntity.h>
#include <agx/GlobalResult.h>
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
#include <agxSDK/Simulation.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.CalculateContactZoneAssignments
      Implementation: ParticlePairContact

      \param job The range job specifying what part of the data set to process
      \param activeParticlePairContacts 
      \param particlePairContact 
      \param particle 
      \param cell 
      \param gridTier 
      \param missingParticlePairContactZones 
      \param tierDependencyParticleContacts 
      \param contactZoneAccumulationLevel 
      */
      void CalculateContactZoneAssignments__ParticlePairContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::UInt32 >& activeParticlePairContacts,
        agx::Physics::ParticlePairContactData& particlePairContact,
        agx::Physics::ParticleData& particle,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agxData::Array< agx::UInt32 >& missingParticlePairContactZones,
        agxData::Array< agx::UInt32 >& tierDependencyParticleContacts,
        const agx::UInt& contactZoneAccumulationLevel
      );


      /**
      Function: Physics.HierarchicalGrid.CalculateContactZoneAssignments
      Implementation: ParticleGeometryContact

      \param job The range job specifying what part of the data set to process
      \param activeParticleGeometryContacts 
      \param particleGeometryContact 
      \param particle 
      \param geometry 
      \param cell 
      \param gridTier 
      \param missingParticleGeometryContactZones 
      \param tierDependencyParticleGeometryContacts 
      \param contactZoneAccumulationLevel 
      */
      void CalculateContactZoneAssignments__ParticleGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::UInt32 >& activeParticleGeometryContacts,
        agx::Physics::ParticleGeometryContactData& particleGeometryContact,
        agx::Physics::ParticleData& particle,
        agx::Physics::GeometryData& geometry,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agxData::Array< agx::UInt32 >& missingParticleGeometryContactZones,
        agxData::Array< agx::UInt32 >& tierDependencyParticleGeometryContacts,
        const agx::UInt& contactZoneAccumulationLevel
      );


      /**
      Function: Physics.HierarchicalGrid.CalculateContactZoneAssignments
      Implementation: GeometryGeometryContact

      \param job The range job specifying what part of the data set to process
      \param activeGeometryContacts 
      \param geometryContact 
      \param cell 
      \param gridTier 
      \param missingGeometryGeometryContactZones 
      \param tierDependencyGeometryContacts 
      \param contactZoneAccumulationLevel 
      \param targetSolveModel 
      */
      void CalculateContactZoneAssignments__GeometryGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::UInt32 >& activeGeometryContacts,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agxData::Array< agx::UInt32 >& missingGeometryGeometryContactZones,
        agxData::Array< agx::UInt32 >& tierDependencyGeometryContacts,
        const agx::UInt& contactZoneAccumulationLevel,
        agx::SolveModel* targetSolveModel
      );


      /**
      Function: Physics.HierarchicalGrid.CalculateContactZoneAssignments
      Implementation: Particle_SolveBody

      \param job The range job specifying what part of the data set to process
      \param particle 
      \param cell 
      \param contactZoneAccumulationLevel 
      */
      void CalculateContactZoneAssignments__Particle_SolveBody
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ParticleData& particle,
        agx::Physics::HierarchicalGrid::CellData& cell,
        const agx::UInt& contactZoneAccumulationLevel
      );


    }
  }
}

#endif
