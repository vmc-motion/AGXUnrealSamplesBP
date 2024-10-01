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

#ifndef AGXFN_PHYSICS_PARTICLESYSTEM_FILTERCONTACTS_H
#define AGXFN_PHYSICS_PARTICLESYSTEM_FILTERCONTACTS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/ParticleEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Vec3.h>
#include <agx/Uuid.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Vec4.h>
#include <agx/Physics/ParticlePairContactEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/RigidBodyState.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/Range6.h>


namespace agx { namespace Physics { namespace ParticleSystem { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace ParticleSystem
    {
      /**
      Function: Physics.ParticleSystem.FilterContacts
      Implementation: ParticlePair

      \param particle 
      \param particlePairContact 
      \param activeParticlePairContacts 
      */
      void FilterContacts__ParticlePair
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::ParticleData& particle,
        agx::Physics::ParticlePairContactData& particlePairContact,
        agxData::Array< agx::UInt32 >& activeParticlePairContacts
      );


      /**
      Function: Physics.ParticleSystem.FilterContacts
      Implementation: ParticleGeometry

      \param particle 
      \param particleGeometryContact 
      \param geometry 
      \param rigidBody 
      \param activeParticleGeometryContacts 
      */
      void FilterContacts__ParticleGeometry
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::ParticleData& particle,
        agx::Physics::ParticleGeometryContactData& particleGeometryContact,
        agx::Physics::GeometryData& geometry,
        agx::Physics::RigidBodyData& rigidBody,
        agxData::Array< agx::UInt32 >& activeParticleGeometryContacts
      );


    }
  }
}

#endif
