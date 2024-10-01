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

#ifndef AGXFN_PHYSICS_PARTICLESYSTEM_CREATEMISSINGCONTACTMATERIALS_H
#define AGXFN_PHYSICS_PARTICLESYSTEM_CREATEMISSINGCONTACTMATERIALS_H

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
#include <agxSDK/MaterialManager.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/RigidBodyEntity.h>


namespace agx { namespace Physics { namespace ParticleSystem { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace ParticleSystem
    {
      /**
      Function: Physics.ParticleSystem.CreateMissingContactMaterials
      Implementation: ParticlePairContact

      \param missingParticlePairContactMaterials 
      \param particlePairContact 
      \param particle 
      \param materialManager 
      */
      void CreateMissingContactMaterials__ParticlePairContact
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt32 >& missingParticlePairContactMaterials,
        agx::Physics::ParticlePairContactData& particlePairContact,
        agx::Physics::ParticleData& particle,
        agxSDK::MaterialManager* materialManager
      );


      /**
      Function: Physics.ParticleSystem.CreateMissingContactMaterials
      Implementation: ParticleGeometryContact

      \param missingParticleGeometryContactMaterials 
      \param particleGeometryContact 
      \param particle 
      \param geometry 
      \param materialManager 
      */
      void CreateMissingContactMaterials__ParticleGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt32 >& missingParticleGeometryContactMaterials,
        agx::Physics::ParticleGeometryContactData& particleGeometryContact,
        agx::Physics::ParticleData& particle,
        agx::Physics::GeometryData& geometry,
        agxSDK::MaterialManager* materialManager
      );


    }
  }
}

#endif
