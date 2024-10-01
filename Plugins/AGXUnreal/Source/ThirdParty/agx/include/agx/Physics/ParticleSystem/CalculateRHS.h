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

#ifndef AGXFN_PHYSICS_PARTICLESYSTEM_CALCULATERHS_H
#define AGXFN_PHYSICS_PARTICLESYSTEM_CALCULATERHS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/Particle/ContactConstraintEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Jacobian.h>
#include <agx/Physics/Particle/ContactConstraintRowEntity.h>
#include <agx/Physics/ParticleEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Vec3.h>
#include <agx/Uuid.h>
#include <agx/Vec4.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Physics/Particle/ParticleBodyContactConstraintEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Range6.h>
#include <agx/Solver.h>


namespace agx { namespace Physics { namespace ParticleSystem { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace ParticleSystem
    {
      /**
      Function: Physics.ParticleSystem.CalculateRHS
      Implementation: ParticlePairContact

      \param constraint 
      \param constraintRow 
      \param particle 
      \param contactMaterial 
      \param timeStep 
      \param damping 
      \param useComplexImpactStage 
      */
      void CalculateRHS__ParticlePairContact
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::Particle::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        agx::Physics::ParticleData& particle,
        agx::Physics::ContactMaterialData& contactMaterial,
        const agx::Real& timeStep,
        const agx::Real& damping,
        const agx::Bool& useComplexImpactStage
      );


      /**
      Function: Physics.ParticleSystem.CalculateRHS
      Implementation: ParticleGeometryContact

      \param constraint 
      \param constraintRow 
      \param particle 
      \param rigidBody 
      \param contactMaterial 
      \param timeStep 
      \param damping 
      \param useComplexImpactStage 
      */
      void CalculateRHS__ParticleGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::Particle::ParticleBodyContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        agx::Physics::ParticleData& particle,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::ContactMaterialData& contactMaterial,
        const agx::Real& timeStep,
        const agx::Real& damping,
        const agx::Bool& useComplexImpactStage
      );


    }
  }
}

#endif
