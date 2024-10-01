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

#ifndef AGXFN_PHYSICS_GRANULARBODY_STORECONTACTFORCES_H
#define AGXFN_PHYSICS_GRANULARBODY_STORECONTACTFORCES_H

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
#include <agx/Physics/GranularBody/ContactConstraintEntity.h>
#include <agx/Jacobian.h>
#include <agx/Physics/Particle/ContactConstraintRowEntity.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/GranularBody/ContactConstraint32Entity.h>
#include <agx/Physics/Particle/ContactConstraintRow32Entity.h>
#include <agx/Solver.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      /**
      Function: Physics.GranularBody.StoreContactForces
      Implementation: GranularGranularContact

      \param contact 
      \param constraint 
      \param constraintRow 
      \param granularGranularJacobianStorage 
      \param timeStep 
      */
      void StoreContactForces__GranularGranularContact
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::ParticlePairContactData& contact,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement >& granularGranularJacobianStorage,
        const agx::Real& timeStep
      );


      /**
      Function: Physics.GranularBody.StoreContactForces
      Implementation: GranularGeometryContact

      \param contact 
      \param constraint 
      \param constraintRow 
      \param geometry 
      \param granularGeometryJacobianStorage 
      \param timeStep 
      */
      void StoreContactForces__GranularGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::ParticleGeometryContactData& contact,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        agx::Physics::GeometryData& geometry,
        const agxData::Array< agx::Jacobian6DOFElement >& granularGeometryJacobianStorage,
        const agx::Real& timeStep
      );


      /**
      Function: Physics.GranularBody.StoreContactForces
      Implementation: GranularGranularContact32

      \param contact 
      \param constraint 
      \param constraintRow 
      \param granularGranularJacobianStorage 
      \param timeStep 
      */
      void StoreContactForces__GranularGranularContact32
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::ParticlePairContactData& contact,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement32 >& granularGranularJacobianStorage,
        const agx::Real& timeStep
      );


      /**
      Function: Physics.GranularBody.StoreContactForces
      Implementation: GranularGranularContact_LocalJacobians

      \param contact 
      \param constraint 
      \param constraintRow 
      \param contactMode 
      \param timeStep 
      */
      void StoreContactForces__GranularGranularContact_LocalJacobians
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::ParticlePairContactData& contact,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agx::UInt& contactMode,
        const agx::Real& timeStep
      );


      /**
      Function: Physics.GranularBody.StoreContactForces
      Implementation: GranularGranularContact_LocalJacobians32

      \param contact 
      \param constraint 
      \param constraintRow 
      \param contactMode 
      \param timeStep 
      */
      void StoreContactForces__GranularGranularContact_LocalJacobians32
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::ParticlePairContactData& contact,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agx::UInt& contactMode,
        const agx::Real& timeStep
      );


    }
  }
}

#endif
