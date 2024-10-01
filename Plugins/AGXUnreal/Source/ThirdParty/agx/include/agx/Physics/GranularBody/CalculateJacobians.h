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

#ifndef AGXFN_PHYSICS_GRANULARBODY_CALCULATEJACOBIANS_H
#define AGXFN_PHYSICS_GRANULARBODY_CALCULATEJACOBIANS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/GranularBody/ContactConstraintEntity.h>
#include <agx/Jacobian.h>
#include <agx/Physics/Particle/ContactConstraintRowEntity.h>
#include <agx/Physics/ParticlePairContactEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Vec3.h>
#include <agx/SpinMutex.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/GranularBodyEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Uuid.h>
#include <agx/Vec4.h>
#include <agx/Quat.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Range6.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      /**
      Function: Physics.GranularBody.CalculateJacobians
      Implementation: GranularGranularContact

      \param job The range job specifying what part of the data set to process
      \param constraint 
      \param constraintRow 
      \param contact 
      \param contactMaterial 
      \param particle 
      \param granularGranularJacobianStorage 
      \param timeStep 
      \param impactCondition 
      \param contactMode 
      \param granularPoissonRatio 
      \param nonlinearCoefficient 
      */
      void CalculateJacobians__GranularGranularContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        agx::Physics::ParticlePairContactData& contact,
        agx::Physics::ContactMaterialData& contactMaterial,
        agx::Physics::GranularBodyData& particle,
        agxData::Array< agx::Jacobian6DOFElement >& granularGranularJacobianStorage,
        const agx::Real& timeStep,
        const agx::Real& impactCondition,
        const agx::UInt& contactMode,
        const agx::Real& granularPoissonRatio,
        const agx::Real& nonlinearCoefficient
      );


      /**
      Function: Physics.GranularBody.CalculateJacobians
      Implementation: GranularGeometryContact

      \param job The range job specifying what part of the data set to process
      \param constraint 
      \param constraintRow 
      \param contact 
      \param contactMaterial 
      \param particle 
      \param rigidBody 
      \param granularGeometryJacobianStorage 
      \param timeStep 
      \param impactCondition 
      \param contactMode 
      \param granularPoissonRatio 
      \param nonlinearCoefficient 
      */
      void CalculateJacobians__GranularGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        agx::Physics::ParticleGeometryContactData& contact,
        agx::Physics::ContactMaterialData& contactMaterial,
        agx::Physics::GranularBodyData& particle,
        agx::Physics::RigidBodyData& rigidBody,
        agxData::Array< agx::Jacobian6DOFElement >& granularGeometryJacobianStorage,
        const agx::Real& timeStep,
        const agx::Real& impactCondition,
        const agx::UInt& contactMode,
        const agx::Real& granularPoissonRatio,
        const agx::Real& nonlinearCoefficient
      );


    }
  }
}

#endif
