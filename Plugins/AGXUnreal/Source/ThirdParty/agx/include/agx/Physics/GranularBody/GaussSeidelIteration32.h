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

#ifndef AGXFN_PHYSICS_GRANULARBODY_GAUSSSEIDELITERATION32_H
#define AGXFN_PHYSICS_GRANULARBODY_GAUSSSEIDELITERATION32_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/GranularBody/ContactConstraint32Entity.h>
#include <agx/Jacobian.h>
#include <agx/Physics/Particle/ContactConstraintRow32Entity.h>
#include <agx/Physics/SolveBody32Entity.h>
#include <agx/Vec3.h>
#include <agx/SpinMutex.h>
#include <agx/Physics/SolveMaterial32Entity.h>
#include <agx/Vec2.h>
#include <agx/Physics/GranularBody/ContactConstraintEntity.h>
#include <agx/Physics/Particle/ContactConstraintRowEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Range6.h>
#include <agx/Physics/SolveMaterialEntity.h>
#include <agx/Solver.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGranularContact_NORMAL

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGranularContact_NORMAL
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement32 >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement32 >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::SolveMaterial32Data& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGranularContact_FRICTION

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGranularContact_FRICTION
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement32 >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement32 >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::SolveMaterial32Data& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGranularContact_ROLLING_RESISTANCE

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGranularContact_ROLLING_RESISTANCE
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement32 >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement32 >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::SolveMaterial32Data& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGranularContact_TWIST_RESISTANCE

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGranularContact_TWIST_RESISTANCE
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement32 >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement32 >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::SolveMaterial32Data& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGeometryContact_NORMAL

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param rigidBody 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGeometryContact_NORMAL
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::SolveMaterialData& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGeometryContact_FRICTION

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param rigidBody 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGeometryContact_FRICTION
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::SolveMaterialData& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGeometryContact_ROLLING_RESISTANCE

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param rigidBody 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGeometryContact_ROLLING_RESISTANCE
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::SolveMaterialData& solveMaterial
      );


      /**
      Function: Physics.GranularBody.GaussSeidelIteration32
      Implementation: GranularGeometryContact_TWIST_RESISTANCE

      \param constraint 
      \param constraintRow 
      \param jacobianStorage 
      \param GMinvStorage 
      \param particle 
      \param rigidBody 
      \param solveMaterial 
      */
      void GaussSeidelIteration32__GranularGeometryContact_TWIST_RESISTANCE
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement >& jacobianStorage,
        const agxData::Array< agx::Jacobian6DOFElement >& GMinvStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::SolveMaterialData& solveMaterial
      );


    }
  }
}

#endif
