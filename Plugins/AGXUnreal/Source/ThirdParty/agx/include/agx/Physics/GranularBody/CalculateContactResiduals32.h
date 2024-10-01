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

#ifndef AGXFN_PHYSICS_GRANULARBODY_CALCULATECONTACTRESIDUALS32_H
#define AGXFN_PHYSICS_GRANULARBODY_CALCULATECONTACTRESIDUALS32_H

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
#include <agx/Solver.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      /**
      Function: Physics.GranularBody.CalculateContactResiduals32
      Implementation: GranularGranularContact

      \param constraint 
      \param constraintRow 
      \param granularGranularJacobianStorage 
      \param particle 
      \param contactMode 
      \param iterationResiduals 
      \param iterationResidualMutex 
      */
      void CalculateContactResiduals32__GranularGranularContact
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement32 >& granularGranularJacobianStorage,
        agx::Physics::SolveBody32Data& particle,
        const agx::UInt& contactMode,
        agxData::Array< agx::Real >& iterationResiduals,
        agx::SpinMutex& iterationResidualMutex
      );


      /**
      Function: Physics.GranularBody.CalculateContactResiduals32
      Implementation: GranularGranularContact_LocalJacobians

      \param constraint 
      \param constraintRow 
      \param particle 
      \param contactMode 
      \param iterationResiduals 
      \param iterationResidualMutex 
      */
      void CalculateContactResiduals32__GranularGranularContact_LocalJacobians
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraint32Data& constraint,
        agx::Physics::Particle::ContactConstraintRow32Data& constraintRow,
        agx::Physics::SolveBody32Data& particle,
        const agx::UInt& contactMode,
        agxData::Array< agx::Real >& iterationResiduals,
        agx::SpinMutex& iterationResidualMutex
      );


      /**
      Function: Physics.GranularBody.CalculateContactResiduals32
      Implementation: GranularGeometryContact

      \param constraint 
      \param constraintRow 
      \param granularGeometryJacobianStorage 
      \param particle 
      \param rigidBody 
      \param contactMode 
      \param iterationResiduals 
      \param iterationResidualMutex 
      */
      void CalculateContactResiduals32__GranularGeometryContact
      (
        /* Parameter list automatically generated, do not edit */
        agx::SolveContext context,
        agx::Physics::GranularBody::ContactConstraintData& constraint,
        agx::Physics::Particle::ContactConstraintRowData& constraintRow,
        const agxData::Array< agx::Jacobian6DOFElement >& granularGeometryJacobianStorage,
        agx::Physics::SolveBody32Data& particle,
        agx::Physics::RigidBodyData& rigidBody,
        const agx::UInt& contactMode,
        agxData::Array< agx::Real >& iterationResiduals,
        agx::SpinMutex& iterationResidualMutex
      );


    }
  }
}

#endif
