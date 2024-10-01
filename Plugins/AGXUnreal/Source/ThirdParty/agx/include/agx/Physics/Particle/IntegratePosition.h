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

#ifndef AGXFN_PHYSICS_PARTICLE_INTEGRATEPOSITION_H
#define AGXFN_PHYSICS_PARTICLE_INTEGRATEPOSITION_H

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


namespace agx { namespace Physics { namespace Particle { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Particle
    {
      /**
      Function: Physics.Particle.IntegratePosition
      Implementation: (default)

      \param particle 
      \param clock_timeStep 
      */
      void IntegratePosition
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::ParticleInstance particle,
        const agx::Real& clock_timeStep
      );


      /**
      Function: Physics.Particle.IntegratePosition
      Implementation: (default)

      \param particle 
      \param clock_timeStep 
      */
      void IntegratePosition
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::ParticlePtr particle,
        const agx::Real& clock_timeStep
      );


      /**
      Function: Physics.Particle.IntegratePosition
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param particle 
      \param clock_timeStep 
      */
      void IntegratePosition
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ParticleData& particle,
        const agx::Real& clock_timeStep
      );


    }
  }
}

#endif
