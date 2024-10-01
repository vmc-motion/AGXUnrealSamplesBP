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

#ifndef AGXFN_PHYSICS_PARTICLESYSTEM_HANDLEREMOVEDPARTICLES_H
#define AGXFN_PHYSICS_PARTICLESYSTEM_HANDLEREMOVEDPARTICLES_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/ParticleState.h>
#include <agx/GlobalResult.h>


namespace agx { namespace Physics { namespace ParticleSystem { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace ParticleSystem
    {
      /**
      Function: Physics.ParticleSystem.HandleRemovedParticles
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param particle_state 
      \param deadParticles 
      */
      void HandleRemovedParticles
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agx::ParticleState >& particle_state,
        agxData::Array< agx::UInt32 >& deadParticles
      );


    }
  }
}

#endif
