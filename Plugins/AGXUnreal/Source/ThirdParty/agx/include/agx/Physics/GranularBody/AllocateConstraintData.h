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

#ifndef AGXFN_PHYSICS_GRANULARBODY_ALLOCATECONSTRAINTDATA_H
#define AGXFN_PHYSICS_GRANULARBODY_ALLOCATECONSTRAINTDATA_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Solver.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      /**
      Function: Physics.GranularBody.AllocateConstraintData
      Implementation: (default)

      \param particleSystem_ActiveParticlePairContacts 
      \param particleSystem_ActiveParticleGeometryContacts 
      \param particleParticleConstraintStorage 
      \param particleBodyConstraintStorage 
      \param particleParticleRowStorage 
      \param particleBodyRowStorage 
      \param granularGranularJacobianStorage 
      \param granularGranularGMInvStorage 
      \param granularGeometryJacobianStorage 
      \param granularGeometryGMInvStorage 
      \param contactMode 
      \param solver 
      */
      void AllocateConstraintData
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt32 >& particleSystem_ActiveParticlePairContacts,
        const agxData::Array< agx::UInt32 >& particleSystem_ActiveParticleGeometryContacts,
        agxData::EntityStorage* particleParticleConstraintStorage,
        agxData::EntityStorage* particleBodyConstraintStorage,
        agxData::EntityStorage* particleParticleRowStorage,
        agxData::EntityStorage* particleBodyRowStorage,
        agxData::Buffer* granularGranularJacobianStorage,
        agxData::Buffer* granularGranularGMInvStorage,
        agxData::Buffer* granularGeometryJacobianStorage,
        agxData::Buffer* granularGeometryGMInvStorage,
        const agx::UInt& contactMode,
        agx::Solver* solver
      );


    }
  }
}

#endif
