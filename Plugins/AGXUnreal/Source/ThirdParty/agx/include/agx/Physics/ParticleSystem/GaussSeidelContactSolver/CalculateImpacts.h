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

#ifndef AGXFN_PHYSICS_PARTICLESYSTEM_GAUSSSEIDELCONTACTSOLVER_CALCULATEIMPACTS_H
#define AGXFN_PHYSICS_PARTICLESYSTEM_GAUSSSEIDELCONTACTSOLVER_CALCULATEIMPACTS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>


namespace agx { namespace Physics { namespace ParticleSystem { namespace GaussSeidelContactSolver { } } } }

namespace agxFn
{
  namespace Physics
  {
    namespace ParticleSystem
    {
      namespace GaussSeidelContactSolver
      {
        /**
        Function: Physics.ParticleSystem.GaussSeidelContactSolver.CalculateImpacts
        Implementation: ContactZone

        \param job The range job specifying what part of the data set to process
        \param contactZone_isImpacting 
        \param contactZone_contactList 
        \param constraint_isImpacting 
        */
        void CalculateImpacts__ContactZone
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agxData::Array< agx::Bool >& contactZone_isImpacting,
          const agxData::Array< agxData::Array< agx::UInt32 > >& contactZone_contactList,
          const agxData::Array< agx::Bool >& constraint_isImpacting
        );


      }
    }
  }
}

#endif
