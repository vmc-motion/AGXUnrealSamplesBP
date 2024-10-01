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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_TRIGGERSUBSYSTEMREORDERING_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_TRIGGERSUBSYSTEMREORDERING_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.TriggerSubsystemReordering
      Implementation: (default)

      \param reorderSubsystems 
      \param clock_time 
      \param clock_timeStep 
      \param subsystemReorderingFrequency 
      \param lastReorderTime 
      */
      void TriggerSubsystemReordering
      (
        /* Parameter list automatically generated, do not edit */
        agx::Task* reorderSubsystems,
        const agx::Real& clock_time,
        const agx::Real& clock_timeStep,
        const agx::Real& subsystemReorderingFrequency,
        agx::Real& lastReorderTime
      );


    }
  }
}

#endif
