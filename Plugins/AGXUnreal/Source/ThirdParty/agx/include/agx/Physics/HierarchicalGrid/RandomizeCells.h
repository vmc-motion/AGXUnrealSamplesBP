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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_RANDOMIZECELLS_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_RANDOMIZECELLS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/HierarchicalGrid/CellEntity.h>
#include <agx/Vec3.h>
#include <agx/IndexRange.h>
#include <agx/Physics/HierarchicalGrid/ContactZoneEntity.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.RandomizeCells
      Implementation: (default)

      \param cell 
      \param sortedCells 
      \param sortedCellsBuffer 
      */
      void RandomizeCells
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::HierarchicalGrid::CellData& cell,
        agxData::Array< agx::UInt32 >& sortedCells,
        agxData::Buffer* sortedCellsBuffer
      );


    }
  }
}

#endif
