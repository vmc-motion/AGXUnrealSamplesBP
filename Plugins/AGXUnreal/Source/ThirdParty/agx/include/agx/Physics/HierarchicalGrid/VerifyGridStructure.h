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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_VERIFYGRIDSTRUCTURE_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_VERIFYGRIDSTRUCTURE_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>
#include <agx/Physics/HierarchicalGrid/Common.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.VerifyGridStructure
      Implementation: (default)

      \param cell_state 
      \param cell_tier 
      \param cell_id 
      \param cell_parent 
      \param cell_children 
      \param cell_numChildren 
      \param cell_neighbors 
      \param gridTier_cellTable 
      */
      void VerifyGridStructure
      (
        /* Parameter list automatically generated, do not edit */
        const agxData::Array< agx::UInt8 >& cell_state,
        const agxData::Array< agx::UInt8 >& cell_tier,
        const agxData::Array< agx::Vec3i >& cell_id,
        const agxData::Array< agx::UInt32 >& cell_parent,
        const agxData::Array< agx::UInt32 >& cell_children,
        const agxData::Array< agx::UInt8 >& cell_numChildren,
        const agxData::Array< agx::UInt32 >& cell_neighbors,
        const agxData::Array< agx::GridCellTable* >& gridTier_cellTable
      );


    }
  }
}

#endif
