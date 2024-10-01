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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_RENDERING_GENERATECELLLINES_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_RENDERING_GENERATECELLLINES_H

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
#include <agx/Physics/HierarchicalGrid/GridTierEntity.h>
#include <agx/Line.h>
#include <agx/Vec4.h>
#include <agx/Vec2.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { namespace Rendering { } } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      namespace Rendering
      {
        /**
        Function: Physics.HierarchicalGrid.Rendering.GenerateCellLines
        Implementation: (default)

        \param job The range job specifying what part of the data set to process
        \param cell 
        \param gridTier 
        \param gridLines 
        \param lineColors 
        */
        void GenerateCellLines
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::HierarchicalGrid::CellData& cell,
          agx::Physics::HierarchicalGrid::GridTierData& gridTier,
          agxData::Array< agx::Line32 >& gridLines,
          agxData::Array< agx::Vec4f >& lineColors
        );


        /**
        Function: Physics.HierarchicalGrid.Rendering.GenerateCellLines
        Implementation: ZoneTags

        \param job The range job specifying what part of the data set to process
        \param cell 
        \param gridTier 
        \param gridLines 
        \param lineColors 
        */
        void GenerateCellLines__ZoneTags
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          agx::Physics::HierarchicalGrid::CellData& cell,
          agx::Physics::HierarchicalGrid::GridTierData& gridTier,
          agxData::Array< agx::Line32 >& gridLines,
          agxData::Array< agx::Vec4f >& lineColors
        );


        /**
        Function: Physics.HierarchicalGrid.Rendering.GenerateCellLines
        Implementation: 2D

        \param job The range job specifying what part of the data set to process
        \param cell_tier 
        \param cell_id 
        \param gridTier_size 
        \param gridLines 
        \param lineColors 
        */
        void GenerateCellLines__2D
        (
          /* Parameter list automatically generated, do not edit */
          const agx::RangeJob& job,
          const agxData::Array< agx::UInt8 >& cell_tier,
          const agxData::Array< agx::Vec2i >& cell_id,
          const agxData::Array< agx::Real >& gridTier_size,
          agxData::Array< agx::Line32 >& gridLines,
          agxData::Array< agx::Vec4f >& lineColors
        );


      }
    }
  }
}

#endif
