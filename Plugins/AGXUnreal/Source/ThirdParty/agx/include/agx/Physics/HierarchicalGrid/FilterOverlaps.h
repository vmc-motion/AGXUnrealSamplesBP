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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_FILTEROVERLAPS_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_FILTEROVERLAPS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/HierarchicalGrid/GridOverlapEntity.h>
#include <agx/Physics/HierarchicalGrid/CollisionObjectEntity.h>
#include <agx/Vec3.h>
#include <agx/GlobalResult.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.FilterOverlaps
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param gridOverlap 
      \param collisionObject 
      \param geometryPairOverlap 
      \param particlePairOverlap 
      \param particleGeometryOverlap 
      */
      void FilterOverlaps
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::HierarchicalGrid::GridOverlapData& gridOverlap,
        agx::Physics::HierarchicalGrid::CollisionObjectData& collisionObject,
        agx::Physics::HierarchicalGrid::GridOverlapData& geometryPairOverlap,
        agx::Physics::HierarchicalGrid::GridOverlapData& particlePairOverlap,
        agx::Physics::HierarchicalGrid::GridOverlapData& particleGeometryOverlap
      );


    }
  }
}

#endif
