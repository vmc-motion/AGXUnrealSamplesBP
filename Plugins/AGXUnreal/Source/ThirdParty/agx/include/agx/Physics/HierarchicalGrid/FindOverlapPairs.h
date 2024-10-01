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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_FINDOVERLAPPAIRS_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_FINDOVERLAPPAIRS_H

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
#include <agx/Physics/HierarchicalGrid/CollisionObjectEntity.h>
#include <agx/Physics/HierarchicalGrid/OrientedGeometryBoundEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Bound.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/HierarchicalGrid/GridOverlapEntity.h>
#include <agx/GlobalResult.h>
#include <agxCollide/CollisionGroupManager.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.FindOverlapPairs
      Implementation: IterateCells

      \param job The range job specifying what part of the data set to process
      \param sortedCells 
      \param cell 
      \param gridTier 
      \param collisionObject 
      \param orientedGeometryBound 
      \param gridOverlap 
      \param collisionGroupManager 
      */
      void FindOverlapPairs__IterateCells
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::UInt32 >& sortedCells,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agx::Physics::HierarchicalGrid::CollisionObjectData& collisionObject,
        agx::Physics::HierarchicalGrid::OrientedGeometryBoundData& orientedGeometryBound,
        agx::Physics::HierarchicalGrid::GridOverlapData& gridOverlap,
        agxCollide::CollisionGroupManager* collisionGroupManager
      );


      /**
      Function: Physics.HierarchicalGrid.FindOverlapPairs
      Implementation: ExternalQueries

      \param numObjects 
      \param rootCells 
      \param cell 
      \param gridTier 
      \param collisionObject 
      \param orientedGeometryBound 
      \param gridOverlap 
      \param collisionGroupManager 
      */
      void FindOverlapPairs__ExternalQueries
      (
        /* Parameter list automatically generated, do not edit */
        const agx::UInt& numObjects,
        agxData::Array< agx::Physics::HierarchicalGrid::CellPtr >& rootCells,
        agx::Physics::HierarchicalGrid::CellData& cell,
        agx::Physics::HierarchicalGrid::GridTierData& gridTier,
        agx::Physics::HierarchicalGrid::CollisionObjectData& collisionObject,
        agx::Physics::HierarchicalGrid::OrientedGeometryBoundData& orientedGeometryBound,
        agx::Physics::HierarchicalGrid::GridOverlapData& gridOverlap,
        agxCollide::CollisionGroupManager* collisionGroupManager
      );


      /**
      Function: Physics.HierarchicalGrid.FindOverlapPairs
      Implementation: 2D_IterateCells

      */
      void FindOverlapPairs__2D_IterateCells
      (
        /* Parameter list automatically generated, do not edit */
      );


      /**
      Function: Physics.HierarchicalGrid.FindOverlapPairs
      Implementation: 2D_BottomUp

      */
      void FindOverlapPairs__2D_BottomUp
      (
        /* Parameter list automatically generated, do not edit */
      );


    }
  }
}

#endif
