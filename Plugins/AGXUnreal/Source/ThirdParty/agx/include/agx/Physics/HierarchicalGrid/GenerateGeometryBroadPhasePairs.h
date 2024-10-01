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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_GENERATEGEOMETRYBROADPHASEPAIRS_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_GENERATEGEOMETRYBROADPHASEPAIRS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/HierarchicalGrid/GridOverlapEntity.h>
#include <agx/Physics/HierarchicalGrid/CollisionObjectEntity.h>
#include <agx/Vec3.h>
#include <agx/Physics/HierarchicalGrid/OrientedGeometryBoundEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Bound.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/MaterialEntity.h>
#include <agxCollide/Space.h>
#include <agx/Physics/BroadPhasePairEntity.h>
#include <agx/Physics/GeometryContactEntity.h>
#include <agx/Physics/WarmStartingDataEntity.h>
#include <agx/GlobalResult.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.GenerateGeometryBroadPhasePairs
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param geometryPairOverlap 
      \param collisionObject 
      \param orientedGeometryBound 
      \param geometry 
      \param space 
      \param broadPhasePair 
      */
      void GenerateGeometryBroadPhasePairs
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::HierarchicalGrid::GridOverlapData& geometryPairOverlap,
        agx::Physics::HierarchicalGrid::CollisionObjectData& collisionObject,
        agx::Physics::HierarchicalGrid::OrientedGeometryBoundData& orientedGeometryBound,
        agx::Physics::GeometryData& geometry,
        agxCollide::Space* space,
        agx::Physics::BroadPhasePairData& broadPhasePair
      );


    }
  }
}

#endif
