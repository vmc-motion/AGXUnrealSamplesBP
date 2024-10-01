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

#ifndef AGXFN_PHYSICS_GEOMETRY_FILTERBROADPHASEOVERLAPS_H
#define AGXFN_PHYSICS_GEOMETRY_FILTERBROADPHASEOVERLAPS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/BroadPhasePairEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Physics/GeometryContactEntity.h>
#include <agx/Physics/WarmStartingDataEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/SpinMutex.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/Vec3.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Range6.h>
#include <agx/Physics/HierarchicalGrid/OrientedGeometryBoundEntity.h>
#include <agx/Bound.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Physics/MaterialEntity.h>
#include <agxData/EntityStorage.h>
#include <agx/GlobalResult.h>
#include <agx/Physics/GeometryPairEntity.h>
#include <agxCollide/Space.h>


namespace agx { namespace Physics { namespace Geometry { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Geometry
    {
      /**
      Function: Physics.Geometry.FilterBroadPhaseOverlaps
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param broadPhasePair 
      \param rigidBody 
      \param orientedGeometryBound 
      \param geometry 
      \param orientedGeometryBoundStorage 
      \param filteredBroadPhasePairs 
      \param separationPair 
      \param space 
      */
      void FilterBroadPhaseOverlaps
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::BroadPhasePairData& broadPhasePair,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::HierarchicalGrid::OrientedGeometryBoundData& orientedGeometryBound,
        agx::Physics::GeometryData& geometry,
        agxData::EntityStorage* orientedGeometryBoundStorage,
        agxData::Array< agx::UInt32 >& filteredBroadPhasePairs,
        agx::Physics::GeometryPairData& separationPair,
        agxCollide::Space* space
      );


    }
  }
}

#endif
