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

#ifndef AGXFN_PHYSICS_GEOMETRY_COMPUTENARROWPHASECONTACTS_H
#define AGXFN_PHYSICS_GEOMETRY_COMPUTENARROWPHASECONTACTS_H

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
#include <agx/SpinMutex.h>
#include <agx/Name.h>
#include <agxCollide/GeometryState.h>
#include <agx/Physics/Geometry/ShapeEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Vec3.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/ContactPointEntity.h>
#include <agx/Vec4.h>
#include <agx/GlobalResult.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/GeometryPairEntity.h>
#include <agxSDK/MaterialManager.h>


namespace agx { namespace Physics { namespace Geometry { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Geometry
    {
      /**
      Function: Physics.Geometry.ComputeNarrowPhaseContacts
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param filteredBroadPhasePairs 
      \param broadPhasePair 
      \param geometry 
      \param contactPoint 
      \param geometryContact 
      \param separationPair 
      \param materialManager 
      \param materialLessGeometryContacts 
      \param contactReductionEnable 
      \param contactReductionThreshold 
      \param contactReductionBinResolution 
      */
      void ComputeNarrowPhaseContacts
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        const agxData::Array< agx::UInt32 >& filteredBroadPhasePairs,
        agx::Physics::BroadPhasePairData& broadPhasePair,
        agx::Physics::GeometryData& geometry,
        agx::Physics::ContactPointData& contactPoint,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::GeometryPairData& separationPair,
        agxSDK::MaterialManager* materialManager,
        agxData::Array< agx::Physics::GeometryContactPtr >& materialLessGeometryContacts,
        const agx::Bool& contactReductionEnable,
        const agx::UInt& contactReductionThreshold,
        const agx::UInt& contactReductionBinResolution
      );


    }
  }
}

#endif
