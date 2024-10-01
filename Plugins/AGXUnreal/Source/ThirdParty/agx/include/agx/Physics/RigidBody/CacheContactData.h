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

#ifndef AGXFN_PHYSICS_RIGIDBODY_CACHECONTACTDATA_H
#define AGXFN_PHYSICS_RIGIDBODY_CACHECONTACTDATA_H

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
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/ContactPointEntity.h>
#include <agx/Vec3.h>
#include <agx/Vec4.h>
#include <agx/Physics/Geometry/ShapeEntity.h>


namespace agx { namespace Physics { namespace RigidBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace RigidBody
    {
      /**
      Function: Physics.RigidBody.CacheContactData
      Implementation: (default)

      \param broadPhasePair 
      \param geometryContact 
      \param contactPoint 
      \param warmStartingData 
      */
      void CacheContactData
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::BroadPhasePairData& broadPhasePair,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::ContactPointData& contactPoint,
        agx::Physics::WarmStartingDataData& warmStartingData
      );


    }
  }
}

#endif
