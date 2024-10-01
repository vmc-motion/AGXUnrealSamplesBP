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

#ifndef AGXFN_PHYSICS_GEOMETRY_SYNCHRONIZESHAPEBOUNDS_H
#define AGXFN_PHYSICS_GEOMETRY_SYNCHRONIZESHAPEBOUNDS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/Geometry/SphereEntity.h>
#include <agx/SpinMutex.h>
#include <agxCollide/BoundingAABB.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/Geometry/BoxEntity.h>
#include <agx/Vec3.h>
#include <agx/Physics/Geometry/CapsuleEntity.h>
#include <agx/Physics/Geometry/ConeEntity.h>
#include <agx/Physics/Geometry/HollowConeEntity.h>
#include <agx/Physics/Geometry/CylinderEntity.h>
#include <agx/Physics/Geometry/HollowCylinderEntity.h>
#include <agx/Physics/Geometry/LineEntity.h>
#include <agx/Line.h>
#include <agx/Physics/Geometry/PlaneEntity.h>
#include <agx/Plane.h>
#include <agx/Physics/Geometry/TrimeshEntity.h>
#include <agx/Physics/Geometry/HeightFieldEntity.h>
#include <agx/Physics/Geometry/ConvexEntity.h>
#include <agx/Physics/Geometry/ShapeGroupEntity.h>
#include <agx/Physics/Geometry/ShapeEntity.h>


namespace agx { namespace Physics { namespace Geometry { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Geometry
    {
      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Sphere

      \param job The range job specifying what part of the data set to process
      \param sphere 
      */
      void SynchronizeShapeBounds__Sphere
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::SphereData& sphere
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Box

      \param job The range job specifying what part of the data set to process
      \param box 
      */
      void SynchronizeShapeBounds__Box
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::BoxData& box
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Capsule

      \param job The range job specifying what part of the data set to process
      \param capsule 
      */
      void SynchronizeShapeBounds__Capsule
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::CapsuleData& capsule
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: WireShape

      \param job The range job specifying what part of the data set to process
      \param wireShape_boundingAABB 
      \param wireShape_transform 
      \param wireShape_height 
      \param wireShape_radius 
      \param wireShape_previousPoint0 
      \param wireShape_previousPoint1 
      */
      void SynchronizeShapeBounds__WireShape
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agxData::Array< agxCollide::BoundingAABB >& wireShape_boundingAABB,
        const agxData::Array< agx::AffineMatrix4x4 >& wireShape_transform,
        const agxData::Array< agx::Real >& wireShape_height,
        const agxData::Array< agx::Real >& wireShape_radius,
        const agxData::Array< agx::Vec3 >& wireShape_previousPoint0,
        const agxData::Array< agx::Vec3 >& wireShape_previousPoint1
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Cone

      \param job The range job specifying what part of the data set to process
      \param cone 
      */
      void SynchronizeShapeBounds__Cone
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::ConeData& cone
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: HollowCone

      \param job The range job specifying what part of the data set to process
      \param hollowCone 
      */
      void SynchronizeShapeBounds__HollowCone
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::HollowConeData& hollowCone
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Cylinder

      \param job The range job specifying what part of the data set to process
      \param cylinder 
      */
      void SynchronizeShapeBounds__Cylinder
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::CylinderData& cylinder
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: HollowCylinder

      \param job The range job specifying what part of the data set to process
      \param hollowCylinder 
      */
      void SynchronizeShapeBounds__HollowCylinder
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::HollowCylinderData& hollowCylinder
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Line

      \param job The range job specifying what part of the data set to process
      \param line 
      */
      void SynchronizeShapeBounds__Line
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::LineData& line
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Plane

      \param job The range job specifying what part of the data set to process
      \param plane 
      */
      void SynchronizeShapeBounds__Plane
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::PlaneData& plane
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Trimesh

      \param job The range job specifying what part of the data set to process
      \param trimesh 
      */
      void SynchronizeShapeBounds__Trimesh
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::TrimeshData& trimesh
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: HeightField

      \param job The range job specifying what part of the data set to process
      \param heightField 
      */
      void SynchronizeShapeBounds__HeightField
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::HeightFieldData& heightField
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: Convex

      \param job The range job specifying what part of the data set to process
      \param convex 
      */
      void SynchronizeShapeBounds__Convex
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::ConvexData& convex
      );


      /**
      Function: Physics.Geometry.SynchronizeShapeBounds
      Implementation: ShapeGroup

      \param job The range job specifying what part of the data set to process
      \param shapeGroup 
      */
      void SynchronizeShapeBounds__ShapeGroup
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::Geometry::ShapeGroupData& shapeGroup
      );


    }
  }
}

#endif
