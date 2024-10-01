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

#ifndef AGXFN_PHYSICS_HIERARCHICALGRID_VERIFYOVERLAPS_H
#define AGXFN_PHYSICS_HIERARCHICALGRID_VERIFYOVERLAPS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Vec3.h>
#include <agx/IndexRange.h>
#include <agx/Vec2.h>


namespace agx { namespace Physics { namespace HierarchicalGrid { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace HierarchicalGrid
    {
      /**
      Function: Physics.HierarchicalGrid.VerifyOverlaps
      Implementation: Particle

      \param particle_instance 
      \param particle_radius 
      \param particle_position 
      \param particle_cellIndex 
      \param particle_cellSlot 
      \param cell_tier 
      \param cell_id 
      \param cell_parent 
      \param cell_collisionObjects 
      \param gridTier_size 
      \param gridOverlap_collisionObject1 
      \param gridOverlap_collisionObject2 
      */
      void VerifyOverlaps__Particle
      (
        /* Parameter list automatically generated, do not edit */
        agxData::Array< agxData::EntityPtr >& particle_instance,
        const agxData::Array< agx::Real >& particle_radius,
        const agxData::Array< agx::Vec3 >& particle_position,
        const agxData::Array< agx::UInt32 >& particle_cellIndex,
        const agxData::Array< agx::UInt16 >& particle_cellSlot,
        const agxData::Array< agx::UInt8 >& cell_tier,
        const agxData::Array< agx::Vec3i >& cell_id,
        const agxData::Array< agx::UInt32 >& cell_parent,
        const agxData::Array< agx::IndexRange32 >& cell_collisionObjects,
        const agxData::Array< agx::Real >& gridTier_size,
        const agxData::Array< agx::UInt32 >& gridOverlap_collisionObject1,
        const agxData::Array< agx::UInt32 >& gridOverlap_collisionObject2
      );


      /**
      Function: Physics.HierarchicalGrid.VerifyOverlaps
      Implementation: Particle_2D

      \param particle_instance 
      \param particle_radius 
      \param particle_position 
      \param particle_cellIndex 
      \param particle_cellSlot 
      \param cell_tier 
      \param cell_id 
      \param cell_parent 
      \param cell_collisionObjects 
      \param gridTier_size 
      \param gridOverlap_collisionObject1 
      \param gridOverlap_collisionObject2 
      */
      void VerifyOverlaps__Particle_2D
      (
        /* Parameter list automatically generated, do not edit */
        agxData::Array< agxData::EntityPtr >& particle_instance,
        const agxData::Array< agx::Real >& particle_radius,
        const agxData::Array< agx::Vec3 >& particle_position,
        const agxData::Array< agx::UInt32 >& particle_cellIndex,
        const agxData::Array< agx::UInt16 >& particle_cellSlot,
        const agxData::Array< agx::UInt8 >& cell_tier,
        const agxData::Array< agx::Vec2i >& cell_id,
        const agxData::Array< agx::UInt32 >& cell_parent,
        const agxData::Array< agx::IndexRange32 >& cell_collisionObjects,
        const agxData::Array< agx::Real >& gridTier_size,
        const agxData::Array< agx::UInt32 >& gridOverlap_collisionObject1,
        const agxData::Array< agx::UInt32 >& gridOverlap_collisionObject2
      );


    }
  }
}

#endif
