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

#ifndef AGXFN_PHYSICS_PARTITIONER_GATHERDRYFRICTIONCONSTRAINTS_H
#define AGXFN_PHYSICS_PARTITIONER_GATHERDRYFRICTIONCONSTRAINTS_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/SolveIslandEntity.h>
#include <agx/Vec4.h>
#include <agx/Physics/SolveGroupEntity.h>
#include <agxSDK/Simulation.h>
#include <agx/Physics/GeometryContactEntity.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/Physics/ContactPointEntity.h>
#include <agx/Physics/BroadPhasePairEntity.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>

namespace agx { class SolveModel; }

namespace agx { namespace Physics { namespace Partitioner { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Partitioner
    {
      /**
      Function: Physics.Partitioner.GatherDryFrictionConstraints
      Implementation: (default)

      \param solveIsland 
      */
      void GatherDryFrictionConstraints
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::SolveIslandData& solveIsland
      );


      /**
      Function: Physics.Partitioner.GatherDryFrictionConstraints
      Implementation: old

      \param solveIsland 
      \param simulation 
      \param dryFrictionConstraintElementBuffer 
      \param dryFrictionConstraintGroup 
      \param generateDebugRendering 
      \param geometryContact 
      \param binaryConstraint 
      \param manyBodyConstraint 
      */
      void GatherDryFrictionConstraints__old
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::SolveIslandData& solveIsland,
        agxSDK::Simulation* simulation,
        agxData::Buffer* dryFrictionConstraintElementBuffer,
        agx::Physics::SolveGroupData& dryFrictionConstraintGroup,
        const agx::Bool& generateDebugRendering,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::BinaryConstraintData& binaryConstraint,
        agx::Physics::ManyBodyConstraintData& manyBodyConstraint
      );


      /**
      Function: Physics.Partitioner.GatherDryFrictionConstraints
      Implementation: oldest

      \param geometryContact 
      \param graphNode 
      \param dryFrictionConstraintGroupStorage 
      \param dryFrictionConstraintElementBuffer 
      \param dryFrictionConstraintGroup_nodes 
      \param dryFrictionConstraintGroup_solveModel 
      \param dryFrictionConstraintGroup_nodes_elements 
      \param generateDebugRendering 
      */
      void GatherDryFrictionConstraints__oldest
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::GraphNodeData& graphNode,
        agxData::EntityStorage* dryFrictionConstraintGroupStorage,
        agxData::Buffer* dryFrictionConstraintElementBuffer,
        agxData::Array< agxData::Array< agx::UInt > >& dryFrictionConstraintGroup_nodes,
        agxData::Array< agx::SolveModel * >& dryFrictionConstraintGroup_solveModel,
        agxData::Array< agx::UInt >& dryFrictionConstraintGroup_nodes_elements,
        const agx::Bool& generateDebugRendering
      );


    }
  }
}

#endif
