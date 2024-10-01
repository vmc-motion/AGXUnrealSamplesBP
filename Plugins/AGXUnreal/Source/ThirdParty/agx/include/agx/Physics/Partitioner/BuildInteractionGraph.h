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

#ifndef AGXFN_PHYSICS_PARTITIONER_BUILDINTERACTIONGRAPH_H
#define AGXFN_PHYSICS_PARTITIONER_BUILDINTERACTIONGRAPH_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/GraphNodeEntity.h>
#include <agx/Physics/RigidBodyEntity.h>
#include <agx/SpinMutex.h>
#include <agx/RigidBodyState.h>
#include <agx/Name.h>
#include <agx/Vec3.h>
#include <agx/SPDMatrix3x3.h>
#include <agx/Matrix3x3.h>
#include <agx/AffineMatrix4x4.h>
#include <agx/Physics/GeometryEntity.h>
#include <agx/Range6.h>
#include <agx/Physics/ParticleEntity.h>
#include <agx/ParticleState.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Physics/CollisionGroupSetEntity.h>
#include <agx/Uuid.h>
#include <agx/Vec4.h>
#include <agx/Physics/ContactConstraintEntity.h>
#include <agx/IndexRange.h>
#include <agx/Physics/GeometryContactEntity.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/Physics/ContactPointEntity.h>
#include <agx/Physics/BroadPhasePairEntity.h>
#include <agx/Physics/ParticlePairContactEntity.h>
#include <agx/Physics/ParticleGeometryContactEntity.h>
#include <agx/Physics/BinaryConstraintEntity.h>
#include <agx/Physics/ManyBodyConstraintEntity.h>
#include <agx/Physics/StrongInteractionEntity.h>
#include <agxSDK/Simulation.h>


namespace agx { namespace Physics { namespace Partitioner { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Partitioner
    {
      /**
      Function: Physics.Partitioner.BuildInteractionGraph
      Implementation: (default)

      \param graphNode 
      \param graphNode_edges_elements 
      \param graphEdgeElementBuffer 
      \param rigidBody 
      \param particle 
      \param activeGeometryContacts 
      \param activeParticlePairContacts 
      \param activeParticleGeometryContacts 
      \param contactConstraint 
      \param geometryContact 
      \param particlePairContact 
      \param particleGeometryContact 
      \param binaryConstraint 
      \param manyBodyConstraint 
      \param strongInteraction 
      \param simulation 
      */
      void BuildInteractionGraph
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::GraphNodeData& graphNode,
        agxData::Array< agx::UInt >& graphNode_edges_elements,
        agxData::Buffer* graphEdgeElementBuffer,
        agx::Physics::RigidBodyData& rigidBody,
        agx::Physics::ParticleData& particle,
        const agxData::Array< agx::UInt32 >& activeGeometryContacts,
        const agxData::Array< agx::UInt32 >& activeParticlePairContacts,
        const agxData::Array< agx::UInt32 >& activeParticleGeometryContacts,
        agx::Physics::ContactConstraintData& contactConstraint,
        agx::Physics::GeometryContactData& geometryContact,
        agx::Physics::ParticlePairContactData& particlePairContact,
        agx::Physics::ParticleGeometryContactData& particleGeometryContact,
        agx::Physics::BinaryConstraintData& binaryConstraint,
        agx::Physics::ManyBodyConstraintData& manyBodyConstraint,
        agx::Physics::StrongInteractionData& strongInteraction,
        agxSDK::Simulation* simulation
      );


    }
  }
}

#endif
