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

#ifndef AGXFN_PHYSICS_PARTITIONER_INTERACTIONGRAPHSPLITTER_H
#define AGXFN_PHYSICS_PARTITIONER_INTERACTIONGRAPHSPLITTER_H

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
#include <agxSDK/Simulation.h>


namespace agx { namespace Physics { namespace Partitioner { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace Partitioner
    {
      /**
      Function: Physics.Partitioner.InteractionGraphSplitter
      Implementation: (default)

      \param graphNode 
      \param rigidBody 
      \param simulation 
      */
      void InteractionGraphSplitter
      (
        /* Parameter list automatically generated, do not edit */
        agx::Physics::GraphNodeData& graphNode,
        agx::Physics::RigidBodyData& rigidBody,
        agxSDK::Simulation* simulation
      );


    }
  }
}

#endif
