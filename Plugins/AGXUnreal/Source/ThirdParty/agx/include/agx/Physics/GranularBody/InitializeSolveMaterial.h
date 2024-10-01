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

#ifndef AGXFN_PHYSICS_GRANULARBODY_INITIALIZESOLVEMATERIAL_H
#define AGXFN_PHYSICS_GRANULARBODY_INITIALIZESOLVEMATERIAL_H

#include <agxData/Array.h>
#include <agxData/EntityPtr.h>
#include <agx/Integer.h>
#include <agx/Real.h>
#include <agx/Math.h>
#include <agx/Job.h>
#include <agx/Physics/ContactMaterialEntity.h>
#include <agx/SpinMutex.h>
#include <agx/Physics/MaterialEntity.h>
#include <agx/Vec3.h>
#include <agx/Physics/SolveMaterialEntity.h>
#include <agx/Vec2.h>
#include <agx/Physics/SolveMaterial32Entity.h>


namespace agx { namespace Physics { namespace GranularBody { } } }

namespace agxFn
{
  namespace Physics
  {
    namespace GranularBody
    {
      /**
      Function: Physics.GranularBody.InitializeSolveMaterial
      Implementation: (default)

      \param job The range job specifying what part of the data set to process
      \param contactMaterial 
      \param solveMaterial 
      \param timeStep 
      */
      void InitializeSolveMaterial
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ContactMaterialData& contactMaterial,
        agx::Physics::SolveMaterialData& solveMaterial,
        const agx::Real& timeStep
      );


      /**
      Function: Physics.GranularBody.InitializeSolveMaterial
      Implementation: Real32

      \param job The range job specifying what part of the data set to process
      \param contactMaterial 
      \param solveMaterial 
      \param solveMaterial32 
      \param timeStep 
      */
      void InitializeSolveMaterial__Real32
      (
        /* Parameter list automatically generated, do not edit */
        const agx::RangeJob& job,
        agx::Physics::ContactMaterialData& contactMaterial,
        agx::Physics::SolveMaterialData& solveMaterial,
        agx::Physics::SolveMaterial32Data& solveMaterial32,
        const agx::Real& timeStep
      );


    }
  }
}

#endif
