/*
Copyright 2007-2024. Algoryx Simulation AB.

All AGX source code, intellectual property, documentation, sample code,
tutorials, scene files and technical white papers, are copyrighted, proprietary
and confidential material of Algoryx Simulation AB. You may not download, read,
store, distribute, publish, copy or otherwise disseminate, use or expose this
material unless having a written signed agreement with Algoryx Simulation AB, or having been
advised so by Algoryx Simulation AB for a time limited evaluation, or having purchased a
valid commercial license from Algoryx Simulation AB.

Algoryx Simulation AB disclaims all responsibilities for loss or damage caused
from using this software, unless otherwise stated in written agreements with
Algoryx Simulation AB.
*/

#pragma once

#include <agx/config/AGX_USE_AGXTERRAIN.h>
#include <agx/OrientedFrictionModels.h>
#include <agxCollide/Contacts.h>
#include <array>



namespace agxTerrain
{
  /**
  Friction model used in the Shovel <-> Aggregate contacts that use the
  cached normal force per-point from the previous time-step in order to
  determine the	friction limits in the current step.
  */
  AGX_DECLARE_POINTER_TYPES( ShovelAggregateFrictionModel );
  class ShovelAggregateFrictionModel : public agx::OrientedBoxFrictionModel
  {
  public:
    ShovelAggregateFrictionModel( agx::Frame* frame, agx::FrictionModel::SolveType solveType = agx::FrictionModel::DIRECT );
    ~ShovelAggregateFrictionModel();

    virtual void initialize( const agx::Physics::GeometryContactPtr geometryContact,
      size_t pointIndex,
      agx::Real dt,
      agx::RangeReal& ret_boundU,
      agx::RangeReal& ret_boundV ) const override;

    void setCachedNormalForce( int pointIndex, agx::Real normalForce );
    void onPre();
    void clear();

  private:
    // Shovel contacts always contains up to 4 points. They all
    // appear in the same order.
    std::array<agx::Real, 4> m_cachedNormalForce;
    mutable size_t m_currentPointIndex;
  };

  agx::Real getAverageContactForce( agxCollide::GeometryContactPtrVector matches, bool print = false );
}