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

#ifndef AGXMODEL_WINDANDWATERALGORITHMS_H
#define AGXMODEL_WINDANDWATERALGORITHMS_H

#include <agxModel/WindAndWaterController.h>

namespace agxModel {
  template<typename T>
  void triangleIterator( T* obj, WindAndWaterController::ObjectData& objectData, const GlobalTriangleData::Container& trianglePool )
  {
    if ( objectData.getRigidBody() == nullptr )
      return;

    auto triangleUpdate = [ & ] ( const agx::RangeJob& job )
    {
      agx::Vec3 force;
      agx::Vec3 torque;
      for ( size_t k = job.range().begin(); k < job.range().end(); ++k ) {
        size_t i = k + objectData.getStartIndexT();
        GlobalTriangleData& tData = trianglePool[ i ];
        if ( tData.isClipped() )
          for ( agx::UInt j = 0; j < tData.clipped.size(); ++j )
            obj->calculate( tData.clipped[ j ], objectData, force, torque );
        else
          obj->calculate( tData, objectData, force, torque );
      }
      objectData.lock();
      objectData.force() += force;
      objectData.torque() += torque;
      switch (obj->getForceType())
      {
        case WindAndWaterController::ForceType::BUOYANCY:
        {
          objectData.forceComponent(WindAndWaterController::ForceType::BUOYANCY) += force;
          objectData.torqueComponent(WindAndWaterController::ForceType::BUOYANCY) += torque;
          break;
        }
        case WindAndWaterController::ForceType::PRESSURE_DRAG:
        {
          objectData.forceComponent(WindAndWaterController::ForceType::PRESSURE_DRAG) += force;
          objectData.torqueComponent(WindAndWaterController::ForceType::PRESSURE_DRAG) += torque;
          break;
        }
        case WindAndWaterController::ForceType::VISCOUS_DRAG:
        {
          objectData.forceComponent(WindAndWaterController::ForceType::VISCOUS_DRAG) += force;
          objectData.torqueComponent(WindAndWaterController::ForceType::VISCOUS_DRAG) += torque;
          break;
        }
        case WindAndWaterController::ForceType::LIFT:
        {
          objectData.forceComponent(WindAndWaterController::ForceType::LIFT) += force;
          objectData.torqueComponent(WindAndWaterController::ForceType::LIFT) += torque;
          break;
        }
        case WindAndWaterController::ForceType::OTHER:
        {
          objectData.forceComponent(WindAndWaterController::ForceType::OTHER) += force;
          objectData.torqueComponent(WindAndWaterController::ForceType::OTHER) += torque;
          break;
        }
        default:
          LOGGER_WARNING() << "Unknown force type in WindAndWaterAlgorithms: " << obj->getForceType() << LOGGER_ENDL();
      }
      objectData.unLock();
    };

    auto sizeCallback = [ & ] ()
    {
      return objectData.getNumTriangles();
    };

    if ( objectData.getNumTriangles() > 500 ) {
      agx::LambdaKernelRef computeKernel = new agx::LambdaKernel( "ComputeKernel", triangleUpdate, sizeCallback );
      computeKernel->execute();
    }
    else {
      agx::RangeJob range;
      range.init( agx::IndexRange( 0, objectData.getNumTriangles() ) );
      triangleUpdate( range );
    }
  }

  class AGXMODEL_EXPORT BuoyancyAlgorithm : public agxModel::WindAndWaterController::Algorithm
  {
  public:
    BuoyancyAlgorithm();

    virtual void updateInteraction( agxModel::WindAndWaterController::ObjectData& objectData,
                                    const GlobalTriangleData::Container& container ) override
    {
      triangleIterator( this, objectData, container );
    }

    void calculate( const agxModel::TriangleData& tData,
                    agxModel::WindAndWaterController::ObjectData& objectData,
                    agx::Vec3& force,
                    agx::Vec3& torque ) const;

    void pre( agxModel::WindAndWaterController* ) override;

    virtual WindAndWaterController::ForceType getForceType() const override { return WindAndWaterController::ForceType::BUOYANCY; }

  protected:
    virtual ~BuoyancyAlgorithm();

    DOXYGEN_START_INTERNAL_BLOCK()

    // Used for scaling buoyancy forces for primitive shapes.
    class VolumeScalingDataContainer {
    public:
      agx::Real getVolumeScaling( const agxCollide::Shape* shape,
                                  agxModel::WindAndWaterParameters::ShapeTessellation tessellation );

      void garbageCollect();

    private:
      struct Data
      {
        Data( agx::Real volumeScaling,
              agx::UInt32 modifiedCount,
              WindAndWaterParameters::ShapeTessellation tessellation,
              const agxCollide::Shape* shape )
          : volumeScaling( volumeScaling )
          , modifiedCount( modifiedCount )
          , tessellation( tessellation )
          , shape( shape )
        {
        }

        agx::Real                                 volumeScaling;
        agx::UInt32                               modifiedCount;
        WindAndWaterParameters::ShapeTessellation tessellation;
        agxCollide::ShapeConstObserver            shape;
      };

      using Container = agx::HashVector< const agxCollide::Shape*, Data >;
      Container m_data;
      std::mutex m_mutex;
    };

    DOXYGEN_END_INTERNAL_BLOCK()

    mutable VolumeScalingDataContainer m_volumeScaling;
  };

  class AGXMODEL_EXPORT CenterOfBuoyancyAlgorithm : public agxModel::WindAndWaterController::Algorithm
  {
  public:
    CenterOfBuoyancyAlgorithm();

    virtual void updateInteraction(agxModel::WindAndWaterController::ObjectData& objectData,
                                   const GlobalTriangleData::Container& container) override;

    void calculate(const agxModel::TriangleData& tData,
                   agxModel::WindAndWaterController::ObjectData& objectData,
                   agx::Vec3& force,
                   agx::Vec3& torque);

  protected:
    virtual ~CenterOfBuoyancyAlgorithm();

    agx::Real calculateTetrahedronVolume(const agx::Vec3& edge0, const agx::Vec3& edge1, const agx::Vec3& edge2) const;

    void calculatePyramidCenterOfBuoyancy(agx::Vec3* basePoints, agx::Vec3* shiftedPoints, agx::Real& vol, agx::Vec3& pos);

    void calculateLocalCenterOfBuoyancy(const TriangleData& tData, const agx::Vec3& up, agx::Real& vol, agx::Vec3& pos);

    agx::Vec3 m_center;
    agx::Real m_waterVolume;
    bool m_enabled;
  };

  class AGXMODEL_EXPORT PressureDragForceAlgorithm : public agxModel::WindAndWaterController::Algorithm
  {
  public:
    PressureDragForceAlgorithm();

    virtual void updateInteraction( agxModel::WindAndWaterController::ObjectData& objectData,
                                    const GlobalTriangleData::Container& container ) override
    {
      triangleIterator( this, objectData, container );
    }

    void calculate( agxModel::TriangleData& tData,
                    agxModel::WindAndWaterController::ObjectData& objectData,
                    agx::Vec3& force,
                    agx::Vec3& torque ) const;

    virtual WindAndWaterController::ForceType getForceType() const override { return WindAndWaterController::ForceType::PRESSURE_DRAG; }

  protected:
    virtual ~PressureDragForceAlgorithm();
  };

  class AGXMODEL_EXPORT LiftForceAlgorithm : public agxModel::WindAndWaterController::Algorithm
  {
  public:
    LiftForceAlgorithm();

    virtual void updateInteraction( agxModel::WindAndWaterController::ObjectData& objectData,
                                    const GlobalTriangleData::Container& container ) override
    {
      triangleIterator( this, objectData, container );
    }

    void calculate( agxModel::TriangleData& tData,
                    agxModel::WindAndWaterController::ObjectData& objectData,
                    agx::Vec3& force,
                    agx::Vec3& torque ) const;

    virtual WindAndWaterController::ForceType getForceType() const override { return WindAndWaterController::ForceType::LIFT; }

  protected:
    virtual ~LiftForceAlgorithm();
  };

  class AGXMODEL_EXPORT ViscousDragForceAlgorithm : public agxModel::WindAndWaterController::Algorithm
  {
  public:
    ViscousDragForceAlgorithm();

    virtual void updateInteraction( agxModel::WindAndWaterController::ObjectData& objectData,
                                    const GlobalTriangleData::Container& container ) override
    {
      triangleIterator( this, objectData, container );
    }

    void calculate( agxModel::TriangleData& tData,
                    agxModel::WindAndWaterController::ObjectData& objectData,
                    agx::Vec3& force,
                    agx::Vec3& torque ) const;

    virtual WindAndWaterController::ForceType getForceType() const override { return WindAndWaterController::ForceType::VISCOUS_DRAG; }
  protected:
    virtual ~ViscousDragForceAlgorithm();
  };

  class AGXMODEL_EXPORT AddedMassAlgorithm : public agxModel::WindAndWaterController::Algorithm
  {
  public:
    AddedMassAlgorithm();

    virtual void updateInteraction( agxModel::WindAndWaterController::ObjectData& objectData,
                                    const GlobalTriangleData::Container& container ) override;

  protected:
    virtual ~AddedMassAlgorithm();
  };
}

#endif // AGXMODEL_WINDANDWATERALGORITHMS_H
