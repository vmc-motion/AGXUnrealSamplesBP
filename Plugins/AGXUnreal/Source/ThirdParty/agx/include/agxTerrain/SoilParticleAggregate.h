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

#include <agxTerrain/PrimaryActiveZone.h>
#include <agxTerrain/SoilSimulationInterface.h>
#include <agx/RigidBody.h>
#include <agx/LockJoint.h>
#include <agx/Plane.h>

namespace agxTerrain
{
  class Shovel;
  class TerrainToolCollection;
  class ActiveZone;
  class TerrainMaterial;

  typedef agx::Physics::GranularBodyPtr SoilParticlePtr;
  typedef agx::Physics::GranularBodyPtrVector SoilParticlePtrVector;
  typedef agx::HashSet<agx::Vec3i> VoxelHashSet;

  AGX_DECLARE_POINTER_TYPES( SoilParticleAggregate );
  AGX_DECLARE_VECTOR_TYPES(SoilParticleAggregate);

  class AGXTERRAIN_EXPORT SoilParticleAggregate : public agx::Referenced
  {
    public:
      /**
      Construct given shovel.
      */
      SoilParticleAggregate( agx::RigidBody* connectedBody = nullptr );

      /**
      \note It's undefined to make changes to this body since all its
            properties are calculated given the soil particles it
            represent.
      \return inner rigid body used as soil particle aggregate
      */
      agx::RigidBody* getInnerBody() const;

      /**
      \note It's undefined to make changes to this body since all its
            properties are calculated given the soil particles it
            represent.
      \return the utmost enabled wedge partition body in the soil particle
              aggregate that is in contact with the terrain.
      */
      const agx::RigidBody* getContactingWedgeBody() const;

      /**
      Return the wedge partition bodies of the wedge part of the soil aggregate.
      \note It's undefined to make changes the bodies since all their
            properties are calculated given the soil particles it
            represent.
      \param onlyEnabled - true if the function should only return enabled
                           wedge bodies, false otherwise. (Default: true)
      \return the wedge partition bodies inside the soil aggregate.
      */
      agx::RigidBodyPtrVector getWedgeBodies(bool onlyEnabled=true) const;

      /**
      Return the wedge lock joints of the wedge part of the soil aggregate.
      \note It's undefined to make changes the locks since all their
            properties are calculated given the soil particles it
            represent.
      \param onlyEnabled - true if the function should only return enabled
                           wedge locks, false otherwise. (Default: true)
      \return the wedge partition lock joints inside the soil aggregate.
      */
      agx::ConstraintPtrVector getWedgeLockJoints(bool onlyEnabled=true) const;

      /**
      \note It's undefined to make changes to this geometry since all its
            properties are calculated given the soil particles it
            represent.
      \return geometry used as soil particle aggregate
      */
      const agxCollide::Geometry* getInnerGeometry() const;

      /**
      \note It's undefined to make changes to this geometry since all its
            properties are calculated given the soil particles it
            represent.
      \return geometry used as soil particle aggregate
      */
      const agxCollide::Geometry* getContactingWedgeGeometry() const;

      /**
      \note It's undefined to make changes to this lock since all its
            properties are calculated during the aggregate update step.
            ALSO NOTE, this lock is only relevant if the body aggregate
            is part of a primary active zone.
      \return lock between the inner body and the wedge shape, IF used by primary active zone.
      */
      const agx::LockJoint* getInnerWedgeLockJoint() const;

      /**
      Assign material used for the contact between the shovel and this
      soil particle aggregate (the soil particles implicitly).
      \param material - new material
      */
      void setMaterial( agx::Material* material );

      /**
      \return the material used between the shovel and this soil particle aggregate
      */
      agx::Material* getMaterial() const;

      /**
      \return the mass of this aggregate, representing both soil particles and fluid mass
      */
      agx::Real getMass() const;

      /**
      \return the soil particles that the inner body of the soil particle aggregate represents
      */
      const SoilParticlePtrVector& getInnerBodyParticles() const;

      /**
      \return the soil particles that the wedge body of the soil particle aggregate represents
      */
      const SoilParticlePtrVector& getWedgeBodyParticles() const;

      /**
      * \return total wedge body mass of all enabled soil wedge partitions.
      */
      agx::Real getTotalWedgeBodyMass() const;

      /**
      \return the mass of all the rigid bodies (both inner and wedge bodies) present in the aggregate.
      */
      agx::Real getTotalAggregateMass() const;

      agx::Vec3 getInnerParticleMomentum() const;
      agx::Vec3 getInnerParticleAngularMomentum() const;

      agx::Vec3 getInnerMomentum() const;
      agx::Vec3 getInnerAngularMomentum() const;

    public:
      /**
      \internal

      On addNotification of Terrain and Shovel.
      */
      void addNotification( agxSDK::Simulation* simulation, Terrain* terrain );

      /**
      \internal

      On removeNotification of Terrain and Shovel.
      */
      void removeNotification( agxSDK::Simulation* simulation, Terrain* terrain );

      /**
      \internal

      On pre-collide step.
      */
      void onPreCollide( TerrainToolCollection* collection );

      /**
      \internal

      On pre step of Terrain and Shovel.
      */
      void onPre( TerrainToolCollection* collection, ActiveZone* activeZone );

      /**
      \internal

      On post step of Terrain and Shovel.
      */
      void onPost( TerrainToolCollection* collection );

      /**
      \internal

      Called when Shovel enable is changed.
      */
      void onEnableChange( bool enable );

      static agx::Plane transformPlaneToWorld(const agx::Frame* localFrame, const agx::Plane& localPlane);

      /**
      * Set a connected inner body to the soil aggregate. This will replace
      * the innerbody of the aggregate with the specified body when connecting
      * the wedge bodies with lock joints.
      */
      void setConnectedInnerBody(agx::RigidBody* connectedInnerBody);

      /**
      * Set a connected inner body to the soil aggregate. This will replace
      * the innerbody of the aggregate with the specified body when connecting
      * the wedge bodies with lock joints.
      */
      agx::RigidBody* getInnerOrConnectedBody() const;

      /**
      * Return true if this soil aggregate contains a connected rigid
      * body, false otherwise.
      */
      bool isConnectedAggregate() const;

    protected:
      /**
      Reference counted object - protected destructor.
      */
      virtual ~SoilParticleAggregate();

      /**
      Internal mutable geometry accessor.
      */
      agxCollide::Geometry* _getInnerGeometry() const;

      /**
      Internal mutable geometry accessor.
      */
      agxCollide::Geometry* _getWedgeGeometry() const;

    protected:
      /**
       Pure data class used to hold information about a
       single partition in the wedge aggregate.
      */
      struct WedgePartitionData
      {
        agx::RigidBodyRef wedgeBody;
        agx::LockJointRef lock;

        agx::Vec3 lockFramePosition;
        agx::Vec3 lockPlaneNormal;

        SoilParticlePtrVector splitWedgeParticles;
        VoxelHashSet voxels;

        agx::Vec3 wedgeBodyMomentum;
        agx::Vec3 wedgeBodyAngularMomentum;

        using RealArray = std::array<agx::Real, 6>;
        RealArray violations;
      };

      /**
      Class that handles and creates a lumped element approximation of
      the soil wedge aggregate. This is done to approximate internal
      shearing effects/limits of the soil aggregate during vertical movements
      instead of treating the whole soil wedge aggregate as a rigid object.

      The wedge aggregate is split inteo different bodies according to
      pre-defined planes. Each body is connected with a lock joint that
      holds frictional limits in the plane of the lock z-direction to
      approximate internal soil shearing during excavation.
      \note Only a maximum of 2-soil wedges are supported at the moment.
      */
      class SoilWedgeContainer
      {
      public:
        /**
        Constructor
        */
        SoilWedgeContainer( size_t numWedges );

        /**
        Initializes the lumped element structure of the wedge aggregate.
        \param innerBody - the inner body of the shovel that will connect to the
                           lumped elment structure of the wedge body.
        */
        void init(agx::RigidBody* innerBody);

        /**
        \internal

        On addNotification of Terrain and Shovel.
        */
        void addNotification( agxSDK::Simulation* simulation, Terrain* terrain );

        /**
        \internal

        On removeNotification of Terrain and Shovel.
        */
        void removeNotification( agxSDK::Simulation* simulation, Terrain* terrain );

        /**
        \internal

        Called when Shovel enable is changed.
        */
        void onEnableChange(bool enable);

        /**
        Set the material of each of the geometries in the lumped element structure.
        */
        void setMaterial( agx::Material* material );

        /**
        Calculates all the normals used to defined the internal partition
        splitting of the wedge volume.
        */
        void calculatePartitionPlaneData( TerrainToolCollection* collection );

        /**
        Split the wedge particles into different partition elements of the wedge
        aggregate. Each partition holds a plane that governs the splitting delimiter
        for the particles.
        */
        void splitWedgeParticles( const TerrainToolCollection* collection,
                                  const SoilParticlePtrVector& soilParticles );

        /**
        Split the wedge voxels into different partition elements of the wedge
        aggregate. Each partition holds a plane that governs the splitting delimiter
        for the particles.
        */
        void splitWedgeVoxels( const TerrainToolCollection* collection,
                               VoxelHashSet& voxels );

        /**
        This function returns the outermost enabled body in the lumped-element
        approximation of the wedge aggregate. This will be used when creating
        terrain-aggregate contacts if applicable.
        */
        const agx::RigidBody* getContactingWedgeBody() const;

        /**
        Update the a specific partition body velocity and angular velocity
        from updated inertia and cached momentum.
        */
        void updatePartitionBodyMomentum(
          agx::RigidBody* partitionBody,
          agx::Real totalMass,
          const agx::Matrix3x3& invWorldInertiaTensor );

        /**
        Updates the cached momentum in each partition for the wedge bodies.
        \note - Should be called after post step when new velocities have been computed.
        */
        void updateCachedWedgeMomentum();

        /**
        Computes lock violations of the lock joints in the lumped structure.
        */
        void computeLockViolations();

        /**
        Update the lumped element lock joints according to partition data.
        */
        void synchronizeLockJoints( TerrainToolCollection* collection );

        void clearParticles( size_t reserve );

        void clearVoxelsAndParticles();

        agx::Real getTotalWedgeMass() const;

        /**
        Set single partition body mode when no splitting is occuring.
        \note Single wedge body connected with a single lock to the inner body.
        */
        void setSinglePartitionMode();

        /**
        Set deformer mode.
        \note Single body active with no lock joints
        */
        void setDeformerWedgeMode();

        /**
        \returns true if the specified body is a wedge partition body, false otherwise.
        */
        bool isWedgePartitionBody( const agx::RigidBody* body ) const;

        /**
        \returns the partition data for a specified partition body.
        */
        const WedgePartitionData& getPartitionData( const agx::RigidBody* partitionBody ) const;

        const agx::Vec3& getCuttingVectorFlat() const;

        const agx::Vec3& getCutTopVector() const;

        const agx::Vec3& getForwardDiggingPlane() const;

        inline const WedgePartitionData& operator[] ( size_t index ) const {
          return m_wedgePartitions[ index ];
        }

        inline WedgePartitionData& operator[] ( size_t index ) {
          return m_wedgePartitions[ index ];
        }

        size_t size() const noexcept
        {
          return m_wedgePartitions.size();
        }

      protected:
        agx::Vec3 m_cuttingVectorFlat;
        agx::Vec3 m_topCutVector;
        agx::Vec3 m_forwardDiggingPlane;

        std::vector<WedgePartitionData> m_wedgePartitions;
      };

    private:
      VoxelHashSet calculateWedgeVoxels(TerrainToolCollection* collection, ActiveZone* activeZone);

      VoxelHashSet filterWedgeVoxelsSplittingPlane(TerrainToolCollection* collection,
                                                               ActiveZone* activeZone,
                                                               const VoxelHashSet& wedgeVoxels);

      void updateDynamicProperties(const SoilParticlePtrVector& soilParticles,
                                   agx::RigidBody* rb,
                                   VoxelHashSet& voxels,
                                   TerrainToolCollection* collection);

      void computeParticlesInActiveZone(TerrainToolCollection* collection,
                                         ActiveZone* activeZone);

      void sortParticlesBySeparatingPlane( Terrain* terrain,
                                           ActiveZone* activeZone,
                                           SoilParticlePtrVector& innerParticles,
                                           SoilParticlePtrVector& wedgeParticles );

      void computeParticlesInInnerShape( PrimaryActiveZone* activeZone,
                                         SoilParticlePtrVector& innerParticles,
                                         agx::Real radiusMultiplier);

      SoilParticlePtrVector computeParticlesInWedgeShape( ActiveZone* activeZone,
                                                          SoilParticlePtrVector& wedgeParticleCandidates );

      void computeParticlesAboveShovel( TerrainToolCollection* collection,
                                        const agx::Vec3& up,
                                        SoilParticlePtrVector& wedgeParticles,
                                        agx::Real radiusMultiplier );

      bool isPointAbovePlane( const agx::Plane& plane,
                              const agx::Vec3& point,
                              agx::Real threshold = 0.0 ) const;

      bool isSphereIntersectingPlane( const agx::Plane& plane,
                                      const agx::Vec3& point,
                                      agx::Real radius ) const;

      bool shouldSplitWedgeAggregate( const TerrainToolCollection* collection );

      void clearParticles( TerrainToolCollection* collection );

      void debugRenderParticles( TerrainToolCollection* collection, bool shouldSplit);

    private:
      agx::RigidBodyRef m_innerBody;

      agx::Vec3         m_innerBodyMomentum;
      agx::Vec3         m_innerBodyAngularMomentum;

      agx::Vec3         m_innerParticleMomentum;
      agx::Vec3         m_innerParticleAngularMomentum;

      agx::Vec3         m_innerCachedMomentum;
      agx::Vec3         m_innerCachedAngularMomentum;

      SoilParticlePtrVector m_innerParticles;
      SoilParticlePtrVector m_wedgeParticles;

      SoilWedgeContainer     m_soilWedgeContainer;
      agx::RigidBodyObserver m_connectedInnerBody;
  };



  inline bool SoilParticleAggregate::isPointAbovePlane( const agx::Plane& plane,
                                                        const agx::Vec3& point,
                                                        agx::Real threshold ) const
  {
    return plane.signedDistanceToPoint(point) > threshold;
  }



  inline bool SoilParticleAggregate::isSphereIntersectingPlane( const agx::Plane& plane,
                                                                const agx::Vec3& point,
                                                                agx::Real radius ) const
  {
    return std::abs(plane.signedDistanceToPoint(point)) < radius;
  }




}
