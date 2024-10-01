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

#include <agx/agx.h>
#include <agx/ParticleSystem.h>
#include <agxSDK/Simulation.h>
#include <agx/Journal.h>
#include <agxSDK/StepEventListener.h>

namespace agx
{
  class RigidBody;
}


namespace agxSDK
{
  DOXYGEN_START_INTERNAL_BLOCK()


  AGX_DECLARE_POINTER_TYPES(SimulationAddEventListener);
  /**
  This class will count the number of active NSS rigid bodies and granular particles.
  It counts bodies via a listener on the RigidBody add/remove events in the simulation object
  and counts particles via a listener on the entity storage of the granular particle system.

  This class also counts all bodies/particles loaded via a journal but misses particles
  that are both created and removed before the journal is saved.
  */
  class AGXPHYSICS_EXPORT SimulationAddEventListener : public agxSDK::StepEventListener
  {
  private:
    /**
    This entity storage event listener will register to all particle create/destroy callbacks in the ParticleSystem
    and notify the main class that a particle is created/destroyed.
    */
    class ParticleStorageListener : public agxData::EntityStorage::EventListener
    {

    public:
      ParticleStorageListener(SimulationAddEventListener& listener);

      virtual void destroyCallback(agxData::EntityStorage* storage) override;
      virtual void createInstanceCallback(agxData::EntityStorage* storage, agxData::EntityPtr instance) override;
      virtual void createInstancesCallback(agxData::EntityStorage* storage, agxData::EntityRange range) override;
      virtual void destroyInstanceCallback(agxData::EntityStorage* storage, agxData::EntityPtr instance) override;
      virtual void destroyInstancesCallback(agxData::EntityStorage* storage, agxData::Array<agxData::EntityPtr> instances) override;
      virtual void createInstancesCallback(agxData::EntityStorage* storage, agxData::Array<agxData::EntityPtr> instances) override;
      virtual void destroyInstancesCallback(agxData::EntityStorage* storage, agxData::EntityRange instance) override;
      virtual void permuteCallback(agxData::EntityStorage* storage, agxData::Array< agx::Index > permutation) override;

      //////////////////////////////////////////////////////////////////////////
      // Variables
      //////////////////////////////////////////////////////////////////////////
    protected:
      SimulationAddEventListener& m_listener;
    };

  public:

    /**
    Constructor
    */
    SimulationAddEventListener();

    /**
    Return the total number of emitted NSS rigid bodies
    */
    size_t getNumberOfCreatedNss() const;

    /**
    Return the total number of emitted granular particles. Note that emitters may continue to emit particles
    after the license limit is reached but they are then removed from the simulation.
    */
    size_t getNumberOfCreatedParticles() const;

    /**
    Return the total number of active non spherical shapes.
    By active we here mean rigid bodies that are valid and currenlty a part of the simulation.
    */
    size_t getNumberOfActiveNss() const;

    /**
    Return the total number of active granular particles.
    By active we here mean particles that are valid and currenlty a part of the simulation.
    */
    size_t getNumberOfActiveParticles() const;

    /**
    Return true if the emitters in the simulation are allowed to create granular particles.
    */
    bool getEnableParticleCreation();

    /**
    Return true if the emitters in the simulation are allowed to create non spherical shapes.
    */
    bool getEnableNss();

    /**
    Return true if we have reached the license limitation of number of active non spherical shapes.
    */
    bool hasReachedActiveNssLimit() const;

    /**
    Return true if we have reached the license limitation of max number of emitted non spherical shapes.
    */
    bool hasReachedEmittedNssLimit() const;

    /**
    Return true if we have reached the license limitation of number of active granular particles.
    */
    bool hasReachedActiveParticlesLimit() const;

    /**
    Return true if we have reached the license limitation of max number of emitted granular particles.
    */
    bool hasReachedEmittedParticlesLimit() const;

    /**
    True if we have reached any of the four license limits,
    hasReachedActiveNssLimit, hasReachedEmittedNssLimit, hasReachedActiveParticlesLimit or hasReachedEmittedParticlesLimit
    */
    bool isGranularBlocked();

    /**
    Return the license limitation of max number of active particles.
    */
    size_t getMaxNumberOfActiveParticles() const;

    /**
    Return the license limitation of max number of emitted particles.
    */
    size_t getMaxNumberOfEmittedParticles() const;

    /**
    Return the license limitation of max number of active NSS.
    */
    size_t getMaxNumberOfActiveNss() const;

    /**
    Return the license limitation of max number of emitted NSS.
    */
    size_t getMaxNumberOfEmittedNss() const;

    /**
    Set the limitation of max number of active particles. This number can't be set higher than the limitation in the license.
    */
    void setMaxNumberOfActiveParticles(size_t max);

    /**
    Set the limitation of max number of emitted particles. This number can't be set higher than the limitation in the license.
    */
    void setMaxNumberOfEmittedParticles(size_t max);

    /**
    Set the limitation of max number of active NSS. This number can't be set higher than the limitation in the license.
    */
    void setMaxNumberOfActiveNss(size_t max);

    /**
    Set the limitation of max number of emitted NSS. This number can't be set higher than the limitation in the license.
    */
    void setMaxNumberOfEmittedNss(size_t max);

    virtual void addNotification() override;

    virtual void removeNotification() override;

    /**
    If the simulation contains a agxSDK::SimulationAddEventListener then that listener is returned,
    else a nullptr is returned.
    */
    static agxSDK::SimulationAddEventListener* getListener(agxSDK::Simulation* simulation);

    /**
    Return false if we have reached a granular license limit. Else return true
    If the limit is reached this will also print a couple of warning messages.
    */
    static bool validateAndWarnGranularLicenseLimits(agxSDK::Simulation* simulation);

    /**
    Call this method when a journal is attached to the simulation
    */
    void journalAttachedCallback(agxSDK::Simulation* simulation, agx::Journal* journal);

    AGXSTREAM_DECLARE_SERIALIZABLE(agxSDK::SimulationAddEventListener);

  protected:
    /**
    Destructor
    */
    virtual ~SimulationAddEventListener();

    void addRigidBodyCallback(agxSDK::Simulation* sim, agx::RigidBody* body);

    void removeRigidBodyCallback(agxSDK::Simulation* sim, agx::RigidBody* body);

    void addGranularParticle(agxData::EntityPtr entity);

    void removeGranularParticle();

    void addGranularSystemCallback(agx::Component* parent, agx::Object* child);

    void removeGranularSystemCallback(agx::Component* parent, agx::Object* child);

    void journalJumpFrameCallback(agx::Journal* journal);

    void granularSystemAdded(agx::ParticleSystem* particleSystem);

    void granularSystemRemoved(agx::ParticleSystem* particleSystem);

  private:
    bool updateLicenseLimit();

    bool hasLicenseLimitations();

    void setLicenseLimit(size_t max, const std::string& limitName, size_t& limitValue);

    void resetCounters();

  private:

    size_t m_numberEmittedBodies;
    size_t m_numberActiveBodies;
    size_t m_numberEmittedGranularParticles;
    size_t m_numberActiveGranularParticles;

    size_t m_maxEmittedPartcles;
    size_t m_maxActivePartcles;
    size_t m_maxEmittedNss;
    size_t m_maxActiveNss;

    // Callbacks for rigid body add/remove events
    agxSDK::Simulation::RigidBodyEvent::CallbackType m_addBodyCallback;
    agxSDK::Simulation::RigidBodyEvent::CallbackType m_removeRigidBodyCallback;

    // Callbacks on the simulation object to listen for add/remove of particle systems
    agx::Component::ObjectEvent::CallbackType m_addGranularSystemCallback;
    agx::Component::ObjectEvent::CallbackType m_removeGranularSystemCallback;

    agx::Journal::FrameJumpEvent::CallbackType m_frameJumpCallback;

    agxSDK::Simulation::JournalAttachEvent::CallbackType m_journalAttachedCallback;

    ParticleStorageListener m_particleStorageListener;

    agx::ParticleSystemRef m_particleSystem;

    bool m_doResetCounters;
  };

  AGX_FORCE_INLINE bool SimulationAddEventListener::isGranularBlocked()
  {
    return !getEnableNss() || !getEnableParticleCreation();
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getNumberOfCreatedNss() const
  {
    return m_numberEmittedBodies;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getNumberOfCreatedParticles() const
  {
    return m_numberEmittedGranularParticles;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getNumberOfActiveNss() const
  {
    return m_numberActiveBodies;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getNumberOfActiveParticles() const
  {
    return m_numberActiveGranularParticles;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getMaxNumberOfActiveParticles() const
  {
    return m_maxActivePartcles;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getMaxNumberOfEmittedParticles() const
  {
    return m_maxEmittedPartcles;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getMaxNumberOfActiveNss() const
  {
    return m_maxActiveNss;
  }

  AGX_FORCE_INLINE size_t SimulationAddEventListener::getMaxNumberOfEmittedNss() const
  {
    return m_maxEmittedNss;
  }

  DOXYGEN_END_INTERNAL_BLOCK()
}
