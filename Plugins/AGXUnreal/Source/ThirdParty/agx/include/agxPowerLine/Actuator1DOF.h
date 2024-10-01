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

#include <agx/Constraint.h>
#include <agx/DistanceJoint.h>
#include <agx/Hinge.h>
#include <agx/Prismatic.h>
#include <agx/Referenced.h>
#include <agxPowerLine/Actuator.h>
#include <agxPowerLine/TranslationalDimension.h>
#include <agxPowerLine/TranslationalUnit.h>
#include <agxPowerLine/RotationalUnit.h>
#include <agxPowerLine/detail/RotationalActuatorConnector.h>
#include <agxPowerLine/detail/TranslationalActuatorConnector.h>

namespace agx
{
  class RigidBody;
}

namespace agxPowerLine
{

  AGX_DECLARE_POINTER_TYPES(ConstraintGeometry);

  /**
  Description of how an Actuator is allowed to apply forces and torques on the
  body or bodies attached to the actuated constraint.
  */
  class AGXMODEL_EXPORT ConstraintGeometry : public agx::Referenced, public agxStream::Serializable
  {
    public:
      virtual bool isValid() const = 0;

      /**
      Transform constraint attachment frames and updates data.

      Constraints do this by themselves during stepping, but at setup time and
      when very early in the step it may be necessary to trigger an early
      transform update.
      */
      virtual void transform() = 0;

      /**
      \param index The index of the body to get the attachment position for. Either 0 or 1.
      \return The constraint attachment point in world coordinates for one of the attachments.
      */
      virtual agx::Vec3 getWorldPosition(agx::UInt index) const = 0;

      /**
      \return The constraint attachment point in world coordinates for first attachment.
      */
      agx::Vec3 getWorldPosition1() const
      {
        // First body is on index 0.
        return getWorldPosition(0);
      }

      /**
      \return The constraint attachment point in world coordinates for second attachment.
      */
      agx::Vec3 getWorldPosition2() const
      {
        // Second body is on index 1.
        return getWorldPosition(1);
      }

      /**
      * \param index The index of the body to get the attachment direction for. Either 0 or 1.
      * \return The world direction the angle is defined along or about for one of the attachments.
      */
      virtual agx::Vec3 getWorldDirection(agx::UInt index) const = 0;

      /**
      \return The world direction the angle is defined along or about for the first attachment.
      */
      agx::Vec3 getWorldDirection1() const
      {
        // First body is on index 0.
        return getWorldDirection(0);
      }

      /**
      \return The world direction the angle is defined along or about for second attachment.
      */
      agx::Vec3 getWorldDirection2() const
      {
        // Second body is on index 1.
        return getWorldDirection(1);
      }

      /**
      \param index The index of the body to get the CoM-to-attachment point vector for. Either 0 or 1.
      \return The vector from CoM to the attachment point in world coordinates for one of the attachments.
      */
      virtual agx::Vec3 getCmToPosition(agx::UInt index) const = 0;

      /**
      \return The vector from CoM to the attachment point in world coordinates for first attachment.
      */
      agx::Vec3 getCmToPosition1() const
      {
        // First body is at index 1.
        return getCmToPosition(0);
      }

      /**
      \return The vector from CoM to the attachment point in world coordinates for second attachment.
      */
      agx::Vec3 getCmToPosition2() const
      {
        return getCmToPosition(1);
      }

      /**
      \return the separation vector between the two attachments (att1->pos - att2->pos).
      */
      virtual agx::Vec3 getSeparation() const = 0;

      /**
      This method returns the current angle for the constraint. For a prismatic,
      this is the position of the constraint. For a hinge, this is the angle of
      rotation including winding.
      \return the current joint angle including winding (if available).
      */
      virtual agx::Real getAngle() const = 0;

      /**
      Get the range of allowed angles. Corresponds to the Range1D in a
      Constraint1D. Used by e.g. the PistonActuator to compute chamber volumes.
      \return Range of allowed angles.
      */
      virtual agx::RangeReal getAngleRange() const
      {
        return {getMinAngle(), getMaxAngle()};
      }

      /**
      \return The minimum angle that the constraint allows.
      */
      virtual agx::Real getMinAngle() const = 0;

      /**
      \return The minimum angle that the constraint allows.
      */
      virtual agx::Real getMaxAngle() const = 0;

      /**
      Replace the constraint being queried for geometry information. If the new
      constraint is not compatible with the current ConstraintGeometry subclass
      then it is put in an invalid state and false is returned

      \param constraint The constraint to use for future queries
      \return True if the constraint was set. False if wrong constraint type.
      */
      virtual bool setConstraint(agx::Constraint* constraint) = 0;

      virtual const agx::Constraint* getConstraint() const = 0;
      virtual agx::Constraint* getConstraint() = 0;

      AGXSTREAM_DECLARE_ABSTRACT_SERIALIZABLE(agxPoerLine::ConstraintGeometry)
  };



  /**
  ConstraintGeometry for any type of constraint that has a
  BasicControllerConstraint along the axis that the Actutar should operate.
  Examples of BasicControllerConstraints are Motor1D, Lock1D, and Range1D.
  */
  class AGXMODEL_EXPORT ControllerConstraintGeometry : public ConstraintGeometry
  {
    public:
      ControllerConstraintGeometry();

      /**
      Create a ConstraintGeometry aligned with the direction of the given
      BasicControllerConstraint. The controller should be an elementary
      constraint of the given constraint.
      */
      ControllerConstraintGeometry(agx::Constraint* constraint, agx::BasicControllerConstraint* controller);

      /**
      \return True if both the constraint and the controller is non-nullptr, false otherwise.
      */
      virtual bool isValid() const override;

      virtual void transform() override;

      virtual agx::Vec3 getWorldPosition(agx::UInt index) const override;
      virtual agx::Vec3 getWorldDirection(agx::UInt index) const override;
      virtual agx::Vec3 getCmToPosition(agx::UInt index) const override;

      virtual agx::Vec3 getSeparation() const override;
      virtual agx::Real getAngle() const override;
      virtual agx::RangeReal getAngleRange() const override;
      virtual agx::Real getMinAngle() const override;
      virtual agx::Real getMaxAngle() const override;

      virtual bool setConstraint(agx::Constraint* constraint) override;
      bool setConstraint(agx::Constraint* constraint, agx::BasicControllerConstraint* controller);

      virtual const agx::Constraint* getConstraint() const override;
      virtual agx::Constraint* getConstraint() override;

      agx::BasicControllerConstraint* getController();
      const agx::BasicControllerConstraint* getController() const;

      AGXSTREAM_DECLARE_SERIALIZABLE(agxPowerLine::ControllerConstraintGeometry)

      const agx::ConstraintAngleBasedData& getAngleData() const;

    private:
      agx::Constraint* m_constraint;
      agx::BasicControllerConstraint* m_controller;
  };



  /**
  ConstraintGeometry for 1-DOF constraints. Orients the Actuator to act along
  or around the free degree of freedom of the constraint.
  */
  class AGXMODEL_EXPORT Constraint1DOFGeometry : public ControllerConstraintGeometry
  {
    public:
      Constraint1DOFGeometry();
      Constraint1DOFGeometry(agx::Constraint1DOF* constraint);

      virtual bool isValid() const override;

      /**
      Set a new constraint, which must be of type Constraint1DOF. Becomes
      invalid if passed incorrect constraint type.

      \param constraint The new constraint.
      \return True if a new Constraint1DOF was set, false otherwise.
      */
      virtual bool setConstraint(agx::Constraint* constraint) override;

      AGXSTREAM_DECLARE_SERIALIZABLE(agxPowerLine::Constraint1DOFGeometry)

    private:
      // Safe to keep a raw pointer because Actuator1DOF will clear this
      // pointer from its objectDeleted observer_ptr callback.
      agx::Constraint1DOF* m_constraint;
  };



  AGX_DECLARE_POINTER_TYPES(Actuator1DOF);


  /**
  An Actuator that operates on Constraint1DOF constraints. Subclasses,
  TranslationalActuator and RotationalActuator, provide specializations for
  rotational and translational constraints.
  */
  class AGXMODEL_EXPORT Actuator1DOF : public agxPowerLine::Actuator
  {
    public:

      Actuator1DOF(agx::Constraint1DOF* constraint1DOF);
      Actuator1DOF(agx::Constraint* constraint1DOF, ConstraintGeometry* constraintGeometry);

      // Begin agxPowerLine::Actuator interface.
      virtual agx::Real calculateRelativeValue() const override;
      // End PowerLine::Actuator interface.

      // Begin PowerLine::Unit interface.
      virtual agxPowerLine::DimensionAndSide getConnectableDimension(
          agxPowerLine::PhysicalDimension::Type type,
          agxPowerLine::Side side) override;
      // End PowerLine::Unit interface.


      // Begin PowerLine::SubGraph interface.
      virtual bool preUpdate(agx::Real timeStamp) override;
      // End PowerLine::SubGraph interface.

      ConstraintGeometry* getConstraintGeometry();
      const ConstraintGeometry* getConstraintGeometry() const;

      /**
      Synchronize the power-line dimension state directions with the
      constraint frame directions using the ConstraintGeometry.
      */
      void synchronizeDirections();

      /**
      Stores internal data into stream.
      */
      virtual bool store(agxStream::StorageStream& str) const override;

      using agxPowerLine::Actuator::store;

      /**
      Restores internal data from stream.
      */
      virtual bool restore(agxStream::StorageStream& str) override;

      using agxPowerLine::Actuator::restore;

      AGXSTREAM_DECLARE_ABSTRACT_SERIALIZABLE(agxPowerLine::Actuator1DOF);
      void store(agxStream::OutputArchive& out) const override;
      void restore(agxStream::InputArchive& in) override;

    protected:
      Actuator1DOF();

      virtual ~Actuator1DOF();

      void synchronizeUnits(agx::Constraint* constraint);
      void replaceActuatorUnit(Unit* oldUnit, Unit* newUnit);

      void detachActuatorUnitsFromEachOther();

      virtual agx::Vec3 getDir1() const override;
      virtual agx::Vec3 getDir2() const override;
      virtual agx::Vec3 getCmToAnchorPos1() const override;
      virtual agx::Vec3 getCmToAnchorPos2() const override;
      virtual agx::Vec3 getSeparation() const override;


    protected:
      /**
      Observer that not only sets the Constraint pointer to nullptr on object
      deletion, but also informs the owning Actuator so the body units can be updated.
      */
      class ConstraintObserver : public agx::observer_ptr<agx::Constraint>
      {
        public:
          ConstraintObserver(agx::Constraint* actuatedConstraint, Actuator1DOF* owningActuator);
          virtual void objectDeleted(void*) override;

          using agx::observer_ptr<agx::Constraint>::operator=;
        private:
          Actuator1DOF* m_owningActuator;
      };

      friend class ConstraintObserver;

    protected:
      ConstraintObserver m_constraintObserver;
      ConstraintGeometryRef m_constraintGeometry;

      /**

      */
      agx::ref_ptr<agxPowerLine::ActuatorBodyUnit> m_dummyActuatorBodyUnits[2];
  };



  class AGXMODEL_EXPORT RotationalActuator : public agxPowerLine::Actuator1DOF
  {
    public:
      // \todo Not sure what the best type to take here is. Constraint1DOF is
      // too lenient but Hinge may be too restrictive. How do I spell
      // "Constraints with a rotational part."? Does that even make sense?
      // What other types of Constraints would match, more than Hinge and
      // Cylindrical? Don't forget SlackCylindrical. And SplineJoint while we're
      // at it.
      RotationalActuator(agx::Constraint1DOF* constraint);


      /**
       * Constructor for constraints that have a more complicated geometry than
       * a simple Hinge. The Steering constraints, for example.
       *
       * \param constraint The Constraint the should be powered by the Actuator.
       * \param constraintGeometry A description of how the Actuator is allowed
       * to apply forces and torques on the constrained bodies.
       */
      RotationalActuator(agx::Constraint* constraint, agxPowerLine::ConstraintGeometry* constraintGeometry);

      /**
      \returns true if high speed mode is active
      */
      agx::Bool getUseHighSpeedMode( ) const;

      /**
      Set the usage of high speed mode.

      RotationalActuator has a feature enabling for high hinge velocities.
      When enabled the relative angular velocity orthogonal to the hinge axis
      will be the same for the two rigid bodies being hinged.

      This results in the possibility to have hinges with arbitrary high velocities,
      which is needed when using hinges as high speed engines/motors.
      */
      void setUseHighSpeedMode( agx::Bool highSpeedMode );

      /**
      \return The shaft that other power line components may connect to. Rotation
              in this shaft is transfered to the Hinge to which this Rotational-
              Actuator is connected.
      */
      agxPowerLine::RotationalUnit* getInputShaft();


      virtual void setEnable(bool enabled);
      virtual bool getEnable() const;

    // Called by the rest of the power line framework.
    public:
      virtual bool preUpdate(agx::Real timeStamp) override;

      virtual void getConnectableDimensionTypes(
        agxPowerLine::PhysicalDimension::TypeVector& types,
        agxPowerLine::Side side) const override;

      virtual agx::Real calculateRelativeGradient() const override;

      virtual agx::Vec3 calculateWorldDirection(
          Side side,
          agxPowerLine::PhysicalDimension::Type dimensionType) const override;

      virtual agx::Vec3 calculateLocalDirection(
          Side side,
          int dimensionType) const override;

      /**
      Stores internal data into stream.
      */
      virtual bool store(agxStream::StorageStream& out) const override;

      /**
      Restores internal data from stream.
      */
      virtual bool restore(agxStream::StorageStream& in) override;

      AGXSTREAM_DECLARE_SERIALIZABLE(agxPowerLine::RotationalActuator);

    DOXYGEN_START_INTERNAL_BLOCK()
    public:
      virtual agxPowerLine::DimensionAndSide getConnectableDimension(
          agxPowerLine::PhysicalDimension::Type type,
          agxPowerLine::Side side) override;


    protected:
      RotationalActuator();

      /*
      There are subclasses of RotationalActuator that want to create their own
      versions of the input shaft and the internal connector. For example, the
      hydraulics package contains the ImpellerActuator, which uses a FlowUnit
      for input.

      Any subclass that uses the dont_create_input tag when constructing the
      RotationalActuator base class must implement get- and setEnable.

      This part of the inheritence tree could use some redesign. There should
      be one base class, inheriting from Actuator1DOF, that knows about hinges
      and how to get Jacobians from them, and a collection of subclasses
      derived of off that that knows about the type of the input and the custom
      constraint that binds the input PhysicalDimension to the
      ActuatorBodyUnits.

      Perhaps we should even move the concept of input unit and internal
      constraint up a bit more and put it inside Actuator1DOF. In that case we
      don't have to deal with the oddities of the current setEnable
      implementation, which is just bad. A base class implementation does the
      wrong thing for the derived classes.

      Not sure if the name RotationalActuator should be used for the base class
      or the subclass that has a RotationalUnit input. The latter is more
      consistent with the current implementation and a better user-level name
      (the base class is not user-level), but what should the base class be
      named?
      */
      struct AGXMODEL_EXPORT dont_create_input_t {};
      static dont_create_input_t dont_create_input;
      RotationalActuator(agx::Constraint1DOF* constraint, dont_create_input_t);
      RotationalActuator(agx::Constraint* constraint, ConstraintGeometry* constraintGeometry, dont_create_input_t);

      virtual ~RotationalActuator();

      void connectActuatorConnector();
      void disconnectActuatorConnector();
      void reconnectActuatorConnector();
    DOXYGEN_END_INTERNAL_BLOCK()

    protected:
      agxPowerLine::RotationalUnitRef m_inputShaft;
      agxPowerLine::detail::RotationalActuatorConnectorRef m_actuatorConnector;

      agx::Bool m_useHighSpeedMode;
  };

  typedef agx::ref_ptr<RotationalActuator> RotationalActuatorRef;



  class AGXMODEL_EXPORT TranslationalActuator : public agxPowerLine::Actuator1DOF
  {
    public:

      TranslationalActuator(agx::Constraint1DOF* constraint); // \todo How only allow translational constraints? Should we?
      TranslationalActuator(agx::Constraint* constraint, ConstraintGeometry* constraintGeometry);

      agxPowerLine::TranslationalUnit* getInputRod();


    // Called by the rest of the power line framework.
    public:
      virtual void getConnectableDimensionTypes(
          agxPowerLine::PhysicalDimension::TypeVector& types,
          agxPowerLine::Side side) const override;

      /**
      Stores internal data into stream.
      */
      virtual agxPowerLine::DimensionAndSide getConnectableDimension(
            agxPowerLine::PhysicalDimension::Type type,
            agxPowerLine::Side side) override;

      /**
      Restores internal data from stream.
      */
      virtual agx::Real calculateRelativeGradient() const override;

      virtual agx::Vec3 calculateWorldDirection(
          Side side,
          agxPowerLine::PhysicalDimension::Type dimensionType) const override;

      virtual agx::Vec3 calculateLocalDirection(Side side, int dimensionType) const override;

      /**
      Stores internal data into stream.
      */
      virtual bool store(agxStream::StorageStream& str) const override;

      /**
      Restores internal data from stream.
      */
      virtual bool restore(agxStream::StorageStream& str) override;

      AGXSTREAM_DECLARE_SERIALIZABLE(agxPowerLine::TranslationalActuator);

    protected:
      TranslationalActuator();

      struct AGXMODEL_EXPORT dont_create_input_t {};
      static dont_create_input_t dont_create_input;

      TranslationalActuator(agx::Constraint1DOF* constraint, dont_create_input_t);
      TranslationalActuator(
          agx::Constraint* constraint, agxPowerLine::ConstraintGeometry* constraintGeometry, dont_create_input_t);

      virtual ~TranslationalActuator();

      void initialize();

      void connectActuatorConnector();

    private:
      agxPowerLine::TranslationalUnitRef m_inputRod;
      agxPowerLine::detail::TranslationalActuatorConnectorRef m_actuatorConnector;
  };

  typedef agx::ref_ptr<TranslationalActuator> TranslationalActuatorRef;


}
