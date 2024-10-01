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

#pragma once

#include <agxVehicle/export.h>

/// \todo Move ConstraintGeometry to its own header file so we don't have to
/// include the entire Actuator jungle here.
#include <agxPowerLine/Actuator1DOF.h>

#include <agxVehicle/WheelJoint.h>


namespace agxVehicle
{
  AGX_DECLARE_POINTER_TYPES(WheelJointConstraintGeometry);

  /**
  Description of constraint geometry for one of the free degrees of freedom of
  a wheel joint. Used by power-line actuators to transfer force/torque/pressure
  from the 1D domain of the power-line to the 3D domain of the rigid bodies.
  */
  class AGXVEHICLE_EXPORT WheelJointConstraintGeometry : public agxPowerLine::ControllerConstraintGeometry
  {
    public:
      WheelJointConstraintGeometry();

      /**
      \param wheel The WheelJoint that this ConstraintGeometry should align with.
      \param axis The degree of freedom in the WheelJoint that this ConstraintGeometry should align with.
      */
      WheelJointConstraintGeometry(WheelJoint* wheel, WheelJoint::SecondaryConstraint axis);

      bool setConstraint(WheelJoint* wheel);
      bool setConstraint(WheelJoint* wheel, WheelJoint::SecondaryConstraint axis);
      using agxPowerLine::ControllerConstraintGeometry::setConstraint;

      WheelJoint::SecondaryConstraint getAxis() const;

      AGXSTREAM_DECLARE_SERIALIZABLE(agxVehicle::WheelJointConstraintGeometry)

    private:
      WheelJoint* m_wheel;
  };
}
