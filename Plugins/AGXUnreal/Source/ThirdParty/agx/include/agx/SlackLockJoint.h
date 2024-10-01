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

namespace agx
{

  /**
  A constraint that is equal to the \sa LockJoint but also has support for a slack range for the
  three translational DOF and for all rotational DOF.
  This can be used to simulate a gap in which two bodies can move freely.
  */
  class CALLABLE AGXPHYSICS_EXPORT SlackLockJoint : public Constraint
  {
    public:
      /**
      Create Lock joint given one or two rigid bodies. If rb2 is null, rb1 will be attached in world
      with the attachment point at model frame origin.
      \param rb1 - First rigid body (invalid if null)
      \param rb2 - Second rigid body (if null, first rigid body will be attached in world)
      */
      SlackLockJoint( agx::RigidBody* rb1, RigidBody* rb2 = nullptr );

      /**
      Create Lock joint given one or two rigid bodies with corresponding attachment frames. The lock is
      defined to be fulfilled when the two attachment frames coincide in the world.
      \param rb1 - First rigid body (invalid if null)
      \param rb1AttachmentFrame - First rigid body attachment frame (invalid if null)
      \param rb2 - Second rigid body (if null, first rigid body will be attached in world)
      \param rb2AttachmentFrame - Second rigid body attachment frame (invalid if null and second rigid body is non-null)
      */
      SlackLockJoint( agx::RigidBody* rb1, agx::Frame* rb1AttachmentFrame, agx::RigidBody* rb2 = nullptr, agx::Frame* rb2AttachmentFrame = nullptr );

      /**
      Create Lock joint given two rigid bodies and one attachment point (in world coordinates).
      \param rb1 - First rigid body (invalid if null)
      \param rb2 - Second rigid body (invalid if null)
      \param worldAnchorPoint - Anchor point in world coordinates
      */
      SlackLockJoint( agx::RigidBody* rb1, agx::RigidBody* rb2, const agx::Vec3& worldAnchorPoint );

      /**
      Create Lock joint given one rigid body and a local anchor point.
      \param rb1 - First rigid body (invalid if null)
      \param localAnchorPoint - Anchor point given in first rigid body local model frame
      */
      SlackLockJoint( agx::RigidBody* rb1, const agx::Vec3& localAnchorPoint );

      /**
      Enum used for specifying which Degree of Freedom (DOF) that should be accessed in calls to for example:
      constraint->getRegularizationParameters( dof ); constraint->setDamping( damping, dof );
      */
      enum DOF
      {
        ALL_DOF=-1,        /**< Select all degrees of freedom */
        TRANSLATIONAL_1=0, /**< Select DOF for the first translational axis */
        TRANSLATIONAL_2=1, /**< Select DOF for the second translational axis */
        TRANSLATIONAL_3=2, /**< Select DOF for the third translational axis */
        ROTATIONAL_1=3,    /**< Select DOF corresponding to the first rotational axis */
        ROTATIONAL_2=4,    /**< Select DOF corresponding to the second rotational axis */
        ROTATIONAL_3=5,    /**< Select DOF for rotation around Z-axis */
        NUM_DOF=6          /**< Number of DOF available for this constraint */
      };

      /**
      \return the number of DOF for this constraint, not including secondary constraints. -1 if num DOF is undefined for the constraint.
      */
      virtual int getNumDOF() const override;


      /**
      Set slack range parameters. Positional slack parameter is in meters and specifies a range in which
      the constraint allow free movement for the three constrained translational DOF.

      Rotational parameter is in radians and specifies a range in which
      the constraint allows free rotation for all of the rotational DOF.

      The slack range parameters are symmetrical which means that the total range is twice the value used.

      \param pos_uvn Symmetrical positional slack range (m) in the first bodies attachment frame
      \param rot     Symmetrical rotational slack range (rad)
      \return True if parameters could be set.
      */
      bool setSlackParameters( agx::Vec3 pos_uvn, agx::Real rot );


      /**
      Read the current slack parameters.
      \return true if values are written
      */
      bool getSlackParameters( agx::Vec3& pos_uvn, agx::Real& rot ) const;

      AGXSTREAM_DECLARE_SERIALIZABLE( agx::SlackLockJoint );

    protected:
      SlackLockJoint();
      virtual ~SlackLockJoint();

      /**
      Internal method used for debug rendering.
      */
      virtual void render( agxRender::RenderManager *mgr, float scale  ) const override;

    private:

      class SlackLockJointImplementation *m_implementation;

  };

  typedef ref_ptr<SlackLockJoint> SlackLockJointRef;

}

