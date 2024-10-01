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

#ifndef AGX_BALLJOINT_H
#define AGX_BALLJOINT_H

#include <agx/Constraint.h>

namespace agx
{
  /**
  Class for storing the geometric information of a BallJoint
  */
  class CALLABLE AGXPHYSICS_EXPORT BallJointFrame : public ConstraintFrame
  {
    public:
      BallJointFrame();

      /**
       \param center - Ball center position in world coordinates
       */
      BallJointFrame( const Vec3& center );

      /// Destructor
      ~BallJointFrame();

      void setCenter( const Vec3& center );
      const Vec3& getCenter() const;
  };

  /**
  Constraint that removes the three translation DOF between two bodies (or one and the world).
  */
  class CALLABLE AGXPHYSICS_EXPORT BallJoint : public Constraint
  {
    public:
      /**
      Constructor that creates a Ball Joint.
      \param bf - Frame specifying the geometry of the constraint.
      \param rb1 - The first body
      \param rb2 - If null, rb1 is attached to the world, if not, rb1 and rb2 are attached to each other.
      */
      BallJoint( const BallJointFrame& bf, RigidBody* rb1, RigidBody* rb2 = 0 );

      /**
      Create a Ball Joint given one or two bodies with their respective attachment frames. Given two bodies and
      two frames, the ball of the ball joint will be located at these two frames origins. For one body,
      rb2 == rb2AttachmentFrame == 0, the ball of the ball joint will be located at the world position of the
      attachment frame.
      \param rb1 - First rigid body (invalid if null)
      \param rb1AttachmentFrame - First rigid body attachment frame
      \param rb2 - Second rigid body (if null, first rigid body will be attached to world)
      \param rb2AttachmentFrame - Second rigid body attachment frame (invalid if rb2 != 0 and rb2AttachmentFrame == 0)
      \note Valid configurations:   rb1 != 0 && rb1AttachmentFrame != 0,
                                    rb1 != 0 && rb1AttachmentFrame != 0 && rb2 == 0 and
                                    rb1 != 0 && rb1AttachmentFrame != 0 && rb2 != 0 && rb2AttachmentFrame != 0.
      All other configurations are invalid.
      */
      BallJoint( RigidBody* rb1, Frame* rb1AttachmentFrame, RigidBody* rb2 = nullptr, Frame* rb2AttachmentFrame = nullptr );

      /**
      Create a Ball Joint given one or two bodies with relative attachment points. This is similar to attachment
      frames.
      \param rb1 - First rigid body (invalid if null)
      \param rb1LocalAttachPoint - Attach point given in first rigid body local frame
      \param rb2 - Second rigid body (if null, first rigid body will be attached to world)
      \param rb2LocalAttachPoint - Attach point given in second rigid body local frame
      */
      BallJoint( RigidBody* rb1, const Vec3& rb1LocalAttachPoint, RigidBody* rb2 = nullptr, const Vec3& rb2LocalAttachPoint = Vec3() );

      /**
      Enum used for specifying which Degree of Freedom (DOF) that should be accessed in calls to for example:
      constraint->getRegularizationParameters( dof ); constraint->setDamping( damping, dof );
      */
      enum DOF {
        ALL_DOF = -1,        /**< Select all degrees of freedom */
        TRANSLATIONAL_1 = 0, /**< Select DOF for the first translational axis */
        TRANSLATIONAL_2 = 1, /**< Select DOF for the second translational axis */
        TRANSLATIONAL_3 = 2, /**< Select DOF for the third translational axis */
        NUM_DOF = 3          /**< Number of DOF available for this constraint */
      };

      /**
      \return the number of DOF for this constraint, not including secondary constraints
      */
      virtual int getNumDOF() const override;

      /**
      Get ConeLimit, which can limit the movement of the first body in this constraint, relative to the
      second body.
      \return the ConeLimit secondary constraint
      */
      ConeLimit* getConeLimit();

      /**
      Get ConeLimit, which can limit the movement of the first body in this constraint, relative to the
      second body. Two limit angles can be set, which will define an elliptic cone that limits the movement
      of the bodies.
      \return ConeLimit secondary constraint
      */
      const ConeLimit* getConeLimit() const;

      /**
      Get FrictionControllers that control the friction on the limit of the ConeLimit. Note that these
      FrictionControllers will not do anything if the ConeLimit is not active.
      There are two FrictionControllers on the limit, one which controls the friction along the limit,
      and the other one for rotation on the limit.
      \return Vector of FrictionControllers related to the ConeLimit
      */
      FrictionControllerRefSetVector getConeLimitFrictionControllers() const;

      /**
      Enable/disable all the FrictionControllers connected to the ConeLimit, which sets friction on the 
      limit.
      \param enable - Set to true to enable all ConeLimit FrictionControllers and false disable them.
      */
      void setEnableConeLimitFriction(bool enable);

      /**
      Set the friction coefficient for all the FrictionControllers connected to the ConeLimit. The friction
      is only active on the limit. Note that this will override any other values set.
      \param frictionCoefficient - the friction coefficient for all the cone limit FrictionControllers
      */
      void setConeLimitFrictionCoefficients(Real frictionCoefficient);

      /**
      Get the friction coefficients of all the cone limit friction controllers
      \return RealVector containing the friction coefficients
      */
      RealVector getConeLimitFrictionCoefficients() const;

      /**
      Get the FrictionController that acts along the cone limit.
      \return FrictionController that controls part of the ConeLimit friction.
      */
      FrictionController* getConeLimitFrictionControllerLimit() const;

      /**
      Get the FrictionController that acts on rotations on the cone limit.
      \return FrictionController that controls part of the ConeLimit friction.
      */
      FrictionController* getConeLimitFrictionControllerRotational() const;

      /**
      Get all three FrictionControllers that control the rotational friction of the BallJoint.
      \return Vector of FrictionControllers
      */
      FrictionControllerRefSetVector getRotationalFrictionControllers() const;

      /**
      Enable/disable rotational friction for all rotational directions of the BallJoint.
      \param enable - Set to true to enable all rotational FrictionControllers and false disable them.
      */
      void setEnableRotationalFriction(bool enable);

      /**
      Get the FrictionController that controls the friction around the U-axis
      \return FrictionController
      */
      FrictionController* getRotationalFrictionControllerU() const;

      /**
      Get the FrictionController that controls the friction around the V-axis
      \return FrictionController
      */
      FrictionController* getRotationalFrictionControllerV() const;

      /**
      Get the FrictionController that controls the friction around the N-axis
      \return FrictionController
      */
      FrictionController* getRotationalFrictionControllerN() const;

      /**
      Set the friction coefficient for all rotational FrictionControllers of the BallJoint.
      \param frictionCoefficient - the friction coefficient for all rotational FrictionControllers
      */
      void setRotationalFrictionCoefficients(Real frictionCoefficient);

      /**
      Get the friction coefficients of all the rotational friction controllers
      \return RealVector containing the friction coefficients
      */
      RealVector getRotationalFrictionCoefficients() const;

      /**
      Get TwistRange secondary constraint, making it possible to limit the twist of the first frame relative
      to the second one. When enabled no range is set by default.
      \return TwistRange secondary constraint
      */
      TwistRangeController* getTwistRangeController() const;

      AGXSTREAM_DECLARE_SERIALIZABLE(agx::BallJoint);

    protected:
      BallJoint();
      BallJoint( class BallJointImplementation* impl );
      virtual ~BallJoint();

      virtual void render( agxRender::RenderManager* mgr, float scale ) const override;

    private:
      class BallJointImplementation* m_implementation;
  };

  typedef ref_ptr< BallJoint > BallJointRef;

  /**
  Special version of a ball joint where the spatial reference frame is world for
  the body/bodies. This translates to that BallJoint::DOF::TRANSLATIONAL_1 is
  world x axis, BallJoint::DOF::TRANSLATIONAL_2 world y etc..

  Since the reference frame isn't moving, this type of the ball joint tends to
  be more stable for fast moving/rotating objects.
  */
  class CALLABLE AGXPHYSICS_EXPORT WorldFrameBallJoint : public BallJoint
  {
    public:
      /**
      Create a Ball Joint given one or two bodies with their respective attachment frames. Given two bodies and
      two frames, the ball of the ball joint will be located at these two frames origins. For one body,
      rb2 == rb2AttachmentFrame == 0, the ball of the ball joint will be located at the world position of the
      attachment frame.
      \param rb1 - First rigid body (invalid if null)
      \param rb1AttachmentFrame - First rigid body attachment frame
      \param rb2 - Second rigid body (if null, first rigid body will be attached to world)
      \param rb2AttachmentFrame - Second rigid body attachment frame (invalid if rb2 != 0 and rb2AttachmentFrame == 0)
      \note Valid configurations:   rb1 != 0 && rb1AttachmentFrame != 0,
                                    rb1 != 0 && rb1AttachmentFrame != 0 && rb2 == 0 and
                                    rb1 != 0 && rb1AttachmentFrame != 0 && rb2 != 0 && rb2AttachmentFrame != 0.
      All other configurations are invalid.
      */
      WorldFrameBallJoint( RigidBody* rb1, Frame* rb1AttachmentFrame, RigidBody* rb2 = nullptr, Frame* rb2AttachmentFrame = nullptr );

      AGXSTREAM_DECLARE_SERIALIZABLE(agx::WorldFrameBallJoint);

    protected:
      WorldFrameBallJoint();
      virtual ~WorldFrameBallJoint();

    private:
      class WorldFrameBallJointImplementation* m_implementation;
  };

  typedef ref_ptr< WorldFrameBallJoint > WorldFrameBallJointRef;

} // namespace agx

#endif // AGX_BALLJOINT_H
