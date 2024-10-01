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
#include <agxROS2/export.h>
#include <agxROS2/Publisher.h>
#include <agxROS2/Subscriber.h>
#include <agxROS2/Qos.h>

#include <agx/Constraint.h>
#include <agxSDK/StepEventListener.h>
#include <agxSDK/Assembly.h>
#include <agxSDK/Simulation.h>
#include <agx/agx_vector_types.h>

#include <map>
#include <string>
#include <tuple>

namespace agxROS2
{
  AGX_DECLARE_POINTER_TYPES(ROS2ControlInterface);
  /**
   * Possible to add joints (Constraint1DOF) and the listener will publish the
   * position and velocity for the joints as a sensorMsgs/JointState aswell as
   * listens to commands to control said joint. This will typically be used
   * together with the ros2_control and topic_based_ros2_control.
   */
  class AGXROS2_EXPORT ROS2ControlInterface : public agxSDK::StepEventListener
  {
    /**
      The ROS2ControlCommandInterface sets how the joints should be controlled

      There are three different command interfaces possible
      -  POSITION means that the joint is controlled by locking the joints free dof to a position
      -  VELOCITY means that the joint speed about the joints free dof is controlled to keep a set velocity
      -  EFFORT means that the force/torque about the joints free dof is set directly
    */
    public:
      enum ROS2ControlCommandInterface {
        POSITION = 0,
        VELOCITY = 1,
        EFFORT = 2
      };

      /**
      Constructor creates the ROS2ControlInterface and initializes the ROS2 subscriber and publisher that will run in the pre and post steps respectively
      \param jointCommandTopic - The topic that the joint commands are received on
      \param jointStateTopic - The topic that the current joint states are published on
      \param syncWithSystemTime - If true the simulation timestamp is set to be equal to the system time.
      \note - This is useful when controlling with ros2_control node with use_sim_time=true, since it syncs its sleeps against system time.
      */
      ROS2ControlInterface(
        const std::string& jointCommandTopic,
        const std::string& jointStateTopic,
        bool syncWithSystemTime = false
      );

      /**
      Add a joint to be controlled by this listener and publish its state on the jointStateTopic
      \param joint - The Contraint1DOF joint to be added to the listener
      \param commandInterface - enum deciding how this joint should be controlled
      \return true if the joint is successfully added. false otherwise.
      */
      bool addJoint(agx::Constraint1DOF* joint, ROS2ControlCommandInterface commandInterface);

      /**
      Remove a joint that is controlled by this listener
      \param name - The name of the joint to be removed from the listener
      \return true if the joint is siuccessfully removed. false otherwise.
      */
      bool removeJoint(const std::string& name);

      /**
      Remove a joint that is controlled by this listener
      \param joint - The Contraint1DOF joint to be removed from the listener
      \return true if the joint is successfully removed. false otherwise.
      */
      bool removeJoint(agx::Constraint1DOF* joint);

      /**
      Checks if there is an available command message, receives it and sets the commands on the joints
      */
      virtual void pre(const agx::TimeStamp& time) override;

      /**
      Publishes the joint states
      */
      virtual void post(const agx::TimeStamp& time) override;

    private:
      /**
      Private method for populating the JointState message with the position and velocity
      that is published in the post() method
      \param time - The current simulation timeStamp
      */
      void populateJointStateMessage(const agx::TimeStamp& time);
      /**
      Private method for setting the commands on the joints given an received message in pre()
      \param jointStateMsg - The JointState message with the commands
      */
      void setCommandOnJoints(agxROS2::sensorMsgs::JointState& jointStateMsg);
      /**
      Private method for setting the time of the header part of the jointStateMsg
      \param jointStateMsg - The JointState message with the commands
      \param time - The time to set to the header
      */
      void setHeader(agxROS2::sensorMsgs::JointState& jointStateMsg, const agx::TimeStamp& time);

      struct Joint {
        agx::Constraint1DOFRef constraint;
        agx::RangeReal effortLimit;
        ROS2ControlCommandInterface commandInterface;
      };

    private:
      agxROS2::sensorMsgs::PublisherJointState m_pub;
      agxROS2::sensorMsgs::JointState m_stateMsg;

      agxROS2::sensorMsgs::SubscriberJointState m_sub;
      agxROS2::sensorMsgs::JointState m_commandMsg;

      std::map<std::string, Joint> m_joints;

      // ros2_control syncs its control loop against the system clock.
      // even if we publish the simulation time to /clock the ros2_control control
      // loop will sleep with std::this_thread::sleep_until(next_iteration_time);
      // where next_iteration_time is 0 and the system clock is larger so it will not sleep at all.
      // Thus making the controller loop running as fast as possible ~4k hz always depending on hardware.
      // Currently an issue in ros2_control repo https://github.com/ros-controls/ros2_control/issues/859
      // 
      // A workaround is to set the simulation time to the system time. This assumes that the simulation
      // runs in realtime. Not faster or slower. This will not make the control-loop and simulation synchronous,
      // it only makes the controller update rate to be the specified rate. If the simulation runs faster than
      // realtime then control signals will be missing when stepping. If it runs slower then the simulation
      // will miss intermediate control signals.
      bool m_syncWithSystemTime;
  };
}
