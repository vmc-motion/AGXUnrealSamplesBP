# AGXUnrealSamplesBP
UE project containing sample levels showing how to use 'AGX Dynamics for Unreal' using Blueprint Visual Scripting, without any C++ code.

## Development Environment

* Unreal Engine: 5.3
* AGX Dynamics for Unreal: 1.13.1 (included in project)

# Sample Levels

Please be aware that the levels below are just simple samples on how to access various functions of AGX Unreal. In a real project, a more sophisticated and thorough design might be necessary.

## Target Speed Control

This sample shows one way to set the speed of an Hinge constraint using Blueprint Visual Scripting. The same technique can be used for other constraints, like the Prismatic component.


### Level ###
Content/Samples/TargetSpeedControl/L_TargetSpeedControl.umap

### Test ###
1. Open level.
1. Press Play button.
 
   (if necessary, press Shift + F1 to activate editor Mouse Control)

1. Select actor **BP_TargetSpeedControl_Sample** from Outliner window.
1. Modify proprety **Target Speed** from Details windows.

<img src="Documentation/Gifs/TargetSpeedControl_Play.gif" width="600"/>

### Contents ###

The level has one BP actor called **BP_TargetSpeedControl_Sample**.

The actor consists of two rigid body components, **Body_Base** and **Body_Arm**, which are connected by the hinge component **Hinge_Arm**.

<img src="Documentation/Images/TargetSpeedControl_BP_Components.png" width="600"/>

The **TargetSpeedController** of the hinge component has been Enabled from the Details panel.

<img src="Documentation/Images/TargetSpeedControl_BP_Hinge.png" width="500"/>

The actor has one public variable called **TargetSpeed** that the user can modify from the Details windows. By further development, it can be connected to keyboard, gamepad, or an external data source.

As seen below, the actor's Blueprint Event Graph continuously sets the speed of the hinge component's **TargetSpeedController** using the value of our public variable. This is done from an event that is bind to execute before each AGX simulation step.

<img src="Documentation/Images/TargetSpeedControl_BP_Graph.png" width="600"/>


