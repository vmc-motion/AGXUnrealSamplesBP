# AGXUnrealSamplesBP
UE project containing sample levels showing how to use 'AGX Dynamics for Unreal' using Blueprint Visual Scripting, without any C++ code.

# Development Environment

* Unreal Engine: 5.3
* AGX Dynamics for Unreal: 1.13.1 (included in project)

# How To Download

# Option A) Download zip file
1. Click the green button **Code** to open the dropdown menu.
1. Right click **Download ZIP** in the dropdown menu.
1. Select **Save Link As...** from the context menu and choose where to save the zip file.
1. Extract the zip file after it has finished downloading.
1. Double click **AGXUnrealSamplesBP.uproject** to open the project in Unreal Editor.

<img src="Documentation/Images/HowToDownload.png"/>

# Option B) Clone using git
1. Install git.
1. Install git lfs.
1. Open a terminal in desired root folder and clone by standard means like below.
   ```
   git clone https://github.com/vmc-motion/AGXUnrealSamplesBP
   ```
1. Double click **AGXUnrealSamplesBP.uproject** to open the project in Unreal Editor.


# Sample Levels

Below is a overview of the sample levels included in the project.

Please be aware that the levels below are just simple demonstrations on ways to access various functions of AGX Unreal. In a real project, a more sophisticated and thorough design might be necessary.

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

The level has one BP Class actor called **BP_TargetSpeedControl_Sample**.

The actor consists of two rigid body components, **Body_Base** and **Body_Arm**, which are connected by the hinge component **Hinge_Arm**.

<img src="Documentation/Images/TargetSpeedControl_BP_Components.png" width="600"/>

The **TargetSpeedController** of the hinge component has been Enabled from the Details panel.

<img src="Documentation/Images/TargetSpeedControl_BP_Hinge.png" width="500"/>

The actor has one public variable called **TargetSpeed** that the user can modify from the Details windows. By further development, it can be connected to keyboard, gamepad, or an external data source.

As seen below, the actor's Blueprint Event Graph continuously sets the speed of the hinge component's **TargetSpeedController** using the value of our public variable. This is done from an event that is bind to execute before each AGX simulation step.

<img src="Documentation/Images/TargetSpeedControl_BP_Graph.png" width="600"/>


