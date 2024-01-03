# Vein_dVRK_SOFA_simulation-Lab_Training

This repository contains all the files related to my internship done at the BioRobotics Institute.
The activity done was named: "Design of a virtual surgical task and evaluation using the Da Vinci Research Kit(dVRK)".

The surgical task deals with the interaction of a surgical robot, namely the dVRK, with a pulmonary vein.
In the lab a pre-existent physical simulator was present. Clinicians required a training platforms to perform the operation without the limitation of the physical one and capable of giving quantitative assessments/evaluations. That's where the idea to develop a virtual couterpart was born.

The virtual simulator was built using SOFA, an open source framework primarily targeted at real-time physical simulation, with an emphasis on medical simulation.

My setup was:

-PC with intel core i5 processor and Nvidia 1050 (laptop version)

-Ubuntu 20.04 LTS

-SOFA version 23.06

-Blender, Gmsh and FreeCad to create volumetric and surface meshes and to further refine them

-ROS Noetic


Now, let's dive in the folders of the repo:)


1)"Simulation_meshes": it contains all the meshes developed for Vein,setup and dVRK gripper. The ones used for final test are in the folder "final_centered_mesh".

2)"my examples": it contains several examples I created from scratch and/or modified from pre-existing ones. Very useful to understand basic principles and functions of SOFA.

3)"Simulation_scene_development": it contains all the simulation scenes, written in python. Of course the final versions are the "v6" ones.

4)"ROS_simulation_ws": it is the ROS workspace used for final simulation test. One file, "gripper_info_publisher" is needed to command in real time the simulated dVRK gripper.



APPENDIX (some info to interpret file names):

-"DAL" stands for "DefaultAnimationLoop"

-"FMAL" stands for "FreeMotionAnimationLoop"

-"RSSFF" stands for "RestShapeSpringsForceField"

-"PSM" stands for "Patient Side Manipulator"
