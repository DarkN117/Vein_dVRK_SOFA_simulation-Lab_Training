![Screenshot 2023-11-02 174229](https://github.com/DarkN117/Vein_dVRK_SOFA_simulation-Lab_Training/assets/148345993/cf2bfd6d-e545-4f20-a024-cd83c907821c)

# Vein_dVRK_SOFA_simulation-Lab_Training

This repository contains all files related to my internship at the BioRobotics Institute. The project was titled "Design of a Virtual Surgical Task and Evaluation Using the Da Vinci Research Kit (dVRK)."

The surgical task focused on the interaction between the dVRK surgical robot and a pulmonary vein. A pre-existing physical simulator was available in the lab, but clinicians needed a training platform that could overcome the limitations of the physical setup and provide quantitative assessments. This led to the development of a virtual counterpart.

The virtual simulator was built using SOFA, an open-source framework designed for real-time physical simulation, with a focus on medical simulation.

My setup:

-PC with intel core i5 processor and Nvidia 1050 (laptop version)

-Ubuntu 20.04 LTS

-SOFA version 23.06

-Blender, Gmsh and FreeCad to create volumetric and surface meshes and to further refine them

-ROS Noetic

-Python 3.8


Now, let's dive in the folders of the repo:)


1) "Simulation_meshes": Contains all the meshes developed for the vein, setup, and dVRK gripper. The final test meshes are located in the "final_centered_mesh" folder.

2) "my examples": Includes several examples I created from scratch or modified from pre-existing ones. These are useful for understanding the basic principles and functions of SOFA.

3) "Simulation_scene_development": Contains all simulation scenes, written in Python. The final versions are labeled as "v6."

4) "ROS_simulation_ws": The ROS workspace used for the final simulation test. It includes the "gripper_info_publisher" file, which is required to command the simulated dVRK gripper in real time.



APPENDIX (some info to interpret file names):

-"DAL" stands for "DefaultAnimationLoop"

-"FMAL" stands for "FreeMotionAnimationLoop"

-"RSSFF" stands for "RestShapeSpringsForceField"

-"PSM" stands for "Patient Side Manipulator"
