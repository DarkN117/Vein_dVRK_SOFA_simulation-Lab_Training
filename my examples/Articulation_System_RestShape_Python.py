#!/usr/bin/env python3

# Required import for python
import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import rospy
import matplotlib.pyplot as plt

_runAsPythonScript = False
# Choose in your script to activate or not the GUI
USE_GUI = True

def createScene(root):

    root.gravity = [0, -9.81, 0]
    root.dt = 0.005
    
     # ADD PLUGINS
    root.addObject("RequiredPlugin", pluginName=['Sofa.Component.Collision.Detection.Algorithm',
                                                 'Sofa.Component.Collision.Detection.Intersection',  #Needed to use components [DiscreteIntersection]
                                                 'Sofa.Component.Collision.Geometry',
                                                 'Sofa.Component.Collision.Response.Contact',  #Needed to use components [DefaultContactManager]
                                                 'Sofa.Component.Constraint.Projective',  #Needed to use components [FixedConstraint]
                                                 'Sofa.Component.Constraint.Lagrangian.Correction',
                                                 'Sofa.Component.Constraint.Lagrangian.Solver',
                                                 'Sofa.Component.Engine.Select',  # Needed to use components [BoxROI]
                                                 'Sofa.Component.IO.Mesh',  #Needed to use components [MeshOBJLoader SphereLoader]
                                                 'Sofa.Component.LinearSolver.Iterative',  #Needed to use components [CGLinearSolver]
                                                 'Sofa.Component.Mapping.Linear',  #Needed to use components [BarycentricMapping]
                                                 'Sofa.Component.Mass',  #Needed to use components [UniformMass]
                                                 'Sofa.Component.ODESolver.Backward',  #Needed to use components [EulerImplicitSolver]
                                                 'Sofa.Component.SolidMechanics.FEM.Elastic',  #Needed to use components [TriangleFEMForceField]
                                                 'Sofa.Component.StateContainer',  #Needed to use components [MechanicalObject]
                                                 'Sofa.Component.Topology.Container.Dynamic',
                                                 'Sofa.Component.Topology.Container.Constant',  # Needed to use components [MeshTopology]
                                                 'Sofa.Component.Visual',  #Needed to use components [VisualStyle]
                                                 'Sofa.GL.Component.Rendering3D',  #Needed to use components [OglModel]
                                                 'Sofa.Component.MechanicalLoad',  #Needed to use components [ConstantForceField]
                                                 'Sofa.Component.SolidMechanics.Spring',  #Needed to use components [MeshSpringForceField]])
                                                 'Sofa.Component.AnimationLoop',  # Needed to use components [FreeMotionAnimationLoop]
                                                 'Sofa.Component.Mapping.NonLinear',  #Needed to use components [RigidMapping]
                                                 'Sofa.Component.LinearSolver.Direct',  # Needed to use components [SparseLDLSolver]
                                                 'Sofa.Component.SolidMechanics.FEM.HyperElastic',  # Needed to use components [TetrahedronHyperelasticityFEMForceField]
                                                 'Sofa.GL.Component.Shader',  # Needed to use components [DirectionalLight,LightManager,PositionalLight,SpotLight]
                                                 'Sofa.Component.Setting',  # Needed to use components [BackgroundSetting]
                                                 'ArticulatedSystemPlugin' # Needed to use components [ArticulatedHierarchyContainer,ArticulatedSystemMapping,Articulation,ArticulationCenter]   
                                                 ])
                                                 
    root.addObject('DefaultAnimationLoop')
    root.addObject('CollisionPipeline', verbose="0", draw="0")
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    root.addObject('MinProximityIntersection', name="Proximity", alarmDistance="0.0001", contactDistance="0.00005")
    root.addObject('CollisionResponse', name="Response", response="PenalityContactForceField")

    root.addObject('VisualStyle', name="visualStyle1", displayFlags="hideBehaviorModels hideCollisionModels")

    root.addObject('EulerImplicitSolver')
    root.addObject('CGLinearSolver', iterations="100", name="linear solver", threshold="1e-20", tolerance="1e-20")                                             
    
    
    
    Shaft_target_pose = root.addChild('Shaft_target_pose')
    Shaft_target_pose.addObject('MechanicalObject', name="rest_shape", template="Rigid3d", position="0 0.02 0.02 0 0 0 1")
    
    Shaft_pose = root.addChild('Shaft_pose')
    Shaft_pose.addObject('MechanicalObject', name="shaft_pose", template="Rigid3d", position="0 0 0 -0.5652 -0.529 0.3821 -0.5049")
    Shaft_pose.addObject('RestShapeSpringsForceField', name="RestShapeSFF", stiffness="1000", angularStiffness="1000", external_rest_shape="@../Shaft_target_pose/rest_shape")  
    
    Jaws_target_angle = root.addChild('Jaws_target_angle')
    Jaws_target_angle.addObject('MechanicalObject', name="rest_shape", template="Vec1d", position="0.35 -0.35")
    
    
    
    Gripper = root.addChild('Gripper')
    Gripper.addObject('MechanicalObject', name="Articulations", template="Vec1d", position="-0.35 0.35")
    Gripper.addObject('RestShapeSpringsForceField', name="RestShapeSFF", stiffness="1000", angularStiffness="1000", external_rest_shape="@../Jaws_target_angle/rest_shape")
    
    
    Jaws = Gripper.addChild('Jaws')
    Jaws.addObject('EulerImplicitSolver')
    Jaws.addObject('CGLinearSolver', iterations="200", tolerance="1e-09")
    Jaws.addObject('MechanicalObject', name="GripperDOFs", template="Rigid3d", position="0 0 0 -0.5652 -0.529 0.3821 -0.5049   0 0 0 -0.5652 -0.529 0.3821 -0.5049   0 0 0 -0.5652 -0.529 0.3821 -0.5049")
    Jaws.addObject('UniformMass', template="Rigid3d", name="mass", totalMass="0.05", showAxisSizeFactor="0.005")   #####Try to remove mass...
    
    ShaftVisual = Jaws.addChild('ShaftVisual')
    ShaftVisual.addObject('MeshObjLoader', name="meshloader", filename="/home/darkn117/sofa/src/share/mesh/Simulation_meshes/Final_Centered_meshes/PSM_Base_High_Res.obj")
    ShaftVisual.addObject('OglModel', name="model", src="@meshloader")
    ShaftVisual.addObject('RigidMapping', input="@..", output="@model", index="0")
    
    LeftVisual = Jaws.addChild('LeftVisual')
    LeftVisual.addObject('MeshObjLoader', name="meshloader", filename="/home/darkn117/sofa/src/share/mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_High_Res.obj")
    LeftVisual.addObject('OglModel', name="model", src="@meshloader")
    LeftVisual.addObject('RigidMapping', input="@..", output="@model", index="1")
    
    RightVisual = Jaws.addChild('RightVisual')
    RightVisual.addObject('MeshObjLoader', name="meshloader", filename="/home/darkn117/sofa/src/share/mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_High_Res.obj")
    RightVisual.addObject('OglModel', name="model", src="@meshloader")
    RightVisual.addObject('RigidMapping', input="@..", output="@model", index="2")
    
    LeftCollision = Jaws.addChild('LeftCollision')
    LeftCollision.addObject('MeshObjLoader', name="meshloader", filename="/home/darkn117/sofa/src/share/mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_Collision_Low_Res_Surface.obj")
    LeftCollision.addObject('MeshTopology', src="@meshloader")
    LeftCollision.addObject('MechanicalObject', name="model", src="@meshloader")
    LeftCollision.addObject('LineCollisionModel', moving="1", simulated="1", group="1")
    LeftCollision.addObject('PointCollisionModel', moving="1", simulated="1", group="1")
    LeftCollision.addObject('RigidMapping', input="@..", output="@model", index="1")
    
    RightCollision = Jaws.addChild('RightCollision')
    RightCollision.addObject('MeshObjLoader', name="meshloader", filename="/home/darkn117/sofa/src/share/mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_Collision_Low_Res_Surface.obj")
    RightCollision.addObject('MeshTopology', src="@meshloader")
    RightCollision.addObject('MechanicalObject', name="model", src="@meshloader")
    RightCollision.addObject('LineCollisionModel', moving="1", simulated="1", group="1")
    RightCollision.addObject('PointCollisionModel', moving="1", simulated="1", group="1")
    RightCollision.addObject('RigidMapping', input="@..", output="@model", index="2")
    
    Jaws.addObject('ArticulatedSystemMapping', input1="@../Articulations", input2="@../../Shaft_pose/shaft_pose", output="@GripperDOFs")
    
    Gripper.addObject('ArticulatedHierarchyContainer')
    
    Articulation_Centers = Gripper.addChild('Articulation_Centers')
    
    """Articulation_Center_Shaft = Articulation_Centers.addChild('Articulation_Center_Shaft') 
    Articulation_Center_Shaft.addObject('ArticulationCenter', parentIndex="0", childIndex="0", posOnParent="-0.0011 0 0.0005", posOnChild="0 0 0")
    Shaft_Articulation = Articulation_Center_Shaft.addChild('Shaft_Articulation') 
    Shaft_Articulation.addObject('Articulation', translation="0", rotation="0", rotationAxis="1 0 0", articulationIndex="0")"""
    
    Articulation_Center_Right = Articulation_Centers.addChild('Articulation_Center_Right') 
    Articulation_Center_Right.addObject('ArticulationCenter', parentIndex="0", childIndex="2", posOnParent="-0.0011 0 0.0005", posOnChild="0 0 0")
    Right_Articulation = Articulation_Center_Right.addChild('Right_Articulation') 
    Right_Articulation.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="0")

    Articulation_Center_Left = Articulation_Centers.addChild('Articulation_Center_Left') 
    Articulation_Center_Left.addObject('ArticulationCenter', parentIndex="0", childIndex="1", posOnParent="0.0011 0 0.0005", posOnChild="0 0 0")
    Left_Articulation = Articulation_Center_Left.addChild('Left_Articulation') 
    Left_Articulation.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="1")
    
    return root
    

def main():
	
	import SofaRuntime
	import Sofa.Gui
	
	#SofaRuntime.importPlugin("SofaOpenglVisual")
	#SofaRuntime.importPlugin("SofaImplicitOdeSolver")
	SofaRuntime.importPlugin("Sofa.GL.Component.Rendering2D")
	SofaRuntime.importPlugin("Sofa.GL.Component.Rendering3D")
	SofaRuntime.importPlugin("Sofa.GL.Component.Shader")
	SofaRuntime.importPlugin("Sofa.Component.ODESolver.Backward")
	
	# Call the SOFA function to create the root node
	root = Sofa.Core.Node("root")

	# Call the createScene function, as runSofa does
	createScene(root)

	# Once defined, initialization of the scene graph
	Sofa.Simulation.init(root)

	# Launch the GUI (qt or qglviewer)
	Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
	Sofa.Gui.GUIManager.createGUI(root, __file__)
	Sofa.Gui.GUIManager.SetDimension(1080, 800)
	
	Sofa.Simulation.animate(root, root.dt.value)
	
	# Initialization of the scene will be done here
	Sofa.Gui.GUIManager.MainLoop(root)
	Sofa.Gui.GUIManager.closeGUI()
	print("\n******************\nSIMULATION IS OVER\n******************")

