#!/usr/bin/env python3

# Required import for python
import Sofa

# Choose in your script to activate or not the GUI
USE_GUI = True


def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()


def createScene(root):
    root.gravity = [0, -9.81, 0]
    root.dt = 0.01

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
                                                 

    # Default Animation Loop+Reference Frame
    root.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    root.addObject('VisualStyle', displayFlags="hideCollisionModels")
    root.addObject('BackgroundSetting', color="0.7 1 0.7") 
    root.addObject('DefaultAnimationLoop')
    root.addObject('CollisionPipeline', name="CollisionPipeline", verbose="0")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    #root.addObject('DiscreteIntersection')
    #root.addObject('MinProximityIntersection', name="Min-Proximity" , alarmDistance=0.0001, contactDistance=0.00005)
    root.addObject('LocalMinDistance', name="LMD-proximity" , alarmDistance=0.001, contactDistance=0.0001, useLMDFilters=0)
    root.addObject('DefaultContactManager', name="CollisionResponse")
    #root.addObject('LightManager', name="lightManager", listening="1", shadows="0", softShadows="0") #use hard shadow: are less accurate but more computationally convenient
    #root.addObject('OglShadowShader', name="oglShadowShader" )
    #root.addObject('SpotLight', name="light3", color="1 1 1", shadowTextureSize="512", position="0 1 1", shadowFactor="1", cutoff="1000", exponent="1")
    #root.addObject('SpotLight', name="light", color="1 1 1", position="0 1 1", cutoff="1000", exponent="1")
    root.addObject('EulerImplicitSolver')
    root.addObject('CGLinearSolver', iterations="200", name="linear solver", threshold="1e-20", tolerance="1e-20" )
    
    
    # ADD Pulmonary_Vein model
    Vein = root.addChild('Vein')

    # Pumonary Vein MECHANICAL MODEL
    Vein.addObject('EulerImplicitSolver', name="odesolver",rayleighStiffness="0.75", rayleighMass="0.01")
    Vein.addObject('CGLinearSolver', iterations="200", threshold="1e-20", tolerance="1e-20" )
    Vein.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Vein_Low_Res.msh")
    Vein.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    Vein.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    Vein.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    Vein.addObject('MeshMatrixMass', name="mass", totalMass= "0.01", showAxisSizeFactor="0.005") #showAxisSizeFactor="0.005" allow to scale down the local object axis representation
    Vein.addObject('TetrahedralCorotationalFEMForceField',template="Vec3d", youngModulus=150000, poissonRatio=0.4, method="large", computeGlobalMatrix="0", updateStiffnessMatrix="false")
    Vein.addObject('FixedConstraint', indices="0 3 4 67 68 71 72 75 77 139 140 142 144 147 148 211 212 215 216 219 221 283 284 286 288 289")

    # Pumonary Vein COLLISION MODEL
    Vein_collision = Vein.addChild('Vein_collision')
    Vein_collision.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Vein_Low_Res.obj")
    Vein_collision.addObject('MeshTopology', src="@loader")
    Vein_collision.addObject('MechanicalObject', name="Vein_model")
    Vein_collision.addObject('TriangleCollisionModel')
    Vein_collision.addObject('LineCollisionModel')
    Vein_collision.addObject('PointCollisionModel')
    Vein_collision.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@Vein_model")

    # Pumonary Vein VISUAL MODEL
    Vein_visual = Vein.addChild('Vein_visual')
    
    Vein_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Vein_Mid_Res.obj")
    Vein_visual.addObject('OglModel', name="VisualModel", material="Default Diffuse 1 1 0 0 1 Ambient 1 0.2 0 0 1 Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45", src="@loader",color=[1, 0.05, 0.3])
    #Vein_visual.addObject('OglModel', name="VisualModel", src="@loader", texturename="/home/darkn117/Desktop/texture-blood-vessel_36888-212.jpg")
    Vein_visual.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")


    # ADD Setup_Base model
    Base = root.addChild('Base')

    # Setup_Base VISUAL MODEL
    Base_visual = Base.addChild('Visual Model')
    
    Base_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Setup_Base_Mid_Res.obj")
    Base_visual.addObject('OglModel', name="VisualModel", material="Default Diffuse 1 1 0 0 1 Ambient 1 0.2 0 0 1 Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45", src="@loader", color=[0.25, 0.5, 1])


    #ADD Gripper model
    Gripper = root.addChild('Gripper')
    #Mechanical object containing the 2 jaws articulations angles
    angle=-0.2
    Gripper.addObject('MechanicalObject', name="Articulations", template="Vec1d", position=[angle,angle*(-1)])
    
    Gripper_Parts = Gripper.addChild('Gripper_Parts')
    #Mechanical object containing the 3 position/orientation of the base, left jaw and right jaw
    Gripper_Parts.addObject('EulerImplicitSolver')
    Gripper_Parts.addObject('CGLinearSolver', iterations="200", threshold="1e-20", tolerance="1e-20"  )
    Gripper_Parts.addObject('MechanicalObject', name="GripperDOFs", template="Rigid3d", 
    
    position=
    [
	    [0, 0.02, -0.014, 0, 0, 0, 1] ,
	    [0, 0.02, -0.014, 0, 0, 0, 1] ,
	    [0, 0.02, -0.014, 0, 0, 0, 1]	],
    translation=
    [
	    [0, 0, 0] , 
	    [0, 0, 0] ,
	    [0, 0, 0]   ],
    rotation=
    [
	    [0, 0, 0] ,
	    [0, 0, 0] ,
	    [0, 0, 0]	]
    
    )
    Gripper_Parts.addObject('RestShapeSpringsForceField', name="RestShapeSFF", stiffness="1000", angularStiffness="1.5", external_rest_shape="@../Gripper/Articulations")
    Gripper_Parts.addObject('UniformMass', template="Rigid3d", name="mass", totalMass="0", showAxisSizeFactor="0.005") # Remove mass to mimick gravity compensation and to avoid computational issues
    #Gripper_Parts.addObject('LinearMovementConstraint', template="Rigid3", keyTimes="0 1 2", movements="0 0 0  0 0 0   0 0.005 0.0015  -0.35 0 0   0 0.01 0.003  -0.35 0 0")
		
    
    Shaft_Visual = Gripper_Parts.addChild('Shaft_Visual')
    Shaft_Visual.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_Base_High_Res.obj")
    Shaft_Visual.addObject('OglModel', name="model", src="@meshloader")
    Shaft_Visual.addObject('RigidMapping', input="@..", output="@model", index="0")
    
    Right_Jaw_Visual = Gripper_Parts.addChild('Right_Jaw_Visual')
    Right_Jaw_Visual.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_High_Res.obj")
    Right_Jaw_Visual.addObject('OglModel', name="model", src="@meshloader")
    Right_Jaw_Visual.addObject('RigidMapping', input="@..", output="@model", index="1")
    
    Left_Jaw_Visual = Gripper_Parts.addChild('Left_Jaw_Visual')
    Left_Jaw_Visual.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_High_Res.obj")
    Left_Jaw_Visual.addObject('OglModel', name="model", src="@meshloader")
    Left_Jaw_Visual.addObject('RigidMapping', input="@..", output="@model", index="2")
    
    Right_Jaw_Collision = Gripper_Parts.addChild('Right_Jaw_Collision')
    Right_Jaw_Collision.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_Collision_High_Res.obj")
    Right_Jaw_Collision.addObject('MeshTopology', src="@meshloader")
    Right_Jaw_Collision.addObject('MechanicalObject', name="model", src="@meshloader")
    Right_Jaw_Collision.addObject('TriangleCollisionModel',  moving="1", simulated="1", group="1")
    Right_Jaw_Collision.addObject('LineCollisionModel',  moving="1", simulated="1", group="1")
    Right_Jaw_Collision.addObject('PointCollisionModel', moving="1", simulated="1", group="1")
    Right_Jaw_Collision.addObject('RigidMapping', input="@..", output="@model", index="1")
    
    Left_Jaw_Collision = Gripper_Parts.addChild('Left_Jaw_Collision')
    Left_Jaw_Collision.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_Collision_High_Res.obj")
    Left_Jaw_Collision.addObject('MeshTopology', src="@meshloader")
    Left_Jaw_Collision.addObject('MechanicalObject', name="model", src="@meshloader")
    Left_Jaw_Collision.addObject('TriangleCollisionModel',  moving="1", simulated="1", group="1")
    Left_Jaw_Collision.addObject('LineCollisionModel',  moving="1", simulated="1", group="1")
    Left_Jaw_Collision.addObject('PointCollisionModel', moving="1", simulated="1", group="1")
    Left_Jaw_Collision.addObject('RigidMapping', input="@..", output="@model", index="2")
    
    Gripper_Parts.addObject('ArticulatedSystemMapping', input1="@../Articulations", output="@GripperDOFs")
    
    Gripper.addObject('ArticulatedHierarchyContainer')
    
    Articulation_Centers = Gripper.addChild('Articulation_Centers')
    
    #Articulation_Center_Base = Articulation_Centers.addChild('Articulation_Center_Base') 
    #Articulation_Center_Base.addObject('ArticulationCenter', parentIndex="0", childIndex="0", posOnParent="0 0 0", posOnChild="0 0 0")
    #Base_Articulation = Articulation_Center_Base.addChild('Base_Articulation') 
    #Base_Articulation.addObject('Articulation', translation="1", rotation="0", rotationAxis="1 1 1", articulationIndex="0")
    
    Articulation_Center_Right = Articulation_Centers.addChild('Articulation_Center_Right') 
    Articulation_Center_Right.addObject('ArticulationCenter', parentIndex="0", childIndex="1", posOnParent="-0.0011 0 0.0005", posOnChild="0 0 0")
    Right_Articulation = Articulation_Center_Right.addChild('Right_Articulation') 
    Right_Articulation.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="0")

    Articulation_Center_Left = Articulation_Centers.addChild('Articulation_Center_Left') 
    Articulation_Center_Left.addObject('ArticulationCenter', parentIndex="0", childIndex="2", posOnParent="0.0011 0 0.0005", posOnChild="0 0 0")
    Left_Articulation = Articulation_Center_Left.addChild('Left_Articulation') 
    Left_Articulation.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="1")
    
    
    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
