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
    root.dt = 0.1
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
                                                 'Sofa.Component.Setting'  # Needed to use components [BackgroundSetting  
                                                 ])


    root.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    root.addObject('VisualStyle', displayFlags="hideCollisionModels")
    root.addObject('BackgroundSetting', color="0.7 1 0.7") 
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('GenericConstraintSolver', resolutionMethod="false", tolerance="1e-4", maxIterations="250", multithreading="true")
    root.addObject('CollisionPipeline', name="CollisionPipeline", verbose="0")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    #root.addObject('DiscreteIntersection')
    root.addObject('MinProximityIntersection', name="Proximity" , alarmDistance=0.0001, contactDistance=0.00005)
    #root.addObject('DefaultContactManager', name="CollisionResponse", response="FrictionContactConstraint")
    root.addObject('DefaultContactManager', name="CollisionResponse", response="FrictionContactConstraint")
    #root.addObject('LightManager', name="lightManager", listening="1", shadows="1", softShadows="0") #use hard shadow: are less accurate but more computationally convenient
    #root.addObject('OglShadowShader', name="oglShadowShader" )
    #root.addObject('SpotLight', name="light3", color="1 1 1", shadowTextureSize="512", position="0 1 1", shadowFactor="1", cutoff="1000", exponent="1")

    Vein_Mass = 0.01
    #volume = 1.0
    #inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]




    # ADD Pulmonary_Vein model
    Vein = root.addChild('Vein')

    # Pumonary Vein MECHANICAL MODEL
    Vein.addObject('EulerImplicitSolver', name="odesolver",rayleighStiffness="0.75", rayleighMass="0.01")
    Vein.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
    Vein.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Vein_low_res.msh")
    Vein.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    Vein.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    Vein.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    Vein.addObject('MeshMatrixMass', name="mass", totalMass= "0.01", showAxisSizeFactor="0.005") #showAxisSizeFactor="0.005" allow to scale down the local object axis representation
    Vein.addObject('TetrahedralCorotationalFEMForceField',template="Vec3d", youngModulus=200000, poissonRatio=0.4, method="large", computeGlobalMatrix="0", updateStiffnessMatrix="false")
    #box_left= root.addObject('BoxROI', template="Vec3d", name='box_roi_1', box=[-0.0585, -0.01, -0.008, -0.0605, 0.010, 0.010], drawBoxes='True', drawSize="1")
    #box_right = root.addObject('BoxROI', template="Vec3d", name='box_roi_2', box=[0.0585, -0.010, -0.008, 0.0605, 0.01, 0.010], drawBoxes='True', drawSize="1")
    Vein.addObject('FixedConstraint', indices="0 3 4 67 68 71 72 75 77 139 140 142 144 147 148 211 212 215 216 219 221 283 284 286 288 289")
    #Vein.addObject('FixedConstraint', template="Vec3d", name="ROIindices_2", indices="@box_roi_2.indices", showObject="True", drawSize="0.3")
    Vein.addObject('GenericConstraintCorrection', name="Vein_ConstraintCorrection")

    # Pumonary Vein COLLISION MODEL
    Vein_collision = Vein.addChild('Vein_collision')
    Vein_collision.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Vein_low_res.obj")
    Vein_collision.addObject('MeshTopology', src="@loader")
    Vein_collision.addObject('MechanicalObject', name="Vein_model")
    Vein_collision.addObject('TriangleCollisionModel')
    Vein_collision.addObject('LineCollisionModel')
    Vein_collision.addObject('PointCollisionModel')
    Vein_collision.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@Vein_model")

    # Pumonary Vein VISUAL MODEL
    Vein_visual = Vein.addChild('Vein_visual')
    
    Vein_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Vein_high_res.obj")
    Vein_visual.addObject('OglModel', name="VisualModel", material="Default Diffuse 1 1 0 0 1 Ambient 1 0.2 0 0 1 Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45", src="@loader",color=[1, 0.1, 0.3])
    #Vein_visual.addObject('OglModel', name="VisualModel", src="@loader", texturename="/home/darkn117/Desktop/texture-blood-vessel_36888-212.jpg")
    Vein_visual.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")


    # ADD Setup_Base model
    Base = root.addChild('Base')

    # Setup_Base VISUAL MODEL
    Base_visual = Base.addChild('Visual Model')
    
    Base_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Setup_Base.obj")
    Base_visual.addObject('OglModel', name="VisualModel", material="Default Diffuse 1 1 0 0 1 Ambient 1 0.2 0 0 1 Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45", src="@loader", color=[1, 1, 1])




    #ADD Gripper model
    Gripper = root.addChild('Gripper')
    
    
    #ADD Gripper Base model

    Gripper_Base = Gripper.addChild('Gripper_Base')
    
    # Gripper Base MECHANICAL MODEL
    Gripper_Base_Mech = Gripper_Base.addChild('Gripper_Base_Mech')
    
    Gripper_Base_Mech.addObject('EulerImplicitSolver', name="odesolver",rayleighStiffness=0)
    Gripper_Base_Mech.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
    #Gripper_Base_Mech.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    Gripper_Base_Mech.addObject('MechanicalObject', name="Gripper_Base_Mech", template='Rigid3d', position=[0, 0, 0, 0.707, 0, 0, 0.707], translation=[0, 0.025, 0], rotation=[0, 0, 0])
    #Gripper_Base_Mech.addObject('TetrahedronSetGeometryAlgorithms', template="Rigid3d", name="GeomAlgo")
    Gripper_Base_Mech.addObject('UniformMass', totalMass = "0", showAxisSizeFactor="0.005") ###Don't give mass to the gripper for the moment
    #Gripper_Base_Mech.addObject('ConstantForceField', force=[0,0,0,-0.002,0,0])
    Gripper_Base_Mech.addObject('GenericConstraintCorrection', name="R_Gripper_ConstraintCorrection")
    #Gripper_Base_Mech.addObject('LinearMovementConstraint', template="Rigid3d", keyTimes=[5], movements=[0,10,0,0,1,0])
    Gripper_Base_Mech.addObject('LinearMovementConstraint', template="Rigid3d")

    # Gripper Base VISUAL MODEL
    Gripper_Base_Visual = Gripper_Base.addChild('Gripper_Base_Visual')
    
    Gripper_Base_Visual.addObject('MeshObjLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_Base_High_Res.obj")
    Gripper_Base_Visual.addObject('MeshTopology', src="@loader")
    Gripper_Base_Visual.addObject('OglModel', name="Visual_Model", color = [0.3,0.5,0.9])
    Gripper_Base_Visual.addObject('RigidMapping', name="VisualMapping", input="@../Gripper_Base_Mech", output="@Visual_Model")
    
    
    
    #ADD Right Gripper model

    R_Gripper = Gripper.addChild('R_Gripper')
    
    # Right Gripper MECHANICAL MODEL
    R_Gripper_Mech = R_Gripper.addChild('R_Gripper_Mech')
    
    R_Gripper_Mech.addObject('EulerImplicitSolver', name="odesolver",rayleighStiffness=0)
    R_Gripper_Mech.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
    #R_Gripper_Mech.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    R_Gripper_Mech.addObject('MechanicalObject', name="R_Gripper_Mech", template='Rigid3d', position=[0, 0, 0, 0.707, 0, 0, 0.707], translation=[-0.001, 0.025, 0], rotation=[0, 0, 0])
    #R_Gripper_Mech.addObject('TetrahedronSetGeometryAlgorithms', template="Rigid3d", name="GeomAlgo")
    R_Gripper_Mech.addObject('UniformMass', totalMass = "0", showAxisSizeFactor="0.005") ###Don't give mass to the gripper for the moment
    #R_Gripper_Mech.addObject('ConstantForceField', force=[0,0,0,-0.002,0,0])
    R_Gripper_Mech.addObject('GenericConstraintCorrection', name="R_Gripper_ConstraintCorrection")
    #R_Gripper_Mech.addObject('LinearMovementConstraint', template="Rigid3d", keyTimes=[5], movements=[0,10,0,0,1,0])
    R_Gripper_Mech.addObject('LinearMovementConstraint', template="Rigid3d")
    
    # Right Gripper COLLISION MODEL
    R_Gripper_Collision = R_Gripper.addChild('R_Gripper_Collision')
    
    R_Gripper_Collision.addObject('MeshObjLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_Low_Res.obj")
    R_Gripper_Collision.addObject('MeshTopology', src="@loader")
    R_Gripper_Collision.addObject('MechanicalObject', name="Collision_Model")
    R_Gripper_Collision.addObject("TriangleCollisionModel")
    R_Gripper_Collision.addObject("LineCollisionModel")
    R_Gripper_Collision.addObject("PointCollisionModel")
    R_Gripper_Collision.addObject("RigidMapping", input="@../R_Gripper_Mech", output="@Collision_Model")
    
    # Right Gripper VISUAL MODEL
    R_Gripper_Visual=R_Gripper.addChild('R_Gripper_Visual')
    
    R_Gripper_Visual.addObject('MeshObjLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_High_Res.obj")
    R_Gripper_Visual.addObject('MeshTopology', src="@loader")
    R_Gripper_Visual.addObject('OglModel', name="Visual_Model", color = [0.3,0.5,0.9])
    R_Gripper_Visual.addObject('RigidMapping', name="VisualMapping", input="@../R_Gripper_Mech", output="@Visual_Model")



    #ADD Left Gripper model

    L_Gripper = Gripper.addChild('L_Gripper')
    
    # Left Gripper MECHANICAL MODEL
    L_Gripper_Mech = L_Gripper.addChild('L_Gripper_Mech')
    
    L_Gripper_Mech.addObject('EulerImplicitSolver', name="odesolver",rayleighStiffness=0)
    L_Gripper_Mech.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
    L_Gripper_Mech.addObject('MechanicalObject', name="L_Gripper_Mech", template='Rigid3d', position=[0, 0, 0, 0.707, 0, 0, 0.707], translation=[0.001, 0.025, 0], rotation=[0, 0, 0])
    L_Gripper_Mech.addObject('UniformMass', totalMass = "0", showAxisSizeFactor="0.005") ###Don't give mass to the gripper for the moment
    #L_Gripper_Mech.addObject('ConstantForceField', force=[0,0,0,-0.002,0,0])
    L_Gripper_Mech.addObject('GenericConstraintCorrection', name="L_Gripper_ConstraintCorrection")
    #L_Gripper_Mech.addObject('LinearMovementConstraint', template="Rigid3d", keyTimes=[5], movements=[0,10,0,0,1,0])
    L_Gripper_Mech.addObject('LinearMovementConstraint', template="Rigid3d")
    
    # Left Gripper COLLISION MODEL
    L_Gripper_Collision=L_Gripper.addChild('L_Gripper_Collision')
    
    L_Gripper_Collision.addObject('MeshObjLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_Low_Res.obj")
    L_Gripper_Collision.addObject('MeshTopology', src="@loader")
    L_Gripper_Collision.addObject('MechanicalObject', name="Collision_Model")
    L_Gripper_Collision.addObject("TriangleCollisionModel", contactStiffness="10")
    L_Gripper_Collision.addObject("LineCollisionModel", contactStiffness="10")
    L_Gripper_Collision.addObject("PointCollisionModel", contactStiffness="10")
    L_Gripper_Collision.addObject("RigidMapping", input="@../L_Gripper_Mech", output="@Collision_Model")
    
    # Left Gripper VISUAL MODEL
    L_Gripper_Visual=L_Gripper.addChild('L_Gripper_Visual')
    
    L_Gripper_Visual.addObject('MeshObjLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_High_Res.obj")
    L_Gripper_Visual.addObject('MeshTopology', src="@loader")
    L_Gripper_Visual.addObject('OglModel', name="Visual_Model", color = [0.3,0.5,0.9])
    L_Gripper_Visual.addObject('RigidMapping', name="VisualMapping", input="@../L_Gripper_Mech", output="@Visual_Model")
    
    
    

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
