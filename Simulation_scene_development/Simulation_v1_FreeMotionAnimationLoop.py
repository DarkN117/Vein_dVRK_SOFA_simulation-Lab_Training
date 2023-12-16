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
                                                 'Sofa.Component.Collision.Detection.Intersection', #Needed to use components [DiscreteIntersection]
                                                 'Sofa.Component.Collision.Geometry',
                                                 'Sofa.Component.Collision.Response.Contact', #Needed to use components [DefaultContactManager]
                                                 'Sofa.Component.Constraint.Projective', #Needed to use components [FixedConstraint]
                                                 'Sofa.Component.Constraint.Lagrangian.Correction',
                                                 'Sofa.Component.Constraint.Lagrangian.Solver',
                                                 'Sofa.Component.Engine.Select', # Needed to use components [BoxROI]
                                                 'Sofa.Component.IO.Mesh', #Needed to use components [MeshOBJLoader SphereLoader]
                                                 'Sofa.Component.LinearSolver.Iterative', #Needed to use components [CGLinearSolver]
                                                 'Sofa.Component.Mapping.Linear', #Needed to use components [BarycentricMapping]
                                                 'Sofa.Component.Mass', #Needed to use components [UniformMass]
                                                 'Sofa.Component.ODESolver.Backward', #Needed to use components [EulerImplicitSolver]
                                                 'Sofa.Component.SolidMechanics.FEM.Elastic', #Needed to use components [TriangleFEMForceField]
                                                 'Sofa.Component.StateContainer', #Needed to use components [MechanicalObject]
                                                 'Sofa.Component.Topology.Container.Dynamic',
                                                 'Sofa.Component.Topology.Container.Constant', # Needed to use components [MeshTopology]
                                                 'Sofa.Component.Visual', #Needed to use components [VisualStyle]
                                                 'Sofa.GL.Component.Rendering3D', #Needed to use components [OglModel]
                                                 'Sofa.Component.MechanicalLoad', #Needed to use components [ConstantForceField]
                                                 'Sofa.Component.SolidMechanics.Spring', #Needed to use components [MeshSpringForceField]])
                                                 'Sofa.Component.AnimationLoop', # Needed to use components [FreeMotionAnimationLoop]
                                                 'Sofa.Component.Mapping.NonLinear'  #Needed to use components [RigidMapping]
                                                 ])

    # Default Animation Loop+Reference Frame
    root.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    #root.addObject('DefaultAnimationLoop')
    root.addObject('FreeMotionAnimationLoop')


    # Add Collision pipeline
    root.addObject('VisualStyle', displayFlags="hideCollisionModels")
    root.addObject('CollisionPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    root.addObject('DefaultContactManager', name="CollisionResponse", response="PenalityContactForceField")
    #root.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    root.addObject('DiscreteIntersection')
    #root.addObject('LocalMinDistance', alarmDistance=0.001, contactDistance=0.0005, angleCone=0.01)
    #rootNode.addObject('RuleBasedContactManager', responseParams="mu=" + str(0.0), name='Response', response='FrictionContactConstraint')


    Vein_Mass = 0.01
    Base_Mass = 0.75
    #volume = 1.0
    #inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]

    # ADD Pulmonary_Vein model
    root.addObject('MeshOBJLoader', name="Vein_Surface", filename="mesh/Pulmonary_Vein_modified.obj") #seems redundant but is needed otherwise boxroi's do not work

    Vein = root.addChild('Vein')

    # Pumonary Vein MECHANICAL MODEL
    #Vein.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    Vein.addObject('EulerImplicitSolver', name="odesolver", vdamping="4.0"),
    Vein.addObject('CGLinearSolver', iterations="100", name="linear_solver", tolerance="1e-09", threshold="1e-09")
    Vein.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/Pulmonary_Vein_modified.msh")
    Vein.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    Vein.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    Vein.addObject('UniformMass', name="mass", vertexMass=[Vein_Mass])
    Vein.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    Vein.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    #Vein.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="30", computeGlobalMatrix="0")
    Vein.addObject('TriangleFEMForceField', name="linearElasticBehavior", youngModulus=10000, poissonRatio=0.45, method="large")
    #Vein.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="10000", tetrasDamping="10")
    box_left=root.addObject('BoxROI', template="Vec3d", name='box_roi_1', box=[-0.0585, 0.01, -0.002, -0.062, 0.030, 0.014], drawBoxes='True', drawSize="1")
    box_right = root.addObject('BoxROI', template="Vec3d", name='box_roi_2', box=[0.056, 0.01, -0.002, 0.059, 0.030, 0.014], drawBoxes='True', drawSize="1")
    Vein.addObject('FixedConstraint', template="Vec3d", name="ROIindices_1", indices="@box_roi_1.indices", showObject="True", drawSize="0.3")
    Vein.addObject('FixedConstraint', template="Vec3d", name="ROIindices_2", indices="@box_roi_2.indices", showObject="True", drawSize="0.3")

    # Pumonary Vein VISUAL MODEL
    Vein_visual = Vein.addChild('Visual Model')
    Vein_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Pulmonary_Vein_modified.obj")
    Vein_visual.addObject('OglModel', name="VisualModel", src="@loader",color=[0.8, 0.1, 0.3])
    Vein_visual.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

    # Pumonary Vein COLLISION MODEL
    Vein_collision = Vein.addChild('collision')
    Vein_collision.addObject('MeshOBJLoader', name="loader", filename="mesh/Pulmonary_Vein_modified.obj")
    Vein_collision.addObject('MeshTopology', src="@loader")
    Vein_collision.addObject('MechanicalObject', name="Vein_model")
    Vein_collision.addObject('TriangleCollisionModel')
    Vein_collision.addObject('LineCollisionModel')
    Vein_collision.addObject('PointCollisionModel')
    Vein_collision.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@Vein_model")



    # ADD Setup_Base model
    Base = root.addChild('Base')

    # Setup_Base MECHANICAL MODEL
    #Base.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    #Base.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    Base.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/Setup_Base.msh")
    #Base.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    Base.addObject('MechanicalObject', name="mstate", template="Rigid3")
    #Base.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    #Base.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    Base.addObject('UniformMass', name="mass", vertexMass=[Base_Mass])
    #Base.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="10000", tetrasDamping="10")

    # Setup_Base VISUAL MODEL
    Base_visual = Base.addChild('Visual Model')
    Base_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Setup_Base.obj")
    Base_visual.addObject('OglModel', name="VisualModel", src="@loader", color=[0.2, 0.3, 0.8])
    Base_visual.addObject('RigidMapping', name="VisualMapping")

    # Setup_Base COLLISION MODEL
    Base_collision = Base.addChild('collision')
    Base_collision.addObject('MeshOBJLoader', name="loader", filename="mesh/Setup_Base.obj")
    Base_collision.addObject('MeshTopology', src="@loader")
    Base_collision.addObject('MechanicalObject', name="Base_model")
    Base_collision.addObject('TriangleCollisionModel', moving=False, simulated=False)
    Base_collision.addObject('LineCollisionModel', moving=False, simulated=False)
    Base_collision.addObject('PointCollisionModel', moving=False, simulated=False)
    Base_collision.addObject('RigidMapping', name="CollisionMapping")


    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
