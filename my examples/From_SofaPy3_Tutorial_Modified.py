import Sofa
import Sofa.Gui


def main():
# Call the SOFA function to create the root node
 root = Sofa.Core.Node("root")

# Call the createScene function, as runSofa does
 createScene(root)

# Once defined, initialization of the scene graph
 Sofa.Simulation.init(root)

# Launch the GUI (qt or qglviewer)
 Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
 Sofa.Gui.GUIManager.createGUI(root, __file__)
 Sofa.Gui.GUIManager.SetDimension(0.001, 0.001)

# Initialization of the scene will be done here
 Sofa.Gui.GUIManager.MainLoop(root)
 Sofa.Gui.GUIManager.closeGUI()


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='CGALPlugin') ###
    
    
    rootNode.addObject("VisualGrid", nbSubdiv=100, size=10) 

    # Define the root node properties
    rootNode.gravity=[0.0, 0.0 ,0.0]
    rootNode.dt=0.1

    # Loading all required SOFA modules
    confignode = rootNode.addChild("Config")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Solver", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.StateContainer", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Visual", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    confignode.addObject('RequiredPlugin', name='Sofa.Component.MechanicalLoad') # Needed to use components[ConstantForceField]
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")


    # Collision pipeline
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.0), name='Response', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)


    totalMass = 1.0
    volume = 1.0
    inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]




    # Creating the falling object

    sphere = rootNode.addChild("Fegato")
    sphere.addObject('EulerImplicitSolver', name='odesolver')
    sphere.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    sphere.addObject('MechanicalObject', name="mstate", template="Vec3d", translation2=[0., 0.5, 0.], rotation2=[0., 0., 0.], showObjectScale=50)
    sphere.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    sphere.addObject('ConstantForceField', name="CFF", totalForce="0 -1 0 0 0 0" )
    sphere.addObject('UncoupledConstraintCorrection')


    #### Collision subnode for the liver
    collision = sphere.addChild('collision')
    #####collision.addObject('MeshOBJLoader', name="loader", filename="mesh/ball.obj", triangulate="true", scale=45.0)
    collision.addObject('MeshGmshLoader', name="loader", filename="mesh/liver2.msh", triangulate="true", scale=1.0)#!!!!!!!!!!!!!!!!!!
    collision.addObject('MeshTopology', src="@loader")
    collision.addObject('MechanicalObject', name="mstate", translation2=[0., 0.5, 0.], rotation2=[0., 0., 0.], showObjectScale=50)
    collision.addObject('TriangleCollisionModel')
    collision.addObject('LineCollisionModel')
    collision.addObject('PointCollisionModel')
    collision.addObject('IdentityMapping')


    #### Visualization subnode for the liver
    sphereVisu = sphere.addChild("VisualModel")
    sphereVisu.loader = sphereVisu.addObject('MeshOBJLoader', name="loader", filename="mesh/liver2.obj")
    sphereVisu.addObject('OglModel', name="model", src="@loader", scale3d=[1]*3, color=[0., 1., 0.], updateNormals=False)
    sphereVisu.addObject('IdentityMapping')




    # Creating the floor object
    
    floor = rootNode.createChild('Banco+Vena') ###
    ###floor.addObject('Mesh',name='mesh',filename='mesh/Banco_Vena.msh') ###
    floor.addObject('EulerImplicitSolver', name='odesolver')
    floor.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    floor.addObject('MechanicalObject', name="mstate", template="Vec3d", translation2=[0., -1, 0.], rotation2=[0., 0., 0.], showObjectScale=50)
    floor.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    floor.addObject('UncoupledConstraintCorrection')
    #floor.createObject('MeshGenerationFromPolyhedron',inputPoints='@mesh.position', inputTriangles='@mesh.triangles', drawTetras='1') ###
    #floor.createObject('Mesh', position='@gen.outputPoints', tetrahedra='@gen.outputTetras') ###
    #floor.createObject('VTKExporter', filename='Setup v.0', edges='0', tetras='1', exportAtBegin='1') ###
    
    
    #### Collision subnode for the floor
    #floorCollis = floor.addChild('collision')
    #floorCollis.addObject('MeshGmshLoader', name="loader", filename="mesh/Banco_Vena.msh", triangulate="true", scale=5.0)
    #floorCollis.addObject('MeshTopology', src="@loader")
    #floorCollis.addObject('MechanicalObject', name="mstate", template="Vec3", translation2=[0., -1, 0.], rotation2=[0., 0., 0.], showObjectScale=50)
    #floorCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    #floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    #floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    #floorCollis.addObject('RigidMapping')


    #### Visualization subnode for the floor
    floorVisu = floor.addChild("VisualModel")
    floorVisu.loader = floorVisu.addObject('MeshOBJLoader', name="loader", filename="mesh/Setup v.0.obj")
    floorVisu.addObject('OglModel', name="model", src="@loader", scale3d=[5.0]*3, color=[1., 0., 0.], updateNormals=False)
    floorVisu.addObject('IdentityMapping')




    return rootNode


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
