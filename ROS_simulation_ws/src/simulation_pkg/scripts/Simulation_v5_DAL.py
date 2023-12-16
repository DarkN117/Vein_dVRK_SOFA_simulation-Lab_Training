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

t_values = []
pos_values = []
plot = 0

import geometry_msgs.msg # PoseStamped message from PSM pose (position+quaternion)
import sensor_msgs.msg # JointState message from PSM jaws angle (single value)

pose_stamped = geometry_msgs.msg.PoseStamped()
angle_info = sensor_msgs.msg.JointState()


def position_plotter():
    global t_values
    global pos_values
    #print(t_values)
    #print(pos_values)
    plt.plot(t_values, pos_values)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.title('Vein node 240 x coordinate over time')
    plt.show()

    
def jaws_angle_callback(angle_msg):
    global angle_info
    angle_info = angle_msg
    #rospy.loginfo('The copied dVRK gripper angle is:\n' + str()+'\n')
    
def pose_stamped_callback(pose_stamped_msg):
    global pose_stamped
    pose_stamped = pose_stamped_msg
    #rospy.loginfo('The copied MTML pose is:\n' + str(pose_stamped) + '\n')	


def main():
	
	import SofaRuntime
	import Sofa.Gui
	global plot
	
	rospy.init_node('simulation_node', anonymous=True)
	rospy.Subscriber("dvrk/PSM2/position_cartesian_current", geometry_msgs.msg.PoseStamped, pose_stamped_callback)
	rospy.Subscriber("dvrk/PSM2/state_jaw_current", sensor_msgs.msg.JointState, jaws_angle_callback)
	
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
	
	if plot == 1:
		plot = 0
		position_plotter()    



class GripperPoseAngleController(Sofa.Core.Controller):

    """ This is a custom controller to perform actions on the simulation in real time"""
    def __init__(self, *args, **kwargs):
    
    	# These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = kwargs["root"]
        #print("Python::__init__::" + str(self.name.value))
        self.iteration = 0
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.up = 0
        self.down = 0
        self.i = 0
        self.shift_distance = 0.001
        self.spawn_grid = 0
        global plot
        
    def onAnimateBeginEvent(self, eventType): # called at each begin of animation step
        
        #print("\n#of elements of vein and right jaw colliding: "+ str(self.root.TestContactListener_1.getNumberOfContacts()))
        #print("\n#of elements of vein and left jaw colliding: "+ str(self.root.TestContactListener_2.getNumberOfContacts()))
        
        if ( (self.x_axis == 1 or self.y_axis == 1 or self.z_axis == 1) and (self.up == 1 or self.down == 1) ):
        	for i in range(316):  #vein mechanical model has 315 nodes
        		with self.root.Vein.dofs.position.writeableArray() as vein_node_pos:
        			vein_node_pos[i][0] += (self.x_axis * (self.up - self.down)) * self.shift_distance
        			vein_node_pos[i][1] += (self.y_axis * (self.up - self.down)) * self.shift_distance
        			vein_node_pos[i][2] += (self.z_axis * (self.up - self.down)) * self.shift_distance
        			#print(vein_node_pos)
        	with self.root.Base.Base_point.position.writeableArray() as base_pos:
        		base_pos += [(self.x_axis * (self.up - self.down)) * self.shift_distance, (self.y_axis * (self.up - self.down)) * self.shift_distance, (self.z_axis * (self.up - self.down)) * self.shift_distance, 0, 0, 0, 0]
        		#print(base_pos)
        		
        if self.spawn_grid == 1:
        	#visual grid has L="size" so has a "size"*"size" square area. Each cube of the grid has "size"/"nbSubdiv" side lenght:
        	self.root.Vein.Vein_visual.addObject('VisualGrid', plane="z", size=0.5, nbSubdiv=50, thickness=0.1, color=[0.8, 1, 0.8])#color=[0.5, 0.9, 0.1]
        	self.spawn_grid = 0
        
        
    def onAnimateEndEvent(self, event): # called at each end of animation step
        
        global t_values
        global pos_values
        
        time = self.root.time.value
        if self.iteration==0: #solve a bug about time display at initializiation
        	time = 0
        t_values.append(self.root.time.value)
        pos_values.append(self.root.Vein.dofs.position.value[240][0])
        
        print("\n:::::::::: Data from dVRK PSM ::::::::::\n____\nPOSE:\n"+str(pose_stamped)+"\n____\nJAWS ANGLE:\n"+str(angle_info)+"\n____\n\n")
        
        with self.root.Gripper.Gripper_Parts.GripperDOFs.position.writeableArray() as gripperPose:
        		gripperPose[0] = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z, pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
        		
        with self.root.Gripper.Articulations.position.writeableArray() as gripperAngle:
        		###print("\n\n"+str(angle_info.position[0])+"\n\n")
        		gripperAngle[0,0]= (angle_info.position[0]*(-1)) + 0.3
        		gripperAngle[1,0]= angle_info.position[0] - 0.3
        		
        
        print( "\n====================\nGripper Components Pose: \n" + "\nBase : " + str(self.root.Gripper.Gripper_Parts.GripperDOFs.position.value[0]) + "\n\nR_Jaw : " + str(self.root.Gripper.Gripper_Parts.GripperDOFs.position.value[1])+ "\n\nL_Jaw : " + str(self.root.Gripper.Gripper_Parts.GripperDOFs.position.value[2]) )
        
        print( "\n====================\nGripper Jaws angle: \n" + "\nR_Jaw : " + str(self.root.Gripper.Articulations.position.value[0]) + "\n\nL_Jaw : " + str(self.root.Gripper.Articulations.position.value[1]) )
        
        print( "\n++++++++++++++++++++\nVein mesh point index 240 position:" + "\n x: " + str(self.root.Vein.dofs.position.value[240][0])+ " [m]\n y: " + str(self.root.Vein.dofs.position.value[240][1])+ " [m]\n z: " + str(self.root.Vein.dofs.position.value[240][2])+ " [m]" )
        
        print("\n:::::::::: Time elapsed: " + str(time) + " [s], iteration #" + str(self.iteration)+ " ::::::::::")
        self.iteration += 1
        
        
    def onEvent(self, event):    
    
        #print ("Different event in Sofa : "+str(event))
        simulationStop = "'type': 'SimulationStartEvent', 'isHandled': False" in str(event)
        if simulationStop==True:
        	print("\n---------------------------\nSIMULATION HAS BEEN STOPPED\n---------------------------")
        #if(event(type) == 'SimulationStartEvent'):
        #	print("Simulation has ended")
        pass


    def onKeypressedEvent(self, c):
    
        global plot
        
        key = c['key']
        
        if key == "0":
            print("---You can shift along x---")
            self.x_axis = 1
            self.y_axis = 0
            self.z_axis = 0
        if key == "1":
            print("---You can shift along y---")
            self.x_axis = 0
            self.y_axis = 1
            self.z_axis = 0
        if key == "2":
            print("---You can shift along y---")
            self.x_axis = 0
            self.y_axis = 0
            self.z_axis = 1
            
        if key =="3":
            self.spawn_grid = 1
            
        if key=="4":
            plot = 1
            
        if ord(key) == 19:  # up
            print("Shifting along axis positive side")
            self.up = 1 

        if ord(key) == 21:  # down
            print("Shifting along axis negative side")
            self.down = 1
 


    def onKeyreleasedEvent(self, c):
    
        key = c['key']
            
        if ord(key) == 19:  # up
            print("Positive side along axis shifting")
            self.up = 0 

        if ord(key) == 21:  # down
            print("Negative side along axis shifting")
            self.down = 0



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
    root.addObject('LocalMinDistance', name="LMD-proximity" , alarmDistance=0.0005, contactDistance=0.0001, useLMDFilters=0)
    root.addObject('DefaultContactManager', name="CollisionResponse")
    
    ###UNKNOWN ISSUES WITH THE LIGHTS IN THE SCENE WHEN WE RUN THE PROGRAM THROUGH PYTHON (no issue when we run with "runsofa"
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
    Base.addObject('EulerImplicitSolver', name="odesolver",rayleighStiffness="0.8", rayleighMass="0.01")
    Base.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
    #Base.addObject('MechanicalObject', name="Base_point", template="Rigid3d")
    Base.addObject('MechanicalObject', name="Base_point", template="Rigid3d")
    Base.addObject('GenericConstraintCorrection', name="Gripper_ConstraintCorrection")
    # Setup_Base VISUAL MODEL
    Base_visual = Base.addChild('Visual Model')

    Base_visual.addObject('MeshOBJLoader', name="loader", filename="mesh/Simulation_meshes/Final_Centered_meshes/Setup_Base_Mid_Res.obj")
    Base_visual.addObject('OglModel', name="VisualModel", src="@loader", color=[0.25, 0.5, 1])
    Base_visual.addObject('RigidMapping', name="VisualMapping", input="@../Base_point", output="@VisualModel")



    #ADD Gripper model
    Gripper = root.addChild('Gripper')
    #Mechanical object containing the 2 jaws articulations angles
    Gripper.addObject('MechanicalObject', name="Articulations", template="Vec1d", position=[0,0])
    
    Gripper_Parts = Gripper.addChild('Gripper_Parts')
    #Mechanical object containing the 3 position/orientation of the base, left jaw and right jaw
    Gripper_Parts.addObject('EulerImplicitSolver')
    Gripper_Parts.addObject('CGLinearSolver', iterations="200", threshold="1e-20", tolerance="1e-20"  )
    Gripper_Parts.addObject('MechanicalObject', name="GripperDOFs", template="Rigid3d", 
    
    position=
    [
	    [0, 0, -0.02, 0, 0, 0, 1] ,
	    [0, 0, -0.02, 0, 0, 0, 1] ,
	    [0, 0, -0.02, 0, 0, 0, 1]	],
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
    Gripper_Parts.addObject('UniformMass', template="Rigid3d", name="mass", totalMass="0", showAxisSizeFactor="0.005") # Remove mass to mimick gravity compensation and to avoid computational issues
		
    
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
    
    Right_Jaw_Collision.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_R_Jaw_Collision_Low_Res_Surface.obj")
    Right_Jaw_Collision.addObject('MeshTopology', src="@meshloader")
    Right_Jaw_Collision.addObject('MechanicalObject', name="model", src="@meshloader")
    Right_Jaw_Collision.addObject('TriangleCollisionModel',  moving="1", simulated="1", group="1")
    Right_Jaw_Collision.addObject('LineCollisionModel',  moving="1", simulated="1", group="1")
    Right_Jaw_Collision.addObject('PointCollisionModel', moving="1", simulated="1", group="1")
    Right_Jaw_Collision.addObject('RigidMapping', input="@..", output="@model", index="1")
    
    Left_Jaw_Collision = Gripper_Parts.addChild('Left_Jaw_Collision')
    
    Left_Jaw_Collision.addObject('MeshObjLoader', name="meshloader", filename="mesh/Simulation_meshes/Final_Centered_meshes/PSM_L_Jaw_Collision_Low_Res_Surface.obj")
    Left_Jaw_Collision.addObject('MeshTopology', src="@meshloader")
    Left_Jaw_Collision.addObject('MechanicalObject', name="model", src="@meshloader")
    Left_Jaw_Collision.addObject('TriangleCollisionModel',  moving="1", simulated="1", group="1")
    Left_Jaw_Collision.addObject('LineCollisionModel',  moving="1", simulated="1", group="1")
    Left_Jaw_Collision.addObject('PointCollisionModel', moving="1", simulated="1", group="1")
    Left_Jaw_Collision.addObject('RigidMapping', input="@..", output="@model", index="2")
    
    
    Gripper_Parts.addObject('ArticulatedSystemMapping', input1="@../Articulations", output="@GripperDOFs")
    
    
    Gripper.addObject('ArticulatedHierarchyContainer')
    
    
    Articulation_Centers = Gripper.addChild('Articulation_Centers')
    
    Articulation_Center_Right = Articulation_Centers.addChild('Articulation_Center_Right') 
    Articulation_Center_Right.addObject('ArticulationCenter', parentIndex="0", childIndex="1", posOnParent="-0.0011 0 0.0005", posOnChild="0 0 0")
    Right_Articulation = Articulation_Center_Right.addChild('Right_Articulation') 
    Right_Articulation.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="0")

    Articulation_Center_Left = Articulation_Centers.addChild('Articulation_Center_Left') 
    Articulation_Center_Left.addObject('ArticulationCenter', parentIndex="0", childIndex="2", posOnParent="0.0011 0 0.0005", posOnChild="0 0 0")
    Left_Articulation = Articulation_Center_Left.addChild('Left_Articulation') 
    Left_Articulation.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="1")


    #should be used to detect when two objects enter in contact, for example to stop jaw rotation...still to be understood better...
    listener_1 = root.addObject(
        "ContactListener",
        name="TestContactListener_1",
        collisionModel1=Vein_collision.LineCollisionModel.getLinkPath(),
        collisionModel2=Gripper.Gripper_Parts.Right_Jaw_Collision.LineCollisionModel.getLinkPath() )
    listener_2 = root.addObject(
        "ContactListener",
        name="TestContactListener_2",
        collisionModel1=Vein_collision.LineCollisionModel.getLinkPath(),
        collisionModel2=Gripper.Gripper_Parts.Left_Jaw_Collision.LineCollisionModel.getLinkPath() )
        
    # If script is loaded by SOFA (runSofa), this script must be added as a controller
    if not _runAsPythonScript:
        root.addObject(GripperPoseAngleController(name="GripperController", root=root))

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass
