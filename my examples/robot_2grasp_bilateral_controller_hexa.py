# Robot-robot manipulation of deformable object: simulation scene
# Using constraint type BilateralInteractionConstraint
# Using SofaPython3 controller
# A. Koessler 2022-04-05
# coding: utf8
import Sofa
import numpy as np

def createScene(node):
  node.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [LinearSolverConstraintCorrection]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Model') # Needed to use components [BilateralInteractionConstraint]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
  node.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [MeshMatrixMass]  
  node.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
  node.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [HexahedronFEMForceField]  
  node.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid') # Needed to use components [RegularGridTopology]  
  node.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]
  node.addObject("VisualStyle", displayFlags="hideBehaviorModels showForceFields")
	
  node.addObject("FreeMotionAnimationLoop")
  node.addObject("GenericConstraintSolver", maxIterations="1000", tolerance="0.001")
  node.gravity = [0, 0, -9.81] # Set gravity towards -Z
	
  # Deformable object
  system = node.addChild("system")
  deformable = system.addChild("deformable")
  deformable.addObject("EulerImplicitSolver", name="odeSolver", rayleighStiffness="0.1",  rayleighMass="0.1", vdamping="1.0")
  deformable.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixd")
  deformable.addObject('RegularGridTopology', name='topology', n=[12, 12, 3] , min=[-0.5, 0.0, -0.05], max=[0.5, 1.0, 0.05])
  deformable.addObject('MeshTopology',name='foamMesh', src="@topology")
  deformable.addObject('MechanicalObject', template='Vec3d',name='deformableMO')
  deformable.addObject("LinearSolverConstraintCorrection", template="Vec3d")
  deformable.addObject('MeshMatrixMass', massDensity='50')
  deformable.addObject('HexahedronFEMForceField', name='HexaFF', src="@foamMesh", poissonRatio=0.3, youngModulus=10000, method="polar")
  defConstraint = deformable.addChild("defConstraint")
  defConstraint.addObject("MechanicalObject", name="constraintMO", template="Vec3d", position="-0.1 0.9 0 0.1 0.9 0 -0.1 1.0 0.0 0.1 1.0 0.0")
  defConstraint.addObject("BarycentricMapping", input="@..", output="@constraintMO")
  defConstraint2 = deformable.addChild("defConstraint2")
  defConstraint2.addObject("MechanicalObject", name="constraintMO", template="Vec3d", position="-0.1 0.1 0 0.1 0.1 0 -0.1 0.0 0.0 0.1 0.0 0.0")
  defConstraint2.addObject("BarycentricMapping", input="@..", output="@constraintMO")
	
	
	
  # Rigid motions of gripper 1
  gripper = system.addChild("gripper") # No solver, mass or FF -> this MO will not be moved by external actions, only by specifying new position values
  gripper.addObject("MechanicalObject", template="Rigid3d", name="gripperMO", position="0.0 1.0 0.0 0.098806 0.0 0.0 0.995106", showObject="1")
  #gripper.addObject("ReadState", filename="gripper1Disp.data") # Now we use the controller instead
  gripConstraint = gripper.addChild("gripConstraint")
  gripConstraint.addObject("MechanicalObject", name="constraintMO", template="Vec3d", position="-0.1 -0.1 0.0 0.1 -0.1 0.0 -0.1 0.0 0.0 0.1 0.0 0.0") # Define points in space where the constraint is projected
  gripConstraint.addObject("RigidMapping") # Map the motions of points in constraintMO with rigid motions of gripperMO



  # Rigid motions of gripper 2
  gripper2 = system.addChild("gripper2") # No solver, mass or FF -> this MO will not be moved by external actions, only by specifying new position values
  gripper2.addObject("MechanicalObject", template="Rigid3d", name="gripperMO", position="0.0 0.0 0.0 -0.098806 0.0 0.0 0.995106", showObject="1")
  #gripper2.addObject("ReadState", filename="gripper2Disp.data") # Now we use the controller instead
  gripConstraint2 = gripper2.addChild("gripConstraint")
  gripConstraint2.addObject("MechanicalObject", name="constraintMO", template="Vec3d", position="-0.1 0.1 0.0 0.1 0.1 0.0 -0.1 0.0 0.0 0.1 0.0 0.0") # Define points in space where the constraint is projected
  gripConstraint2.addObject("RigidMapping") # Map the motions of points in constraintMO with rigid motions of gripperMO



  # Apply constraints
  system.addObject("BilateralInteractionConstraint", template="Vec3d", object2="@deformable/defConstraint/constraintMO", object1="@gripper/gripConstraint/constraintMO", first_point="0 1 2 3", second_point="0 1 2 3")
  system.addObject("BilateralInteractionConstraint", template="Vec3d", object2="@deformable/defConstraint2/constraintMO", object1="@gripper2/gripConstraint/constraintMO", first_point="0 1 2 3", second_point="0 1 2 3")



  # Use controller to move the grippers
  gripper.addObject( ControlParticles(name="MouvementPinces", Particle1=gripper.gripperMO, Particle2=gripper2.gripperMO, root=node) )
  return 0


class ControlParticles(Sofa.Core.Controller):
	def __init__(self, *args, **kwargs):
		# These are needed (and the normal way to override from a python class)
		Sofa.Core.Controller.__init__(self, *args, **kwargs)
		self.Particle1 = kwargs.get("Particle1")
		self.Particle2 = kwargs.get("Particle2")
		self.root = kwargs.get("root")
		self.time = 0.0

	def onAnimateBeginEvent(self, event): # Compute gripper poses for the next time step
		self.time += self.root.dt.value
		#print('Time: ' + str(self.time)) # For debug
		pos1, pos2 = generator(self.time)
		#print('Computed poses, pos1: ' + str(pos1)) # For debug
		with self.Particle1.position.writeableArray() as wa1:
			wa1[0] = pos1
		with self.Particle2.position.writeableArray() as wa2:
			wa2[0] = pos2
		return 0;

	def onKeypressedEvent(self, event):
		key = event['key']
		if key=="-" :
			print("Reset key was pressed!")
			self.time = 0.0
		return 0


def generator(t,tmin=0, tmax=5): # Makes the same motion than the data files
	t = (t-tmin)/(tmax-tmin)
	if t>1:
		t=1
	y1 = 1 - 0.05*(10*t**3 - 15*t**4 + 6*t**5)
	y2 = 0.05*(10*t**3 - 15*t**4 + 6*t**5)
	qw = 1 - 0.05*(10*t**3 - 15*t**4 + 6*t**5)
	qx1 = np.sqrt(1-qw**2)
	qx2 = -np.sqrt(1-qw**2)
	pos1 = [0, y1, 0, qx1, 0, 0, qw]
	pos2 = [0, y2, 0, qx2, 0, 0, qw]
	return pos1, pos2


def main():
	import SofaRuntime
	import Sofa.Gui
	# Make sure to load all SOFA libraries
	SofaRuntime.importPlugin("SofaBaseMechanics")
	SofaRuntime.importPlugin("SofaOpenglVisual")

	#Create the root node
	root = Sofa.Core.Node("root")
	# Call the below 'createScene' function to create the scene graph
	createScene(root)
	Sofa.Simulation.init(root)

	# Launch the GUI (qt or qglviewer)
	Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
	Sofa.Gui.GUIManager.createGUI(root, __file__)
	Sofa.Gui.GUIManager.SetDimension(1080, 1080)
	# Initialization of the scene will be done here
	Sofa.Gui.GUIManager.MainLoop(root)
	Sofa.Gui.GUIManager.closeGUI()
	print("Simulation is done.")


# Function used only if this script is called from a python environment
if __name__ == '__main__':
	main()
