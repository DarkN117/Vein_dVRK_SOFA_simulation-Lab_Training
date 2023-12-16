import Sofa

def createScene(node):
	node.gravity="0 0 0"
	node.name="root"
	
	childNode=node.createChild("Particle")
	
	childNode.createObject('EulerImplicitSolver')
	childNode.createObject('CGLinearSolver', iterations="200", tolerance="1e-09")
	childNode.createObject('MechanicalObject', template="Rigid3d", name="myParticle", position="0 0 0 0 0 0 1", showObject="1")
	childNode.createObject('UniformMass', totalMass="1")
	childNode.createObject('ConstantForceField', name="CFF", totalForce="1 0 0 0 0 0" )
