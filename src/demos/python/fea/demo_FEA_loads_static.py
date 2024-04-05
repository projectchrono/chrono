# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
import copy

print("Copyright (c) 2017 projectchrono.org ")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh:
mesh = fea.ChMesh()
sys.Add(mesh)

# Create some nodes.
nodeA = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
nodeB = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))

# Default mass for FEM nodes is zero
nodeA.SetMass(0.0)
nodeB.SetMass(0.0)

mesh.AddNode(nodeA)
mesh.AddNode(nodeB)

# Create beam section & material
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.1
beam_wz = 0.2
beam_section.SetAsRectangularSection(beam_wy, beam_wz)
beam_section.SetYoungModulus(0.01e9)
beam_section.SetShearModulus(0.01e9 * 0.3)
beam_section.SetRayleighDamping(0.200)
beam_section.SetDensity(1500)

# Create a beam of Eulero-Bernoulli type:
elementA = fea.ChElementBeamEuler()
elementA.SetNodes(nodeA, nodeB)
elementA.SetSection(beam_section)
mesh.AddElement(elementA)

# Create also a truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create a constraat the end of the beam
constraintA = chrono.ChLinkMateGeneric()
constraintA.Initialize(nodeA, truss, False, nodeA.Frame(), nodeA.Frame())
sys.Add(constraintA)
constraintA.SetConstrainedCoords(True, True, True,   # x, y, z
							     True, True, True)   # Rx, Ry, Rz

# APPLY SOME LOADS!

# First: loads must be added to "load containers",
# and load containers must be added to your system
load_container = chrono.ChLoadContainer()
sys.Add(load_container)

# Example 1:

# Add a vertical load to the end of the beam element:
wrench_load = fea.ChLoadBeamWrench(elementA)
wrench_load.GetLoader().SetApplication(1.0)  # in -1..+1 range, -1: end A, 0: mid, +1: end B
wrench_load.GetLoader().SetForce(chrono.ChVector3d(0, -0.2, 0))
load_container.Add(wrench_load)  # do not forget to add the load to the load container.

# Example 2:

# Add a distributed load along the beam element:
distr_wrench_load = fea.ChLoadBeamWrenchDistributed(elementA)
distr_wrench_load.GetLoader().SetForcePerUnit(chrono.ChVector3d(0, -0.1, 0))  # load per unit length
load_container.Add(distr_wrench_load)

# Example 3:

# Add gravity (constant volumetric load)
gravity_loader = chrono.ChLoaderGravity(elementA)
gravity_load = chrono.ChLoad(gravity_loader)
load_container.Add(gravity_load)

# note that by default all solid elements in the mesh will already
# get gravitational force, if you want to bypass this automatic gravity, do:
mesh.SetAutomaticGravity(False)

### TODO  Examples 4 & 5 in corresponding C++ demo

# Example 6:

# As before, create a custom load with stiff force, acting on a single node, but
# this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
# This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
# As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
# that will be used in statics, implicit integrators, etc.

nodeD = fea.ChNodeFEAxyz(chrono.ChVector3d(2, 10, 3))
mesh.AddNode(nodeD)

class MyLoadCustom(chrono.ChLoadCustom):
    def __init__(self, loadable):
        chrono.ChLoadCustom.__init__(self, loadable)
    # "Virtual" copy constructor (covariant return type).
    def Clone(self):
        newinst = copy.deepcopy(self)
        return  newinst

	# Compute Q=Q(x,v)
	# This is the function that you have to implement. It should return the generalized Q load
	# (i.e.the force in generalized lagrangian coordinates).
	# For ChNodeFEAxyz, Q loads are expected as 3-rows vectors, containing absolute force x,y,z.
	# As this is a stiff force field, dependency from state_x and state_y must be considered.
    def ComputeQ(self,state_x, # state position to evaluate Q
                 state_w):     # state speed to evaluate Q
        if not state_x==None and not state_w==None :
            node_pos = chrono.ChVector3d(state_x.GetItem(0), state_x.GetItem(1), state_x.GetItem(2))
            node_vel = chrono.ChVector3d(state_w.GetItem(0), state_w.GetItem(1), state_w.GetItem(2))
        else:
            mynode = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadable) ))
            node_pos = mynode.GetPos()
            node_vel = mynode.GetPosDt()

        # Just implement a simple force+spring+damper in xy plane,
        # for spring&damper connected to absolute reference:
        Kx = 100
        Ky = 400
        Dx = 0.6
        Dy = 0.9
        x_offset = 2
        y_offset = 10
        x_force = 50
        y_force = 0

        # Store the computed generalized forces in this.load_Q, same x,y,z order as in state_w
        self.load_Q.SetItem(0, x_force - Kx * (node_pos.x - x_offset) - Dx * node_vel.x)
        self.load_Q.SetItem(1, y_force - Ky * (node_pos.y - y_offset) - Dy * node_vel.y)
        self.load_Q.SetItem(2, 0.0)

	# Compute jacobian not available from Python

	# Set this as stiff, to enable the Jacobians
    def IsStiff(self) : 
        return True 

# Instance load object, applying to a node, as in previous example, and add to container:
custom_load = MyLoadCustom(nodeD)
load_container.Add(custom_load)

# Example 7:

# As before, create a custom load with stiff force, acting on MULTIPLE nodes at once.
# This time we will need the ChLoadCustomMultiple as base class.
# Those nodes (ie.e ChLoadable objects) can be added in mesh in whatever order,
# not necessarily contiguous, because the bookkeeping is automated.
# Being a stiff load, a jacobian will be automatically generated
# by default using numerical differentiation but if you want you
# can override ComputeJacobian() and compute mK, mR analytically - see prev.example.

nodeE = fea.ChNodeFEAxyz(chrono.ChVector3d(2, 10, 3))
mesh.AddNode(nodeE)
nodeF = fea.ChNodeFEAxyz(chrono.ChVector3d(2, 11, 3))
mesh.AddNode(nodeF)

class MyLoadCustomMultiple(chrono.ChLoadCustomMultiple):
    def __init__(self, loadables):
        chrono.ChLoadCustomMultiple.__init__(self, loadables)

    # "Virtual" copy constructor (covariant return type).
    def Clone(self):
        print('I am there!')
        newinst = copy.deepcopy(self)
        return  newinst

    # Compute Q=Q(x,v)
    # This is the function that you have to implement. It should return the generalized Q load
    # (i.e.the force in generalized lagrangian coordinates).
    # Since here we have multiple connected ChLoadable objects (the two nodes), the rule is that
    # all the vectors (load_Q, state_x, state_w) are split in the same order that the loadable objects
    # are added to MyLoadCustomMultiple in this case for instance Q={Efx,Efy,Efz,Ffx,Ffy,Ffz}.
    # As this is a stiff force field, dependency from state_x and state_y must be considered.
    def ComputeQ(self,state_x, # state position to evaluate Q
                 state_w):     # state speed to evaluate Q
        #chrono.ChVector3d Enode_pos
        #chrono.ChVector3d Enode_vel
        #chrono.ChVector3d Fnode_pos
        #chrono.ChVector3d Fnode_vel
        if not state_x==None and not state_w==None :
            Enode_pos = chrono.ChVector3d(state_x.GetItem(0), state_x.GetItem(1), state_x.GetItem(2))
            Enode_vel = chrono.ChVector3d(state_w.GetItem(0), state_w.GetItem(1), state_w.GetItem(2))
            Fnode_pos = chrono.ChVector3d(state_x.GetItem(3), state_x.GetItem(4), state_x.GetItem(5))
            Fnode_vel = chrono.ChVector3d(state_w.GetItem(3), state_w.GetItem(4), state_w.GetItem(5))
        else:
            # explicit integrators might call ComputeQ(0,0), null pointers mean
            # that we assume current state, without passing state_x for efficiency
            Enode = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadables[0])))
            Fnode = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadables[1])))
            Enode_pos = Enode.GetPos()
            Enode_vel = Enode.GetPosDt()
            Fnode_pos = Fnode.GetPos()
            Fnode_vel = Fnode.GetPosDt()
  		
        # Just implement two simple force+spring+dampers in xy plane:
  		# ... from node E to ground,
        Kx1 = 60
        Ky1 = 50
        Dx1 = 0.3
        Dy1 = 0.2
        E_x_offset = 2
        E_y_offset = 10
        spring1 = chrono.ChVector3d(-Kx1 * (Enode_pos.x - E_x_offset) - Dx1 * Enode_vel.x, -Ky1 * (Enode_pos.y - E_y_offset) - Dy1 * Enode_vel.y, 0)
  		# ... from node F to node E,
        Ky2 = 10
        Dy2 = 0.2
        EF_dist = 1
        spring2 = chrono.ChVector3d (0, -Ky2 * (Fnode_pos.y - Enode_pos.y - EF_dist) - Dy2 * (Enode_vel.y - Fnode_vel.y), 0)
        Fforcey = 2

  		# store generalized forces as a contiguous vector in this.load_Q, with same order of state_w
        self.load_Q.SetItem(0, spring1.x - spring2.x)    # Fx component of force on 1st node
        self.load_Q.SetItem(1, spring1.y - spring2.y)    # Fy component of force on 1st node
        self.load_Q.SetItem(2, spring1.z - spring2.z)    # Fz component of force on 1st node
        self.load_Q.SetItem(3, spring2.x)                # Fx component of force on 2nd node
        self.load_Q.SetItem(4, spring2.y + Fforcey)      # Fy component of force on 2nd node
        self.load_Q.SetItem(5, spring2.z)                # Fz component of force on 2nd node

	# Set this as stiff, to enable the Jacobians
    def IsStiff(self) : 
        return True 


# Instance load object. This require a list of ChLoadable objects
# (these are our two nodes,pay attention to the sequence order), and add to container.
node_list = chrono.vector_ChLoadable()
node_list.append(nodeE)
node_list.append(nodeF)
custom_multi_load = MyLoadCustomMultiple(node_list)
load_container.Add(custom_multi_load)

# --------------------

# Setup a MINRES solver. For FEA one cannot use the default PSOR type solver.

solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(True)

# Perform a static analysis:
sys.DoStaticLinear()

reaction = constraintA.GetReaction2()

print(" constraintA reaction force  F= " + str(reaction.force))
print( " constraintA reaction torque T= " + str(reaction.torque))

print("nodeD position = ")
print(nodeD.GetPos() )
print("custom_load K jacobian=" ) 
print( custom_load.GetJacobians().K.GetMatr() )

print(" nodeE position = " )
print( nodeE.GetPos() )
print(" nodeF position = " )
print( nodeF.GetPos() )
print(" custom_multi_load K jacobian=" )
print( custom_multi_load.GetJacobians().K.GetMatr() )
