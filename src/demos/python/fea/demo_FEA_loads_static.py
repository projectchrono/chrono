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
mnodeA = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
mnodeB = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(2, 0, 0)))

# Default mass for FEM nodes is zero
mnodeA.SetMass(0.0)
mnodeB.SetMass(0.0)

mesh.AddNode(mnodeA)
mesh.AddNode(mnodeB)

# Create beam section & material
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.1
beam_wz = 0.2
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetGshearModulus(0.01e9 * 0.3)
msection.SetBeamRaleyghDamping(0.200)
msection.SetDensity(1500)

# Create a beam of Eulero-Bernoulli type:
melementA = fea.ChElementBeamEuler()
melementA.SetNodes(mnodeA, mnodeB)
melementA.SetSection(msection)
mesh.AddElement(melementA)

# Create also a truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

# Create a constraat the end of the beam
constr_a = chrono.ChLinkMateGeneric()
constr_a.Initialize(mnodeA, truss, False, mnodeA.Frame(), mnodeA.Frame())
sys.Add(constr_a)
constr_a.SetConstrainedCoords(True, True, True,   # x, y, z
							   True, True, True)  # Rx, Ry, Rz

# APPLY SOME LOADS!

# First: loads must be added to "load containers",
# and load containers must be added to your system
mloadcontainer = chrono.ChLoadContainer()
sys.Add(mloadcontainer)

# Example 1:

# Add a vertical load to the end of the beam element:
mwrench = fea.ChLoadBeamWrench(melementA)
mwrench.loader.SetApplication(1.0)  # in -1..+1 range, -1: end A, 0: mid, +1: end B
mwrench.loader.SetForce(chrono.ChVectorD(0, -0.2, 0))
mloadcontainer.Add(mwrench)  # do not forget to add the load to the load container.

# Example 2:

# Add a distributed load along the beam element:
mwrenchdis = fea.ChLoadBeamWrenchDistributed(melementA)
mwrenchdis.loader.SetForcePerUnit(chrono.ChVectorD(0, -0.1, 0))  # load per unit length
mloadcontainer.Add(mwrenchdis)

# Example 3:

# Add gravity (constant volumetric load)
mgravity = chrono.LoadLoaderGravity(melementA)
mloadcontainer.Add(mgravity)

# note that by default all solid elements in the mesh will already
# get gravitational force, if you want to bypass this automatic gravity, do:
mesh.SetAutomaticGravity(False)

"""
CANNOT INSTANTIATE TEMPLATE 
Example 4 and 5 in C++ equivalent demo require respectively ChLoad<MyLoaderTriangular> and ChLoad<MyLoaderPointStiff>(mnodeC)

"""

# Example 6:

# As before, create a custom load with stiff force, acting on a single node, but
# this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
# This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
# As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
# that will be used in statics, implicit integrators, etc.

mnodeD = fea.ChNodeFEAxyz(chrono.ChVectorD(2, 10, 3))
mesh.AddNode(mnodeD)

class MyLoadCustom(chrono.ChLoadCustom):
    def __init__(self, mloadable):
        chrono.ChLoadCustom.__init__(self, mloadable)
    #/ "Virtual" copy constructor (covariant return type).
    def Clone(self):
        newinst = copy.deepcopy(self)
        return  newinst

	# Compute Q=Q(x,v)
	# This is the function that you have to implement. It should return the generalized Q load
	# (i.e.the force in generalized lagrangian coordinates).
	# For ChNodeFEAxyz, Q loads are expected as 3-rows vectors, containing absolute force x,y,z.
	# As this is a stiff force field, dependency from state_x and state_y must be considered.
    def ComputeQ(self,state_x,      #/< state position to evaluate Q
                 state_w):     #/< state speed to evaluate Q
        if not state_x==None and not state_w==None :
            node_pos = chrono.ChVectorD(state_x[0], state_x[1], state_x[2])
            node_vel = chrono.ChVectorD(state_w[0], state_w[1], state_w[2])
        else:
            mynode = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadable) ))
            node_pos = mynode.GetPos()
            node_vel = mynode.GetPos_dt()
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
        self.load_Q[0] = x_force - Kx * (node_pos.x - x_offset) - Dx * node_vel.x
        self.load_Q[1] = y_force - Ky * (node_pos.y - y_offset) - Dy * node_vel.y
        self.load_Q[2] = 0

	# Compute jacobian not available from Python

	# Remember to set this as stiff, to enable the jacobians
    def IsStiff(self) : 
        return True 

# Instance load object, applying to a node, as in previous example, and add to container:
mloadcustom = MyLoadCustom(mnodeD)
mloadcontainer.Add(mloadcustom)

# Example 7:

# As before, create a custom load with stiff force, acting on MULTIPLE nodes at once.
# This time we will need the ChLoadCustomMultiple as base class.
# Those nodes (ie.e ChLoadable objects) can be added in mesh in whatever order,
# not necessarily contiguous, because the bookkeeping is automated.
# Being a stiff load, a jacobian will be automatically generated
# by default using numerical differentiation but if you want you
# can override ComputeJacobian() and compute mK, mR analytically - see prev.example.

mnodeE = fea.ChNodeFEAxyz(chrono.ChVectorD(2, 10, 3))
mesh.AddNode(mnodeE)
mnodeF = fea.ChNodeFEAxyz(chrono.ChVectorD(2, 11, 3))
mesh.AddNode(mnodeF)

class MyLoadCustomMultiple(chrono.ChLoadCustomMultiple):
    def __init__(self, mloadables):
        chrono.ChLoadCustomMultiple.__init__(self, mloadables)
    #/ "Virtual" copy constructor (covariant return type).
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
    def ComputeQ(self,state_x,      #/< state position to evaluate Q
                 state_w):     #/< state speed to evaluate Q
        #chrono.ChVectorD Enode_pos
        #chrono.ChVectorD Enode_vel
        #chrono.ChVectorD Fnode_pos
        #chrono.ChVectorD Fnode_vel
        if not state_x==None and not state_w==None :
            Enode_pos = chrono.ChVectorD(state_x[0], state_x[1], state_x[2])
            Enode_vel = chrono.ChVectorD(state_w[0], state_w[1], state_w[2])
            Fnode_pos = chrono.ChVectorD(state_x[3], state_x[4], state_x[5])
            Fnode_vel = chrono.ChVectorD(state_w[3], state_w[4], state_w[5])
        else:
            # explicit integrators might call ComputeQ(0,0), null pointers mean
            # that we assume current state, without passing state_x for efficiency
            Enode = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadables[0])))
            Fnode = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadables[1])))
            Enode_pos = Enode.GetPos()
            Enode_vel = Enode.GetPos_dt()
            Fnode_pos = Fnode.GetPos()
            Fnode_vel = Fnode.GetPos_dt()
  		# Just implement two simple force+spring+dampers in xy plane:
  		# ... from node E to ground,
        Kx1 = 60
        Ky1 = 50
        Dx1 = 0.3
        Dy1 = 0.2
        E_x_offset = 2
        E_y_offset = 10
        spring1 = chrono.ChVectorD(-Kx1 * (Enode_pos.x - E_x_offset) - Dx1 * Enode_vel.x, -Ky1 * (Enode_pos.y - E_y_offset) - Dy1 * Enode_vel.y, 0)
  		# ... from node F to node E,
        Ky2 = 10
        Dy2 = 0.2
        EF_dist = 1
        spring2 = chrono.ChVectorD (0, -Ky2 * (Fnode_pos.y - Enode_pos.y - EF_dist) - Dy2 * (Enode_vel.y - Fnode_vel.y), 0)
        Fforcey = 2
  		# store generalized forces as a contiguous vector in this.load_Q, with same order of state_w
        self.load_Q[0] = spring1.x - spring2.x    # Fx component of force on 1st node
        self.load_Q[1] = spring1.y - spring2.y    # Fy component of force on 1st node
        self.load_Q[2] = spring1.z - spring2.z    # Fz component of force on 1st node
        self.load_Q[3] = spring2.x                # Fx component of force on 2nd node
        self.load_Q[4] = spring2.y + Fforcey      # Fy component of force on 2nd node
        self.load_Q[5] = spring2.z                # Fz component of force on 2nd node

	# OPTIONAL: if you want to provide an analytical jacobian, just implement the following:
	#   virtual void ComputeJacobian(...)

	# Remember to set this as stiff, to enable the jacobians
    def IsStiff(self) : 
        return True 


# Instance load object. This require a list of ChLoadable objects
# (these are our two nodes,pay attention to the sequence order), and add to container.
mnodelist = chrono.vector_ChLoadable()
mnodelist.append(mnodeE)
mnodelist.append(mnodeF)
mloadcustommultiple = MyLoadCustomMultiple(mnodelist)
mloadcontainer.Add(mloadcustommultiple)

###################/

# Setup a MINRES solver. For FEA one cannot use the default PSOR type solver.

solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(True)

# Perform a static analysis:
sys.DoStaticLinear()

print(" constr_a reaction force  F= " + str(constr_a.Get_react_force()) )
print( " constr_a reaction torque T= " + str(constr_a.Get_react_torque()))

#print("mnodeC position = " + mnodeC.GetPos() )
#print("mloadstiff K jacobian=" + mloadstiff.GetJacobians().K )

print("mnodeD position = ")
print(mnodeD.GetPos() )
print("mloadcustom K jacobian=" ) 
print( mloadcustom.GetJacobians().K.GetMatr() )

print(" mnodeE position = " )
print( mnodeE.GetPos() )
print(" mnodeF position = " )
print( mnodeF.GetPos() )
print(" mloadcustommultiple K jacobian=" )
print( mloadcustommultiple.GetJacobians().K.GetMatr() )
