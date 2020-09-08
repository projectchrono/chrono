# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Alessandro Tasora
# =============================================================================
#
# FEA for shells of Reissner 6-field type
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
import pychrono.mkl as mkl
import pychrono.irrlicht as chronoirr
import os
import numpy as np
import matplotlib.pyplot as plt


# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS"

print( "Copyright (c) 2017 projectchrono.org")

# Create (if needed) output directory
if not os.path.isdir(out_dir):
    os.mkdir(out_dir)
if not os.path.isdir(out_dir):
	raise NameError("Error creating directory " )


# Create a Chrono::Engine physical system
my_system = chrono.ChSystemSMC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(my_system, "Shells FEA", chronoirr.dimension2du(800, 600), False, True)

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 6.0, -10))

# Create a mesh, that is a container for groups
# of elements and their referenced nodes.
my_mesh = fea.ChMesh()

# Remember to add the mesh to the system!
my_system.Add(my_mesh)

# my_system.Set_G_acc(VNULL) or
my_mesh.SetAutomaticGravity(False)

nodePlotA = fea.ChNodeFEAxyzrot()
nodePlotB = fea.ChNodeFEAxyzrot()
nodesLoad = []

ref_X = chrono.ChFunction_Recorder()
ref_Y = chrono.ChFunction_Recorder()

load_torque = chrono.ChVectorD()
load_force = chrono.ChVectorD()

bench1 = False
bench2 = True
bench3 = False
#
# BENCHMARK n.1
#
# Add an EANS SHELL cantilever:
#

if (bench1):  # set as 'True' to execute this
    rect_thickness = 0.10
    rect_L = 10.0
    rect_W = 1
    # Create a material
    rho = 0.0
    E = 1.2e6
    nu = 0.0
    melasticity =fea.ChElasticityReissnerIsothropic(E, nu, 1.0, 0.01)
    mat = fea.ChMaterialShellReissner(melasticity)
    # In case you need also damping it would add...
    #mdamping = chrono_types::make_shared<ChDampingReissnerRayleigh>(melasticity,0.01)
    #mat = chrono_types::make_shared<ChMaterialShellReissner>(melasticity, nullptr, mdamping)
    mat.SetDensity(rho)
    # Create the nodes
    nels_L = 12
    nels_W = 1
    elarray = [fea.ChElementShellReissner4]*(nels_L * nels_W)
    nodearray = [fea.ChNodeFEAxyzrot]*((nels_L + 1) * (nels_W + 1))
    nodes_start = [fea.ChNodeFEAxyzrot]*(nels_W + 1)
    nodes_end = [fea.ChNodeFEAxyzrot]*(nels_W + 1)
    for il in range(nels_L+1) :
        for iw in range(nels_W +1):           
            # Make nodes
            nodepos = chrono.ChVectorD(rect_L * (il / nels_L), 0, rect_W * (iw / nels_W))
            noderot = chrono.ChQuaternionD(chrono.QUNIT)
            nodeframe = chrono.ChFrameD(nodepos, noderot)
            mnode = fea.ChNodeFEAxyzrot(nodeframe)
            my_mesh.AddNode(mnode)
            for i in range(3):
                mnode.GetInertia()[i,i] = 0   # approx]
            mnode.SetMass(0)
            nodearray[il * (nels_W + 1) + iw] = mnode
            if (il == 0):
                nodes_start[iw] = mnode
            if (il == nels_L):
                nodes_end[iw] = mnode
            # Make elements
            if (il > 0 and iw > 0) :
                melement = fea.ChElementShellReissner4()
                my_mesh.AddElement(melement)
                melement.SetNodes(nodearray[(il - 1) * (nels_W + 1) + (iw - 1)],
								   nodearray[(il) * (nels_W + 1) + (iw - 1)], nodearray[(il) * (nels_W + 1) + (iw)],
								   nodearray[(il - 1) * (nels_W + 1) + (iw)])

                melement.AddLayer(rect_thickness, 0 * chrono.CH_C_DEG_TO_RAD, mat)
                elarray[(il - 1) * (nels_W) + (iw - 1)] = melement
                
    nodesLoad = nodes_end
    nodePlotA = nodes_end[0]
    nodePlotB = nodes_end[-1]
    for startnode in( nodes_start):
        startnode.SetFixed(True)

    # applied load
    # load_force = chrono.ChVectorD(200000,0, 20000)
    load_force = chrono.ChVectorD(0, 4, 0)
    # load_torque = chrono.ChVectorD(0, 0, 50*CH_C_PI/3.0)

    # reference solution for (0, 4, 0) shear to plot
    ref_Y.AddPoint(0.10, 1.309)
    ref_X.AddPoint(0.40, 0.103)
    ref_Y.AddPoint(0.20, 2.493)
    ref_X.AddPoint(0.80, 0.381)
    ref_Y.AddPoint(0.30, 3.488)
    ref_X.AddPoint(1.20, 0.763)
    ref_Y.AddPoint(0.40, 4.292)
    ref_X.AddPoint(1.60, 1.184)
    ref_Y.AddPoint(0.50, 4.933)
    ref_X.AddPoint(2.00, 1.604)
    ref_Y.AddPoint(0.60, 5.444)
    ref_X.AddPoint(2.40, 2.002)
    ref_Y.AddPoint(0.70, 5.855)
    ref_X.AddPoint(2.80, 2.370)
    ref_Y.AddPoint(0.80, 6.190)
    ref_X.AddPoint(3.20, 2.705)
    ref_Y.AddPoint(0.90, 6.467)
    ref_X.AddPoint(3.60, 3.010)
    ref_Y.AddPoint(1.00, 6.698)
    ref_X.AddPoint(4.00, 3.286)

#
# BENCHMARK n.2
#
# Add a SLIT ANNULAR PLATE:
#

if (bench2):  # set as 'True' to execute this
    plate_thickness = 0.03
    plate_Ri = 6
    plate_Ro = 10
    # Create a material
    rho = 0.0
    E = 21e6
    nu = 0.0
    mat = fea.ChMaterialShellReissnerIsothropic(rho, E, nu, 1.0, 0.01)
	# Create the nodes
    nels_U = 60
    nels_W = 10
    arc = chrono.CH_C_2PI * 1 
    elarray = [fea.ChElementShellReissner4]*(nels_U * nels_W)
    nodearray = [fea.ChNodeFEAxyzrot]*((nels_U + 1) * (nels_W + 1))
    nodes_start = [fea.ChNodeFEAxyzrot]*(nels_W + 1)
    nodes_end = [fea.ChNodeFEAxyzrot]*(nels_W + 1)
    
    for iu in range(nels_U +1): 
        for iw in range(nels_W +1) :
            # Make nodes
            u = iu / nels_U
            w = iw / nels_W
            nodepos = chrono.ChVectorD((plate_Ri + (plate_Ro - plate_Ri) * w) * np.cos(u * arc), 0,
							   (plate_Ri + (plate_Ro - plate_Ri) * w) * np.sin(u * arc))
            noderot = chrono.ChQuaternionD(chrono.QUNIT)
            nodeframe = chrono.ChFrameD(nodepos, noderot)
            mnode = fea.ChNodeFEAxyzrot(nodeframe)
            my_mesh.AddNode(mnode)
            for i in range(3):
                mnode.GetInertia()[i,i] = 0
            mnode.SetMass(0)
            nodearray[iu * (nels_W + 1) + iw] = mnode
            if (iu == 0):
                nodes_start[iw] = mnode
            if (iu == nels_U):
                nodes_end[iw] = mnode
			# Make elements
            if (iu > 0 and iw > 0) :
                melement = fea.ChElementShellReissner4()
                my_mesh.AddElement(melement)
                melement.SetNodes(nodearray[(iu) * (nels_W + 1) + (iw)], nodearray[(iu - 1) * (nels_W + 1) + (iw)],
								   nodearray[(iu - 1) * (nels_W + 1) + (iw - 1)],
								   nodearray[(iu) * (nels_W + 1) + (iw - 1)])

                melement.AddLayer(plate_thickness, 0 * chrono.CH_C_DEG_TO_RAD, mat)
                elarray[(iu - 1) * (nels_W) + (iw - 1)] = melement

    nodesLoad = nodes_end
    nodePlotA = nodes_end[0]
    nodePlotB = nodes_end[-1]

    for mstartnode in nodes_start :
        mstartnode.SetFixed(True)

    load_force = chrono.ChVectorD(0, 0.8 * 4, 0)
    load_torque = chrono.VNULL
	# reference solution to plot
    ref_X.AddPoint(0.025, 1.305)
    ref_Y.AddPoint(0.025, 1.789)
    ref_X.AddPoint(0.10, 4.277)
    ref_Y.AddPoint(0.10, 5.876)
    ref_X.AddPoint(0.20, 6.725)
    ref_Y.AddPoint(0.20, 9.160)
    ref_X.AddPoint(0.30, 8.340)
    ref_Y.AddPoint(0.30, 11.213)
    ref_X.AddPoint(0.40, 9.529)
    ref_Y.AddPoint(0.40, 12.661)
    ref_X.AddPoint(0.50, 10.468)
    ref_Y.AddPoint(0.50, 13.768)
    ref_X.AddPoint(0.60, 11.257)
    ref_Y.AddPoint(0.60, 14.674)
    ref_X.AddPoint(0.70, 11.970)
    ref_Y.AddPoint(0.70, 15.469)
    ref_X.AddPoint(0.80, 12.642)
    ref_Y.AddPoint(0.80, 16.202)
    ref_X.AddPoint(0.9, 13.282)
    ref_Y.AddPoint(0.90, 16.886)
    ref_X.AddPoint(1.00, 13.891)
    ref_Y.AddPoint(1.00, 17.528)

#
# BENCHMARK n.3
#
# Add a CLAMPED HALF CYLINDER :
#

if (bench3):
    plate_thickness = 0.03
    plate_R = 1.016
    plate_L = 3.048

	# Create a material
    rho = 0.0
    E = 2.0685e7
    nu = 0.3
    mat = fea.ChMaterialShellReissnerIsothropic(rho, E, nu, 1.0, 0.01)

	# In case you want to test laminated shells, use this:
    mat_ortho =fea.ChMaterialShellReissnerOrthotropic(rho, 2.0685e7, 0.517e7, 0.3, 0.795e7, 0.795e7, 0.795e7, 1.0, 0.01)
	# Create the nodes
    nels_U = 32
    nels_W = 32
    arc = chrono.CH_C_PI
    elarray = [fea.ChElementShellReissner4]*(nels_U * nels_W)
    nodearray = [fea.ChNodeFEAxyzrot]*(nels_U + 1) * (nels_W + 1)
    nodes_start = [fea.ChNodeFEAxyzrot]*(nels_W + 1)
    nodes_end = [fea.ChNodeFEAxyzrot]*(nels_W + 1)
    nodes_left = [fea.ChNodeFEAxyzrot]*(nels_U + 1)
    nodes_right = [fea.ChNodeFEAxyzrot]*(nels_U + 1)

    for iu in range(nels_U +1) :
        for iw in range(nels_W +1) :
			# Make nodes
            u = iu / nels_U
            w = iw / nels_W
            nodepos = chrono.ChVectorD((plate_R)*np.cos(w * arc), (plate_R)*np.sin(w * arc), u * plate_L)
            noderot = chrono.ChQuaternionD(chrono.QUNIT)
            nodeframe = chrono.ChFrameD(nodepos, noderot)
            mnode = fea.ChNodeFEAxyzrot(nodeframe)
            my_mesh.AddNode(mnode)
            for i in range(3):
                mnode.GetInertia()[i,i] = 0
            mnode.SetMass(0.0)
            nodearray[iu * (nels_W + 1) + iw] = mnode
            if (iu == 0):
                nodes_start[iw] = mnode
            if (iu == nels_U):
                nodes_end[iw] = mnode
            if (iw == 0):
                nodes_left[iu] = mnode
            if (iw == nels_W):
                nodes_right[iu] = mnode

			# Make elements
            if (iu > 0 and iw > 0) :
                melement = fea.ChElementShellReissner4()
                my_mesh.AddElement(melement)
                melement.SetNodes(nodearray[(iu) * (nels_W + 1) + (iw)], nodearray[(iu - 1) * (nels_W + 1) + (iw)],
								   nodearray[(iu - 1) * (nels_W + 1) + (iw - 1)],
								   nodearray[(iu) * (nels_W + 1) + (iw - 1)])

                melement.AddLayer(plate_thickness, 0 * chrono.CH_C_DEG_TO_RAD, mat)
				# In case you want to test laminated shells, do instead:
				#  melement.AddLayer(plate_thickness/3, 0 * CH_C_DEG_TO_RAD, mat_ortho)
				#  melement.AddLayer(plate_thickness/3, 90 * CH_C_DEG_TO_RAD, mat_ortho)
				#  melement.AddLayer(plate_thickness/3, 0 * CH_C_DEG_TO_RAD, mat_ortho)

                elarray[(iu - 1) * (nels_W) + (iw - 1)] = melement

    nodesLoad.append(nodes_end[int(len(nodes_end) / 2)])
    nodePlotA = nodes_end[int(len(nodes_end) / 2)]
    nodePlotB = nodes_end[int(len(nodes_end) / 2)]

    for mstartnode in nodes_start :
        mstartnode.SetFixed(True)

    mtruss = chrono.ChBody()
    mtruss.SetBodyFixed(True)
    my_system.Add(mtruss)
    for mendnode in nodes_left :
        mlink = chrono.ChLinkMateGeneric(False, True, False, True, False, True)
        mlink.Initialize(mendnode, mtruss, False, mendnode.Frame(), mendnode.Frame())
        my_system.Add(mlink)
        
    for mendnode in nodes_right :
        mlink = chrono.ChLinkMateGeneric(False, True, False, True, False, True)
        mlink.Initialize(mendnode, mtruss, False, mendnode.Frame(), mendnode.Frame())
        my_system.Add(mlink)

    load_force = chrono.ChVectorD(0, -2000, 0)
    load_torque = chrono.VNULL
    
    # reference solution to plot
    ref_X.AddPoint(0.10, 1 - 0.16)
    ref_X.AddPoint(0.20, 1 - 0.37)
    ref_X.AddPoint(0.30, 1 - 0.66)
    ref_X.AddPoint(0.40, 1 - 1.13)
    ref_X.AddPoint(0.50, 1 - 1.32)
    ref_X.AddPoint(0.60, 1 - 1.44)
    ref_X.AddPoint(0.70, 1 - 1.52)
    ref_X.AddPoint(0.80, 1 - 1.60)
    ref_X.AddPoint(0.90, 1 - 1.66)
    ref_X.AddPoint(1.00, 1 - 1.71)


# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChTriangleMeshShape).
# Do not forget AddAsset() at the end!

mvisualizeshellA = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
my_mesh.AddAsset(mvisualizeshellA)

mvisualizeshellC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizeshellC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizeshellC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
mvisualizeshellC.SetSymbolsThickness(0.05)
mvisualizeshellC.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizeshellC)

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.

application.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

application.AssetUpdateAll()

#
# THE SOFT-REAL-TIME CYCLE
#
# Change solver to MKL
mkl_solver = mkl.ChSolverMKL()
mkl_solver.LockSparsityPattern(True)
my_system.SetSolver(mkl_solver)

# Set integrator
ts = chrono.ChTimestepperEulerImplicit(my_system)
my_system.SetTimestepper(ts)

ts.SetMaxiters(5)
ts.SetAbsTolerances(1e-12, 1e-12)

timestep = 0.1
application.SetTimestep(timestep)
my_system.Setup()
my_system.Update()

rec_X = []
rec_Y = []
load = []
mtime = 0

while (application.GetDevice().run()) :
    application.BeginScene()
    application.DrawAll()
    # .. draw also a grid
    chronoirr.ChIrrTools.drawGrid(application.GetVideoDriver(), 1, 1)
    # ...update load at end nodes, as simple lumped nodal forces
    load_scale = mtime * 0.1
    for mendnode in(nodesLoad) :
        mendnode.SetForce(load_force * load_scale * (1. / len(nodesLoad)))
        mendnode.SetTorque(load_torque * load_scale * (1. / len(nodesLoad)))
    # application.DoStep()
    # mtime = my_system.GetChTime()
    application.GetSystem().DoStaticNonlinear(3)
    # application.GetSystem().DoStaticLinear()
    mtime += timestep

    if (not application.GetPaused() and nodePlotA and nodePlotB) :
        rec_Y.append(nodePlotA.GetPos().y)
        rec_X.append(nodePlotB.GetPos().y)
        load.append(load_scale)
	
    application.EndScene()
    #if (load_scale > 1):
    #    application.GetDevice().closeDevice()

# Outputs results in a MatPlotLib plot:

plt.plot(load, rec_Y)
plt.plot(load, rec_X, ':')
plt.legend(['W','-U'])
plt.xlabel('Torque T/T0')
plt.ylabel('Tip displacement [m]')
plt.savefig(out_dir+'/shell_benchmark.pdf') 
plt.show()
