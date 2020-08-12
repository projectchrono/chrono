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
# Authors: Simone Benatti
# =============================================================================
#
# FEA for thin shells of Kirchhoff-Love type, with BST triangle finite elements
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.mkl as mkl
import os


# Output directory
out_dir = "./FEA_SHELLS"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

#GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n"

# Create (if needed) output directory
if not os.path.isdir(out_dir): 
	os.mkdir(out_dir)
	

# Create a Chrono::Engine physical system
my_system = chrono.ChSystemSMC()



# Create a mesh, that is a container for groups
# of elements and their referenced nodes.
my_mesh = fea.ChMesh()

# Remember to add the mesh to the system!
my_system.Add(my_mesh)

# my_system.Set_G_acc(VNULL) or
#my_mesh.SetAutomaticGravity(False)

nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = [] # std::vector<std::shared_ptr<ChNodeFEAxyz>> 

ref_X = chrono.ChFunction_Recorder()
ref_Y = chrono.ChFunction_Recorder()

load_force = chrono.ChVectorD()

#
# BENCHMARK n.1
#
# Add a single BST element:
#



if (False):  # set as 'true' to execute this


	# Create a material

    density = 0.1
    E = 1.2e3
    nu = 0.3
    thickness = 0.001
    
    melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)
    
    # Create nodes
    L = 1.0
    
    p0 = chrono.ChVectorD(0, 0, 0)
    p1 = chrono.ChVectorD(L, 0, 0)
    p2 = chrono.ChVectorD(0, L, 0)
    p3 = chrono.ChVectorD(L, L, 0)
    p4 = chrono.ChVectorD(-L, L, 0)
    p5 = chrono.ChVectorD(L, -L, 0)
    
    mnode0 = fea.ChNodeFEAxyz(p0)
    mnode1 = fea.ChNodeFEAxyz(p1)
    mnode2 = fea.ChNodeFEAxyz(p2)
    mnode3 = fea.ChNodeFEAxyz(p3)
    mnode4 = fea.ChNodeFEAxyz(p4)
    mnode5 = fea.ChNodeFEAxyz(p5)
    my_mesh.AddNode(mnode0)
    my_mesh.AddNode(mnode1)
    my_mesh.AddNode(mnode2)
    my_mesh.AddNode(mnode3)
    my_mesh.AddNode(mnode4)
    my_mesh.AddNode(mnode5)
    
    # Create element
    
    melement = fea.ChElementShellBST()
    my_mesh.AddElement(melement)
    
    melement.SetNodes(mnode0, mnode1, mnode2,None,None,None)
    
    melement.AddLayer(thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)
    
    # TEST
    my_system.Setup()
    my_system.Update()
    print( "BST initial: \n"
    	+ "Area: " + str(melement.area) + "\n"
    	+ "l0: " + str(melement.l0) + "\n"
    	+ "phi0: " + str(melement.phi0) + "\n"
    	+ "k0: " + str(melement.k0) + "\n"
    	+ "e0: " + str(melement.e0) + "\n")
    
    mnode1.SetPos(mnode1.GetPos() + chrono.ChVectorD(0.1, 0, 0))
    
    my_system.Update()
    Fi = chrono.ChVectorDynamicD(melement.GetNdofs())
    melement.ComputeInternalForces(Fi)
    print( "BST updated: \n" 
    		+ "phi: " + str(melement.phi) + "\n"
    		+ "k: " + str(melement.k) + "\n"
    		+ "e: " + str(melement.e) + "\n"
    		+ "m: " + str(melement.m) + "\n"
    		+ "n: " + str(melement.n) + "\n" )
    resultant = chrono.VNULL
    print( "Fi= \n" )

    for i in range(Fi.Size()-2):
        if (i % 3 == 0) :
            print( "-------" + str(i / 3) + "\n")
            fi = chrono.ChVectorD(Fi[i], Fi[i+1], Fi[i+2])
            resultant += fi
        print( str(Fi[i]) + "\n")
        	
    print("resultant: " + str(resultant) + "\n")
        	#system("pause")



#
# BENCHMARK n.2
#
# Add a rectangular mesh of BST elements:
#

mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = fea.ChElementShellBST()

if (True):  # set as 'true' to execute this

    # Create a material
    density = 100
    E = 6e4
    nu = 0.0
    thickness = 0.01

    melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)

    L_x = 1
    nsections_x = 40
    L_z = 1
    nsections_z = 40

	# Create nodes
    mynodes = [] # for future loop when adding elements
    for iz in range(nsections_z+1) :
        for ix in range(nsections_x+1):
            p = chrono.ChVectorD(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
            mnode = fea.ChNodeFEAxyz(p)
            my_mesh.AddNode(mnode)
            mynodes.append(mnode)
		
	
	# Create elements
    for iz in range(nsections_z):
        for ix in range(nsections_x) :
            melementA = fea.ChElementShellBST()
            my_mesh.AddElement(melementA)

            if (iz == 0 and ix == 1):
                ementmonitor = melementA

            boundary_1 = fea.ChNodeFEAxyz()
            boundary_2 = fea.ChNodeFEAxyz() 
            boundary_3 = fea.ChNodeFEAxyz()
            
            boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
            if (ix > 0):
                boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]
            else:
                boundary_2 = None
            if (iz > 0):
                boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1]
            else:
                boundary_3 = None

            melementA.SetNodes(
				mynodes[(iz    ) * (nsections_x + 1) + ix    ], 
				mynodes[(iz    ) * (nsections_x + 1) + ix + 1], 
				mynodes[(iz + 1) * (nsections_x + 1) + ix    ], 
				boundary_1, boundary_2, boundary_3)

            melementA.AddLayer(thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)

			
            melementB = fea.ChElementShellBST()
            my_mesh.AddElement(melementB)
			
            boundary_1 = mynodes[(iz    ) * (nsections_x + 1) + ix    ]
            if (ix < nsections_x-1):
                boundary_2 = mynodes[(iz   ) * (nsections_x + 1) + ix + 2]
            else:
                boundary_2 = None
            if (iz < nsections_z-1):
                boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix    ]
            else:
                boundary_3 = None

            melementB.SetNodes( 
				mynodes[(iz + 1) * (nsections_x + 1) + ix + 1], 
				mynodes[(iz + 1) * (nsections_x + 1) + ix    ], 
				mynodes[(iz    ) * (nsections_x + 1) + ix + 1],
				boundary_1, boundary_2, boundary_3)

            melementB.AddLayer(thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)
	
    for j in range(30) :
        for k in range(30) :
            mynodes[j * (nsections_x + 1)+k].SetFixed(True);
			
		
		
	
"""
	mynodes[0*(nsections_x+1)].SetFixed(True)
	mynodes[1*(nsections_x+1)].SetFixed(True)

	mynodes[0*(nsections_x+1)+1].SetFixed(True)
	mynodes[1*(nsections_x+1)+1].SetFixed(True)
"""

#
# BENCHMARK n.3
#
# Load and create shells from a .obj file containing a triangle mesh surface
#

if (False) :
	density = 100
	E = 6e5
	nu = 0.0
	thickness = 0.01

	melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
	material = fea.ChMaterialShellKirchhoff(melasticity)
	material.SetDensity(density)

	fea.ChMeshFileLoader.BSTShellFromObjFile(my_mesh, chrono.GetChronoDataFile('cube.obj'), material, thickness)




# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChTriangleMeshShape).
# Do not forget AddAsset() at the end!


mvisualizeshellA = fea.ChVisualizationFEAmesh(my_mesh)
#mvisualizeshellA.SetSmoothFaces(True)
#mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetShellResolution(2)
#mvisualizeshellA.SetBackfaceCull(True)
my_mesh.AddAsset(mvisualizeshellA)

mvisualizeshellB = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizeshellB.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizeshellB.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
my_mesh.AddAsset(mvisualizeshellB)


#
# VISUALIZATION
#

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(my_system, "Shells FEA test: triangle BST elements", chronoirr.dimension2du(1024, 768), False, True)

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1, .3, 1.3), chronoirr.vector3df(.5, -.3, .5))

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

#gmres_solver = chrono_types::make_shared<ChSolverGMRES>()
#gmres_solver.SetMaxIterations(100)
#my_system.SetSolver(gmres_solver)


timestep = 0.005
application.SetTimestep(timestep)
my_system.Setup()
my_system.Update()

rec_X = chrono.ChFunction_Recorder()
rec_Y = chrono.ChFunction_Recorder()

mtime = 0

while (application.GetDevice().run()) :
	application.BeginScene()

	application.DrawAll()

	# .. draw also a grid
	#ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1)

	application.DoStep()

	application.EndScene()



