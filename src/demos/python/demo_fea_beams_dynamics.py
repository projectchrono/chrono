#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Alessandro Tasora
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.mkl as mkl
import pychrono.irrlicht as chronoirr

print ("Example: FEA of the Jeffcott rotor passing through resonance.");

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../../../data/")


# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#


# Create a Chrono::Engine physical system
my_system = chrono.ChSystemNSC()

my_mesh = fea.ChMesh()
my_system.Add(my_mesh)

my_mesh.SetAutomaticGravity(True,2) # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0));

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_C_PI = 3.1456

# Create a section, i.e. thickness and material properties
# for beams. This will be shared among some beams.

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetGwithPoissonRatio(0.3)
melasticity.SetIyy( (CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) )
melasticity.SetIzz( (CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) )
melasticity.SetJ  ( (CH_C_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) )

msection = fea.ChBeamSectionCosserat(melasticity)
msection.SetDensity(7800)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro) # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.
msection.SetArea(CH_C_PI * (pow(beam_ro, 2)- pow(beam_ri, 2)))

# Use the ChBuilderBeamIGA tool for creating a straight rod 
# divided in Nel elements:

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(my_mesh,      # the mesh to put the elements in
	msection,					# section of the beam
	20,							# number of sections (spans)
	chrono.ChVectorD(0, 0, 0),		# start point
	chrono.ChVectorD(beam_L, 0, 0),	# end point
	chrono.VECT_Y,				# suggested Y direction of section
	1)							# order (3 = cubic, etc)

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size()/2.0)]
	
# Create the flywheel and attach it to the center of the beam
	
mbodyflywheel = chrono.ChBodyEasyCylinder(0.24, 0.05, 7800) # R, h, density
mbodyflywheel.SetCoord(
		chrono.ChCoordsysD(node_mid.GetPos() + chrono.ChVectorD(0,0.05,0), # flywheel initial center (plus Y offset)
		chrono.Q_from_AngAxis(CH_C_PI/2.0, chrono.VECT_Z)) # flywheel initial alignment (rotate 90° so cylinder axis is on X)
)
my_system.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
my_system.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
my_system.Add(truss)

# Create the end bearing 
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
	truss,
	chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())
)
my_system.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),                # body A (slave)
	truss,               # body B (master)
	chrono.ChFrameD(builder.GetLastBeamNodes().front().GetPos(), 
                    chrono.Q_from_AngAxis(CH_C_PI/2.0, chrono.VECT_Y)) # motor frame, in abs. coords
)
my_system.Add(rotmotor1)
	
# use a custom function for setting the speed of the motor
class ChFunction_myf (chrono.ChFunction):
    def __init__(self):
         chrono.ChFunction.__init__(self)
    def Get_y(self,x):
        A1 = 0.8
        A2 = 1.2
        T1 = 0.5
        T2 = 1.0
        T3 = 1.25
        w = 60
        if x < T1:
            return A1 * w * (1. - m.cos(CH_C_PI*x / T1)) / 2.0
        elif (x > T1 and x <= T2):
            return A1 * w
        elif (x > T2 and x <= T3):
            return A1 * w + (A2 - A1) * w * (1.0 - m.cos(CH_C_PI*(x - T2) / (T3 - T2))) / 2.0
        else:
            return A2 * w

f_ramp = ChFunction_myf()
#f_ramp = chrono.ChFunction_Sine(0,0.2,40)
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh.

mvisualizebeamA = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
my_mesh.AddAsset(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizebeamC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizebeamC)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#


# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
myapplication = chronoirr.ChIrrApp(my_system, 'Test FEA: the Jeffcott rotor with IGA beams', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chronoirr.vector3df(0,1,4), chronoirr.vector3df(beam_L/2, 0, 0))
myapplication.AddTypicalLights()

# This is needed if you want to see things in Irrlicht 3D view.
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()


# ---------------------------------------------------------------------
#
#  Run the simulation
#


# Set to a more precise HHT timestepper if needed
# my_system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverMKL()
my_system.SetSolver(msolver)

myapplication.SetTimestep(0.002)

my_system.DoStaticLinear()

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()

