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


import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print ("Example: FEA of the Jeffcott rotor passing through resonance.");

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')


# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#


# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,2) # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0,-9.81, 0));

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1456

# Create a section, i.e. thickness and material properties
# for beams. This will be shared among some beams.

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800);
minertia.SetArea(CH_PI * (pow(beam_ro, 2)- pow(beam_ri, 2)));
minertia.SetIyy( (CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) );
minertia.SetIzz( (CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) );
    
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy( (CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) )
melasticity.SetIzz( (CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) )
melasticity.SetJ  ( (CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)) )

msection = fea.ChBeamSectionCosserat(minertia, melasticity)

msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro) # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.


# Use the ChBuilderBeamIGA tool for creating a straight rod 
# divided in Nel elements:

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,      # the mesh to put the elements in
	msection,					# section of the beam
	20,							# number of sections (spans)
	chrono.ChVector3d(0, 0, 0),		# start point
	chrono.ChVector3d(beam_L, 0, 0),	# end point
	chrono.VECT_Y,				# suggested Y direction of section
	1)							# order (3 = cubic, etc)

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size()/2.0)]
	
# Create the flywheel and attach it to the center of the beam
	
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800) # R, h, density
mbodyflywheel.SetCoordsys(
		chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0,0.05,0), # flywheel initial center (plus Y offset)
		chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Z)) # flywheel initial alignment (rotate 90° so cylinder axis is on X)
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the end bearing 
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
	truss,
	chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos())
)
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),                # body A (slave)
	truss,               # body B (master)
	chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(), 
                    chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Y)) # motor frame, in abs. coords
)
sys.Add(rotmotor1)
	
# use a custom function for setting the speed of the motor
class ChFunctionMyFun (chrono.ChFunction):
    def __init__(self):
         chrono.ChFunction.__init__(self)
    def GetVal(self,x):
        A1 = 0.8
        A2 = 1.2
        T1 = 0.5
        T2 = 1.0
        T3 = 1.25
        w = 60
        if x < T1:
            return A1 * w * (1. - m.cos(CH_PI*x / T1)) / 2.0
        elif (x > T1 and x <= T2):
            return A1 * w
        elif (x > T2 and x <= T3):
            return A1 * w + (A2 - A1) * w * (1.0 - m.cos(CH_PI*(x - T2) / (T3 - T2))) / 2.0
        else:
            return A2 * w

f_ramp = ChFunctionMyFun()
#f_ramp = chrono.ChFunctionSine(40,0.2)
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh.

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)


# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L/2, 0, 0))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#


# Set to a more precise HHT timestepper if needed
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)

