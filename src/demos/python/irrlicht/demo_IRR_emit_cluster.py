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
# Demo code about
# - using the ChParticleEmitter to create a cluster of random shapes
# - applying custom force field to particles
# - using Irrlicht to display objects
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import  combinations

#     A callback executed at each particle creation can be attached to the emitter.
#     For example, we need that new particles will be bound to Irrlicht visualization:

class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, application):
        chrono.ChRandomShapeCreator_AddBodyCallback.__init__(self)
        self.airrlicht_application = application#chronoirr.ChIrrApp()

    def OnAddBody(self,
                  mbody,
                  mcoords,
                  mcreator):
        # optional: add further assets, ex for improving visualization:
        mtexture = chrono.ChTexture()
        mtexture.SetTextureFilename(chrono.GetChronoDataFile("textures/bluewhite.png"))
        mbody.AddAsset(mtexture)

        # Enable Irrlicht visualization for all particles
        self.airrlicht_application.AssetBind(mbody)
        self.airrlicht_application.AssetUpdate(mbody)

        # Other stuff, ex. disable gyroscopic forces for increased integrator stability
        mbody.SetNoGyroTorque(True)



print("Copyright (c) 2017 projectchrono.org")

# Create a ChronoENGINE physical system
mphysicalSystem = chrono.ChSystemNSC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(mphysicalSystem, "Particle emitter", chronoirr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 14, -20))

#
# CREATE THE SYSTEM OBJECTS
#

# Example: create a ChBody rigid body.
sphere_mat = chrono.ChMaterialSurfaceNSC()
sphere_mat.SetFriction(0.2)

msphereBody = chrono.ChBodyEasySphere(2.1,          # radius size
                                      1800,         # density
                                      True,         # visualization?
                                      True,         # collision?
                                      sphere_mat)  # contact material
msphereBody.SetPos(chrono.ChVectorD(1, 1, 0))

# optional: add further assets, ex for improving visualization:
mtexture = chrono.ChTexture()
mtexture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
msphereBody.AddAsset(mtexture)

mphysicalSystem.Add(msphereBody)

# Ok, creating particles using ChBody or the ChBodyEasyXXYYZZ shortcuts
# can be enough, ex. if you put in a for() loop you can create a cluster.
# However there is an easier way to the creation of cluster of particles,
# namely the ChEmitter helper class. Here we show hof to use it.

# Create an emitter:

emitter = chrono.ChParticleEmitter()

# Ok, that object will take care of generating particle flows for you.
# It accepts a lot of settings, for creating many different types of particle
# flows, like fountains, outlets of various shapes etc.
# For instance, set the flow rate, etc:

emitter.SetParticlesPerSecond(2000)

emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Our ChParticleEmitter object, among the main settings, it requires
# that you give him four 'randomizer' objects: one is in charge of
# generating random shapes, one is in charge of generating
# random positions, one for random alignements, and one for random velocities.
# In the following we need to instance such objects. (There are many ready-to-use
# randomizer objects already available in chrono, but note that you could also
# inherit your own class from these randomizers if the choice is not enough).

# ---Initialize the randomizer for POSITIONS: random points in a large cube
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
sampled_cube = chrono.ChBox(chrono.VNULL, chrono.ChMatrix33D(chrono.QUNIT), chrono.ChVectorD(50, 50, 50))
emitter_positions.SetGeometry(sampled_cube)

emitter.SetParticlePositioner(emitter_positions)

# ---Initialize the randomizer for ALIGNMENTS
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# ---Initialize the randomizer for VELOCITIES, with statistical distribution
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChMinMaxDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# ---Initialize the randomizer for ANGULAR VELOCITIES, with statistical distribution
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChMinMaxDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# ---Initialize the randomizer for CREATED SHAPES, with statistical distribution


# Create a ChRandomShapeCreator object (ex. here for sphere particles)
#mcreator_spheres = chrono.ChRandomShapeCreatorSpheres>()
#mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution>(0.6, 0.23))
#mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution>(1600))
#
## Finally, tell to the emitter that it must use the creator above:
#emitter.SetParticleCreator(mcreator_spheres)


# ..as an alternative: create odd shapes with convex hulls, like faceted fragments:
mcreator_hulls = chrono.ChRandomShapeCreatorConvexHulls()
mcreator_hulls.SetNpoints(15)
mcreator_hulls.SetChordDistribution(chrono.ChZhangDistribution(1.3, 0.4))
mcreator_hulls.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_hulls)

# --- Optional: what to do by default on ALL newly created particles?
#     A callback executed at each particle creation can be attached to the emitter.
#     For example, we need that new particles will be bound to Irrlicht visualization:

# a- define a class that implement your custom OnAddBody method (see top of source file)
# b- create the callback object...
mcreation_callback = MyCreatorForAll(application)
# c- set callback own data that he might need...
#mcreation_callback.airrlicht_application = application
# d- attach the callback to the emitter!
emitter.RegisterAddBodyCallback(mcreation_callback)

# Use this function for adding a ChIrrNodeAsset to all already created items (ex. a floor, a wall, etc.)
# Otherwise use application.AssetBind(myitem) on a per-item basis, as in the creation callback.
application.AssetBindAll()

# Use this function for 'converting' assets into Irrlicht meshes
application.AssetUpdateAll()

# Modify some setting of the physical system for the simulation, if you want
mphysicalSystem.SetSolverType(chrono.ChSolver.Type_PSOR)
mphysicalSystem.SetSolverMaxIterations(40)

# Turn off default -9.8 downward gravity
mphysicalSystem.Set_G_acc(chrono.ChVectorD(0, 0, 0))

#
# THE SOFT-REAL-TIME CYCLE
#

application.SetTimestep(0.01)

while (application.GetDevice().run()) :
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))

    application.DrawAll()

    # Create particle flow
    emitter.EmitParticles(mphysicalSystem, application.GetTimestep())

    # Apply custom forcefield (brute force approach..)
    # A) reset 'user forces accumulators':
    for body in mphysicalSystem.Get_bodylist() :
        body.Empty_forces_accumulators()


    # B) store user computed force:
    # G_constant = 6.674e-11 # gravitational constant
    G_constant = 6.674e-3  # gravitational constant - HACK to speed up simulation
    mlist = list(combinations(mphysicalSystem.Get_bodylist(), 2))

    for bodycomb in  mlist :
        abodyA = bodycomb[0]
        abodyB = bodycomb[1]
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) /(pow(r_attract, 2))
        F_attract = (D_attract / r_attract) * f_attract
        abodyA.Accumulate_force(F_attract, abodyA.GetPos(), False)
        abodyB.Accumulate_force(-F_attract, abodyB.GetPos(), False)


    # Perform the integration timestep
    application.DoStep()

    application.EndScene()

