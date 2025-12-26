# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban, Jasper Grant
# =============================================================================
#
# Viper rover on CRM terrain (initialized from heightmap image)
#
# =============================================================================

from enum import Enum
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono.fsi import (
    ChFsiFluidSystemSPH,
    ChFsiSystemSPH,
    ElasticMaterialProperties,
    SPHParameters,
    IntegrationScheme_RK2,
    ViscosityMethod_ARTIFICIAL_BILATERAL,
    BoundaryMethod_ADAMI,
    OutputLevel_STATE,
    BoxSide_ALL,
    BoxSide_Z_POS,
    BoxSide_Z_NEG,
    ParticleHeightColorCallback,
    ChSphVisualizationVSG,
)
import pychrono.vsg3d as vsg3d

# ===================================================================================================================


# CRM terrain patch type
class PatchType(Enum):
    RECTANGULAR = 0
    HEIGHT_MAP = 1


patch_type = PatchType.HEIGHT_MAP

# Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
terrain_length = 12
terrain_width = 3

# Simulation parameters
density = 1700
cohesion = 5e3
friction = 0.7
youngs_modulus = 1e6
poisson_ratio = 0.3

tend = 30
step_size = 5e-4
active_box_dim = chrono.ChVector3d(0.6, 0.6, 0.6)

render = True  # use run-time visualization
render_fps = 200  # rendering FPS

visualization_sph = True  # render SPH particles
visualization_bndry_bce = False  # render boundary BCE markers
visualization_rigid_bce = False  # render wheel BCE markers

verbose = True

# ===================================================================================================================

# Create the Chrono system and associated collision system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create rover
print("Create rover...")
wheel_mat = chrono.ChContactMaterialData(
    0.4,  # friction coefficient (mu)
    0.2,  # restitution (cr)
    2e7,  # Young's modulus (Y)
    0.3,  # Poisson ratio (nu)
    2e5,  # normal stiffness (kn)
    40.0,  # normal damping (gn)
    2e5,  # tangential stiffness (kt)
    20.0,  # tangential damping (gt)
)
init_loc = chrono.ChVector3d(1.25, 0.0, 0.55)

driver = viper.ViperDCMotorControl()
rover = viper.Viper(system)
rover.SetDriver(driver)
rover.SetWheelContactMaterial(wheel_mat.CreateMaterial(system.GetContactMethod()))
rover.Initialize(chrono.ChFramed(init_loc, chrono.QUNIT))

# Create the CRM terrain system
initial_spacing = 0.03
terrain = veh.CRMTerrain(system, initial_spacing)
sysFSI: ChFsiSystemSPH = terrain.GetFsiSystemSPH()
sysSPH: ChFsiFluidSystemSPH = terrain.GetFluidSystemSPH()
sysSPH.EnableCudaErrorCheck(False)
terrain.SetVerbose(verbose)
terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
terrain.SetStepSizeCFD(step_size)

mat_props: ElasticMaterialProperties = ElasticMaterialProperties()
mat_props.density = density
mat_props.Young_modulus = youngs_modulus
mat_props.Poisson_ratio = poisson_ratio
mat_props.mu_I0 = 0.04
mat_props.mu_fric_s = friction
mat_props.mu_fric_2 = friction
mat_props.average_diam = 0.005
mat_props.cohesion_coeff = cohesion
terrain.SetElasticSPH(mat_props)

sph_params: SPHParameters = SPHParameters()
sph_params.integration_scheme = IntegrationScheme_RK2
sph_params.initial_spacing = initial_spacing
sph_params.d0_multiplier = 1.0
sph_params.free_surface_threshold = 0.8
sph_params.artificial_viscosity = 0.5
sph_params.use_consistent_gradient_discretization = False
sph_params.use_consistent_laplacian_discretization = False
sph_params.viscosity_method = ViscosityMethod_ARTIFICIAL_BILATERAL
sph_params.boundary_method = BoundaryMethod_ADAMI
sph_params.use_variable_time_step = True
terrain.SetSPHParameters(sph_params)

# Set output level from SPH simulation
terrain.SetOutputLevel(OutputLevel_STATE)

# Add rover wheels as FSI bodies
print("Create wheel BCE markers...")
mesh_file_name = chrono.GetChronoDataFile("robot/viper/obj/viper_cylwheel.obj")
geometry = chrono.ChBodyGeometry()
geometry.materials.push_back(chrono.ChContactMaterialData())
geometry.coll_meshes.push_back(
    chrono.TrimeshShape(chrono.VNULL, chrono.QUNIT, mesh_file_name, chrono.VNULL)
)

for i in range(4):
    wheel_body = rover.GetWheel(i).GetBody()
    terrain.AddRigidBody(wheel_body, geometry, False)

terrain.SetActiveDomain(active_box_dim)

# Construct the terrain
print("Create terrain...")
if patch_type == PatchType.RECTANGULAR:
    # Create a rectangular terrain patch
    terrain.Construct(
        chrono.ChVector3d(
            terrain_length, terrain_width, 0.25
        ),  # length X width X height
        chrono.ChVector3d(terrain_length / 2, 0, 0),  # patch center
        BoxSide_ALL and ~BoxSide_Z_POS,  # all boundaries, except top
    )
elif patch_type == PatchType.HEIGHT_MAP:
    # Create a patch from a height field map image
    terrain.Construct(
        veh.GetVehicleDataFile(
            "terrain/height_maps/bump64.bmp"
        ),  # height map image file
        terrain_length,
        terrain_width,  # length (X) and width (Y)
        chrono.ChVector2d(0.25, 0.55),  # height range
        0.25,  # depth
        True,  # uniform depth
        chrono.ChVector3d(terrain_length / 2, 0, 0),  # patch center
        BoxSide_Z_NEG,  # bottom wall
    )

# Initialize the terrain system
terrain.Initialize()

aabb: chrono.ChAABB = terrain.GetSPHBoundingBox()
print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
print(f"  SPH AABB:          {aabb.min}   {aabb.max}")

# Create run-time visualization
if render:
    # FSI plugin
    col_callback = ParticleHeightColorCallback(aabb.min.z, aabb.max.z)

    visFSI: ChSphVisualizationVSG = ChSphVisualizationVSG(sysFSI)
    visFSI.EnableFluidMarkers(visualization_sph)
    visFSI.EnableBoundaryMarkers(visualization_bndry_bce)
    visFSI.EnableRigidBodyMarkers(visualization_rigid_bce)
    visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)

    # VSG visual system (attach visFSI as plugin)
    visVSG: vsg3d.ChVisualSystemVSG = vsg3d.ChVisualSystemVSG()
    visVSG.AttachPlugin(visFSI)
    visVSG.AttachSystem(system)
    visVSG.SetWindowTitle("Viper rover on CRM deformable terrain")
    visVSG.SetWindowSize(1280, 800)
    visVSG.SetWindowPosition(100, 100)
    visVSG.AddCamera(init_loc + chrono.ChVector3d(0, 6, 0.5), init_loc)
    visVSG.SetLightIntensity(0.9)
    visVSG.Initialize()
    vis = visVSG

# Start the simulation
time = 0
sim_frame = 0
render_frame = 0
exchange_info = 5 * step_size

while time < tend:
    rover.Update()

    # Run-time visualization
    if render and time >= render_frame / render_fps:
        if not vis.Run():
            break
        vis.Render()
        render_frame += 1

    if not render:
        print(f"{time}  {terrain.GetRtfCFD()}  {terrain.GetRtfMBD()}")

    # Advance dynamics of multibody and fluid systems concurrently
    terrain.DoStepDynamics(exchange_info)

    time += exchange_info
    sim_frame += 1
