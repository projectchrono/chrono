import sys
import os
import argparse
import math
import time
from pathlib import Path

import pychrono as chrono
import pychrono.fsi as fsi
import pychrono.vsg3d as vsg3d


# Dimensions of fluid domain
fsize = chrono.ChVector3d(0.8, 0.8, 1.2)

# Object type
OBJ_BOX = "box"
OBJ_SPHERE = "sphere"
OBJ_CYLINDER = "cylinder"
OBJ_MESH = "mesh"
object_shape = OBJ_CYLINDER

# Mesh specification (for object_shape = ObjectShape::MESH)
mesh_obj_filename = chrono.GetChronoDataFile("models/semicapsule.obj")

mesh_scale = 1.0
mesh_bottom_offset = 0.1

# Object density
density = 500.0

# Object initial height above floor (as a ratio of fluid height)
initial_height = 1.05

# Visibility flags
show_rigid = True
show_rigid_bce = False
show_boundary_bce = True
show_particles_sph = True


# -------------------------
# Command-line parsing
# -------------------------
def parse_args(argv):
    parser = argparse.ArgumentParser(description="FSI object drop demo (Python translation)")
    parser.add_argument("--t_end", type=float, default=3.0, help="Simulation duration [s]")
    parser.add_argument("--quiet", action="store_true", help="Disable verbose terminal output")
    parser.add_argument("--output_particle_data", type=str, default="false",
                        help="Enable collection of output files (true/false)")
    parser.add_argument("--output_fps", type=float, default=20.0, help="Output frequency [fps]")
    parser.add_argument("--no_vis", action="store_true", help="Disable run-time visualization")
    parser.add_argument("--render_fps", type=float, default=400.0, help="Render frequency [fps]")
    parser.add_argument("--snapshots", type=str, default="false", help="Enable writing snapshot image files")
    parser.add_argument("--ps_freq", type=int, default=1, help="Frequency of Proximity Search")
    parser.add_argument("--boundary_method", type=str, default="adami",
                        choices=["adami", "holmes"], help="Boundary condition type")
    parser.add_argument("--viscosity_method", type=str, default="artificial_unilateral",
                        choices=["laminar", "artificial_unilateral", "artificial_bilateral"],
                        help="Viscosity type")
    parser.add_argument("--use_variable_time_step", type=str, default="true", help="Use variable time step (true/false)")
    return parser.parse_args(argv)


def parse_bool(s):
    s2 = str(s).lower()
    return s2 in ("1", "true", "yes", "y", "t")


# -------------------------
# Main
# -------------------------
def main(argv):
    args = parse_args(argv)

    # simulation parameters
    initial_spacing = 0.025
    # If variable time step is enabled, this step size is only used for the first time step
    step_size = 1e-4

    t_end = args.t_end
    verbose = not args.quiet
    output = parse_bool(args.output_particle_data)
    output_fps = args.output_fps
    render = not args.no_vis
    render_fps = args.render_fps
    snapshots = parse_bool(args.snapshots)
    ps_freq = args.ps_freq
    use_variable_time_step = parse_bool(args.use_variable_time_step)
    boundary_method = args.boundary_method
    viscosity_method = args.viscosity_method

    # Set output root directory
    chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

    # Create the Chrono system and associated collision system
    sysMBS = chrono.ChSystemNSC()
    sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the FSI problem
    fsiProblemCartesian = fsi.ChFsiProblemCartesian(initial_spacing, sysMBS)
    fsiProblemCartesian.SetVerbose(verbose)
    sysFSI = fsiProblemCartesian.GetFsiSystemSPH()

    # Set gravitational acceleration
    gravity = chrono.ChVector3d(0, 0, -9.8)
    fsiProblemCartesian.SetGravitationalAcceleration(gravity)

    # Set integration step size
    fsiProblemCartesian.SetStepSizeCFD(step_size)
    fsiProblemCartesian.SetStepsizeMBD(step_size)

    # Meta-step (communication interval)
    meta_time_step = 5 * step_size

    # Set CFD fluid properties
    fluid_props = fsi.FluidProperties()
    fluid_props.density = 1000.0
    fluid_props.viscosity = 1.0

    fsiProblemCartesian.SetCfdSPH(fluid_props)

    # Set SPH solution parameters
    sph_params = fsi.SPHParameters()
    sph_params.integration_scheme = fsi.IntegrationScheme_RK2
    sph_params.num_bce_layers = 4
    sph_params.initial_spacing = initial_spacing
    sph_params.d0_multiplier = 1
    # Compute max velocity as root of 2*g*h
    sph_params.max_velocity = 4.538
    sph_params.shifting_method = fsi.ShiftingMethod_XSPH
    sph_params.shifting_xsph_eps = 0.5
    sph_params.artificial_viscosity = 0.03
    sph_params.eos_type = fsi.EosType_TAIT
    sph_params.use_consistent_gradient_discretization = False
    sph_params.use_consistent_laplacian_discretization = False
    sph_params.num_proximity_search_steps = ps_freq
    sph_params.use_delta_sph = True
    sph_params.delta_sph_coefficient = 0.1
    sph_params.use_variable_time_step = use_variable_time_step

    # Set boundary and viscosity types
    if boundary_method == "holmes":
        sph_params.boundary_method = fsi.BoundaryMethod_HOLMES
    else:
        sph_params.boundary_method = fsi.BoundaryMethod_ADAMI

    if viscosity_method == "laminar":
        sph_params.viscosity_method = fsi.ViscosityMethod_LAMINAR
    elif viscosity_method == "artificial_bilateral":
        sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
    else:
        sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_UNILATERAL

    fsiProblemCartesian.SetSPHParameters(sph_params)

    # Set surface reconstruction parameters
    splashsurf_params = fsi.SplashsurfParameters()
    splashsurf_params.smoothing_length = 2.0
    splashsurf_params.cube_size = 0.3
    splashsurf_params.surface_threshold = 0.6
    fsiProblemCartesian.SetSplashsurfParameters(splashsurf_params)

    # Create the rigid body
    bottom_offset = 0.0
    mass = 0.0
    inertia = chrono.ChMatrix33d()
    geometry = chrono.ChBodyGeometry()
    geometry.materials.append(chrono.ChContactMaterialData())

    if object_shape == OBJ_BOX:
        size = chrono.ChVector3d(0.20, 0.20, 0.10)
        bottom_offset = size.z / 2
        box = chrono.ChBox(size)
        mass = density * box.GetVolume()
        inertia = mass * box.GetGyration()
        geometry.coll_boxes.append(chrono.BoxShape(chrono.VNULL, chrono.QUNIT, box))
    elif object_shape == OBJ_SPHERE:
        radius = 0.12
        bottom_offset = radius
        sphere = chrono.ChSphere(radius)
        mass = density * sphere.GetVolume()
        inertia = mass * sphere.GetGyration()
        geometry.coll_spheres.append(chrono.SphereShape(chrono.VNULL, sphere, 0))
    elif object_shape == OBJ_CYLINDER:
        radius = 0.12
        length = 0.20
        bottom_offset = radius
        cylinder = chrono.ChCylinder(radius, length)
        mass = density * cylinder.GetVolume()
        inertia = mass * cylinder.GetGyration()
        geometry.coll_cylinders.append(
            chrono.CylinderShape(chrono.VNULL, chrono.Q_ROTATE_Z_TO_X, cylinder, 0))
    elif object_shape == OBJ_MESH:
        trimesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(mesh_obj_filename, True, True)
        com = chrono.ChVector3d()
        trimesh.ComputeMassProperties(True, mass, com, inertia, mesh_scale)
        mass *= density
        inertia *= density
        bottom_offset = mesh_bottom_offset
        geometry.coll_meshes.append(
            chrono.TrimeshShape(chrono.VNULL, chrono.QUNIT, mesh_obj_filename, chrono.VNULL, mesh_scale, 0.01, 0))
    else:
        raise ValueError("Unsupported object shape: " + object_shape)

    body = chrono.ChBody()
    height = initial_height * fsize.z + bottom_offset
    body.SetName("object")
    body.SetPos(chrono.ChVector3d(0, 0, height))
    body.SetRot(chrono.QUNIT)
    body.SetMass(mass)
    body.SetInertia(inertia)
    body.SetFixed(False)
    body.EnableCollision(False)
    sysMBS.AddBody(body)

    if show_rigid:
        geometry.CreateVisualizationAssets(body, chrono.VisualizationType_COLLISION)

    # Add as an FSI body
    fsiProblemCartesian.AddRigidBody(body, geometry, True, True)

    print("FSI body:")
    print("   initial height = {:.6f}".format(height))
    print("   mass = {:.6f}".format(mass))
    print("   inertia = ")
    print(inertia)

    # Enable depth-based initial pressure for SPH particles
    fsiProblemCartesian.RegisterParticlePropertiesCallback(fsi.DepthPressurePropertiesCallback(fsize.z))

    # Create SPH material and boundaries
    fsiProblemCartesian.Construct(fsize, chrono.ChVector3d(0, 0, 0), fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS)

    # Initialize FSI
    fsiProblemCartesian.Initialize()

    domain_aabb = fsiProblemCartesian.GetComputationalDomain()
    print("Computational domain:")
    print("  min:", domain_aabb.min)
    print("  max:", domain_aabb.max)

    # Output directories
    out_dir = chrono.GetChronoOutputPath() + "FSI_Object_Drop/"
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    out_dir = out_dir + fsiProblemCartesian.GetSphIntegrationSchemeString() + "_" + viscosity_method + "_" + boundary_method + "_ps" + str(ps_freq)
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    if output:
        Path(out_dir + "/particles").mkdir(parents=True, exist_ok=True)
        Path(out_dir + "/fsi").mkdir(parents=True, exist_ok=True)
        Path(out_dir + "/vtk").mkdir(parents=True, exist_ok=True)

    Path(out_dir + "/snapshots").mkdir(parents=True, exist_ok=True)
    Path(out_dir + "/meshes").mkdir(parents=True, exist_ok=True)

    # Visualization setup (VSG) if available
    vis = None
    if render:
        # FSI plugin
        col_callback = fsi.ParticlePressureColorCallback(-1000, 12000, True)
        plane1 = fsi.MarkerPlanesVisibilityCallback_Plane()
        plane1.point = chrono.VNULL
        plane1.normal = chrono.ChVector3d(1, 0, 0)
        plane2 = fsi.MarkerPlanesVisibilityCallback_Plane()
        plane2.point = chrono.VNULL
        plane2.normal = chrono.ChVector3d(0, 1, 0)
        planes = fsi.vector_MarkerPlanesVisibilityCallback_Plane()
        planes.append(plane1)
        planes.append(plane2)

        mode = fsi.MarkerPlanesVisibilityCallback.Mode_ALL
        vis_callback_SPH = fsi.MarkerPlanesVisibilityCallback(planes, mode)
        vis_callback_BCE = fsi.MarkerPlanesVisibilityCallback(planes, mode)

        visFSI = fsi.ChSphVisualizationVSG(sysFSI)
        visFSI.EnableFluidMarkers(show_particles_sph)
        visFSI.EnableBoundaryMarkers(show_boundary_bce)
        visFSI.EnableRigidBodyMarkers(show_rigid_bce)
        visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_RED_BLUE)
        visFSI.SetSPHVisibilityCallback(vis_callback_SPH)
        visFSI.SetBCEVisibilityCallback(vis_callback_BCE)
        

        visVSG = vsg3d.ChVisualSystemVSG()
        visVSG.AttachPlugin(visFSI)
        visVSG.AttachSystem(sysMBS)
        visVSG.SetWindowTitle("Object Drop")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.AddCamera(chrono.ChVector3d(2.5 * fsize.x, 2.5 * fsize.y, 1.5 * fsize.z),
                            chrono.ChVector3d(0, 0, 0.5 * fsize.z))
        visVSG.SetLightIntensity(0.9)
        visVSG.SetLightDirection(math.pi / 2.0, math.pi / 6.0)
        visVSG.Initialize()
        vis = visVSG
        
    # Prepare output files
    if use_variable_time_step:
        out_file = out_dir + "/results_variable_time_step.txt"
    else:
        out_file = out_dir + "/results_fixed_time_step.txt"
    ofile = open(out_file, "w")

    # Start the simulation
    time_sim = 0.0
    sim_frame = 0
    out_frame = 0
    render_frame = 0

    tstart = time.time()
    try:
        while time_sim < t_end:
            body_height = body.GetPos().z
            ofile.write(f"{time_sim}\t{body_height}\n")

            if output and time_sim >= out_frame / output_fps:
                if verbose:
                    print(f" -- Output frame {out_frame} at t = {time_sim:.6f}")
                fsi.SaveOutputData(time_sim, out_dir + "/particles", out_dir + "/fsi")
                out_frame += 1

            # Render FSI system
            if render and vis is not None and time_sim >= render_frame / render_fps:
                if not vis.Run():
                    break
                
                vis.Render()

                if snapshots:
                    if verbose:
                        print(f" -- Snapshot frame {render_frame} at t = {time_sim:.6f}")
                    filename = f"{out_dir}/snapshots/img_{render_frame+1:05d}.bmp"
                    vis.WriteImageToFile(filename)
                
                if 70 <= render_frame < 80:
                    meshname = f"mesh_{render_frame+1:05d}"
                    fsiProblemCartesian.WriteReconstructedSurface(out_dir + "/meshes", meshname, True)

                render_frame += 1

            # Advance dynamics of the FSI system to next communication time
            fsiProblemCartesian.DoStepDynamics(meta_time_step)

            time_sim += meta_time_step
            sim_frame += 1

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        tend = time.time()
        ofile.close()

    print("End Time:", t_end)
    print("\nSimulation time: {:.6f} seconds\n".format(tend - tstart))

    # Write RTF file
    if use_variable_time_step:
        rtf_name = out_dir + "/rtf_variable_time_step.txt"
    else:
        rtf_name = out_dir + "/rtf_fixed_time_step.txt"
    with open(rtf_name, "w") as rtf_file:
        rtf_file.write("time (s)\twall clock time (s)\tRTF\n")
        wall = tend - tstart
        rtf = wall / t_end if t_end > 0 else 0.0
        rtf_file.write(f"{t_end}\t{wall}\t{rtf}\n")

    if use_variable_time_step:
        fsiProblemCartesian.PrintStats()
        fsiProblemCartesian.PrintTimeSteps(out_dir + "/time_steps.txt")


if __name__ == "__main__":
    main(sys.argv[1:])
