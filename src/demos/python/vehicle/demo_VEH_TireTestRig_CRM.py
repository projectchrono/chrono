import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
import pychrono.vsg3d as vsg3d
import pychrono.postprocess as postprocess

import importlib.util
from pathlib import Path

# include "demos/SetChronoSolver.h"
file_path = Path(__file__).resolve().parents[1] / "SetChronoSolver.py"

spec = importlib.util.spec_from_file_location("SetChronoSolver", file_path)
SetChronoSolver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(SetChronoSolver)

# -----------------------------------------------------------------------------

# Tire specification file
tire_json = "Polaris/Polaris_RigidMeshTire.json"
# tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json"

# Wheel specification file
wheel_json = "Polaris/Polaris_Wheel.json"

render_fps = 100
debug_output = False
gnuplot_output = True
blender_output = False

render = True

# -----------------------------------------------------------------------------

def main():
    # --------------------------------
    # Create wheel and tire subsystems
    # --------------------------------
    wheel = veh.ReadWheelJSON(veh.GetVehicleDataFile(wheel_json))
    tire = veh.ReadTireJSON(veh.GetVehicleDataFile(tire_json))

    fea_tire = isinstance(tire, veh.ChDeformableTire)

    # Set tire contact surface (relevant for FEA tires only)
    if fea_tire:
        collision_family = 7
        surface_type = veh.ChTire.ContactSurfaceType_TRIANGLE_MESH
        surface_dim = 0.0
        tire.SetContactSurfaceType(surface_type, surface_dim, collision_family)

    # ---------------------------------------------------------
    # Create system and set default solver and integrator types
    # ---------------------------------------------------------
    sys = None
    step_size = 0
    solver_type = None
    integrator_type = None

    if fea_tire:
        sys = chrono.ChSystemSMC()
        step_size = 1e-5
        solver_type = chrono.ChSolver.Type_PARDISO_MKL
        integrator_type = chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED
    else:
        sys = chrono.ChSystemNSC()
        step_size = 2e-4
        solver_type = chrono.ChSolver.Type_BARZILAIBORWEIN
        integrator_type = chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED

    # Set collision system
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Number of OpenMP threads used in Chrono (SCM ray-casting and FEA)
    num_threads_chrono = min(8, chrono.ChOMP.GetNumProcs())

    # Number of threads used in collision detection
    num_threads_collision = 1

    # Number of threads used by Eigen
    num_threads_eigen = 1

    # Number of threads used by PardisoMKL
    num_threads_pardiso = min(8, chrono.ChOMP.GetNumProcs())

    sys.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen)
    SetChronoSolver.SetChronoSolver(sys, solver_type, integrator_type, num_threads_pardiso)
    tire.SetStepsize(step_size)

    # -----------------------------
    # Create and configure test rig
    # -----------------------------
    rig = veh.ChTireTestRig(wheel, tire, sys)

    rig.SetGravitationalAcceleration(9.8)
    rig.SetNormalLoad(2500)

    rig.SetTireStepsize(step_size)
    rig.SetTireVisualizationType(chrono.VisualizationType_COLLISION)

    size = veh.TerrainPatchSize()
    size.length = 10
    size.width = 1
    size.depth = 0.2

    params = veh.TerrainParamsCRM()
    params.sph_params.initial_spacing = 0.02
    params.mat_props.density = 1700
    params.mat_props.Young_modulus = 2e6
    params.mat_props.cohesion_coeff = 1e2

    rig.SetTerrainCRM(size, params)

    # -----------------
    # Set test scenario
    # -----------------
    rig.SetLongSpeedFunction(chrono.ChFunctionConst(0.2))
    rig.SetAngSpeedFunction(chrono.ChFunctionConst(10 * chrono.CH_RPM_TO_RAD_S))

    input_time_delay = 1.0
    rig.SetTimeDelay(input_time_delay)
    rig.Initialize(veh.ChTireTestRig.Mode_TEST, 0.05)

    # Optionally, modify tire visualization (can be done only after initialization)
    if isinstance(tire, veh.ChDeformableTire):
        tire_def = veh.CastToChDeformableTire(tire)
        visFEA = chrono.ChVisualShapeFEA()
        visFEA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
        visFEA.SetShellResolution(3)
        visFEA.SetWireframe(False)
        visFEA.SetColormapRange(0.0, 5.0)
        visFEA.SetSmoothFaces(True)
        tire_def.AddVisualShapeFEA(visFEA)

    # -----------------
    # Initialize output
    # -----------------
    out_dir = chrono.GetChronoOutputPath() + "TIRE_TEST_RIG"
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # ---------------------------------
    # Create the run-time visualization
    # ---------------------------------
    vis = None
    if render:
        # FSI plugin
        sysFSI = veh.CastToCRMTerrain(rig.GetTerrain()).GetFsiSystemSPH()        
        visFSI = fsi.ChSphVisualizationVSG(sysFSI)

        visFSI.EnableFluidMarkers(True)
        visFSI.EnableBoundaryMarkers(True)
        visFSI.EnableRigidBodyMarkers(True)

        visVSG = vsg3d.ChVisualSystemVSG()
        visVSG.AttachPlugin(visFSI)
        visVSG.AttachSystem(sys)
        visVSG.SetWindowTitle("Tire Test Rig on CRM deformable terrain")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.AddCamera(chrono.ChVector3d(1.0, 2.5, 1.0), chrono.ChVector3d(0, 1, 0))
        visVSG.SetLightIntensity(0.9)
        visVSG.SetLightDirection(chrono.CH_PI_2, chrono.CH_PI / 6)

        visVSG.Initialize()
        vis = visVSG

    # ---------------
    # Simulation loop
    # ---------------

    # Timers and counters
    timer = chrono.ChTimer() # timer for measuring total run time
    time = 0.0 # simulated time
    sim_time = 0.0 # simulation time
    render_frame = 0 # render frame counter
    sim_time_max = 10.0

    # Data collection
    long_slip_fct = chrono.ChFunctionInterp()
    slip_angle_fct = chrono.ChFunctionInterp()
    camber_angle_fct = chrono.ChFunctionInterp()

    timer.start()
    while time < sim_time_max:
        time = sys.GetChTime()

        if time >= render_frame / render_fps:
            loc = rig.GetPos()
            vis.UpdateCamera(loc + chrono.ChVector3d(1.0, 2.5, 0.5), loc + chrono.ChVector3d(0, 0.25, -0.25))

            if not vis.Run():
                break
            vis.Render()
            render_frame += 1

        rig.Advance(step_size)
        sim_time += sys.GetTimerStep()

        long_slip = rig.GetLongitudinalSlip()
        slip_angle = rig.GetSlipAngle() * chrono.CH_RAD_TO_DEG
        camber_angle = rig.GetCamberAngle() * chrono.CH_RAD_TO_DEG

        if debug_output and rig.OutputEnabled():
            print(f"{time}")
            print(f"   {long_slip} {slip_angle} {camber_angle}")
            tforce = rig.ReportTireForce()
            frc = tforce.force
            pnt = tforce.point
            trq = tforce.moment
            print(f"   {frc.x} {frc.y} {frc.z}")
            print(f"   {pnt.x} {pnt.y} {pnt.z}")
            print(f"   {trq.x} {trq.y} {trq.z}")
        else:
            print(f"\rRTF: {sys.GetRTF()}", end="")
            
    timer.stop()
    step_time = timer.GetTimeSeconds()
    
    print(f"\nSimulated time: {time}")
    print(f"Run time (simulation): {sim_time}  |  RTF: {sim_time / time if time > 0 else 0}")
    print(f"Run time (total):      {step_time}  |  RTF: {step_time / time if time > 0 else 0}")

if __name__ == "__main__":
    main()