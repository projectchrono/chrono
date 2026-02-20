import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.vsg3d as vsg3d
import math
import os
from enum import Enum

import importlib.util
from pathlib import Path

# include "demos/SetChronoSolver.h"
file_path = Path(__file__).resolve().parents[1] / "SetChronoSolver.py"

spec = importlib.util.spec_from_file_location("SetChronoSolver", file_path)
SetChronoSolver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(SetChronoSolver)

# -----------------------------------------------------------------------------

# Run-time visualization system (IRRLICHT or VSG)
vis_type = chrono.ChVisualSystem.Type_VSG

# Terrain type (RIGID or SCM)
class TerrainType(Enum):
    RIGID = 1
    SCM = 2

terrain_type = TerrainType.RIGID;

# Tire specification file
# tire_json = "hmmwv/tire/HMMWV_RigidTire.json"
# tire_json = "hmmwv/tire/HMMWV_TMeasyTire.json"
# tire_json = "hmmwv/tire/HMMWV_FialaTire.json"
# tire_json = "hmmwv/tire/HMMWV_Pac89Tire.json"
# tire_json = "hmmwv/tire/HMMWV_Pac02Tire.json"
# tire_json = "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json"
# tire_json = "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json"
# tire_json = "hmmwv/tire/HMMWV_ReissnerTire.json"
tire_json = "Polaris/Polaris_TMeasyTire.json"
# tire_json = "Polaris/Polaris_RigidTire.json"
# tire_json = "Polaris/Polaris_RigidMeshTire.json"
# tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json"

# Wheel specification file
# wheel_json = "hmmwv/wheel/HMMWV_Wheel.json"
wheel_json = "Polaris/Polaris_Wheel.json"

render_fps = 120
debug_output = False

def main():
    # --------------------------------
    # Create wheel and tire subsystems
    # --------------------------------
    wheel = veh.ReadWheelJSON(veh.GetVehicleDataFile(wheel_json))
    tire = veh.ReadTireJSON(veh.GetVehicleDataFile(tire_json))

    # Identify Tire Type
    # Note: In PyChrono, we check types using isinstance or generic casts if necessary
    # Here we assume standard handling unless Deformable is detected
    handling_tire = isinstance(tire, veh.ChForceElementTire)
    fea_tire = isinstance(tire, veh.ChDeformableTire)

    if handling_tire and terrain_type == TerrainType.SCM:
        print("ERROR: Handling tire models cannot be used with SCM terrain.")
        return

    # Set tire contact surface (relevant for FEA tires only)
    if fea_tire:
        collision_family = 7
        surface_type = veh.ChTire.ContactSurfaceType_NODE_CLOUD
        surface_dim = 0.02
        if terrain_type == TerrainType.SCM:
            surface_type = veh.ChTire.ContactSurfaceType_TRIANGLE_MESH
            surface_dim = 0
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
        step_size = 5e-5
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
    SetChronoSolver.SetChronoSolver(sys, solver_type, integrator_type, num_threads_pardiso);
    tire.SetStepsize(step_size)

    # -----------------------------
    # Create and configure test rig
    # -----------------------------

    rig = veh.ChTireTestRig(wheel, tire, sys)

    rig.SetGravitationalAcceleration(9.8)
    rig.SetNormalLoad(3000)
    
    # rig.SetCamberAngle(15 * chrono.CH_DEG_TO_RAD)

    rig.SetTireStepsize(step_size)
    rig.SetTireCollisionType(veh.ChTire.CollisionType_FOUR_POINTS)
    rig.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # Terrain Setup
    patch_size = veh.TerrainPatchSize()
    patch_size.length = 10
    patch_size.width = 1

    if terrain_type == TerrainType.RIGID:
        params = veh.TerrainParamsRigid()
        params.friction = 0.8
        params.restitution = 0
        params.Young_modulus = 2e7
        rig.SetTerrainRigid(patch_size, params)
    else:
        params = veh.TerrainParamsSCM()
        params.Bekker_Kphi = 2e6
        params.Bekker_Kc = 0
        params.Bekker_n = 1.1
        params.Mohr_cohesion = 0
        params.Mohr_friction = 30
        params.Janosi_shear = 0.01
        params.grid_spacing = 0.05
        rig.SetTerrainSCM(patch_size, params)

    # -----------------
    # Set test scenario
    # -----------------
    
    # Scenario: driven wheel
    # rig.SetAngSpeedFunction(chrono.ChFunctionConst(10.0));
    # rig.Initialize();

    # Scenario: pulled wheel
    # rig.SetLongSpeedFunction(chrono.ChFunctionConst(1.0));
    # rig.Initialize();

    # Scenario: imobilized wheel
    # rig.SetLongSpeedFunction(chrono.ChFunctionConst(0.0));
    # rig.SetAngSpeedFunction(chrono.ChFunctionConst(0.0));
    # rig.Initialize();

    # Scenario: prescribe all motion functions
    #   longitudinal speed: 0.2 m/s
    #   angular speed: 10 RPM
    #   slip angle: sinusoidal +- 5 deg with 5 s period
    rig.SetLongSpeedFunction(chrono.ChFunctionConst(0.2))
    rig.SetAngSpeedFunction(chrono.ChFunctionConst(10 * chrono.CH_RPM_TO_RAD_S))
    rig.SetSlipAngleFunction(chrono.ChFunctionSine(5 * chrono.CH_DEG_TO_RAD, 0.2))

    # Scenario: specified longitudinal slip (overrrides other definitions of motion functions)
    # rig.SetConstantLongitudinalSlip(0.2, 0.1);

    # Set delay before applying inputs (settling time)
    input_time_delay = 1.0
    rig.SetTimeDelay(input_time_delay)

    # Initialize the tire test rig; in TEST mode set a drop speed of 0.05
    # rig.Initialize(veh.ChTireTestRig.Mode_SUSPEND)
    # rig.Initialize(veh.ChTireTestRig.Mode_DROP)
    rig.Initialize(veh.ChTireTestRig.Mode_TEST, 0.05)

    # Optionally, modify tire visualization (can be done only after initialization)
    if (isinstance(tire, veh.ChDeformableTire)):
        tire_def = chrono.CastToChDeformableTire(tire)

        visFEA = chrono.ChVisualShapeFEA()
        visFEA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
        visFEA.SetShellResolution(3)
        visFEA.SetWireframe(False)
        visFEA.SetColormapRange(0.0, 5.0)
        visFEA.SetSmoothFaces(True)
        tire_def.AddVisualShapeFEA(visFEA)

    # -----------------
    # Initialize Output
    # -----------------
    out_dir = os.path.join(chrono.GetChronoOutputPath(), "TIRE_TEST_RIG")
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # ---------------------------------
    # Create the run-time visualization
    # ---------------------------------

    vis = None
    if vis_type == chrono.ChVisualSystem.Type_IRRLICHT:
        vis_irr = chronoirr.ChVisualSystemIrrlicht()
        vis_irr.AttachSystem(sys)
        vis_irr.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis_irr.SetWindowSize(1200, 600)
        vis_irr.SetWindowTitle("Tire Test Rig")        
        vis_irr.Initialize()
        vis_irr.AddLogo()
        vis_irr.AddSkyBox()
        vis_irr.AddCamera(chrono.ChVector3d(1.0, 2.5, 1.0))
        vis_irr.AddLightDirectional()

        # vis_irr->GetActiveCamera()->setFOV(irr::core::PI / 4.5f);

        vis = vis_irr
    if vis_type == chrono.ChVisualSystem.Type_VSG:
        vis_vsg = vsg3d.ChVisualSystemVSG()
        vis_vsg.AttachSystem(sys)
        vis_vsg.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis_vsg.SetWindowSize(1280, 800)
        vis_vsg.SetWindowTitle("Tire Test Rig")        
        vis_vsg.AddCamera(chrono.ChVector3d(1.0, 2.5, 1.0))
        vis_vsg.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
        vis_vsg.EnableShadows()
        vis_vsg.Initialize()

        # vis_vsg.GetActiveCamera().setFOV(irr::core::PI / 4.5f);

        vis = vis_vsg

    # ----------------
    # Simulation loop
    # ----------------

    # Timers and counters
    timer = chrono.ChTimer() # timer for measuring total run time
    time = 0 # simulated time
    sim_time = 0 # simulation time
    render_frame = 0 # render frame counter
    
    timer.start()    
    while vis.Run():
        time = sys.GetChTime()

        # Render Logic
        if time >= render_frame / render_fps:
            loc = rig.GetPos()
            vis.UpdateCamera(loc + chrono.ChVector3d(1.0, 2.5, 0.5), 
                                 loc + chrono.ChVector3d(0, 0.25, -0.25))
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        rig.Advance(step_size)
        sim_time += sys.GetTimerStep()

        long_slip = rig.GetLongitudinalSlip()
        slip_angle = rig.GetSlipAngle() * chrono.CH_RAD_TO_DEG
        camber_angle = rig.GetCamberAngle() * chrono.CH_RAD_TO_DEG

        if (debug_output and rig.OutputEnabled()):
            print(time)
            print(f"   {long_slip} {slip_angle} {camber_angle}")
            tforce = rig.ReportTireForce()
            frc = tforce.force
            pnt = tforce.point
            trq = tforce.moment
            print(f"   {frc.x} {frc.y} {frc.z}")
            print(f"   {pnt.x} {pnt.y} {pnt.z}")
            print(f"   {trq.x} {trq.y} {trq.z}")
        else:
            print(f"RTF: {sys.GetRTF()}", end = "\r")
    
    timer.stop()

    total_time = timer()
    print(f"Simulated time: {time}")
    print(f"Run time (simulation): {sim_time}  |  RTF: {sim_time / time}")
    print(f"Run time (total):      {total_time}  |  RTF: {total_time / time}")


if __name__ == "__main__":
    main()