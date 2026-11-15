import pychrono as chrono
import pychrono.vehicle as veh
import pytest

# Run-time selection of the SCM backend (CPU or GPU) from Python.
# The API is the same in every build; without a usable GPU, a GPU request runs on the CPU.

AUTO = veh.SCMTerrain.Backend_AUTO
CPU = veh.SCMTerrain.Backend_CPU
GPU = veh.SCMTerrain.Backend_GPU


def make_scene():
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetNumThreads(1)

    # Fixed 2 m cube (triangle mesh, as required by GPU ray casting) pressed 1 cm into the soil
    mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(chrono.GetChronoDataFile("models/cube.obj"), True, True)
    material = chrono.ChContactMaterialSMC()
    body = chrono.ChBody()
    body.SetFixed(True)
    body.SetPos(chrono.ChVector3d(0, 0, 0.99))
    body.AddCollisionShape(chrono.ChCollisionShapeTriangleMesh(material, mesh, True, False, 0))
    body.EnableCollision(True)
    sys.AddBody(body)

    terrain = veh.SCMTerrain(sys, False)
    terrain.AddActiveDomain(body, chrono.VNULL, chrono.ChVector3d(2.2, 2.2, 2.2))
    return sys, body, terrain


def run(sys, terrain, num_steps=3):
    terrain.Initialize(4, 4, 0.1)
    for _ in range(num_steps):
        sys.DoStepDynamics(1e-3)


def vertical_force(terrain, body):
    force = chrono.ChVector3d()
    torque = chrono.ChVector3d()
    assert terrain.GetContactForceBody(body, force, torque)
    return force.z


def test_backend_selection():
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    terrain = veh.SCMTerrain(sys, False)

    assert terrain.GetRayCastBackend() == AUTO
    assert terrain.GetContactForceBackend() == AUTO

    terrain.SetBackend(CPU)
    assert terrain.GetRayCastBackend() == CPU
    assert terrain.GetContactForceBackend() == CPU

    terrain.SetRayCastBackend(GPU)
    assert terrain.GetRayCastBackend() == GPU
    assert terrain.GetContactForceBackend() == CPU

    settings = veh.GpuSettings()
    settings.min_hits = 17
    settings.raycast_fp64 = True
    terrain.SetGpuSettings(settings)
    assert terrain.GetGpuSettings().min_hits == 17
    assert terrain.GetGpuSettings().raycast_fp64

    assert isinstance(veh.SCMTerrain.IsGpuAvailable(), bool)


def test_cpu_backend():
    sys, body, terrain = make_scene()
    terrain.SetBackend(CPU)
    run(sys, terrain)

    assert terrain.GetNumRaycastGpuSteps() == 0
    assert terrain.GetNumContactForceGpuSteps() == 0
    assert terrain.GetNumRayCasts() > 0
    assert vertical_force(terrain, body) > 0


def test_gpu_backend():
    sys, body, terrain = make_scene()
    terrain.SetBackend(GPU)
    run(sys, terrain)

    if veh.SCMTerrain.IsGpuAvailable():
        assert terrain.GetNumRaycastGpuSteps() > 0
        assert terrain.GetNumContactForceGpuSteps() > 0
    else:
        assert terrain.GetNumRaycastGpuSteps() == 0
        assert terrain.GetNumContactForceGpuSteps() == 0
    assert vertical_force(terrain, body) > 0


def test_cpu_and_gpu_forces_agree():
    if not veh.SCMTerrain.IsGpuAvailable():
        pytest.skip("No usable GPU")

    forces = []
    for backend in (CPU, GPU):
        sys, body, terrain = make_scene()
        terrain.SetRayCastBackend(CPU)  # same hits for both runs
        terrain.SetContactForceBackend(backend)
        run(sys, terrain)
        forces.append(vertical_force(terrain, body))

    assert forces[1] == pytest.approx(forces[0], rel=1e-6)
