# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban, Bocheng Zou
# =============================================================================
#
# Demonstration of the Chrono::VSG run-time visualization system
#
# =============================================================================

import pychrono.core as chrono
import pychrono.vsg3d as vsg
import os

def CreateMeshShape(filename):
    """Create a mesh shape from a Wavefront OBJ file."""
    trimesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(filename, True, True)
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(trimesh)
    # Set name from filename (equivalent to filesystem::path(filename).stem())
    name = os.path.splitext(os.path.basename(filename))[0]
    trimesh_shape.SetName(name)
    trimesh_shape.SetMutable(False)
    
    return trimesh_shape

def main():
    # Create a Chrono physical system
    sys = chrono.ChSystemNSC()

    # Create the VSG visualization system
    vis = vsg.ChVisualSystemVSG()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(chrono.ChVector2i(1200, 800))
    vis.SetWindowPosition(chrono.ChVector2i(100, 300))
    vis.SetWindowTitle("Chrono VSG Assets")
    vis.AddCamera(chrono.ChVector3d(8.0, 12.3, 3.0), chrono.ChVector3d(-0.1, 1.0, 0.4))
    vis.SetCameraAngleDeg(40)
    vis.SetLightIntensity(1.0)
    vis.SetLightDirection(chrono.CH_PI_2, chrono.CH_PI_4)

    # Vehicle positions
    bus_pos = chrono.ChVector3d(3, -4.5, 0)
    polaris_pos = chrono.ChVector3d(-3, -1.5, 0)
    suv_pos = chrono.ChVector3d(-3, 1.5, 0)
    uaz_pos = chrono.ChVector3d(-3, 4.5, 0)
    hmmwv_pos = chrono.ChVector3d(3, -1.5, 0)
    audi_pos = chrono.ChVector3d(3, 1.5, 0)
    gator_pos = chrono.ChVector3d(3, 4.5, 0)

    # Create Bus
    bus_wpos = [chrono.ChVector3d(0, 1.128, 0.5), chrono.ChVector3d(0, -1.128, 0.5),
                chrono.ChVector3d(-7.184, 0.96, 0.5), chrono.ChVector3d(-7.184, -0.96, 0.5)]
    bus = chrono.ChVisualModel()
    bus_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/citybus/CityBus_Vis.obj"))
    bus_wheel = CreateMeshShape(chrono.GetChronoDataFile("vehicle/citybus/CityBusRim.obj"))
    bus_tire = CreateMeshShape(chrono.GetChronoDataFile("vehicle/citybus/CityBusTire.obj"))
    bus.AddShape(bus_chassis)
    for i in range(4):
        bus.AddShape(bus_wheel, chrono.ChFramed(bus_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
        bus.AddShape(bus_tire, chrono.ChFramed(bus_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
    vis.AddVisualModel(bus, chrono.ChFramed(bus_pos, chrono.QUNIT))

    # Create HMMWV
    hmmwv_wpos = [chrono.ChVector3d(1.64, 0.910, -0.026), chrono.ChVector3d(1.64, -0.910, -0.026),
                  chrono.ChVector3d(-1.64, 0.910, -0.026), chrono.ChVector3d(-1.64, -0.910, -0.026)]
    hmmwv = chrono.ChVisualModel()
    hmmwv_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"))
    hmmwv_wheel = CreateMeshShape(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_rim.obj"))
    hmmwv_tireL = CreateMeshShape(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_left.obj"))
    hmmwv_tireR = CreateMeshShape(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_right.obj"))
    hmmwv.AddShape(hmmwv_chassis)
    for i in range(4):
        hmmwv.AddShape(hmmwv_wheel, chrono.ChFramed(hmmwv_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
    hmmwv.AddShape(hmmwv_tireL, chrono.ChFramed(hmmwv_wpos[0], chrono.QuatFromAngleZ(chrono.CH_PI * 0)))
    hmmwv.AddShape(hmmwv_tireR, chrono.ChFramed(hmmwv_wpos[1], chrono.QuatFromAngleZ(chrono.CH_PI * 1)))
    hmmwv.AddShape(hmmwv_tireL, chrono.ChFramed(hmmwv_wpos[2], chrono.QuatFromAngleZ(chrono.CH_PI * 2)))
    hmmwv.AddShape(hmmwv_tireR, chrono.ChFramed(hmmwv_wpos[3], chrono.QuatFromAngleZ(chrono.CH_PI * 3)))
    vis.AddVisualModel(hmmwv, chrono.ChFramed(hmmwv_pos, chrono.QUNIT))

    # Create Gator
    gator_wpos = [chrono.ChVector3d(0.97, 0.56, -0.02), chrono.ChVector3d(0.97, -0.56, -0.02),
                  chrono.ChVector3d(-0.97, 0.62, 0), chrono.ChVector3d(-0.97, -0.62, 0)]
    gator = chrono.ChVisualModel()
    gator_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/gator/gator_chassis.obj"))
    gator_wheelF = CreateMeshShape(chrono.GetChronoDataFile("vehicle/gator/gator_wheel_FL.obj"))
    gator_wheelR = CreateMeshShape(chrono.GetChronoDataFile("vehicle/gator/gator_wheel_RL.obj"))
    gator.AddShape(gator_chassis)
    gator.AddShape(gator_wheelF, chrono.ChFramed(gator_wpos[0], chrono.QuatFromAngleZ(chrono.CH_PI * 0)))
    gator.AddShape(gator_wheelF, chrono.ChFramed(gator_wpos[1], chrono.QuatFromAngleZ(chrono.CH_PI * 1)))
    gator.AddShape(gator_wheelR, chrono.ChFramed(gator_wpos[2], chrono.QuatFromAngleZ(chrono.CH_PI * 2)))
    gator.AddShape(gator_wheelR, chrono.ChFramed(gator_wpos[3], chrono.QuatFromAngleZ(chrono.CH_PI * 3)))
    vis.AddVisualModel(gator, chrono.ChFramed(gator_pos, chrono.QUNIT))

    # Create Audi
    audi_wpos = [chrono.ChVector3d(1.44, 0.8, 0.12), chrono.ChVector3d(1.44, -0.8, 0.12),
                 chrono.ChVector3d(-1.48, 0.8, 0.12), chrono.ChVector3d(-1.48, -0.8, 0.12)]
    audi = chrono.ChVisualModel()
    audi_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/audi/audi_chassis_white.obj"))
    audi_wheel = CreateMeshShape(chrono.GetChronoDataFile("vehicle/audi/audi_rim.obj"))
    audi_tire = CreateMeshShape(chrono.GetChronoDataFile("vehicle/audi/audi_tire.obj"))
    audi.AddShape(audi_chassis)
    for i in range(4):
        audi.AddShape(audi_wheel, chrono.ChFramed(audi_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
        audi.AddShape(audi_tire, chrono.ChFramed(audi_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
    vis.AddVisualModel(audi, chrono.ChFramed(audi_pos, chrono.QUNIT))

    # Create UAZ
    uaz_wpos = [chrono.ChVector3d(0, 0.733, 0), chrono.ChVector3d(0, -0.733, 0),
                chrono.ChVector3d(-2.3, 0.733, 0), chrono.ChVector3d(-2.3, -0.733, 0)]
    uaz = chrono.ChVisualModel()
    uaz_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/uaz/uazbus_chassis.obj"))
    uaz_wheel = CreateMeshShape(chrono.GetChronoDataFile("vehicle/uaz/uaz_rim.obj"))
    uaz_tire = CreateMeshShape(chrono.GetChronoDataFile("vehicle/uaz/uaz_tire.obj"))
    uaz.AddShape(uaz_chassis)
    for i in range(4):
        uaz.AddShape(uaz_wheel, chrono.ChFramed(uaz_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
        uaz.AddShape(uaz_tire, chrono.ChFramed(uaz_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
    vis.AddVisualModel(uaz, chrono.ChFramed(uaz_pos, chrono.QUNIT))

    # Create SUV
    suv_wpos = [chrono.ChVector3d(0, 0.960, 0.1), chrono.ChVector3d(0, -0.960, 0.1),
                chrono.ChVector3d(-3.336, 1.010, 0.05), chrono.ChVector3d(-3.336, -1.010, 0.05)]
    suv = chrono.ChVisualModel()
    suv_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/Nissan_Patrol/suv_chassis.obj"))
    suv_wheel = CreateMeshShape(chrono.GetChronoDataFile("vehicle/Nissan_Patrol/suv_rim.obj"))
    suv_tire = CreateMeshShape(chrono.GetChronoDataFile("vehicle/Nissan_Patrol/suv_tire.obj"))
    suv.AddShape(suv_chassis)
    for i in range(4):
        suv.AddShape(suv_wheel, chrono.ChFramed(suv_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
        suv.AddShape(suv_tire, chrono.ChFramed(suv_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
    vis.AddVisualModel(suv, chrono.ChFramed(suv_pos, chrono.QUNIT))

    # Create Polaris
    polaris_wpos = [chrono.ChVector3d(0, 0.616, 0.397), chrono.ChVector3d(0, -0.616, 0.397),
                    chrono.ChVector3d(-2.715, 0.616, 0.405), chrono.ChVector3d(-2.715, -0.616, 0.405)]
    polaris = chrono.ChVisualModel()
    polaris_chassis = CreateMeshShape(chrono.GetChronoDataFile("vehicle/Polaris/meshes/Polaris_chassis.obj"))
    polaris_wheel = CreateMeshShape(chrono.GetChronoDataFile("vehicle/Polaris/meshes/Polaris_wheel.obj"))
    polaris_tire = CreateMeshShape(chrono.GetChronoDataFile("vehicle/Polaris/meshes/Polaris_tire.obj"))
    polaris.AddShape(polaris_chassis)
    for i in range(4):
        polaris.AddShape(polaris_wheel, chrono.ChFramed(polaris_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
        polaris.AddShape(polaris_tire, chrono.ChFramed(polaris_wpos[i], chrono.QuatFromAngleZ(chrono.CH_PI * i)))
    vis.AddVisualModel(polaris, chrono.ChFramed(polaris_pos, chrono.QUNIT))

    # Initialize the visualization system
    vis.Initialize()

    # Simulation loop
    while vis.Run():
        vis.Render()
        sys.DoStepDynamics(0.01)

if __name__ == "__main__":
    main()
