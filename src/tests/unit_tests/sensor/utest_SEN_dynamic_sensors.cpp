// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
//
// VERIFICATION: do the non-rendering ("dynamic") Chrono::Sensor sensors work in a Metal RT build?
// GPS, IMU (accelerometer / gyroscope / magnetometer) and tachometer derive their readings from body
// state via ChDynamicsManager, not from the ray tracer, so they should be backend-independent. This
// runs all five on a driving Audi -- alongside a Metal RT camera, to prove they coexist with the
// render backend -- and prints live readings so the values can be sanity-checked, not just the plumbing.
//
// =============================================================================

#include "gtest/gtest.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

TEST(ChSensorDynamicSensors, all_five_produce_data_on_a_render_build) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    RigidTerrain terrain(&sys);
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300.0, 120.0);
    patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 60, 24);
    terrain.Initialize();

    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));
    audi.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.55), QUNIT));
    audi.SetChassisVisualizationType(VisualizationType::MESH);
    audi.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetWheelVisualizationType(VisualizationType::MESH);
    {
        auto engine = ReadEngineJSON(GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"));
        auto transmission = ReadTransmissionJSON(GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"));
        audi.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));
        for (auto& axle : audi.GetAxles())
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("audi/json/audi_TMeasyTire.json"));
                tire->SetStepsize(1e-4);
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.02625f, 0.50710f);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));

    auto body = audi.GetChassisBody();
    // Cartesian2GPS treats the reference as (LONGITUDE, LATITUDE, altitude) -- not (lat, lon, alt).
    const ChVector3d gps_ref(-89.400, 43.073, 260.0);  // Madison, WI
    auto none = chrono_types::make_shared<ChNoiseNone>();

    // --- a Metal RT camera, purely to prove the render backend and the dynamic sensors coexist ---
    ChVector3d cam_off(-8.0, -3.0, 3.0), look(4.0, 0.0, 0.6);
    ChVector3d cd = (look - cam_off).GetNormalized();
    auto cam = chrono_types::make_shared<ChCameraSensor>(body, 10.0f, ChFrame<double>(cam_off, QuatFromAngleZ(std::atan2(cd.y(), cd.x())) * QuatFromAngleY(-std::asin(cd.z()))),
                                                         640, 360, (float)(CH_PI / 3), 1, CameraLensModelType::PINHOLE, false, false);
    cam->SetName("coexist_cam");
    manager->AddSensor(cam);

    // --- the five non-rendering sensors ---
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(body, 20.0f, ChFrame<double>(), none);
    acc->SetName("accelerometer");
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    manager->AddSensor(acc);

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(body, 20.0f, ChFrame<double>(), none);
    gyro->SetName("gyroscope");
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    manager->AddSensor(gyro);

    auto mag = chrono_types::make_shared<ChMagnetometerSensor>(body, 20.0f, ChFrame<double>(), none, gps_ref);
    mag->SetName("magnetometer");
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    manager->AddSensor(mag);

    auto gps = chrono_types::make_shared<ChGPSSensor>(body, 20.0f, ChFrame<double>(), gps_ref, none);
    gps->SetName("gps");
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);

    // Mount the tachometer on a WHEEL SPINDLE, not the chassis: it reports the angular rate of its parent
    // body about the given axis, and the chassis does not spin (that would read ~0 rpm while driving).
    auto spindle = audi.GetAxle(0)->GetWheel(LEFT)->GetSpindle();
    auto tach = chrono_types::make_shared<ChTachometerSensor>(spindle, 20.0f, ChFrame<double>(), ChTachometerSensor::Axis::Y);
    tach->SetName("tachometer");
    tach->PushFilter(chrono_types::make_shared<ChFilterTachometerAccess>());
    manager->AddSensor(tach);

    printf("Dynamic-sensor check on the Metal RT build (accel / gyro / magnet / GPS / tach + a Metal camera)\n");
    printf("%6s %28s %28s %30s %26s %8s\n", "t[s]", "accel [m/s^2]", "gyro [rad/s]", "magnet [T]", "GPS lat/lon/alt", "rpm");

    const double step = 1e-3;
    double time = 0;
    int got_acc = 0, got_gyro = 0, got_mag = 0, got_gps = 0, got_tach = 0;
    double next_print = 0.0;

    while (time < 4.0) {
        DriverInputs in;
        in.m_throttle = 0.5;
        in.m_steering = 0.25 * std::sin(1.1 * time);  // weave -> non-trivial yaw rate for the gyro
        in.m_braking = 0.0;
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();

        auto a = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        auto g = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
        auto m = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
        auto p = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
        auto t = tach->GetMostRecentBuffer<UserTachometerBufferPtr>();
        if (a && a->Buffer)
            got_acc++;
        if (g && g->Buffer)
            got_gyro++;
        if (m && m->Buffer)
            got_mag++;
        if (p && p->Buffer)
            got_gps++;
        if (t && t->Buffer)
            got_tach++;

        if (time >= next_print && a && a->Buffer && g && g->Buffer && m && m->Buffer && p && p->Buffer && t && t->Buffer) {
            printf("%6.2f  %8.3f %8.3f %8.3f  %8.4f %8.4f %8.4f  %9.2e %9.2e %9.2e  %10.6f %11.6f %7.1f %8.1f\n", time, a->Buffer[0].X, a->Buffer[0].Y, a->Buffer[0].Z,
                   g->Buffer[0].Roll, g->Buffer[0].Pitch, g->Buffer[0].Yaw, m->Buffer[0].X, m->Buffer[0].Y, m->Buffer[0].Z, p->Buffer[0].Latitude, p->Buffer[0].Longitude,
                   p->Buffer[0].Altitude, t->Buffer[0].rpm);
            next_print += 0.5;
        }
        time += step;
    }

    printf("\nbuffers delivered (non-zero => sensor is live): accel=%d gyro=%d magnet=%d gps=%d tach=%d\n", got_acc, got_gyro, got_mag, got_gps, got_tach);
    EXPECT_GT(got_acc, 0) << "accelerometer produced no data";
    EXPECT_GT(got_gyro, 0) << "gyroscope produced no data";
    EXPECT_GT(got_mag, 0) << "magnetometer produced no data";
    EXPECT_GT(got_gps, 0) << "GPS produced no data";
    EXPECT_GT(got_tach, 0) << "tachometer produced no data";
}
