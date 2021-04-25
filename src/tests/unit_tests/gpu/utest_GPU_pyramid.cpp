// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Luning Fang, Jason Zhou
// =============================================================================
// Pyramid test: two spheres settled on the ground with gap in between, a third
// sphere set on top to form a pyramid. With more friction (rolling/sliding) the
// stack can support the top particle, otherwise the top particle drops
// This test is to verify the rolling friction of the chrono::gpu
// =============================================================================
#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/ChGpuData.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// declare global variables
ChVector<float> pos_t;
ChVector<float> velo_t;
ChVector<float> angular_velo_t;
float mass = 4 / 3 * 3.14 * 0.5 * 0.5 * 0.5 * 1.90986;
float inertia = 2 / 5 * mass * 0.5 * 0.5;
float precision = 1e-3;
float radius = 0.5;

int main(int argc, char* argv[]) {    
    // load the gpu system using checkpointing
    std::string settled_filename = GetChronoDataPath() + "testing/gpu/utest_GPU_pyramid/checkpoint.dat";
    ChSystemGpu gpu_sys(settled_filename);

    // overwrite fixity to allow the top particle to drop
    std::vector<bool> body_fixity;
    body_fixity.push_back(false);
    body_fixity.push_back(false);
    body_fixity.push_back(false);

    gpu_sys.SetParticleFixed(body_fixity);

    // set sliding frictin coefficient
    double mu_k = 0.5;   
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(mu_k);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(mu_k);

    // generate the ground for the pyramid
    ChVector<float> ground_plate_pos(0.0, 0.0, 0.0);
    ChVector<float> ground_plate_normal(0.0, 0.0, 1.0f);
    size_t ground_plate_id = gpu_sys.CreateBCPlane(ground_plate_pos, ground_plate_normal, true);

    // gpu_sys.SetRecordingContactInfo(true);
    gpu_sys.Initialize();

    float curr_time = 0;
    int fps = 1000;
    int currframe = 0;

    float step_size = 1e-4;
    float endTime = 1.5;
    while (curr_time < endTime){
        gpu_sys.AdvanceSimulation(step_size);
        curr_time += step_size;
        currframe++;
        if (currframe%100 == 0){
            std::cout<<"curr: "<<curr_time<<" tot: 1.5"<<std::endl;
        }

    }

    pos_t = gpu_sys.GetParticlePosition(2);
    velo_t = gpu_sys.GetParticleVelocity(2);
    angular_velo_t = gpu_sys.GetParticleAngVelocity(2);
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// check the top particle y and z component, pos_t.z() should be larger than sphere radius.
TEST(gpuFrictionSliding, check) {
    // check position y and z component
    EXPECT_NEAR(pos_t.y(), 0, precision);
    EXPECT_EQ(pos_t.z()>radius, true);

    // check top particle KE
    // the system should be into steady state
    float KE = 0.5 * mass * velo_t.Length() * velo_t.Length() + 0.5 * inertia * angular_velo_t.Length() * angular_velo_t.Length();
    EXPECT_NEAR(KE, 0, precision);
}
