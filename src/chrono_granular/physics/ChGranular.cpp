// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
/*! \file */

#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>
#include <vector>
#include "ChGranular.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/core/ChVector.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranularBoundaryConditions.h"

namespace chrono {
namespace granular {

ChSystemGranular_MonodisperseSMC::ChSystemGranular_MonodisperseSMC(float radiusSPH, float density)
    : sphere_radius_UU(radiusSPH),
      sphere_density_UU(density),
      time_stepping(GRAN_TIME_STEPPING::ADAPTIVE),
      nDEs(0),
      elapsedSimTime(0),
      K_n_s2s_UU(0),
      K_n_s2w_UU(0),
      K_t_s2s_UU(0),
      K_t_s2w_UU(0),
      Gamma_n_s2s_UU(0),
      Gamma_n_s2w_UU(0),
      Gamma_t_s2s_UU(0),
      Gamma_t_s2w_UU(0) {
    gpuErrchk(cudaMallocManaged(&gran_params, sizeof(ChGranParams), cudaMemAttachGlobal));
    gran_params->psi_T = PSI_T_DEFAULT;
    gran_params->psi_h = PSI_h_DEFAULT;
    gran_params->psi_L = PSI_L_DEFAULT;
    gran_params->friction_mode = FRICTIONLESS;
    this->friction_mode = FRICTIONLESS;
    gran_params->time_integrator = FORWARD_EULER;
    this->time_integrator = FORWARD_EULER;
    gran_params->force_model = HOOKE;
    this->force_model = HOOKE;
}

ChSystemGranular_MonodisperseSMC::~ChSystemGranular_MonodisperseSMC() {
    gpuErrchk(cudaFree(gran_params));
}

void ChSystemGranular_MonodisperseSMC::packSphereDataPointers(sphereDataStruct& packed) {
    // Set data from system
    packed.pos_X = pos_X.data();
    packed.pos_Y = pos_Y.data();
    packed.pos_Z = pos_Z.data();
    packed.pos_X_dt = pos_X_dt.data();
    packed.pos_Y_dt = pos_Y_dt.data();
    packed.pos_Z_dt = pos_Z_dt.data();

    if (friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
        packed.sphere_Omega_X = sphere_Omega_X.data();
        packed.sphere_Omega_Y = sphere_Omega_Y.data();
        packed.sphere_Omega_Z = sphere_Omega_Z.data();
        packed.sphere_ang_acc_X = sphere_ang_acc_X.data();
        packed.sphere_ang_acc_Y = sphere_ang_acc_Y.data();
        packed.sphere_ang_acc_Z = sphere_ang_acc_Z.data();
    }

    packed.sphere_force_X = sphere_force_X.data();
    packed.sphere_force_Y = sphere_force_Y.data();
    packed.sphere_force_Z = sphere_force_Z.data();

    if (time_integrator == GRAN_TIME_INTEGRATOR::CHUNG) {
        packed.sphere_force_X_old = sphere_force_X_old.data();
        packed.sphere_force_Y_old = sphere_force_Y_old.data();
        packed.sphere_force_Z_old = sphere_force_Z_old.data();
        packed.sphere_ang_acc_X_old = sphere_ang_acc_X_old.data();
        packed.sphere_ang_acc_Y_old = sphere_ang_acc_Y_old.data();
        packed.sphere_ang_acc_Z_old = sphere_ang_acc_Z_old.data();
    }

    packed.SD_NumOf_DEs_Touching = SD_NumOf_DEs_Touching.data();
    packed.DEs_SD_offsets = DEs_SD_offsets.data();
    packed.DEs_in_SD_composite = DEs_in_SD_composite.data();

    if (friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        packed.sphere_contact_map = sphere_contact_map.data();
        packed.contact_history_map = contact_history_map.data();
    }
}

// size_t ChSystemGranular_MonodisperseSMC::Create_BC_AABox(float hdims[3], float center[3], bool outward_normal) {
//     BC_params_t<float, float3> p;
//     printf("UU bounds are %f,%f,%f,%f,%f,%f", center[0] + hdims[0], center[1] + hdims[1], center[2] + hdims[2],
//            center[0] - hdims[0], center[1] - hdims[1], center[2] - hdims[2]);
//
//     // Find two corners to describe box
//     p.AABox_params.max_corner.x = center[0] + hdims[0];
//     p.AABox_params.max_corner.y = center[1] + hdims[1];
//     p.AABox_params.max_corner.z = center[2] + hdims[2];
//     p.AABox_params.min_corner.x = center[0] - hdims[0];
//     p.AABox_params.min_corner.y = center[1] - hdims[1];
//     p.AABox_params.min_corner.z = center[2] - hdims[2];
//
//     printf("SU bounds are %d, %d, %d, %d, %d, %d", p.AABox_params.max_corner.x, p.AABox_params.max_corner.y,
//            p.AABox_params.max_corner.z, p.AABox_params.min_corner.x, p.AABox_params.min_corner.y,
//            p.AABox_params.min_corner.z);
//
//     if (outward_normal) {
//         // negate forces to push particles outward
//         p.AABox_params.normal_sign = -1;
//     } else {
//         // normal is inward, flip force sign
//         p.AABox_params.normal_sign = 1;
//     }
//     BC_type_list.push_back(BC_type::AA_BOX);
//     BC_params_list_UU.push_back(p);
//     // get my index in the new array
//     return BC_type_list.size() - 1;
// }

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Sphere(float center[3], float radius, bool outward_normal) {
    BC_params_t<float, float3> p;
    // set center, radius, norm
    p.sphere_params.sphere_center.x = center[0];
    p.sphere_params.sphere_center.y = center[1];
    p.sphere_params.sphere_center.z = center[2];
    p.sphere_params.radius = radius;

    if (outward_normal) {
        // negate forces to push particles outward
        p.sphere_params.normal_sign = -1;
    } else {
        p.sphere_params.normal_sign = 1;
    }

    BC_type_list.push_back(BC_type::SPHERE);
    BC_params_list_UU.push_back(p);
    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Cone_Z(float cone_tip[3],
                                                          float slope,
                                                          float hmax,
                                                          float hmin,
                                                          bool outward_normal) {
    BC_params_t<float, float3> p;
    // set center, radius, norm
    p.cone_params.cone_tip.x = cone_tip[0];
    p.cone_params.cone_tip.y = cone_tip[1];
    p.cone_params.cone_tip.z = cone_tip[2];
    p.cone_params.hmax = hmax;
    p.cone_params.hmin = hmin;
    p.cone_params.slope = slope;

    if (outward_normal) {
        // negate forces to push particles outward
        p.cone_params.normal_sign = -1;
    } else {
        p.cone_params.normal_sign = 1;
    }

    BC_type_list.push_back(BC_type::CONE);
    BC_params_list_UU.push_back(p);
    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Plane(float plane_pos[3], float plane_normal[3]) {
    BC_params_t<float, float3> p;
    p.plane_params.position.x = plane_pos[0];
    p.plane_params.position.y = plane_pos[1];
    p.plane_params.position.z = plane_pos[2];

    p.plane_params.normal.x = plane_normal[0];
    p.plane_params.normal.y = plane_normal[1];
    p.plane_params.normal.z = plane_normal[2];

    BC_type_list.push_back(BC_type::PLANE);
    BC_params_list_UU.push_back(p);
    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Cyl_Z(float center[3], float radius, bool outward_normal) {
    BC_params_t<float, float3> p;
    p.cyl_params.center.x = center[0];
    p.cyl_params.center.y = center[1];
    p.cyl_params.center.z = center[2];

    p.cyl_params.radius = radius;

    if (outward_normal) {
        // negate forces to push particles outward
        p.cyl_params.normal_sign = -1;
    } else {
        p.cyl_params.normal_sign = 1;
    }

    BC_type_list.push_back(BC_type::CYLINDER);
    BC_params_list_UU.push_back(p);
    // get my index in the new array
    return BC_type_list.size() - 1;
}

void ChSystemGranular_MonodisperseSMC::determineNewStepSize_SU() {
    // std::cerr << "determining new step!\n";
    if (time_stepping == GRAN_TIME_STEPPING::ADAPTIVE) {
        static float new_step_stop = 0;
        if (elapsedSimTime >= new_step_stop) {
            // printf("-------------------------\n");
            // std::cerr << "elapsed is " << elapsedSimTime << ", stop is " << new_step_stop << std::endl;
            new_step_stop += new_step_freq;  // assumes we never have a timestep larger than new_step_freq
            float max_v = get_max_vel();
            if (max_v <= 0) {
                // clearly we have an issue, just fallback to the fixed step
                stepSize_SU = fixed_step_UU / gran_params->TIME_UNIT;
            } else {
                // maximum number of gravity displacements we allow moving in one timestep
                constexpr float num_disp_grav = 100;
                // maximum fraction of radius we allow moving in one timestep
                constexpr float num_disp_radius = .1;
                float max_displacement_grav = num_disp_grav * gran_params->psi_T;
                float max_displacement_radius = num_disp_radius * gran_params->sphereRadius_SU;

                // TODO consider gravity drift

                // find the highest position displacement we allow
                float max_displacement = std::min(max_displacement_grav, max_displacement_radius);
                float suggested_SU = max_displacement / max_v;
                float max_step_SU = max_adaptive_step_UU / gran_params->TIME_UNIT;
                float min_step_SU = 1e-5 / gran_params->TIME_UNIT;
                printf("grav step is %f, rad step is %f\n", max_displacement_grav / max_v,
                       max_displacement_radius / max_v);

                // don't go above max
                stepSize_SU = std::max(std::min(suggested_SU, max_step_SU), min_step_SU);
            }
            printf("new timestep is %f SU, %f UU\n", stepSize_SU, stepSize_SU * gran_params->TIME_UNIT);
        }
    } else {
        stepSize_SU = fixed_step_UU / gran_params->TIME_UNIT;
    }
    // if step ize changed, update it on device
    if (gran_params->stepSize_SU != stepSize_SU) {
        gran_params->stepSize_SU = stepSize_SU;
    }
}

double ChSystemGranular_MonodisperseSMC::get_max_K() {
    return std::max(K_n_s2s_UU, K_n_s2w_UU);
}

void ChSystemGranular_MonodisperseSMC::convertBCUnits() {
    for (int i = 0; i < BC_type_list.size(); i++) {
        auto bc_type = BC_type_list.at(i);
        BC_params_t<float, float3> params_UU = BC_params_list_UU.at(i);
        BC_params_t<int, int3> params_SU;
        switch (bc_type) {
            case BC_type::SPHERE: {
                printf("adding sphere!\n");
                // set center, radius, norm
                params_SU.sphere_params.sphere_center.x =
                    convertToPosSU<int, float>(params_UU.sphere_params.sphere_center.x);
                params_SU.sphere_params.sphere_center.y =
                    convertToPosSU<int, float>(params_UU.sphere_params.sphere_center.y);
                params_SU.sphere_params.sphere_center.z =
                    convertToPosSU<int, float>(params_UU.sphere_params.sphere_center.z);
                params_SU.sphere_params.radius = convertToPosSU<int, float>(params_UU.sphere_params.radius);
                params_SU.sphere_params.normal_sign = params_UU.sphere_params.normal_sign;
                params_SU.active = true;

                BC_params_list_SU.push_back(params_SU);
                break;
            }

                // case BC_type::AA_BOX:{
                //     printf("adding box!\n");
                //
                //     // note that these are correct but the BC formulation is not complete
                //     // TODO fix AABox formulation
                //     // Find two corners to describe box
                //     params_SU.AABox_params.max_corner.x = convertToPosSU<int,
                //     float>(params_UU.AABox_params.max_corner.x); params_SU.AABox_params.max_corner.y =
                //     convertToPosSU<int, float>(params_UU.AABox_params.max_corner.y);
                //     params_SU.AABox_params.max_corner.z = convertToPosSU<int,
                //     float>(params_UU.AABox_params.max_corner.z); params_SU.AABox_params.min_corner.x =
                //     convertToPosSU<int, float>(params_UU.AABox_params.min_corner.x);
                //     params_SU.AABox_params.min_corner.y = convertToPosSU<int,
                //     float>(params_UU.AABox_params.min_corner.y); params_SU.AABox_params.min_corner.z =
                //     convertToPosSU<int, float>(params_UU.AABox_params.min_corner.z);
                //
                //     params_SU.AABox_params.normal_sign = params_UU.AABox_params.normal_sign;
                //     params_SU.active = true;
                //
                //     BC_params_list_SU.push_back(params_SU);
                //     break;}

            case BC_type::CONE: {
                printf("adding cone!\n");

                params_SU.cone_params.cone_tip.x = convertToPosSU<int, float>(params_UU.cone_params.cone_tip.x);
                params_SU.cone_params.cone_tip.y = convertToPosSU<int, float>(params_UU.cone_params.cone_tip.y);
                params_SU.cone_params.cone_tip.z = convertToPosSU<int, float>(params_UU.cone_params.cone_tip.z);

                params_SU.cone_params.hmax = convertToPosSU<int, float>(params_UU.cone_params.hmax);
                params_SU.cone_params.hmin = convertToPosSU<int, float>(params_UU.cone_params.hmin);
                params_SU.cone_params.slope = params_UU.cone_params.slope;
                params_SU.cone_params.normal_sign = params_UU.cone_params.normal_sign;
                params_SU.active = true;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            case BC_type::PLANE: {
                printf("adding plane!\n");
                params_SU.plane_params.position.x = convertToPosSU<int, float>(params_UU.plane_params.position.x);
                params_SU.plane_params.position.y = convertToPosSU<int, float>(params_UU.plane_params.position.y);
                params_SU.plane_params.position.z = convertToPosSU<int, float>(params_UU.plane_params.position.z);

                // normal is unitless
                // TODO normalize this just in case
                // float abs = Length(params_UU);
                params_SU.plane_params.normal.x = params_UU.plane_params.normal.x;
                params_SU.plane_params.normal.y = params_UU.plane_params.normal.y;
                params_SU.plane_params.normal.z = params_UU.plane_params.normal.z;
                params_SU.active = true;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            case BC_type::CYLINDER: {
                printf("adding cylinder!\n");
                params_SU.cyl_params.center.x = convertToPosSU<int, float>(params_UU.cyl_params.center.x);
                params_SU.cyl_params.center.y = convertToPosSU<int, float>(params_UU.cyl_params.center.y);
                params_SU.cyl_params.center.z = convertToPosSU<int, float>(params_UU.cyl_params.center.z);

                // normal is unitless
                // TODO normalize this just in case
                // float abs = Length(params_UU);
                params_SU.cyl_params.radius = convertToPosSU<int, float>(params_UU.cyl_params.radius);
                params_SU.cyl_params.normal_sign = params_UU.cyl_params.normal_sign;

                params_SU.active = true;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            default: {
                printf("ERROR: Unsupported BC Type!\n");
                exit(1);
            }
        }
    }
}

void ChSystemGranular_MonodisperseSMC::initializeSpheres() {
    switchToSimUnits();
    generateDEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setupSimulation();
    copyConstSphereDataToDevice();

    determineNewStepSize_SU();
    convertBCUnits();

    // Seed arrays that are populated by the kernel call
    resetBroadphaseInformation();

    printf("doing priming!\n");
    printf("max possible composite offset with 256 limit is %zu\n", (size_t)nSDs * MAX_COUNT_OF_DEs_PER_SD);
    runBroadphase();
    printf("priming finished!\n");

    int dev_ID;
    cudaGetDevice(&dev_ID);
    cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetReadMostly, dev_ID);

    printf("z grav term with timestep %f is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gran_params->gravAcc_Z_SU);
    printf("running at approximate timestep %f\n", stepSize_SU * gran_params->TIME_UNIT);
}

// mean to be overriden by children
void ChSystemGranular_MonodisperseSMC::initialize() {
    initializeSpheres();
}

// set up sphere-sphere data structures
void ChSystemGranular_MonodisperseSMC::setupSimulation() {
    partitionBD();
    // allocate mem for array saying for each SD how many spheres touch it
    SD_NumOf_DEs_Touching.resize(nSDs);
    DEs_SD_offsets.resize(nSDs);
    // assume each sphere touches 2 SDs on average
    // NOTE that this will get resized again later, this is just the first estimate
    DEs_in_SD_composite.resize(2 * nDEs);
}

// Set particle positions in UU
void ChSystemGranular_MonodisperseSMC::setParticlePositions(const std::vector<ChVector<float>>& points) {
    user_sphere_positions = points;  // Copy points to class vector
}

void ChSystemGranular_MonodisperseSMC::generateDEs() {
    // Each fills user_sphere_positions with positions to be copied
    if (user_sphere_positions.size() == 0) {
        printf("ERROR: no sphere positions given!\n");
        exit(1);
    }

    // dump these into SU, we no longer need their UU componenets
    for (auto& point : user_sphere_positions) {
        point /= gran_params->LENGTH_UNIT;
    }

    nDEs = (unsigned int)user_sphere_positions.size();
    std::cout << nDEs << " balls added!" << std::endl;

    // Allocate space for new bodies
    pos_X.resize(nDEs);
    pos_Y.resize(nDEs);
    pos_Z.resize(nDEs);
    pos_X_dt.resize(nDEs, 0);
    pos_Y_dt.resize(nDEs, 0);
    pos_Z_dt.resize(nDEs, 0);
    sphere_force_X.resize(nDEs, 0);
    sphere_force_Y.resize(nDEs, 0);
    sphere_force_Z.resize(nDEs, 0);

    if (friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
        // add rotational DOFs
        sphere_Omega_X.resize(nDEs, 0);
        sphere_Omega_Y.resize(nDEs, 0);
        sphere_Omega_Z.resize(nDEs, 0);

        // add torques
        sphere_ang_acc_X.resize(nDEs, 0);
        sphere_ang_acc_Y.resize(nDEs, 0);
        sphere_ang_acc_Z.resize(nDEs, 0);
    }

    if (friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        contactDataStruct null_data;
        null_data.active = false;
        null_data.body_B = NULL_GRANULAR_ID;
        sphere_contact_map.resize(12 * nDEs, null_data);
        contact_history_map.resize(12 * nDEs, {0., 0., 0.});
    }

    if (time_integrator == GRAN_TIME_INTEGRATOR::CHUNG) {
        sphere_force_X_old.resize(nDEs, 0);
        sphere_force_Y_old.resize(nDEs, 0);
        sphere_force_Z_old.resize(nDEs, 0);
    }

    // Copy from array of structs to 3 arrays
    for (unsigned int i = 0; i < nDEs; i++) {
        auto vec = user_sphere_positions.at(i);
        pos_X.at(i) = (int)(vec.x());
        pos_Y.at(i) = (int)(vec.y());
        pos_Z.at(i) = (int)(vec.z());
    }
}

/**
This method figures out how big a SD is, and how many SDs are going to be necessary
in order to cover the entire BD.
BD: Big domain.
SD: Sub-domain.
*/
void ChSystemGranular_MonodisperseSMC::partitionBD() {
    double sd_length_scale = 2. * sphere_radius_UU * AVERAGE_SPHERES_PER_SD_X_DIR;

    unsigned int nSDs_X = (unsigned int)(std::ceil(box_size_X / sd_length_scale));
    // work with an even kFac to hit the CM of the box.
    if (nSDs_X & 1)
        nSDs_X++;
    int SD_size_X = (unsigned int)std::ceil(box_size_X / (nSDs_X * gran_params->LENGTH_UNIT));

    unsigned int nSDs_Y = (unsigned int)(std::ceil(box_size_Y / sd_length_scale));
    // work with an even kFac to hit the CM of the box.
    if (nSDs_Y & 1)
        nSDs_Y++;
    int SD_size_Y = (unsigned int)std::ceil(box_size_Y / (nSDs_Y * gran_params->LENGTH_UNIT));

    unsigned int nSDs_Z = (unsigned int)(std::ceil(box_size_Z / sd_length_scale));
    // work with an even kFac to hit the CM of the box.
    if (nSDs_Z & 1)
        nSDs_Z++;
    int SD_size_Z = (unsigned int)std::ceil(box_size_Z / (nSDs_Z * gran_params->LENGTH_UNIT));

    nSDs = nSDs_X * nSDs_Y * nSDs_Z;

    // copy these to device right away
    gran_params->nSDs = nSDs;
    gran_params->nSDs_X = nSDs_X;
    gran_params->nSDs_Y = nSDs_Y;
    gran_params->nSDs_Z = nSDs_Z;

    gran_params->SD_size_X_SU = SD_size_X;
    gran_params->SD_size_Y_SU = SD_size_Y;
    gran_params->SD_size_Z_SU = SD_size_Z;

    // Place BD frame at bottom-left corner, one half-length in each direction
    // Can change later if desired
    gran_params->BD_frame_X = -.5 * (nSDs_X * SD_size_X);
    gran_params->BD_frame_Y = -.5 * (nSDs_Y * SD_size_Y);
    gran_params->BD_frame_Z = -.5 * (nSDs_Z * SD_size_Z);
    // BD starts at rest
    gran_params->BD_frame_X_dot = 0;
    gran_params->BD_frame_Y_dot = 0;
    gran_params->BD_frame_Z_dot = 0;
    printf("%u Sds as %u, %u, %u\n", gran_params->nSDs, gran_params->nSDs_X, gran_params->nSDs_Y, gran_params->nSDs_Z);
}

/**
This method defines the mass, time, length Simulation Units. It also sets several other constants that enter the scaling
of various physical quantities set by the user.
*/
void ChSystemGranular_MonodisperseSMC::switchToSimUnits() {
    double massSphere = 4. / 3. * M_PI * sphere_radius_UU * sphere_radius_UU * sphere_radius_UU * sphere_density_UU;
    gran_params->MASS_UNIT = massSphere / gran_params->sphere_mass_SU;
    double K_stiffness = get_max_K();
    gran_params->TIME_UNIT = sqrt(massSphere / (gran_params->psi_h * K_stiffness)) / gran_params->psi_T;

    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);
    gran_params->LENGTH_UNIT = massSphere * magGravAcc / (gran_params->psi_L * K_stiffness);

    gran_params->sphereRadius_SU = sphere_radius_UU / gran_params->LENGTH_UNIT;

    float g_scalingFactor =
        ((float)gran_params->psi_L) / (gran_params->psi_T * gran_params->psi_T * gran_params->psi_h);
    gran_params->gravAcc_X_SU = g_scalingFactor * X_accGrav / magGravAcc;
    gran_params->gravAcc_Y_SU = g_scalingFactor * Y_accGrav / magGravAcc;
    gran_params->gravAcc_Z_SU = g_scalingFactor * Z_accGrav / magGravAcc;

    /// SU values for normal stiffnesses for S2S and S2W
    float K_scalingFactor = 1.f / (1.f * gran_params->psi_T * gran_params->psi_T * gran_params->psi_h);
    gran_params->K_n_s2s_SU = K_scalingFactor * (K_n_s2s_UU / K_stiffness);
    gran_params->K_n_s2w_SU = K_scalingFactor * (K_n_s2w_UU / K_stiffness);

    gran_params->K_t_s2s_SU = K_scalingFactor * (K_t_s2s_UU / K_stiffness);
    gran_params->K_t_s2w_SU = K_scalingFactor * (K_t_s2w_UU / K_stiffness);

    float Gamma_scalingFactor = 1.f / (gran_params->psi_T * std::sqrt(K_stiffness * gran_params->psi_h / massSphere));
    gran_params->Gamma_n_s2s_SU = Gamma_scalingFactor * Gamma_n_s2s_UU;
    gran_params->Gamma_n_s2w_SU = Gamma_scalingFactor * Gamma_n_s2w_UU;
    gran_params->Gamma_t_s2s_SU = Gamma_scalingFactor * Gamma_t_s2s_UU;
    gran_params->Gamma_t_s2w_SU = Gamma_scalingFactor * Gamma_t_s2w_UU;

    gran_params->cohesionAcc_s2s = cohesion_over_gravity * g_scalingFactor;
    gran_params->adhesionAcc_s2w = adhesion_s2w_over_gravity * g_scalingFactor;

    // Handy debug output
    printf("UU mass is %f\n", gran_params->MASS_UNIT);
    printf("SU gravity is %f, %f, %f\n", gran_params->gravAcc_X_SU, gran_params->gravAcc_Y_SU,
           gran_params->gravAcc_Z_SU);
    printf("SU radius is %u\n", gran_params->sphereRadius_SU);
    float dt_safe_estimate = sqrt(massSphere / K_n_s2s_UU);
    printf("Safe timestep is about %f\n", dt_safe_estimate);
}
}  // namespace granular
}  // namespace chrono