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
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "chrono_granular/physics/ChGranularBoundaryConditions.h"
#include <climits>

// define it here, once and for all
size_t gran_approx_bytes_used = 0;

namespace chrono {
namespace granular {

ChSystemGranular_MonodisperseSMC::ChSystemGranular_MonodisperseSMC(float radiusSPH, float density, float3 boxDims)
    : sphere_radius_UU(radiusSPH),
      sphere_density_UU(density),
      box_size_X(boxDims.x),
      box_size_Y(boxDims.y),
      box_size_Z(boxDims.z),
      time_stepping(GRAN_TIME_STEPPING::ADAPTIVE),
      nSpheres(0),
      elapsedSimTime(0),
      verbose_runtime(false),
      // load_checkpoint(false),
      file_write_mode(CSV),
      X_accGrav(0),
      Y_accGrav(0),
      Z_accGrav(0),
      cohesion_over_gravity(0),
      adhesion_s2w_over_gravity(0),
      K_n_s2s_UU(0),
      K_n_s2w_UU(0),
      K_t_s2s_UU(0),
      K_t_s2w_UU(0),
      Gamma_n_s2s_UU(0),
      Gamma_n_s2w_UU(0),
      Gamma_t_s2s_UU(0),
      Gamma_t_s2w_UU(0) {
    gpuErrchk(cudaMallocManaged(&gran_params, sizeof(ChGranParams), cudaMemAttachGlobal));
    gpuErrchk(cudaMallocManaged(&sphere_data, sizeof(ChGranSphereData), cudaMemAttachGlobal));
    gran_params->psi_T = PSI_T_DEFAULT;
    gran_params->psi_h = PSI_h_DEFAULT;
    gran_params->psi_L = PSI_L_DEFAULT;
    gran_params->friction_mode = FRICTIONLESS;
    gran_params->rolling_mode = NO_RESISTANCE;
    this->friction_mode = FRICTIONLESS;
    this->rolling_mode = NO_RESISTANCE;
    gran_params->time_integrator = EXTENDED_TAYLOR;
    this->time_integrator = EXTENDED_TAYLOR;
    setMaxSafeVelocity_SU((float)UINT_MAX);
    set_static_friction_coeff(0);  // default to zero
    set_rolling_coeff(0);

    createWallBCs();
    setBDWallsMotionFunction(GranPosFunction_default);
}

void ChSystemGranular_MonodisperseSMC::createWallBCs() {
    float plane_center_bot_X[3] = {-box_size_X / 2, 0, 0};
    float plane_center_top_X[3] = {box_size_X / 2, 0, 0};
    float plane_center_bot_Y[3] = {0, -box_size_Y / 2, 0};
    float plane_center_top_Y[3] = {0, box_size_Y / 2, 0};
    float plane_center_bot_Z[3] = {0, 0, -box_size_Z / 2};
    float plane_center_top_Z[3] = {0, 0, box_size_Z / 2};
    // face in upwards
    float plane_normal_bot_X[3] = {1, 0, 0};
    float plane_normal_top_X[3] = {-1, 0, 0};
    float plane_normal_bot_Y[3] = {0, 1, 0};
    float plane_normal_top_Y[3] = {0, -1, 0};
    float plane_normal_bot_Z[3] = {0, 0, 1};
    float plane_normal_top_Z[3] = {0, 0, -1};

    // create wall BCs
    size_t plane_BC_X_bot = Create_BC_Plane(plane_center_bot_X, plane_normal_bot_X, false);
    size_t plane_BC_X_top = Create_BC_Plane(plane_center_top_X, plane_normal_top_X, false);
    size_t plane_BC_Y_bot = Create_BC_Plane(plane_center_bot_Y, plane_normal_bot_Y, false);
    size_t plane_BC_Y_top = Create_BC_Plane(plane_center_top_Y, plane_normal_top_Y, false);
    size_t plane_BC_Z_bot = Create_BC_Plane(plane_center_bot_Z, plane_normal_bot_Z, false);
    size_t plane_BC_Z_top = Create_BC_Plane(plane_center_top_Z, plane_normal_top_Z, false);

    // verify that we have the right IDs for these walls
    assert(plane_BC_X_bot == BD_WALL_ID_X_BOT);
    assert(plane_BC_X_top == BD_WALL_ID_X_TOP);
    assert(plane_BC_Y_bot == BD_WALL_ID_Y_BOT);
    assert(plane_BC_Y_top == BD_WALL_ID_Y_TOP);
    assert(plane_BC_Z_bot == BD_WALL_ID_Z_BOT);
    assert(plane_BC_Z_top == BD_WALL_ID_Z_TOP);
}

ChSystemGranular_MonodisperseSMC::~ChSystemGranular_MonodisperseSMC() {
    gpuErrchk(cudaFree(gran_params));
}

size_t ChSystemGranular_MonodisperseSMC::estimateMemUsage() const {
    return gran_approx_bytes_used;
}

void ChSystemGranular_MonodisperseSMC::packSphereDataPointers() {
    // Set data from system
    sphere_data->sphere_local_pos_X = sphere_local_pos_X.data();
    sphere_data->sphere_local_pos_Y = sphere_local_pos_Y.data();
    sphere_data->sphere_local_pos_Z = sphere_local_pos_Z.data();
    sphere_data->pos_X_dt = pos_X_dt.data();
    sphere_data->pos_Y_dt = pos_Y_dt.data();
    sphere_data->pos_Z_dt = pos_Z_dt.data();

    sphere_data->sphere_owner_SDs = sphere_owner_SDs.data();

    if (friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
        sphere_data->sphere_Omega_X = sphere_Omega_X.data();
        sphere_data->sphere_Omega_Y = sphere_Omega_Y.data();
        sphere_data->sphere_Omega_Z = sphere_Omega_Z.data();
        sphere_data->sphere_ang_acc_X = sphere_ang_acc_X.data();
        sphere_data->sphere_ang_acc_Y = sphere_ang_acc_Y.data();
        sphere_data->sphere_ang_acc_Z = sphere_ang_acc_Z.data();
    }

    sphere_data->sphere_acc_X = sphere_acc_X.data();
    sphere_data->sphere_acc_Y = sphere_acc_Y.data();
    sphere_data->sphere_acc_Z = sphere_acc_Z.data();

    if (time_integrator == GRAN_TIME_INTEGRATOR::CHUNG) {
        sphere_data->sphere_acc_X_old = sphere_acc_X_old.data();
        sphere_data->sphere_acc_Y_old = sphere_acc_Y_old.data();
        sphere_data->sphere_acc_Z_old = sphere_acc_Z_old.data();
        if (friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
            sphere_data->sphere_ang_acc_X_old = sphere_ang_acc_X_old.data();
            sphere_data->sphere_ang_acc_Y_old = sphere_ang_acc_Y_old.data();
            sphere_data->sphere_ang_acc_Z_old = sphere_ang_acc_Z_old.data();
        }
    }

    sphere_data->SD_NumSpheresTouching = SD_NumSpheresTouching.data();
    sphere_data->SD_SphereCompositeOffsets = SD_SphereCompositeOffsets.data();
    sphere_data->spheres_in_SD_composite = spheres_in_SD_composite.data();

    if (friction_mode == GRAN_FRICTION_MODE::MULTI_STEP || friction_mode == GRAN_FRICTION_MODE::SINGLE_STEP) {
        sphere_data->contact_partners_map = contact_partners_map.data();
        sphere_data->contact_active_map = contact_active_map.data();
    }

    if (friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        sphere_data->contact_history_map = contact_history_map.data();
    }

    // force prefetch the sphere data pointer after update
    int dev_ID;
    gpuErrchk(cudaGetDevice(&dev_ID));
    gpuErrchk(cudaMemPrefetchAsync(sphere_data, sizeof(*sphere_data), dev_ID));
}

// // Checkpoint the entire system's data
// void ChSystemGranular_MonodisperseSMC::writeCheckpoint(std::string ofile) const {
//     // CSV is much slower but requires less postprocessing
//     std::ofstream ptFile(ofile + ".dat", std::ios::out);
//
//     // Dump to a stream, write to file only at end
//     std::ostringstream outstrstream;
//     // no header
//     outstrstream << "x,y,z";
//
//     outstrstream << "\n";
//     for (unsigned int n = 0; n < nSpheres; n++) {
//         float x_UU = sphere_local_pos_X[n] * gran_params->LENGTH_UNIT;
//         float y_UU = sphere_local_pos_Y[n] * gran_params->LENGTH_UNIT;
//         float z_UU = sphere_local_pos_Z[n] * gran_params->LENGTH_UNIT;
//
//         outstrstream << x_UU << "," << y_UU << "," << z_UU << "\n";
//     }
//
//     ptFile << outstrstream.str();
//
// }  // namespace granular
//
// void tokenizeCSVLine(std::ifstream& istream, std::vector<float> data) {
//     std::string line;
//     std::getline(istream, line);  // load in current line
//     std::stringstream lineStream(line);
//     std::string cell;
//
//     // iterate over cells
//     while (std::getline(lineStream, cell, ',')) {
//         data.push_back(std::stof(cell));
//     }
// }
//
// // Load froma checkpoint file
// void ChSystemGranular_MonodisperseSMC::loadCheckpoint(std::string infile) {
//     // CSV is much slower but requires less postprocessing
//     std::ifstream ptFile(infile + ".dat");
//
//     std::string line;
//     unsigned int curr_sphere_id = 0;
//     while (ptFile.good()) {
//         std::vector<float> line_data;
//         tokenizeCSVLine(ptFile, line_data);
//         sphere_local_pos_X[curr_sphere_id] = line_data.at(0);
//         sphere_local_pos_Y[curr_sphere_id] = line_data.at(1);
//         sphere_local_pos_Z[curr_sphere_id] = line_data.at(2);
//     }
// }  // namespace granular

// This can belong to the superclass but does reference deCounts which may not be a thing when DVI rolls around
void ChSystemGranular_MonodisperseSMC::writeFile(std::string ofile) const {
    // The file writes are a pretty big slowdown in CSV mode
    if (file_write_mode == GRAN_OUTPUT_MODE::BINARY) {
        // TODO implement this
        // Write the data as binary to a file, requires later postprocessing that can be done in parallel, this is a
        // much faster write due to no formatting
        std::ofstream ptFile(ofile + ".raw", std::ios::out | std::ios::binary);

        for (unsigned int n = 0; n < nSpheres; n++) {
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n)) *
                         (gran_params->LENGTH_UNIT / gran_params->TIME_UNIT);

            unsigned int ownerSD = sphere_owner_SDs.at(n);
            int3 ownerSD_trip = getSDTripletFromID(ownerSD);
            float x_UU = sphere_local_pos_X[n] * gran_params->LENGTH_UNIT;
            float y_UU = sphere_local_pos_Y[n] * gran_params->LENGTH_UNIT;
            float z_UU = sphere_local_pos_Z[n] * gran_params->LENGTH_UNIT;

            x_UU += gran_params->BD_frame_X * gran_params->LENGTH_UNIT;
            y_UU += gran_params->BD_frame_Y * gran_params->LENGTH_UNIT;
            z_UU += gran_params->BD_frame_Z * gran_params->LENGTH_UNIT;

            x_UU += ((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * gran_params->LENGTH_UNIT;
            y_UU += ((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * gran_params->LENGTH_UNIT;
            z_UU += ((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * gran_params->LENGTH_UNIT;

            ptFile.write((const char*)&x_UU, sizeof(float));
            ptFile.write((const char*)&y_UU, sizeof(float));
            ptFile.write((const char*)&z_UU, sizeof(float));
            ptFile.write((const char*)&absv, sizeof(float));

            if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
                ptFile.write((const char*)&sphere_Omega_X.at(n), sizeof(float));
                ptFile.write((const char*)&sphere_Omega_Y.at(n), sizeof(float));
                ptFile.write((const char*)&sphere_Omega_Z.at(n), sizeof(float));
            }
        }
    } else if (file_write_mode == GRAN_OUTPUT_MODE::CSV) {
        // CSV is much slower but requires less postprocessing
        std::ofstream ptFile(ofile + ".csv", std::ios::out);

        // Dump to a stream, write to file only at end
        std::ostringstream outstrstream;
        outstrstream << "x,y,z,absv";

        if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
            outstrstream << ",wx,wy,wz";
        }
        outstrstream << "\n";
        for (unsigned int n = 0; n < nSpheres; n++) {
            unsigned int ownerSD = sphere_owner_SDs.at(n);
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n)) *
                         (gran_params->LENGTH_UNIT / gran_params->TIME_UNIT);
            int3 ownerSD_trip = getSDTripletFromID(ownerSD);
            float x_UU = sphere_local_pos_X[n] * gran_params->LENGTH_UNIT;
            float y_UU = sphere_local_pos_Y[n] * gran_params->LENGTH_UNIT;
            float z_UU = sphere_local_pos_Z[n] * gran_params->LENGTH_UNIT;

            x_UU += gran_params->BD_frame_X * gran_params->LENGTH_UNIT;
            y_UU += gran_params->BD_frame_Y * gran_params->LENGTH_UNIT;
            z_UU += gran_params->BD_frame_Z * gran_params->LENGTH_UNIT;

            x_UU += ((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * gran_params->LENGTH_UNIT;
            y_UU += ((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * gran_params->LENGTH_UNIT;
            z_UU += ((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * gran_params->LENGTH_UNIT;

            outstrstream << x_UU << "," << y_UU << "," << z_UU << "," << absv;

            if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
                outstrstream << "," << sphere_Omega_X.at(n) << "," << sphere_Omega_Y.at(n) << ","
                             << sphere_Omega_Z.at(n);
            }
            outstrstream << "\n";
        }

        ptFile << outstrstream.str();
    } else if (file_write_mode == GRAN_OUTPUT_MODE::NONE) {
        // Do nothing, only here for symmetry
    }
}

// Reset broadphase data structures
void ChSystemGranular_MonodisperseSMC::resetBCForces() {
    // zero out reaction forces on each BC
    for (unsigned int i = 0; i < BC_params_list_SU.size(); i++) {
        if (BC_params_list_SU.at(i).track_forces) {
            BC_params_list_SU.at(i).reaction_forces = {0, 0, 0};
        }
    }
}

/// Copy constant sphere data to device, this should run at start
void ChSystemGranular_MonodisperseSMC::copyConstSphereDataToDevice() {
    gran_params->max_x_pos_unsigned = ((int64_t)gran_params->SD_size_X_SU * gran_params->nSDs_X);
    gran_params->max_y_pos_unsigned = ((int64_t)gran_params->SD_size_Y_SU * gran_params->nSDs_Y);
    gran_params->max_z_pos_unsigned = ((int64_t)gran_params->SD_size_Z_SU * gran_params->nSDs_Z);

    printf("max pos is is %lu, %lu, %lu\n", gran_params->max_x_pos_unsigned, gran_params->max_y_pos_unsigned,
           gran_params->max_z_pos_unsigned);

    int64_t true_max_pos = std::max(std::max(gran_params->max_x_pos_unsigned, gran_params->max_y_pos_unsigned),
                                    gran_params->max_z_pos_unsigned);

    if (true_max_pos >= INT_MAX) {
        printf("WARNING! Max possible position is greater than INT_MAX!!!\n");
    }

    if (true_max_pos >= UINT_MAX) {
        printf("BIG WARNING! Max possible position is greater than UINT_MAX!!!\n");
        printf("You are now relying on Conlain's local coordinate implementation.\n");
    }

    if (true_max_pos >= LLONG_MAX) {
        printf("ERROR! Max possible position is greater than LLONG_MAX!!!\n");
        printf("Not even local coordinates can save you now.\n");
        exit(1);
    }

    // NOTE: Assumes mass = 1
    gran_params->sphereInertia_by_r = (2.f / 5.f) * gran_params->sphere_mass_SU * gran_params->sphereRadius_SU;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Sphere(float center[3],
                                                          float radius,
                                                          bool outward_normal,
                                                          bool track_forces) {
    BC_params_t<float, float3> p;
    // set center, radius, norm
    p.sphere_params.sphere_center.x = center[0];
    p.sphere_params.sphere_center.y = center[1];
    p.sphere_params.sphere_center.z = center[2];
    p.sphere_params.radius = radius;
    p.active = true;
    p.fixed = true;
    p.track_forces = track_forces;

    if (outward_normal) {
        // negate forces to push particles outward
        p.sphere_params.normal_sign = -1;
    } else {
        p.sphere_params.normal_sign = 1;
    }

    BC_type_list.push_back(BC_type::SPHERE);
    BC_params_list_UU.push_back(p);
    BC_offset_function_list.push_back(GranPosFunction_default);
    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Cone_Z(float cone_tip[3],
                                                          float slope,
                                                          float hmax,
                                                          float hmin,
                                                          bool outward_normal,
                                                          bool track_forces) {
    BC_params_t<float, float3> p;
    // set center, radius, norm
    p.cone_params.cone_tip.x = cone_tip[0];
    p.cone_params.cone_tip.y = cone_tip[1];
    p.cone_params.cone_tip.z = cone_tip[2];
    p.cone_params.hmax = hmax;
    p.cone_params.hmin = hmin;
    p.cone_params.slope = slope;
    p.active = true;
    p.fixed = true;
    p.track_forces = track_forces;

    if (outward_normal) {
        // negate forces to push particles outward
        p.cone_params.normal_sign = -1;
    } else {
        p.cone_params.normal_sign = 1;
    }

    BC_type_list.push_back(BC_type::CONE);
    BC_params_list_UU.push_back(p);
    BC_offset_function_list.push_back(GranPosFunction_default);

    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Plane(float plane_pos[3], float plane_normal[3], bool track_forces) {
    BC_params_t<float, float3> p;
    p.plane_params.position.x = plane_pos[0];
    p.plane_params.position.y = plane_pos[1];
    p.plane_params.position.z = plane_pos[2];

    p.plane_params.normal.x = plane_normal[0];
    p.plane_params.normal.y = plane_normal[1];
    p.plane_params.normal.z = plane_normal[2];
    p.active = true;
    p.fixed = true;

    p.track_forces = track_forces;

    BC_type_list.push_back(BC_type::PLANE);
    BC_params_list_UU.push_back(p);
    BC_offset_function_list.push_back(GranPosFunction_default);

    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGranular_MonodisperseSMC::Create_BC_Cyl_Z(float center[3],
                                                         float radius,
                                                         bool outward_normal,
                                                         bool track_forces) {
    BC_params_t<float, float3> p;
    p.cyl_params.center.x = center[0];
    p.cyl_params.center.y = center[1];
    p.cyl_params.center.z = center[2];

    p.cyl_params.radius = radius;
    p.active = true;
    p.fixed = true;

    p.track_forces = track_forces;

    if (outward_normal) {
        // negate forces to push particles outward
        p.cyl_params.normal_sign = -1;
    } else {
        p.cyl_params.normal_sign = 1;
    }

    BC_type_list.push_back(BC_type::CYLINDER);
    BC_params_list_UU.push_back(p);
    BC_offset_function_list.push_back(GranPosFunction_default);

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

double ChSystemGranular_MonodisperseSMC::get_max_K() const {
    return std::max(K_n_s2s_UU, K_n_s2w_UU);
}

// set the position of a BC and account for the offset
void ChSystemGranular_MonodisperseSMC::setBCOffset(const BC_type& bc_type,
                                                   const BC_params_t<float, float3>& params_UU,
                                                   BC_params_t<int64_t, int64_t3>& params_SU,
                                                   double3 offset_UU) {
    int64_t3 old_pos = {0, 0, 0};
    int64_t3 new_pos = {0, 0, 0};
    switch (bc_type) {
        case BC_type::SPHERE: {
            old_pos = params_SU.sphere_params.sphere_center;
            params_SU.sphere_params.sphere_center.x =
                convertToPosSU<int64_t, float>(params_UU.sphere_params.sphere_center.x + offset_UU.x);
            params_SU.sphere_params.sphere_center.y =
                convertToPosSU<int64_t, float>(params_UU.sphere_params.sphere_center.y + offset_UU.y);
            params_SU.sphere_params.sphere_center.z =
                convertToPosSU<int64_t, float>(params_UU.sphere_params.sphere_center.z + offset_UU.z);
            new_pos = params_SU.sphere_params.sphere_center;

            break;
        }

        case BC_type::CONE: {
            old_pos = params_SU.cone_params.cone_tip;
            params_SU.cone_params.cone_tip.x =
                convertToPosSU<int64_t, float>(params_UU.cone_params.cone_tip.x + offset_UU.x);
            params_SU.cone_params.cone_tip.y =
                convertToPosSU<int64_t, float>(params_UU.cone_params.cone_tip.y + offset_UU.y);
            params_SU.cone_params.cone_tip.z =
                convertToPosSU<int64_t, float>(params_UU.cone_params.cone_tip.z + offset_UU.z);
            new_pos = params_SU.cone_params.cone_tip;

            params_SU.cone_params.hmax = convertToPosSU<int64_t, float>(params_UU.cone_params.hmax + offset_UU.z);
            params_SU.cone_params.hmin = convertToPosSU<int64_t, float>(params_UU.cone_params.hmin + offset_UU.z);
            break;
        }
        case BC_type::PLANE: {
            old_pos = params_SU.plane_params.position;

            params_SU.plane_params.position.x =
                convertToPosSU<int64_t, float>(params_UU.plane_params.position.x + offset_UU.x);
            params_SU.plane_params.position.y =
                convertToPosSU<int64_t, float>(params_UU.plane_params.position.y + offset_UU.y);
            params_SU.plane_params.position.z =
                convertToPosSU<int64_t, float>(params_UU.plane_params.position.z + offset_UU.z);
            new_pos = params_SU.plane_params.position;

            break;
        }
        case BC_type::CYLINDER: {
            old_pos = params_SU.cyl_params.center;

            params_SU.cyl_params.center.x = convertToPosSU<int64_t, float>(params_UU.cyl_params.center.x + offset_UU.x);
            params_SU.cyl_params.center.y = convertToPosSU<int64_t, float>(params_UU.cyl_params.center.y + offset_UU.y);
            params_SU.cyl_params.center.z = convertToPosSU<int64_t, float>(params_UU.cyl_params.center.z + offset_UU.z);
            new_pos = params_SU.cyl_params.center;

            break;
        }
        default: {
            printf("ERROR: Unsupported BC Type!\n");
            exit(1);
        }

            // do midpoint approx for velocity
            params_SU.vel_SU.x = (new_pos.x - old_pos.x) / stepSize_SU;
            params_SU.vel_SU.y = (new_pos.y - old_pos.y) / stepSize_SU;
            params_SU.vel_SU.z = (new_pos.z - old_pos.z) / stepSize_SU;
    }
}

void ChSystemGranular_MonodisperseSMC::convertBCUnits() {
    for (int i = 0; i < BC_type_list.size(); i++) {
        auto bc_type = BC_type_list.at(i);
        BC_params_t<float, float3> params_UU = BC_params_list_UU.at(i);
        BC_params_t<int64_t, int64_t3> params_SU;

        params_SU.active = params_UU.active;
        params_SU.fixed = params_UU.fixed;
        params_SU.track_forces = params_UU.track_forces;
        // always start at rest
        params_SU.vel_SU = {0, 0, 0};
        switch (bc_type) {
            case BC_type::SPHERE: {
                printf("adding sphere!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));
                params_SU.sphere_params.radius = convertToPosSU<int64_t, float>(params_UU.sphere_params.radius);
                params_SU.sphere_params.normal_sign = params_UU.sphere_params.normal_sign;

                BC_params_list_SU.push_back(params_SU);
                break;
            }

            case BC_type::CONE: {
                printf("adding cone!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));

                params_SU.cone_params.slope = params_UU.cone_params.slope;
                params_SU.cone_params.normal_sign = params_UU.cone_params.normal_sign;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            case BC_type::PLANE: {
                printf("adding plane!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));

                // normal is unitless
                // TODO normalize this just in case
                // float abs = Length(params_UU);
                params_SU.plane_params.normal.x = params_UU.plane_params.normal.x;
                params_SU.plane_params.normal.y = params_UU.plane_params.normal.y;
                params_SU.plane_params.normal.z = params_UU.plane_params.normal.z;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            case BC_type::CYLINDER: {
                printf("adding cylinder!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));

                // normal is unitless
                // TODO normalize this just in case
                // float abs = Length(params_UU);
                params_SU.cyl_params.radius = convertToPosSU<int64_t, float>(params_UU.cyl_params.radius);
                params_SU.cyl_params.normal_sign = params_UU.cyl_params.normal_sign;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            default: {
                printf("ERROR: Unsupported BC Type!\n");
                exit(1);
            }
        }

        // always start at rest
        params_SU.vel_SU = {0, 0, 0};
    }
}

void ChSystemGranular_MonodisperseSMC::initializeSpheres() {
    switchToSimUnits();

    // Set aside memory for holding data structures worked with. Get some initializations going
    partitionBD();

    copyConstSphereDataToDevice();

    determineNewStepSize_SU();
    convertBCUnits();
    setupSphereDataStructures();

    // Seed arrays that are populated by the kernel call
    resetBroadphaseInformation();
    resetBCForces();

    printf("Doing initial broadphase!\n");
    printf("max possible composite offset with 256 limit is %zu\n", (size_t)nSDs * MAX_COUNT_OF_SPHERES_PER_SD);
    runSphereBroadphase();
    printf("Initial broadphase finished!\n");

    int dev_ID;
    gpuErrchk(cudaGetDevice(&dev_ID));
    // these two will be mostly read by everyone
    gpuErrchk(cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetReadMostly, dev_ID));
    gpuErrchk(cudaMemAdvise(sphere_data, sizeof(*sphere_data), cudaMemAdviseSetReadMostly, dev_ID));

    printf("z grav term with timestep %f is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gran_params->gravAcc_Z_SU);
    printf("running at approximate timestep %f\n", stepSize_SU * gran_params->TIME_UNIT);
}

// mean to be overriden by children
void ChSystemGranular_MonodisperseSMC::initialize() {
    initializeSpheres();
    size_t approx_mem_usage = estimateMemUsage();
    printf("Approx mem usage is %s\n", pretty_format_bytes(approx_mem_usage).c_str());
}

// Set particle positions in UU
void ChSystemGranular_MonodisperseSMC::setParticlePositions(const std::vector<ChVector<float>>& points) {
    user_sphere_positions = points;  // Copy points to class vector
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
    gran_params->BD_frame_X = -.5 * ((int64_t)nSDs_X * SD_size_X);
    gran_params->BD_frame_Y = -.5 * ((int64_t)nSDs_Y * SD_size_Y);
    gran_params->BD_frame_Z = -.5 * ((int64_t)nSDs_Z * SD_size_Z);

    // permanently cache the initial frame
    BD_rest_frame_SU = make_longlong3(gran_params->BD_frame_X, gran_params->BD_frame_Y, gran_params->BD_frame_Z);

    printf("%u Sds as %u, %u, %u\n", gran_params->nSDs, gran_params->nSDs_X, gran_params->nSDs_Y, gran_params->nSDs_Z);

    // allocate mem for array saying for each SD how many spheres touch it
    TRACK_VECTOR_RESIZE(SD_NumSpheresTouching, nSDs, "SD_numSpheresTouching", 0);
    TRACK_VECTOR_RESIZE(SD_SphereCompositeOffsets, nSDs, "SD_SphereCompositeOffsets", 0);
}

/**
This method defines the mass, time, length Simulation Units. It also sets several other constants that enter the
scaling of various physical quantities set by the user.
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

    float rolling_scalingFactor = 1.f;
    if (gran_params->rolling_mode == GRAN_ROLLING_MODE::VISCOUS) {
        rolling_scalingFactor = 1.f / gran_params->TIME_UNIT;
    }
    gran_params->rolling_coeff_SU = rolling_scalingFactor * rolling_coeff_UU;

    gran_params->cohesionAcc_s2s = cohesion_over_gravity * g_scalingFactor;
    gran_params->adhesionAcc_s2w = adhesion_s2w_over_gravity * g_scalingFactor;

    // Handy debug output
    printf("UU mass is %f\n", gran_params->MASS_UNIT);
    printf("SU gravity is %f, %f, %f\n", gran_params->gravAcc_X_SU, gran_params->gravAcc_Y_SU,
           gran_params->gravAcc_Z_SU);
    printf("SU radius is %u\n", gran_params->sphereRadius_SU);
    float dt_safe_estimate = sqrt(massSphere / K_n_s2s_UU);
    printf("Safe timestep is about %f\n", dt_safe_estimate);
    printf("Length unit is %0.16f\n", gran_params->LENGTH_UNIT);

    // speed at bottom if dropped from top
    float vdrop_UU = sqrt(abs(2. * box_size_Z / Z_accGrav));
    // same but in SU
    float vdrop_SU = vdrop_UU * (gran_params->TIME_UNIT / gran_params->LENGTH_UNIT);

    // damping force at bottom if we dropped a ball from the top
    float drop_damping_force = 0.5 * gran_params->sphere_mass_SU * vdrop_SU * gran_params->K_n_s2w_SU;

    // force of gravity (or the restorative force of one sphere resting on the bottom)
    float grav_force = abs(gran_params->sphere_mass_SU * gran_params->gravAcc_Z_SU);
    printf("max drop speed is %f, damp force is %f, grav force is %f, ratio is %f\n", vdrop_SU, drop_damping_force,
           grav_force, grav_force / drop_damping_force);
}
}  // namespace granular
}  // namespace chrono