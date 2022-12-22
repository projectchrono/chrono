// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut, Luning Fang, Radu Serban
// =============================================================================

#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <climits>

#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/core/ChVector.h"

#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChGpuBoundaryConditions.h"
#include "chrono_gpu/utils/ChGpuUtilities.h"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"

#include "chrono_thirdparty/chpf/particle_writer.hpp"

#ifdef USE_HDF5
    #include "H5Cpp.h"
#endif

// define it here, once and for all
size_t gran_approx_bytes_used = 0;

namespace chrono {
namespace gpu {

ChSystemGpu_impl::ChSystemGpu_impl(float sphere_rad, float density, float3 boxDims, float3 O)
    : sphere_radius_UU(sphere_rad),
      sphere_density_UU(density),
      box_size_X(boxDims.x),
      box_size_Y(boxDims.y),
      box_size_Z(boxDims.z),
      user_coord_O_X(O.x),
      user_coord_O_Y(O.y),
      user_coord_O_Z(O.z),
      stepSize_UU(1e-4f),
      nSpheres(0),
      elapsedSimTime(0.f),
      verbosity(CHGPU_VERBOSITY::INFO),
      use_min_length_unit(true),
      defragment_on_start(true),
      file_write_mode(CHGPU_OUTPUT_MODE::CSV),
      X_accGrav(0.f),
      Y_accGrav(0.f),
      Z_accGrav(0.f),
      cohesion_over_gravity(0.f),
      adhesion_s2w_over_gravity(0.f),
      K_n_s2s_UU(0.0),
      K_n_s2w_UU(0.0),
      K_t_s2s_UU(0.0),
      K_t_s2w_UU(0.0),
      Gamma_n_s2s_UU(0.0),
      Gamma_n_s2w_UU(0.0),
      Gamma_t_s2s_UU(0.0),
      Gamma_t_s2w_UU(0.0),
      use_mat_based(true),
      YoungsModulus_sphere_UU(0.0),
      YoungsModulus_wall_UU(0.0),
      COR_sphere_UU(0.0),
      COR_wall_UU(0.0),
      PoissonRatio_sphere_UU(0.0),
      PoissonRatio_wall_UU(0.0),
      rolling_coeff_s2s_UU(0.0),
      rolling_coeff_s2w_UU(0.0),
      spinning_coeff_s2s_UU(0.0),
      spinning_coeff_s2w_UU(0.0) {
    gpuErrchk(cudaMallocManaged(&gran_params, sizeof(GranParams), cudaMemAttachGlobal));
    gpuErrchk(cudaMallocManaged(&sphere_data, sizeof(SphereData), cudaMemAttachGlobal));
    psi_T = PSI_T_DEFAULT;
    psi_L = PSI_L_DEFAULT;
    psi_R = PSI_R_DEFAULT;
    gran_params->friction_mode = CHGPU_FRICTION_MODE::FRICTIONLESS;
    gran_params->rolling_mode = CHGPU_ROLLING_MODE::NO_RESISTANCE;
    gran_params->time_integrator = CHGPU_TIME_INTEGRATOR::EXTENDED_TAYLOR;
    this->time_integrator = CHGPU_TIME_INTEGRATOR::EXTENDED_TAYLOR;
    this->output_flags = ABSV;  // | VEL_COMPONENTS and | ANG_VEL_COMPONENTS are reasonable additions

    gran_params->max_safe_vel = (float)UINT_MAX;
    gran_params->recording_contactInfo = false;

    gran_params->static_friction_coeff_s2s = 0;
    gran_params->static_friction_coeff_s2w = 0;

    // Reserve seats for big box domain BCs
    BC_type_list.resize(NUM_RESERVED_BC_IDS);
    BC_params_list_UU.resize(NUM_RESERVED_BC_IDS);
    BC_offset_function_list.resize(NUM_RESERVED_BC_IDS);
}

void ChSystemGpu_impl::CreateWallBCs() {
    float plane_center_bot_X[3] = {-box_size_X / 2.f + user_coord_O_X, 0, 0};
    float plane_center_top_X[3] = {box_size_X / 2.f + user_coord_O_X, 0, 0};
    float plane_center_bot_Y[3] = {0, -box_size_Y / 2.f + user_coord_O_Y, 0};
    float plane_center_top_Y[3] = {0, box_size_Y / 2.f + user_coord_O_Y, 0};
    float plane_center_bot_Z[3] = {0, 0, -box_size_Z / 2.f + user_coord_O_Z};
    float plane_center_top_Z[3] = {0, 0, box_size_Z / 2.f + user_coord_O_Z};
    // face in upwards
    float plane_normal_bot_X[3] = {1, 0, 0};
    float plane_normal_top_X[3] = {-1, 0, 0};
    float plane_normal_bot_Y[3] = {0, 1, 0};
    float plane_normal_top_Y[3] = {0, -1, 0};
    float plane_normal_bot_Z[3] = {0, 0, 1};
    float plane_normal_top_Z[3] = {0, 0, -1};

    // create wall BCs, and put them at their reserved seats
    size_t plane_BC_X_bot = CreateBCPlane(plane_center_bot_X, plane_normal_bot_X, false, 0);
    size_t plane_BC_X_top = CreateBCPlane(plane_center_top_X, plane_normal_top_X, false, 1);
    size_t plane_BC_Y_bot = CreateBCPlane(plane_center_bot_Y, plane_normal_bot_Y, false, 2);
    size_t plane_BC_Y_top = CreateBCPlane(plane_center_top_Y, plane_normal_top_Y, false, 3);
    size_t plane_BC_Z_bot = CreateBCPlane(plane_center_bot_Z, plane_normal_bot_Z, false, 4);
    size_t plane_BC_Z_top = CreateBCPlane(plane_center_top_Z, plane_normal_top_Z, false, 5);

    // verify that we have the right IDs for these walls
    assert(plane_BC_X_bot == BD_WALL_ID_X_BOT);
    assert(plane_BC_X_top == BD_WALL_ID_X_TOP);
    assert(plane_BC_Y_bot == BD_WALL_ID_Y_BOT);
    assert(plane_BC_Y_top == BD_WALL_ID_Y_TOP);
    assert(plane_BC_Z_bot == BD_WALL_ID_Z_BOT);
    assert(plane_BC_Z_top == BD_WALL_ID_Z_TOP);
}

ChSystemGpu_impl::~ChSystemGpu_impl() {
    gpuErrchk(cudaFree(gran_params));
}

size_t ChSystemGpu_impl::EstimateMemUsage() const {
    return gran_approx_bytes_used;
}

void ChSystemGpu_impl::packSphereDataPointers() {
    // Set data from system
    sphere_data->sphere_local_pos_X = sphere_local_pos_X.data();
    sphere_data->sphere_local_pos_Y = sphere_local_pos_Y.data();
    sphere_data->sphere_local_pos_Z = sphere_local_pos_Z.data();
    sphere_data->pos_X_dt = pos_X_dt.data();
    sphere_data->pos_Y_dt = pos_Y_dt.data();
    sphere_data->pos_Z_dt = pos_Z_dt.data();

    sphere_data->sphere_owner_SDs = sphere_owner_SDs.data();

    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
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

    if (time_integrator == CHGPU_TIME_INTEGRATOR::CHUNG) {
        sphere_data->sphere_acc_X_old = sphere_acc_X_old.data();
        sphere_data->sphere_acc_Y_old = sphere_acc_Y_old.data();
        sphere_data->sphere_acc_Z_old = sphere_acc_Z_old.data();
        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            sphere_data->sphere_ang_acc_X_old = sphere_ang_acc_X_old.data();
            sphere_data->sphere_ang_acc_Y_old = sphere_ang_acc_Y_old.data();
            sphere_data->sphere_ang_acc_Z_old = sphere_ang_acc_Z_old.data();
        }
    }

    sphere_data->sphere_fixed = sphere_fixed.data();

    sphere_data->sphere_stats_buffer = sphere_stats_buffer.data();
    sphere_data->sphere_stats_buffer_int = sphere_stats_buffer_int.data();

    sphere_data->SD_NumSpheresTouching = SD_NumSpheresTouching.data();
    sphere_data->SD_SphereCompositeOffsets = SD_SphereCompositeOffsets.data();
    sphere_data->SD_SphereCompositeOffsets_SP = SD_SphereCompositeOffsets_ScratchPad.data();
    sphere_data->spheres_in_SD_composite = spheres_in_SD_composite.data();

    if (gran_params->friction_mode == CHGPU_FRICTION_MODE::MULTI_STEP ||
        gran_params->friction_mode == CHGPU_FRICTION_MODE::SINGLE_STEP) {
        sphere_data->contact_partners_map = contact_partners_map.data();
        sphere_data->contact_active_map = contact_active_map.data();
    }

    if (gran_params->friction_mode == CHGPU_FRICTION_MODE::MULTI_STEP) {
        sphere_data->contact_history_map = contact_history_map.data();
        sphere_data->contact_duration = contact_duration.data();
    }

    if (gran_params->recording_contactInfo == true) {
        sphere_data->normal_contact_force = normal_contact_force.data();

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            sphere_data->tangential_friction_force = tangential_friction_force.data();
        }

        if (gran_params->rolling_mode != CHGPU_ROLLING_MODE::NO_RESISTANCE) {
            sphere_data->rolling_friction_torque = rolling_friction_torque.data();
            sphere_data->char_collision_time = char_collision_time.data();
            sphere_data->v_rot_array = v_rot_array.data();
        }
    }
}

void ChSystemGpu_impl::WriteRawParticles(std::ofstream& ptFile) const {
    for (unsigned int n = 0; n < nSpheres; n++) {
        unsigned int ownerSD = sphere_owner_SDs.at(n);
        int3 ownerSD_trip = getSDTripletFromID(ownerSD);
        float x_UU = (float)(sphere_local_pos_X[n] * LENGTH_SU2UU);
        float y_UU = (float)(sphere_local_pos_Y[n] * LENGTH_SU2UU);
        float z_UU = (float)(sphere_local_pos_Z[n] * LENGTH_SU2UU);

        x_UU += (float)(gran_params->BD_frame_X * LENGTH_SU2UU);
        y_UU += (float)(gran_params->BD_frame_Y * LENGTH_SU2UU);
        z_UU += (float)(gran_params->BD_frame_Z * LENGTH_SU2UU);

        x_UU += (float)(((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * LENGTH_SU2UU);
        y_UU += (float)(((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * LENGTH_SU2UU);
        z_UU += (float)(((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * LENGTH_SU2UU);

        ptFile.write((const char*)&x_UU, sizeof(float));
        ptFile.write((const char*)&y_UU, sizeof(float));
        ptFile.write((const char*)&z_UU, sizeof(float));

        if (GET_OUTPUT_SETTING(VEL_COMPONENTS)) {
            float vx_UU = (float)(pos_X_dt[n] * LENGTH_SU2UU / TIME_SU2UU);
            float vy_UU = (float)(pos_Y_dt[n] * LENGTH_SU2UU / TIME_SU2UU);
            float vz_UU = (float)(pos_Z_dt[n] * LENGTH_SU2UU / TIME_SU2UU);

            ptFile.write((const char*)&vx_UU, sizeof(float));
            ptFile.write((const char*)&vy_UU, sizeof(float));
            ptFile.write((const char*)&vz_UU, sizeof(float));
        }

        if (GET_OUTPUT_SETTING(ABSV)) {
            float absv = (float)(std::sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                                           pos_Z_dt.at(n) * pos_Z_dt.at(n)) *
                                 VEL_SU2UU);

            ptFile.write((const char*)&absv, sizeof(float));
        }

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS && GET_OUTPUT_SETTING(ANG_VEL_COMPONENTS)) {
            float omega_x_UU = (float)(sphere_Omega_X.at(n) / TIME_SU2UU);
            float omega_y_UU = (float)(sphere_Omega_Y.at(n) / TIME_SU2UU);
            float omega_z_UU = (float)(sphere_Omega_Z.at(n) / TIME_SU2UU);
            ptFile.write((const char*)&omega_x_UU, sizeof(float));
            ptFile.write((const char*)&omega_y_UU, sizeof(float));
            ptFile.write((const char*)&omega_z_UU, sizeof(float));
        }
    }
}

void ChSystemGpu_impl::WriteCsvParticles(std::ofstream& ptFile) const {
    // Dump to a stream, write to file only at end
    std::ostringstream outstrstream;
    outstrstream << "x,y,z";
    if (GET_OUTPUT_SETTING(VEL_COMPONENTS)) {
        outstrstream << ",vx,vy,vz";
    }
    if (GET_OUTPUT_SETTING(ABSV)) {
        outstrstream << ",absv";
    }
    if (GET_OUTPUT_SETTING(FIXITY)) {
        outstrstream << ",fixed";
    }

    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS && GET_OUTPUT_SETTING(ANG_VEL_COMPONENTS)) {
        outstrstream << ",wx,wy,wz";
    }

    if (GET_OUTPUT_SETTING(FORCE_COMPONENTS)) {
        outstrstream << ",fx,fy,fz";
    }

    outstrstream << "\n";
    for (unsigned int n = 0; n < nSpheres; n++) {
        unsigned int ownerSD = sphere_owner_SDs.at(n);
        int3 ownerSD_trip = getSDTripletFromID(ownerSD);

        float x_UU = (float)(sphere_local_pos_X[n] * LENGTH_SU2UU);
        float y_UU = (float)(sphere_local_pos_Y[n] * LENGTH_SU2UU);
        float z_UU = (float)(sphere_local_pos_Z[n] * LENGTH_SU2UU);

        x_UU += (float)(gran_params->BD_frame_X * LENGTH_SU2UU);
        y_UU += (float)(gran_params->BD_frame_Y * LENGTH_SU2UU);
        z_UU += (float)(gran_params->BD_frame_Z * LENGTH_SU2UU);

        x_UU += (float)(((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * LENGTH_SU2UU);
        y_UU += (float)(((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * LENGTH_SU2UU);
        z_UU += (float)(((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * LENGTH_SU2UU);

        outstrstream << x_UU << "," << y_UU << "," << z_UU;

        if (GET_OUTPUT_SETTING(VEL_COMPONENTS)) {
            float vx_UU = (float)(pos_X_dt[n] * LENGTH_SU2UU / TIME_SU2UU);
            float vy_UU = (float)(pos_Y_dt[n] * LENGTH_SU2UU / TIME_SU2UU);
            float vz_UU = (float)(pos_Z_dt[n] * LENGTH_SU2UU / TIME_SU2UU);

            outstrstream << "," << vx_UU << "," << vy_UU << "," << vz_UU;
        }

        if (GET_OUTPUT_SETTING(ABSV)) {
            float absv = (float)(std::sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                                           pos_Z_dt.at(n) * pos_Z_dt.at(n)) *
                                 VEL_SU2UU);
            outstrstream << "," << absv;
        }

        if (GET_OUTPUT_SETTING(FIXITY)) {
            int fixed = (int)sphere_fixed[n];
            outstrstream << "," << fixed;
        }

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS && GET_OUTPUT_SETTING(ANG_VEL_COMPONENTS)) {
            outstrstream << "," << sphere_Omega_X.at(n) / TIME_SU2UU << "," << sphere_Omega_Y.at(n) / TIME_SU2UU << ","
                         << sphere_Omega_Z.at(n) / TIME_SU2UU;
        }

        if (GET_OUTPUT_SETTING(FORCE_COMPONENTS)) {
            double fx = (sphere_acc_X.at(n) - gran_params->gravAcc_X_SU) * gran_params->sphere_mass_SU * FORCE_SU2UU;
            double fy = (sphere_acc_Y.at(n) - gran_params->gravAcc_Y_SU) * gran_params->sphere_mass_SU * FORCE_SU2UU;
            double fz = (sphere_acc_Z.at(n) - gran_params->gravAcc_Z_SU) * gran_params->sphere_mass_SU * FORCE_SU2UU;
            outstrstream << "," << fx << "," << fy << "," << fz;
        }

        outstrstream << "\n";
    }

    ptFile << outstrstream.str();
}

void ChSystemGpu_impl::WriteChPFParticles(std::ofstream& ptFile) const {
    ParticleFormatWriter pw;

    std::vector<float> v_x_UU(sphere_local_pos_X.size());
    std::vector<float> v_y_UU(sphere_local_pos_Y.size());
    std::vector<float> v_z_UU(sphere_local_pos_Z.size());
    std::vector<float> sphere_radius(gran_params->nSpheres);

    for (unsigned int n = 0; n < nSpheres; n++) {
        unsigned int ownerSD = sphere_owner_SDs.at(n);
        int3 ownerSD_trip = getSDTripletFromID(ownerSD);
        float x_UU = (float)(sphere_local_pos_X[n] * LENGTH_SU2UU);
        float y_UU = (float)(sphere_local_pos_Y[n] * LENGTH_SU2UU);
        float z_UU = (float)(sphere_local_pos_Z[n] * LENGTH_SU2UU);

        x_UU += (float)(gran_params->BD_frame_X * LENGTH_SU2UU);
        y_UU += (float)(gran_params->BD_frame_Y * LENGTH_SU2UU);
        z_UU += (float)(gran_params->BD_frame_Z * LENGTH_SU2UU);

        x_UU += (float)(((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * LENGTH_SU2UU);
        y_UU += (float)(((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * LENGTH_SU2UU);
        z_UU += (float)(((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * LENGTH_SU2UU);

        v_x_UU[n] = x_UU;
        v_y_UU[n] = y_UU;
        v_z_UU[n] = z_UU;
        sphere_radius[n] = gran_params->sphereRadius_SU * LENGTH_SU2UU;
    }

    pw.write(ptFile, ParticleFormatWriter::CompressionType::NONE, v_x_UU, v_y_UU, v_z_UU, sphere_radius);
}

#ifdef USE_HDF5
void ChSystemGpu_impl::WriteH5Particles(H5::H5File& ptFile) const {
    float* x = new float[nSpheres];
    float* y = new float[nSpheres];
    float* z = new float[nSpheres];

    for (size_t n = 0; n < nSpheres; n++) {
        unsigned int ownerSD = sphere_owner_SDs.at(n);
        int3 ownerSD_trip = getSDTripletFromID(ownerSD);
        float x_UU = sphere_local_pos_X[n] * LENGTH_SU2UU;
        float y_UU = sphere_local_pos_Y[n] * LENGTH_SU2UU;
        float z_UU = sphere_local_pos_Z[n] * LENGTH_SU2UU;

        x_UU += gran_params->BD_frame_X * LENGTH_SU2UU;
        y_UU += gran_params->BD_frame_Y * LENGTH_SU2UU;
        z_UU += gran_params->BD_frame_Z * LENGTH_SU2UU;

        x_UU += ((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * LENGTH_SU2UU;
        y_UU += ((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * LENGTH_SU2UU;
        z_UU += ((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * LENGTH_SU2UU;

        x[n] = x_UU;
        y[n] = y_UU;
        z[n] = z_UU;
    }

    hsize_t dims[1] = {nSpheres};
    H5::DataSpace dataspace(1, dims);

    H5::DataSet ds_x = ptFile.createDataSet("x", H5::PredType::NATIVE_FLOAT, dataspace);
    ds_x.write(x, H5::PredType::NATIVE_FLOAT);

    H5::DataSet ds_y = ptFile.createDataSet("y", H5::PredType::NATIVE_FLOAT, dataspace);
    ds_y.write(y, H5::PredType::NATIVE_FLOAT);

    H5::DataSet ds_z = ptFile.createDataSet("z", H5::PredType::NATIVE_FLOAT, dataspace);
    ds_z.write(z, H5::PredType::NATIVE_FLOAT);
    delete[] x, y, z;

    if (GET_OUTPUT_SETTING(VEL_COMPONENTS)) {
        float* vx = new float[nSpheres];
        float* vy = new float[nSpheres];
        float* vz = new float[nSpheres];
        for (size_t n = 0; n < nSpheres; n++) {
            vx[n] = pos_X_dt[n] * LENGTH_SU2UU / TIME_SU2UU;
            vy[n] = pos_Y_dt[n] * LENGTH_SU2UU / TIME_SU2UU;
            vz[n] = pos_Z_dt[n] * LENGTH_SU2UU / TIME_SU2UU;
        }

        H5::DataSet ds_vx = ptFile.createDataSet("vx", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_vx.write(vx, H5::PredType::NATIVE_FLOAT);

        H5::DataSet ds_vy = ptFile.createDataSet("vy", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_vy.write(vy, H5::PredType::NATIVE_FLOAT);

        H5::DataSet ds_vz = ptFile.createDataSet("vz", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_vz.write(vz, H5::PredType::NATIVE_FLOAT);

        delete[] vx, vy, vz;
    }

    if (GET_OUTPUT_SETTING(ABSV)) {
        float* absv = new float[nSpheres];
        for (size_t n = 0; n < nSpheres; n++) {
            absv[n] = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                           pos_Z_dt.at(n) * pos_Z_dt.at(n)) *
                      VEL_SU2UU;
        }
        H5::DataSet ds_absv = ptFile.createDataSet("absv", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_absv.write(absv, H5::PredType::NATIVE_FLOAT);

        delete[] absv;
    }

    if (GET_OUTPUT_SETTING(FIXITY)) {
        unsigned char* fixed = new unsigned char[nSpheres];
        for (size_t n = 0; n < nSpheres; n++) {
            fixed[n] = (unsigned char)sphere_fixed[n];
        }
        H5::DataSet ds_fixed = ptFile.createDataSet("fixed", H5::PredType::NATIVE_UCHAR, dataspace);
        ds_fixed.write(fixed, H5::PredType::NATIVE_UCHAR);

        delete[] fixed;
    }

    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS && GET_OUTPUT_SETTING(ANG_VEL_COMPONENTS)) {
        float* wx = new float[nSpheres];
        float* wy = new float[nSpheres];
        float* wz = new float[nSpheres];
        for (size_t n = 0; n < nSpheres; n++) {
            wx[n] = sphere_Omega_X[n] / TIME_SU2UU;
            wy[n] = sphere_Omega_Y[n] / TIME_SU2UU;
            wz[n] = sphere_Omega_Z[n] / TIME_SU2UU;
        }

        H5::DataSet ds_wx = ptFile.createDataSet("wx", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_wx.write(wx, H5::PredType::NATIVE_FLOAT);

        H5::DataSet ds_wy = ptFile.createDataSet("wy", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_wy.write(wy, H5::PredType::NATIVE_FLOAT);

        H5::DataSet ds_wz = ptFile.createDataSet("wz", H5::PredType::NATIVE_FLOAT, dataspace);
        ds_wz.write(wz, H5::PredType::NATIVE_FLOAT);

        delete[] wx, wy, wz;
    }
}
#endif

/// Get rolling friction torque between body i and j, return 0 if not in contact
float3 ChSystemGpu_impl::getRollingFrictionTorque(unsigned int i, unsigned int j) {
    if (gran_params->recording_contactInfo == false) {
        printf("ERROR: recording_contactInfo set to false!\n");
        exit(1);
    }

    if (gran_params->rolling_mode == CHGPU_ROLLING_MODE::NO_RESISTANCE) {
        return make_float3(0.0f, 0.0f, 0.0f);
    }

    unsigned int bodyAoffset = i * MAX_SPHERES_TOUCHED_BY_SPHERE;
    // go through all possible neighbors
    for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
        unsigned int theirSphereMappingID = bodyAoffset + neighborID;
        unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];

        if (theirSphereID == j) {
            return make_float3(rolling_friction_torque[theirSphereMappingID].x * FORCE_SU2UU * LENGTH_SU2UU,
                               rolling_friction_torque[theirSphereMappingID].y * FORCE_SU2UU * LENGTH_SU2UU,
                               rolling_friction_torque[theirSphereMappingID].z * FORCE_SU2UU * LENGTH_SU2UU);
        }
    }
    return make_float3(0.0f, 0.0f, 0.0f);
}

void ChSystemGpu_impl::getNeighbors(unsigned int ID, std::vector<unsigned int>& neighborList) {
    unsigned int bodyAoffset = ID * MAX_SPHERES_TOUCHED_BY_SPHERE;
    // go through all possible neighbors
    for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
        unsigned int theirSphereMappingID = bodyAoffset + neighborID;
        unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];

        if (theirSphereID != -1) {
            neighborList.push_back(theirSphereID);
        }
    }
}

// get rolling friction v_rot
float3 ChSystemGpu_impl::getRollingVrot(unsigned int i, unsigned int j) {
    if (gran_params->recording_contactInfo == false) {
        printf("ERROR: recording_contactInfo set to false!\n");
        exit(1);
    }

    if (gran_params->rolling_mode == CHGPU_ROLLING_MODE::NO_RESISTANCE) {
        return make_float3(0.0f, 0.0f, 0.0f);
    }

    unsigned int bodyAoffset = i * MAX_SPHERES_TOUCHED_BY_SPHERE;
    // go through all possible neighbors
    for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
        unsigned int theirSphereMappingID = bodyAoffset + neighborID;
        unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];

        if (theirSphereID == j) {
            return make_float3(v_rot_array[theirSphereMappingID].x * LENGTH_SU2UU / TIME_SU2UU,
                               v_rot_array[theirSphereMappingID].y * LENGTH_SU2UU / TIME_SU2UU,
                               v_rot_array[theirSphereMappingID].z * LENGTH_SU2UU / TIME_SU2UU);
        }
    }
    return make_float3(0.0f, 0.0f, 0.0f);
}

/// get rolling characterisitc contact time
float ChSystemGpu_impl::getRollingCharContactTime(unsigned int i, unsigned int j) {
    if (gran_params->recording_contactInfo == false) {
        printf("ERROR: recording_contactInfo set to false!\n");
        exit(1);
    }

    if (gran_params->rolling_mode == CHGPU_ROLLING_MODE::NO_RESISTANCE) {
        return 0.0f;
    }

    unsigned int bodyAoffset = i * MAX_SPHERES_TOUCHED_BY_SPHERE;
    // go through all possible neighbors
    for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
        unsigned int theirSphereMappingID = bodyAoffset + neighborID;
        unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];

        if (theirSphereID == j) {
            return char_collision_time[theirSphereMappingID] * TIME_SU2UU;
        }
    }
    return 0.0f;
}

/// Get tangential friction force between body i and j, return 0 if not in contact
float3 ChSystemGpu_impl::getSlidingFrictionForce(unsigned int i, unsigned int j) {
    if (gran_params->recording_contactInfo == false) {
        printf("ERROR: recording_contactInfo set to false!\n");
        exit(1);
    }

    if (i >= nSpheres) {
        int tmp = i;
        i = j;
        j = tmp;
    }

    unsigned int bodyAoffset = i * MAX_SPHERES_TOUCHED_BY_SPHERE;
    // go through all possible neighbors
    for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
        unsigned int theirSphereMappingID = bodyAoffset + neighborID;
        unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];

        if (theirSphereID == j) {
            return make_float3(tangential_friction_force[theirSphereMappingID].x * FORCE_SU2UU,
                               tangential_friction_force[theirSphereMappingID].y * FORCE_SU2UU,
                               tangential_friction_force[theirSphereMappingID].z * FORCE_SU2UU);
        }
    }
    return make_float3(0.0f, 0.0f, 0.0f);
}

/// Get normal friction force between body i and j, return 0 if not in contact
/// for sphere-sphere contact, index sequence does not matter
/// for sphere-wall contact, force only stored in one spot, associated with sphere, so i has to be smaller than j, where
/// j is the BC index, i is sphere ID
float3 ChSystemGpu_impl::getNormalForce(unsigned int i, unsigned int j) {
    if (gran_params->recording_contactInfo == false) {
        printf("ERROR: recording_contactInfo set to false!\n");
        exit(1);
    }

    if (i >= nSpheres) {
        int tmp = i;
        i = j;
        j = tmp;
    }

    unsigned int bodyAoffset = i * MAX_SPHERES_TOUCHED_BY_SPHERE;
    // go through all possible neighbors
    for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
        unsigned int theirSphereMappingID = bodyAoffset + neighborID;
        unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];

        if (theirSphereID == j) {
            return make_float3(normal_contact_force[theirSphereMappingID].x * FORCE_SU2UU,
                               normal_contact_force[theirSphereMappingID].y * FORCE_SU2UU,
                               normal_contact_force[theirSphereMappingID].z * FORCE_SU2UU);
        }
    }
    return make_float3(0.0f, 0.0f, 0.0f);
}

void ChSystemGpu_impl::WriteContactInfoFile(const std::string& outfilename) const {
    if ((gran_params->recording_contactInfo == false) ||
        (gran_params->friction_mode == CHGPU_FRICTION_MODE::FRICTIONLESS)) {
        CHGPU_ERROR("ERROR! You did not enable contact info recording or are using frictionless model!\n");
    } else {
        // write contact info as an csv style in the following format
        // body i, body j, n_mag, fx, fy, fz, mx, my, mz

        std::ofstream ptFile(outfilename, std::ios::out);
        // Dump to a stream, write to file only at end
        std::ostringstream outstrstream;
        outstrstream << "bi, bj, n_mag";

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            outstrstream << ", fx, fy, fz";
        }

        if (gran_params->rolling_mode != CHGPU_ROLLING_MODE::NO_RESISTANCE) {
            outstrstream << ", mx, my, mz";
        }
        outstrstream << "\n";
        for (unsigned int n = 0; n < nSpheres; n++) {
            unsigned int bodyAoffset = n * MAX_SPHERES_TOUCHED_BY_SPHERE;
            // go through all possible neighbors
            for (unsigned int neighborID = 0; neighborID < MAX_SPHERES_TOUCHED_BY_SPHERE; neighborID++) {
                unsigned int theirSphereMappingID = bodyAoffset + neighborID;
                unsigned int theirSphereID = contact_partners_map[theirSphereMappingID];
                // only write when bi < bj
                if (theirSphereID >= n && theirSphereID < nSpheres) {
                    outstrstream << n << ", " << theirSphereID;

                    outstrstream << ", " << Length(normal_contact_force[theirSphereMappingID]) * FORCE_SU2UU;

                    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
                        outstrstream << ", " << tangential_friction_force[theirSphereMappingID].x * FORCE_SU2UU << ", "
                                     << tangential_friction_force[theirSphereMappingID].y * FORCE_SU2UU << ", "
                                     << tangential_friction_force[theirSphereMappingID].z * FORCE_SU2UU;
                    }

                    if (gran_params->rolling_mode != CHGPU_ROLLING_MODE::NO_RESISTANCE) {
                        outstrstream << ", "
                                     << rolling_friction_torque[theirSphereMappingID].x * FORCE_SU2UU * LENGTH_SU2UU
                                     << ", "
                                     << rolling_friction_torque[theirSphereMappingID].y * FORCE_SU2UU * LENGTH_SU2UU
                                     << ", "
                                     << rolling_friction_torque[theirSphereMappingID].z * FORCE_SU2UU * LENGTH_SU2UU;
                    }
                    outstrstream << "\n";
                }
            }
        }
        ptFile << outstrstream.str();
    }
}

// Reset broadphase data structures
void ChSystemGpu_impl::resetBCForces() {
    // zero out reaction forces on each BC
    for (unsigned int i = 0; i < BC_params_list_SU.size(); i++) {
        if (BC_params_list_SU.at(i).track_forces) {
            BC_params_list_SU.at(i).reaction_forces = {0, 0, 0};

            if (BC_type_list.at(i) == BC_type::SPHERE) {
                BC_params_list_SU.at(i).sphere_params.reaction_torques = {0, 0, 0};
            }
        }
    }
}

// Copy constant sphere data to device, this should run at start
void ChSystemGpu_impl::copyConstSphereDataToDevice() {
    gran_params->max_x_pos_unsigned = ((int64_t)gran_params->SD_size_X_SU * gran_params->nSDs_X);
    gran_params->max_y_pos_unsigned = ((int64_t)gran_params->SD_size_Y_SU * gran_params->nSDs_Y);
    gran_params->max_z_pos_unsigned = ((int64_t)gran_params->SD_size_Z_SU * gran_params->nSDs_Z);

    INFO_PRINTF("max pos is is %llu, %llu, %llu\n", gran_params->max_x_pos_unsigned, gran_params->max_y_pos_unsigned,
                gran_params->max_z_pos_unsigned);

    int64_t true_max_pos = std::max(std::max(gran_params->max_x_pos_unsigned, gran_params->max_y_pos_unsigned),
                                    gran_params->max_z_pos_unsigned);

    // if (true_max_pos >= INT_MAX) {
    //     printf("WARNING! Max possible position is greater than INT_MAX!!!\n");
    // }

    // if (true_max_pos >= UINT_MAX) {
    //     printf("BIG WARNING! Max possible position is greater than UINT_MAX!!!\n");
    //     printf("You are now relying on Conlain's local coordinate implementation.\n");
    // }

    if (true_max_pos >= LLONG_MAX) {
        printf("ERROR! Max possible position is greater than LLONG_MAX!!!\n");
        printf("Not even local coordinates can save you now.\n");
        exit(1);
    }

    // NOTE: Assumes mass = 1
    gran_params->sphereInertia_by_r = (float)((2.0 / 5.0) * gran_params->sphere_mass_SU * gran_params->sphereRadius_SU);
}

size_t ChSystemGpu_impl::CreateBCSphere(float center[3],
                                        float radius,
                                        bool outward_normal,
                                        bool track_forces,
                                        float mass) {
    BC_params_t<float, float3> p;
    // set center, radius, norm
    p.sphere_params.sphere_center.x = center[0];
    p.sphere_params.sphere_center.y = center[1];
    p.sphere_params.sphere_center.z = center[2];
    p.sphere_params.radius = radius;

    p.sphere_params.sphere_velo.x = 0.0f;
    p.sphere_params.sphere_velo.y = 0.0f;
    p.sphere_params.sphere_velo.z = 0.0f;

    p.sphere_params.sphere_angularVelo.x = 0.0f;
    p.sphere_params.sphere_angularVelo.y = 0.0f;
    p.sphere_params.sphere_angularVelo.z = 0.0f;

    p.sphere_params.reaction_torques.x = 0.0f;
    p.sphere_params.reaction_torques.y = 0.0f;
    p.sphere_params.reaction_torques.z = 0.0f;

    p.sphere_params.mass = mass;

    p.active = true;
    p.fixed = false;
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

size_t ChSystemGpu_impl::CreateBCConeZ(float cone_tip[3],
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

size_t ChSystemGpu_impl::CreateBCPlane(float plane_pos[3], float plane_normal[3], bool track_forces, size_t position) {
    BC_params_t<float, float3> p;
    p.plane_params.position.x = plane_pos[0];
    p.plane_params.position.y = plane_pos[1];
    p.plane_params.position.z = plane_pos[2];

    double len = std::sqrt(plane_normal[0] * plane_normal[0] + plane_normal[1] * plane_normal[1] +
                           plane_normal[2] * plane_normal[2]);

    p.plane_params.normal.x = (float)(plane_normal[0] / len);
    p.plane_params.normal.y = (float)(plane_normal[1] / len);
    p.plane_params.normal.z = (float)(plane_normal[2] / len);
    p.active = true;
    p.fixed = true;

    p.track_forces = track_forces;

    p.plane_params.rotation_center = make_float3(0.0f, 0.0f, 0.0f);
    p.plane_params.angular_acc = make_float3(0.0f, 0.0f, 0.0f);

    // If position is SIZE_MAX then just use standard push_back
    if (position == SIZE_MAX) {
        BC_type_list.push_back(BC_type::PLANE);
        BC_params_list_UU.push_back(p);
        BC_offset_function_list.push_back(GranPosFunction_default);
    } else {
        BC_type_list.at(position) = BC_type::PLANE;
        BC_params_list_UU.at(position) = p;
        BC_offset_function_list.at(position) = GranPosFunction_default;
    }

    // get my index in the new array
    if (position == SIZE_MAX)
        return BC_type_list.size() - 1;
    else
        return position;
}

size_t ChSystemGpu_impl::CreateCustomizedPlate(float plate_pos_center[3], float plate_normal[3], float hdim_y) {
    BC_params_t<float, float3> p;
    p.plate_params.plate_center.x = plate_pos_center[0];
    p.plate_params.plate_center.y = plate_pos_center[1];
    p.plate_params.plate_center.z = plate_pos_center[2];

    double len = std::sqrt(plate_normal[0] * plate_normal[0] + plate_normal[1] * plate_normal[1] +
                           plate_normal[2] * plate_normal[2]);

    p.plate_params.normal.x = (float)(plate_normal[0] / len);
    p.plate_params.normal.y = (float)(plate_normal[1] / len);
    p.plate_params.normal.z = (float)(plate_normal[2] / len);

    p.plate_params.h_dim_y = hdim_y;

    p.active = true;
    p.fixed = false;

    p.track_forces = false;

    BC_type_list.push_back(BC_type::PLATE);
    BC_params_list_UU.push_back(p);
    BC_offset_function_list.push_back(GranPosFunction_default);

    // get my index in the new array
    return BC_type_list.size() - 1;
}

size_t ChSystemGpu_impl::CreateBCCylinderZ(float center[3], float radius, bool outward_normal, bool track_forces) {
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

double ChSystemGpu_impl::get_max_K() const {
    double maxK;
    if (gran_params->use_mat_based == true) {
        // material pparameter sigma for different surface
        // see reference eq 2.13, 2.14 in Book Contact Force Model, Flores and Lankarani
        double sigma_sphere = (1 - std::pow(PoissonRatio_sphere_UU, 2)) / YoungsModulus_sphere_UU;
        double sigma_wall = (1 - std::pow(PoissonRatio_sphere_UU, 2)) / YoungsModulus_wall_UU;

        maxK = 4.0 / (3.0 * (sigma_sphere + std::min(sigma_sphere, sigma_wall))) * std::sqrt(sphere_radius_UU);

        INFO_PRINTF("Use material based contact force model, maximum effective stiffnes is %e\n", maxK);
        return maxK;

    } else {
        maxK = std::max(K_n_s2s_UU, K_n_s2w_UU);
        INFO_PRINTF("Use user defined contact force model, maximum effective stiffnes is %e\n", maxK);
        return maxK;
    }
}

// set the position of a BC and account for the offset
void ChSystemGpu_impl::setBCOffset(const BC_type& bc_type,
                                   const BC_params_t<float, float3>& params_UU,
                                   BC_params_t<int64_t, int64_t3>& params_SU,
                                   double3 offset_UU) {
    int64_t3 old_pos = {0, 0, 0};
    int64_t3 new_pos = {0, 0, 0};
    switch (bc_type) {
        case BC_type::SPHERE: {
            if (params_SU.active) {
                old_pos = params_SU.sphere_params.sphere_center;
                // TODO: flag for this part? original implementation is commented out... need to figure out the
                // structure
                // TODO sphere update can be different dependent on the integrator
                float3 sphere_acc = params_SU.reaction_forces * 1.f / params_SU.sphere_params.mass;

                sphere_acc.x += gran_params->gravAcc_X_SU;
                sphere_acc.y += gran_params->gravAcc_Y_SU;
                sphere_acc.z += gran_params->gravAcc_Z_SU;

                // add influence from gravity
                float velo_update_x = sphere_acc.x * stepSize_SU;
                float velo_update_y = sphere_acc.y * stepSize_SU;
                float velo_update_z = sphere_acc.z * stepSize_SU;

                params_SU.sphere_params.sphere_velo.x += velo_update_x;
                params_SU.sphere_params.sphere_velo.y += velo_update_y;
                params_SU.sphere_params.sphere_velo.z += velo_update_z;

                params_SU.sphere_params.sphere_center.x +=
                    (lround)(params_SU.sphere_params.sphere_velo.x * stepSize_SU);
                params_SU.sphere_params.sphere_center.y +=
                    (lround)(params_SU.sphere_params.sphere_velo.y * stepSize_SU);
                params_SU.sphere_params.sphere_center.z +=
                    (lround)(params_SU.sphere_params.sphere_velo.z * stepSize_SU);

                float sphere_inertia = 0.4f * params_SU.sphere_params.mass * params_SU.sphere_params.radius *
                                       params_SU.sphere_params.radius;

                float3 ang_acc = params_SU.sphere_params.reaction_torques * 1.f / sphere_inertia;

                // angular velocity update
                params_SU.sphere_params.sphere_angularVelo.x += ang_acc.x * stepSize_SU;
                params_SU.sphere_params.sphere_angularVelo.y += ang_acc.y * stepSize_SU;
                params_SU.sphere_params.sphere_angularVelo.z += ang_acc.z * stepSize_SU;

                new_pos = params_SU.sphere_params.sphere_center;

                break;
            }
        }

        case BC_type::CONE: {
            old_pos = params_SU.cone_params.cone_tip;
            params_SU.cone_params.cone_tip.x =
                convertToPosSU<int64_t, float>((float)(params_UU.cone_params.cone_tip.x + offset_UU.x));
            params_SU.cone_params.cone_tip.y =
                convertToPosSU<int64_t, float>((float)(params_UU.cone_params.cone_tip.y + offset_UU.y));
            params_SU.cone_params.cone_tip.z =
                convertToPosSU<int64_t, float>((float)(params_UU.cone_params.cone_tip.z + offset_UU.z));
            new_pos = params_SU.cone_params.cone_tip;

            params_SU.cone_params.hmax =
                convertToPosSU<int64_t, float>((float)(params_UU.cone_params.hmax + offset_UU.z));
            params_SU.cone_params.hmin =
                convertToPosSU<int64_t, float>((float)(params_UU.cone_params.hmin + offset_UU.z));
            break;
        }
        case BC_type::PLANE: {
            old_pos = params_SU.plane_params.position;

            params_SU.plane_params.position.x =
                convertToPosSU<int64_t, float>((float)(params_UU.plane_params.position.x + offset_UU.x));
            params_SU.plane_params.position.y =
                convertToPosSU<int64_t, float>((float)(params_UU.plane_params.position.y + offset_UU.y));
            params_SU.plane_params.position.z =
                convertToPosSU<int64_t, float>((float)(params_UU.plane_params.position.z + offset_UU.z));
            new_pos = params_SU.plane_params.position;

            break;
        }
        case BC_type::CYLINDER: {
            old_pos = params_SU.cyl_params.center;

            params_SU.cyl_params.center.x =
                convertToPosSU<int64_t, float>((float)(params_UU.cyl_params.center.x + offset_UU.x));
            params_SU.cyl_params.center.y =
                convertToPosSU<int64_t, float>((float)(params_UU.cyl_params.center.y + offset_UU.y));
            params_SU.cyl_params.center.z =
                convertToPosSU<int64_t, float>((float)(params_UU.cyl_params.center.z + offset_UU.z));
            new_pos = params_SU.cyl_params.center;

            break;
        }
        default: {
            printf("ERROR: Unsupported BC Type!\n");
            exit(1);
        }
    }

    // do midpoint approx for velocity
    params_SU.vel_SU.x = (new_pos.x - old_pos.x) / stepSize_SU;
    params_SU.vel_SU.y = (new_pos.y - old_pos.y) / stepSize_SU;
    params_SU.vel_SU.z = (new_pos.z - old_pos.z) / stepSize_SU;
}

float3 ChSystemGpu_impl::GetBCPlanePosition(size_t plane_id) const {
    BC_params_t<float, float3> p = BC_params_list_UU[plane_id];
    auto offset_function = BC_offset_function_list[plane_id];
    double3 offset_UU = offset_function(elapsedSimTime);
    float3 currPos;
    currPos.x = p.plane_params.position.x + offset_UU.x;
    currPos.y = p.plane_params.position.y + offset_UU.y;
    currPos.z = p.plane_params.position.z + offset_UU.z;
    return currPos;
}

float3 ChSystemGpu_impl::GetBCSpherePosition(size_t bc_id) const {
    BC_params_t<int64_t, int64_t3> p = BC_params_list_SU[bc_id];
    float3 pos_uu;
    pos_uu.x = (float)(p.sphere_params.sphere_center.x * LENGTH_SU2UU);
    pos_uu.y = (float)(p.sphere_params.sphere_center.y * LENGTH_SU2UU);
    pos_uu.z = (float)(p.sphere_params.sphere_center.z * LENGTH_SU2UU);

    return pos_uu;
}

void ChSystemGpu_impl::SetBCSpherePosition(size_t bc_id, const float3 pos) {
    BC_params_list_SU[bc_id].sphere_params.sphere_center.x = (int)(pos.x / LENGTH_SU2UU);
    BC_params_list_SU[bc_id].sphere_params.sphere_center.y = (int)(pos.y / LENGTH_SU2UU);
    BC_params_list_SU[bc_id].sphere_params.sphere_center.z = (int)(pos.z / LENGTH_SU2UU);
}

float3 ChSystemGpu_impl::GetBCSphereVelocity(size_t bc_id) const {
    BC_params_t<int64_t, int64_t3> p = BC_params_list_SU[bc_id];
    float3 pos_uu;
    pos_uu.x = (float)(p.sphere_params.sphere_velo.x * LENGTH_SU2UU / TIME_SU2UU);
    pos_uu.y = (float)(p.sphere_params.sphere_velo.y * LENGTH_SU2UU / TIME_SU2UU);
    pos_uu.z = (float)(p.sphere_params.sphere_velo.z * LENGTH_SU2UU / TIME_SU2UU);

    return pos_uu;
}

void ChSystemGpu_impl::SetBCSphereVelocity(size_t bc_id, const float3 velo) {
    BC_params_list_SU[bc_id].sphere_params.sphere_velo.x = velo.x / LENGTH_SU2UU * TIME_SU2UU;
    BC_params_list_SU[bc_id].sphere_params.sphere_velo.y = velo.y / LENGTH_SU2UU * TIME_SU2UU;
    BC_params_list_SU[bc_id].sphere_params.sphere_velo.z = velo.z / LENGTH_SU2UU * TIME_SU2UU;
}

void ChSystemGpu_impl::SetBCPlaneRotation(size_t plane_id, double3 rotation_center, double3 rotation_omega) {
    BC_params_list_UU[plane_id].plane_params.rotation_center.x = (int64_t)rotation_center.x;
    BC_params_list_UU[plane_id].plane_params.rotation_center.y = (int64_t)rotation_center.y;
    BC_params_list_UU[plane_id].plane_params.rotation_center.z = (int64_t)rotation_center.z;

    BC_params_list_UU[plane_id].plane_params.angular_acc.x = rotation_omega.x;
    BC_params_list_UU[plane_id].plane_params.angular_acc.y = rotation_omega.y;
    BC_params_list_UU[plane_id].plane_params.angular_acc.z = rotation_omega.z;
}

bool ChSystemGpu_impl::GetBCReactionForces(size_t BC_id, float3& force) const {
    size_t max_id = BC_params_list_SU.size();
    if (BC_id >= max_id) {
        printf("ERROR: Trying to get forces for invalid BC ID %zu\n", BC_id);
        return false;
    }
    if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
        printf("ERROR: Trying to modify reserved BC ID %zu\n", BC_id);
        return false;
    }
    if (BC_params_list_SU.at(BC_id).track_forces == false) {
        printf("ERROR: Trying to get forces for non-force-tracking BC ID %zu\n", BC_id);
        return false;
    }
    if (BC_params_list_SU.at(BC_id).active == false) {
        printf("ERROR: Trying to get forces for inactive BC ID %zu\n", BC_id);
        return false;
    }
    float3 reaction_forces = BC_params_list_SU.at(BC_id).reaction_forces;

    // conversion from SU to UU force
    force.x = (float)(reaction_forces.x * FORCE_SU2UU);
    force.y = (float)(reaction_forces.y * FORCE_SU2UU);
    force.z = (float)(reaction_forces.z * FORCE_SU2UU);

    return true;
}

void ChSystemGpu_impl::convertBCUnits() {
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
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));
                params_SU.sphere_params.radius =
                    (unsigned int)convertToPosSU<int64_t, float>((params_UU.sphere_params.radius));
                params_SU.sphere_params.normal_sign = (int64_t)params_UU.sphere_params.normal_sign;

                // add BC mass
                params_SU.sphere_params.mass = params_UU.sphere_params.mass / MASS_SU2UU;

                // BC position
                params_SU.sphere_params.sphere_center.x =
                    (int)convertToPosSU<int64_t, float>((float)(params_UU.sphere_params.sphere_center.x));
                params_SU.sphere_params.sphere_center.y =
                    (int)convertToPosSU<int64_t, float>((float)(params_UU.sphere_params.sphere_center.y));
                params_SU.sphere_params.sphere_center.z =
                    (int)convertToPosSU<int64_t, float>((float)(params_UU.sphere_params.sphere_center.z));

                // BC velocity
                params_SU.sphere_params.sphere_velo.x = params_UU.sphere_params.sphere_velo.x / VEL_SU2UU;
                params_SU.sphere_params.sphere_velo.y = params_UU.sphere_params.sphere_velo.y / VEL_SU2UU;
                params_SU.sphere_params.sphere_velo.z = params_UU.sphere_params.sphere_velo.z / VEL_SU2UU;

                // BC angular velocity
                params_SU.sphere_params.sphere_angularVelo.x =
                    params_UU.sphere_params.sphere_angularVelo.x * TIME_SU2UU;
                params_SU.sphere_params.sphere_angularVelo.y =
                    params_UU.sphere_params.sphere_angularVelo.y * TIME_SU2UU;
                params_SU.sphere_params.sphere_angularVelo.z =
                    params_UU.sphere_params.sphere_angularVelo.z * TIME_SU2UU;

                // BC torque
                params_SU.sphere_params.reaction_torques.x = 0.0f;
                params_SU.sphere_params.reaction_torques.y = 0.0f;
                params_SU.sphere_params.reaction_torques.z = 0.0f;

                BC_params_list_SU.push_back(params_SU);
                break;
            }

            case BC_type::CONE: {
                INFO_PRINTF("adding cone!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));

                params_SU.cone_params.slope = params_UU.cone_params.slope;
                params_SU.cone_params.normal_sign = (int64_t)params_UU.cone_params.normal_sign;

                BC_params_list_SU.push_back(params_SU);
                break;
            }
            case BC_type::PLANE: {
                INFO_PRINTF("adding plane!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));

                // normal is unitless
                // TODO normalize this just in case
                // float abs = Length(params_UU);
                params_SU.plane_params.normal.x = (float)params_UU.plane_params.normal.x;
                params_SU.plane_params.normal.y = (float)params_UU.plane_params.normal.y;
                params_SU.plane_params.normal.z = (float)params_UU.plane_params.normal.z;

                params_SU.plane_params.rotation_center.x =
                    (int64_t)((float)params_UU.plane_params.rotation_center.x / LENGTH_SU2UU);
                params_SU.plane_params.rotation_center.y =
                    (int64_t)((float)params_UU.plane_params.rotation_center.x / LENGTH_SU2UU);
                params_SU.plane_params.rotation_center.z =
                    (int64_t)((float)params_UU.plane_params.rotation_center.x / LENGTH_SU2UU);

                params_SU.plane_params.angular_acc.x = params_UU.plane_params.angular_acc.x * TIME_SU2UU;
                params_SU.plane_params.angular_acc.y = params_UU.plane_params.angular_acc.y * TIME_SU2UU;
                params_SU.plane_params.angular_acc.z = params_UU.plane_params.angular_acc.z * TIME_SU2UU;
                BC_params_list_SU.push_back(params_SU);
                break;
            }
            case BC_type::CYLINDER: {
                INFO_PRINTF("adding cylinder!\n");
                setBCOffset(bc_type, params_UU, params_SU, make_double3(0, 0, 0));

                // normal is unitless
                // TODO normalize this just in case
                // float abs = Length(params_UU);
                params_SU.cyl_params.radius = convertToPosSU<int64_t, float>(params_UU.cyl_params.radius);
                params_SU.cyl_params.normal_sign = (int64_t)params_UU.cyl_params.normal_sign;

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

void ChSystemGpu_impl::initializeSpheres() {
    switchToSimUnits();
    INFO_PRINTF("Initialize spheres.\n");
    // Set aside memory for holding data structures worked with. Get some initializations going
    partitionBD();

    copyConstSphereDataToDevice();

    convertBCUnits();
    setupSphereDataStructures();

    // Seed arrays that are populated by the kernel call
    resetBroadphaseInformation();
    resetBCForces();

    INFO_PRINTF("Doing initial broadphase!\n");
    INFO_PRINTF("max possible composite offset with 256 limit is %zu\n", (size_t)nSDs * MAX_COUNT_OF_SPHERES_PER_SD);
    runSphereBroadphase();
    INFO_PRINTF("Initial broadphase finished!\n");

    int dev_ID;
    gpuErrchk(cudaGetDevice(&dev_ID));
    // these two will be mostly read by everyone
    gpuErrchk(cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetReadMostly, dev_ID));
    gpuErrchk(cudaMemAdvise(sphere_data, sizeof(*sphere_data), cudaMemAdviseSetReadMostly, dev_ID));

    INFO_PRINTF("z grav term with timestep %f is %f\n", stepSize_SU,
                stepSize_SU * stepSize_SU * gran_params->gravAcc_Z_SU);
    INFO_PRINTF("running at approximate timestep %f\n", stepSize_SU * TIME_SU2UU);
}

// Set particle positions in UU
void ChSystemGpu_impl::SetParticles(const std::vector<float3>& points,
                                    const std::vector<float3>& vels,
                                    const std::vector<float3>& ang_vels) {
    user_sphere_positions = points;  // Copy points to class vector
    user_sphere_vel = vels;
    user_sphere_ang_vel = ang_vels;
}

// void ChSystemGranularSMC::setParticleFixed(const std::vector<bool>& fixed) {
//     user_sphere_fixed = fixed;
// }

void ChSystemGpu_impl::SetParticleVelocity(int id, const double3& velocity) {
    // LULUTODO: sphere_data->
    sphere_data->pos_X_dt[id] = (float)(velocity.x * TIME_SU2UU / LENGTH_SU2UU);
    sphere_data->pos_Y_dt[id] = (float)(velocity.y * TIME_SU2UU / LENGTH_SU2UU);
    sphere_data->pos_Z_dt[id] = (float)(velocity.z * TIME_SU2UU / LENGTH_SU2UU);
}

// return position in user units given sphere index
float3 ChSystemGpu_impl::GetParticlePosition(int nSphere) const {
    // owner SD
    unsigned int ownerSD = sphere_owner_SDs.at(nSphere);
    int3 ownerSD_trip = getSDTripletFromID(ownerSD);
    // local position
    float x_UU = (float)(sphere_local_pos_X[nSphere] * LENGTH_SU2UU);
    float y_UU = (float)(sphere_local_pos_Y[nSphere] * LENGTH_SU2UU);
    float z_UU = (float)(sphere_local_pos_Z[nSphere] * LENGTH_SU2UU);
    // add big domain position
    x_UU += (float)(gran_params->BD_frame_X * LENGTH_SU2UU);
    y_UU += (float)(gran_params->BD_frame_Y * LENGTH_SU2UU);
    z_UU += (float)(gran_params->BD_frame_Z * LENGTH_SU2UU);
    // add subdomainNum * subdomain size
    x_UU += (float)(((int64_t)ownerSD_trip.x * gran_params->SD_size_X_SU) * LENGTH_SU2UU);
    y_UU += (float)(((int64_t)ownerSD_trip.y * gran_params->SD_size_Y_SU) * LENGTH_SU2UU);
    z_UU += (float)(((int64_t)ownerSD_trip.z * gran_params->SD_size_Z_SU) * LENGTH_SU2UU);
    return make_float3(x_UU, y_UU, z_UU);
}

/// Set particle position
void ChSystemGpu_impl::SetParticlePosition(int nSphere, double3 position) {
    // // convert user unit to simulation unit
    int64_t global_pos_X = (int64_t)((double)position.x / (double)LENGTH_SU2UU);
    int64_t global_pos_Y = (int64_t)((double)position.y / (double)LENGTH_SU2UU);
    int64_t global_pos_Z = (int64_t)((double)position.z / (double)LENGTH_SU2UU);

    // find SD for the particle center
    // offset w.r.t BD frame
    int64_t sphCenter_X_modified = -gran_params->BD_frame_X + global_pos_X;
    int64_t sphCenter_Y_modified = -gran_params->BD_frame_Y + global_pos_Y;
    int64_t sphCenter_Z_modified = -gran_params->BD_frame_Z + global_pos_Z;

    int3 ownerSD;
    // Get the SD of the sphere's center in the x, y and z direction
    ownerSD.x = (sphCenter_X_modified / (int64_t)gran_params->SD_size_X_SU);
    ownerSD.y = (sphCenter_Y_modified / (int64_t)gran_params->SD_size_Y_SU);
    ownerSD.z = (sphCenter_Z_modified / (int64_t)gran_params->SD_size_Z_SU);

    int ownerSD_x = ownerSD.x;
    int ownerSD_y = ownerSD.y;
    int ownerSD_z = ownerSD.z;

    // compute local position to that SD
    int sphere_pos_local_X = (int)(sphCenter_X_modified - (int64_t)ownerSD.x * gran_params->SD_size_X_SU);
    int sphere_pos_local_Y = (int)(sphCenter_Y_modified - (int64_t)ownerSD.y * gran_params->SD_size_Y_SU);
    int sphere_pos_local_Z = (int)(sphCenter_Z_modified - (int64_t)ownerSD.z * gran_params->SD_size_Z_SU);

    unsigned int SDID =
        ownerSD_x * gran_params->nSDs_Y * gran_params->nSDs_Z + ownerSD_y * gran_params->nSDs_Z + ownerSD_z;

    // write local pos back to global memory
    sphere_data->sphere_local_pos_X[nSphere] = sphere_pos_local_X;
    sphere_data->sphere_local_pos_Y[nSphere] = sphere_pos_local_Y;
    sphere_data->sphere_local_pos_Z[nSphere] = sphere_pos_local_Z;

    // if (SDID >= gran_params->nSDs) {
    //     printf("ERROR! Sphere %u has invalid SD %u, max is %u, triplet %d, %d, %d\n", nSphere, SDID,
    //                     gran_params->nSDs, ownerSD.x, ownerSD.y, ownerSD.z);
    // }

    // write back which SD currently owns this sphere
    sphere_data->sphere_owner_SDs[nSphere] = SDID;
}

float ChSystemGpu_impl::ComputeTotalKE() {
    size_t nSpheres = pos_X_dt.size();
    if (nSpheres == 0)
        return 0.f;

    // Compute sum(v^2) and sum(w^2)
    float v2_UU = computeArray3SquaredSum(pos_X_dt, pos_Y_dt, pos_Z_dt, nSpheres);
    v2_UU *= VEL_SU2UU * VEL_SU2UU;
    float w2_UU = computeArray3SquaredSum(sphere_Omega_X, sphere_Omega_Y, sphere_Omega_Z, nSpheres);
    w2_UU /= TIME_SU2UU * TIME_SU2UU;
    float m = (4. / 3.) * CH_C_PI * sphere_radius_UU * sphere_radius_UU * sphere_radius_UU * sphere_density_UU;

    // Then, KE = 0.5 * m * sum(v^2) + 0.2 * m * r^2 * sum(w^2)
    return 0.5 * m * v2_UU + 0.2 * m * sphere_radius_UU * sphere_radius_UU * w2_UU;
}

// return absolute velocity
float ChSystemGpu_impl::getAbsVelocity(int nSphere) {
    float absv_SU = std::sqrt(pos_X_dt[nSphere] * pos_X_dt[nSphere] + pos_Y_dt[nSphere] * pos_Y_dt[nSphere] +
                              pos_Z_dt[nSphere] * pos_Z_dt[nSphere]);
    float absv_UU = (float)(absv_SU * LENGTH_SU2UU / TIME_SU2UU);
    return absv_UU;
}

// return velocity
float3 ChSystemGpu_impl::GetParticleLinVelocity(int nSphere) const {
    float vx_UU = (float)(pos_X_dt[nSphere] * LENGTH_SU2UU / TIME_SU2UU);
    float vy_UU = (float)(pos_Y_dt[nSphere] * LENGTH_SU2UU / TIME_SU2UU);
    float vz_UU = (float)(pos_Z_dt[nSphere] * LENGTH_SU2UU / TIME_SU2UU);
    return make_float3(vx_UU, vy_UU, vz_UU);
}

// get angular velocity of a particle
float3 ChSystemGpu_impl::GetParticleAngVelocity(int nSphere) const {
    float wx_UU = sphere_Omega_X.at(nSphere) / TIME_SU2UU;
    float wy_UU = sphere_Omega_Y.at(nSphere) / TIME_SU2UU;
    float wz_UU = sphere_Omega_Z.at(nSphere) / TIME_SU2UU;
    return make_float3(wx_UU, wy_UU, wz_UU);
}

// get particle acceleration
float3 ChSystemGpu_impl::GetParticleLinAcc(int nSphere) const {
    double acc_x = sphere_acc_X.at(nSphere) * LENGTH_SU2UU / (TIME_SU2UU * TIME_SU2UU);
    double acc_y = sphere_acc_Y.at(nSphere) * LENGTH_SU2UU / (TIME_SU2UU * TIME_SU2UU);
    double acc_z = sphere_acc_Z.at(nSphere) * LENGTH_SU2UU / (TIME_SU2UU * TIME_SU2UU);

    return make_float3(acc_x, acc_y, acc_z);
}

// whether or not the particle is fixed
bool ChSystemGpu_impl::IsFixed(int nSphere) const {
    return sphere_fixed[nSphere];
}

// Return number of particle-particle contacts
int ChSystemGpu_impl::GetNumContacts() const {
    auto contact_itr = contact_partners_map.begin();
    int total_nc = 0;

    while (contact_itr != contact_partners_map.end()) {
        int body_j = *contact_itr;
        contact_itr++;

        if (body_j != -1) {
            total_nc++;
        }
    }

    return total_nc / 2;
}

// Partitions the big domain (BD) and sets the number of SDs that BD is split in.
void ChSystemGpu_impl::partitionBD() {
    // Create BC walls and potential trajactory function
    CreateWallBCs();
    setBDWallsMotionFunction(GranPosFunction_default);

    double sd_length_scale = 2.0 * sphere_radius_UU * AVERAGE_SPHERES_PER_SD_X_DIR;

    unsigned int nSDs_X = (unsigned int)(std::ceil(box_size_X / sd_length_scale));
    // work with an even kFac to hit the CM of the box.
    if (nSDs_X & 1)
        nSDs_X++;
    int SD_size_X = (unsigned int)std::ceil(box_size_X / (nSDs_X * LENGTH_SU2UU));

    unsigned int nSDs_Y = (unsigned int)(std::ceil(box_size_Y / sd_length_scale));
    // work with an even kFac to hit the CM of the box.
    if (nSDs_Y & 1)
        nSDs_Y++;
    int SD_size_Y = (unsigned int)std::ceil(box_size_Y / (nSDs_Y * LENGTH_SU2UU));

    unsigned int nSDs_Z = (unsigned int)(std::ceil(box_size_Z / sd_length_scale));
    // work with an even kFac to hit the CM of the box.
    if (nSDs_Z & 1)
        nSDs_Z++;
    int SD_size_Z = (unsigned int)std::ceil(box_size_Z / (nSDs_Z * LENGTH_SU2UU));

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
    gran_params->BD_frame_X = (int64_t)(-0.5 * ((int64_t)nSDs_X * SD_size_X) + (double)user_coord_O_X / LENGTH_SU2UU);
    gran_params->BD_frame_Y = (int64_t)(-0.5 * ((int64_t)nSDs_Y * SD_size_Y) + (double)user_coord_O_Y / LENGTH_SU2UU);
    gran_params->BD_frame_Z = (int64_t)(-0.5 * ((int64_t)nSDs_Z * SD_size_Z) + (double)user_coord_O_Z / LENGTH_SU2UU);

    // permanently cache the initial frame
    BD_rest_frame_SU = make_longlong3(gran_params->BD_frame_X, gran_params->BD_frame_Y, gran_params->BD_frame_Z);

    INFO_PRINTF("%u Sds as %u, %u, %u\n", gran_params->nSDs, gran_params->nSDs_X, gran_params->nSDs_Y,
                gran_params->nSDs_Z);

    // allocate mem for array saying for each SD how many spheres touch it
    TRACK_VECTOR_RESIZE(SD_NumSpheresTouching, nSDs, "SD_numSpheresTouching", 0);
    TRACK_VECTOR_RESIZE(SD_SphereCompositeOffsets, nSDs, "SD_SphereCompositeOffsets", 0);
    TRACK_VECTOR_RESIZE(SD_SphereCompositeOffsets_ScratchPad, nSDs, "SD_SphereCompositeOffsets_ScratchPad", 0);
}

void ChSystemGpu_impl::combineMaterialSurface() {
    // effective youngs modulus and shear modulus in user units
    double E_eff_s2s_uu, E_eff_s2w_uu, G_eff_s2s_uu, G_eff_s2w_uu;

    materialPropertyCombine(YoungsModulus_sphere_UU, YoungsModulus_sphere_UU, PoissonRatio_sphere_UU,
                            PoissonRatio_sphere_UU, E_eff_s2s_uu, G_eff_s2s_uu);

    materialPropertyCombine(YoungsModulus_sphere_UU, YoungsModulus_wall_UU, PoissonRatio_sphere_UU,
                            PoissonRatio_wall_UU, E_eff_s2w_uu, G_eff_s2w_uu);

    // unit conversion of youngs modulus
    double E_SU2UU = this->MASS_SU2UU / (this->LENGTH_SU2UU * this->TIME_SU2UU * this->TIME_SU2UU);

    // convert effective youngs modulus and shear modulus to simulation units
    gran_params->E_eff_s2s_SU = (float)E_eff_s2s_uu / E_SU2UU;
    gran_params->E_eff_s2w_SU = (float)E_eff_s2w_uu / E_SU2UU;
    gran_params->G_eff_s2s_SU = (float)G_eff_s2s_uu / E_SU2UU;
    gran_params->G_eff_s2w_SU = (float)G_eff_s2w_uu / E_SU2UU;

    // assign coefficient of restitution in simulation units
    gran_params->COR_s2s_SU = COR_sphere_UU;
    gran_params->COR_s2w_SU = std::min(COR_sphere_UU, COR_wall_UU);
}

// Convert unit parameters from UU to SU
void ChSystemGpu_impl::switchToSimUnits() {
    // Compute sphere mass, highest system stiffness, and gravity magnitude
    double massSphere =
        (4. / 3.) * CH_C_PI * sphere_radius_UU * sphere_radius_UU * sphere_radius_UU * sphere_density_UU;
    double K_star = get_max_K();

    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);

    // These two are independent of hooke/hertz
    this->MASS_SU2UU = massSphere / gran_params->sphere_mass_SU;
    this->TIME_SU2UU = sqrt(massSphere / K_star) / psi_T;
    // copy this to gran_params for device to use
    gran_params->TIME_UNIT = this->TIME_SU2UU;
    gran_params->MASS_UNIT = this->MASS_SU2UU;

    // old hooke way
    // LENGTH_SU2UU = massSphere * magGravAcc / (psi_L * K_star);
    // new hertz way
    this->LENGTH_SU2UU =
        std::pow(massSphere * massSphere * magGravAcc * magGravAcc * sphere_radius_UU / (K_star * K_star), 1. / 3.) /
        psi_L;
    this->LENGTH_SU2UU = std::min((double)(sphere_radius_UU * psi_R), this->LENGTH_SU2UU);

    // If we can get better precision by just dividing the box as fine as possible
    // using single ints, do that. Conservative by a factor of 2.
    if (this->use_min_length_unit) {
        this->LENGTH_SU2UU =
            std::min((double)std::max(box_size_X, std::max(box_size_Y, box_size_Z)) / std::numeric_limits<int>::max(),
                     this->LENGTH_SU2UU);
    }

    stepSize_SU = (float)(stepSize_UU / TIME_SU2UU);
    gran_params->stepSize_SU = stepSize_SU;

    // compute temporary composite quantities
    double ACC_SU2UU = LENGTH_SU2UU / (TIME_SU2UU * TIME_SU2UU);
    double K_SU2UU = MASS_SU2UU / (TIME_SU2UU * TIME_SU2UU);
    double GAMMA_SU2UU = 1. / TIME_SU2UU;

    // compute quantities to store back in class
    this->VEL_SU2UU = LENGTH_SU2UU / TIME_SU2UU;
    this->FORCE_SU2UU = MASS_SU2UU * ACC_SU2UU;
    this->TORQUE_SU2UU = FORCE_SU2UU * LENGTH_SU2UU;
    // copy into gran params for now
    gran_params->LENGTH_UNIT = LENGTH_SU2UU;
    gran_params->sphereRadius_SU = (unsigned int)(sphere_radius_UU / LENGTH_SU2UU);

    gran_params->gravAcc_X_SU = (float)(X_accGrav / ACC_SU2UU);
    gran_params->gravAcc_Y_SU = (float)(Y_accGrav / ACC_SU2UU);
    gran_params->gravAcc_Z_SU = (float)(Z_accGrav / ACC_SU2UU);

    gran_params->cohesionAcc_s2s = (float)(magGravAcc * cohesion_over_gravity / ACC_SU2UU);
    gran_params->adhesionAcc_s2w = (float)(magGravAcc * adhesion_s2w_over_gravity / ACC_SU2UU);

    // LULUTODO: INITIALIZATION DEPENDS ON WHETHER OR NOT MATERIAL PROPERTY IS USED
    /// SU values for normal stiffnesses for S2S and S2W
    gran_params->K_n_s2s_SU = (float)(K_n_s2s_UU / K_SU2UU);
    gran_params->K_n_s2w_SU = (float)(K_n_s2w_UU / K_SU2UU);

    gran_params->K_t_s2s_SU = (float)(K_t_s2s_UU / K_SU2UU);
    gran_params->K_t_s2w_SU = (float)(K_t_s2w_UU / K_SU2UU);

    gran_params->Gamma_n_s2s_SU = (float)(Gamma_n_s2s_UU / GAMMA_SU2UU);
    gran_params->Gamma_n_s2w_SU = (float)(Gamma_n_s2w_UU / GAMMA_SU2UU);
    gran_params->Gamma_t_s2s_SU = (float)(Gamma_t_s2s_UU / GAMMA_SU2UU);
    gran_params->Gamma_t_s2w_SU = (float)(Gamma_t_s2w_UU / GAMMA_SU2UU);

    // SU values for youngs modulus and coefficient of restitution
    combineMaterialSurface();

    gran_params->rolling_coeff_s2s_SU = (float)rolling_coeff_s2s_UU;
    gran_params->rolling_coeff_s2w_SU = (float)rolling_coeff_s2w_UU;

    // Handy debug output
    INFO_PRINTF("UU mass is %f\n", MASS_SU2UU);
    INFO_PRINTF("SU gravity is %f, %f, %f\n", gran_params->gravAcc_X_SU, gran_params->gravAcc_Y_SU,
                gran_params->gravAcc_Z_SU);
    INFO_PRINTF("SU radius is %u\n", gran_params->sphereRadius_SU);
    float dt_safe_estimate = (float)std::sqrt(massSphere / K_n_s2s_UU);
    INFO_PRINTF("CFL timestep is about %f\n", dt_safe_estimate);
    INFO_PRINTF("Length unit is %0.16f\n", gran_params->LENGTH_UNIT);
}
}  // namespace gpu
}  // namespace chrono
