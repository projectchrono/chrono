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
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <cmath>
#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "ChGranularTriMesh.h"

using chrono::geometry::ChTriangleMeshConnected;

namespace chrono {
namespace granular {

ChSystemGranularSMC_trimesh::ChSystemGranularSMC_trimesh(float sphere_rad, float density, float3 boxDims)
    : ChSystemGranularSMC(sphere_rad, density, boxDims),
      K_n_s2m_UU(0),
      K_t_s2m_UU(0),
      Gamma_n_s2m_UU(0),
      Gamma_t_s2m_UU(0),
      rolling_coeff_s2m_UU(0),
      spinning_coeff_s2m_UU(0),
      adhesion_s2m_over_gravity(0) {
    // Allocate triangle collision parameters
    gpuErrchk(cudaMallocManaged(&tri_params, sizeof(ChGranParams_trimesh), cudaMemAttachGlobal));

    // Allocate the device soup storage
    gpuErrchk(cudaMallocManaged(&meshSoup, sizeof(ChTriangleSoup<float3>), cudaMemAttachGlobal));
    // start with no triangles
    meshSoup->nTrianglesInSoup = 0;
    meshSoup->numTriangleFamilies = 0;

    set_static_friction_coeff_SPH2MESH(0);
}

ChSystemGranularSMC_trimesh::~ChSystemGranularSMC_trimesh() {
    // work to do here
    cleanupTriMesh();
}
double ChSystemGranularSMC_trimesh::get_max_K() const {
    return std::max(std::max(K_n_s2s_UU, K_n_s2w_UU), K_n_s2m_UU);
}

void ChSystemGranularSMC_trimesh::initializeTriangles() {
    double K_SU2UU = MASS_SU2UU / (TIME_SU2UU * TIME_SU2UU);
    double GAMMA_SU2UU = 1. / TIME_SU2UU;

    tri_params->K_n_s2m_SU = (float)(K_n_s2m_UU / K_SU2UU);
    tri_params->K_t_s2m_SU = (float)(K_t_s2m_UU / K_SU2UU);

    tri_params->Gamma_n_s2m_SU = (float)(Gamma_n_s2m_UU / GAMMA_SU2UU);
    tri_params->Gamma_t_s2m_SU = (float)(Gamma_t_s2m_UU / GAMMA_SU2UU);

    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);
    tri_params->adhesionAcc_s2m = (float)(adhesion_s2m_over_gravity * magGravAcc);

    for (unsigned int fam = 0; fam < meshSoup->numTriangleFamilies; fam++) {
        meshSoup->familyMass_SU[fam] = (float)(meshSoup->familyMass_SU[fam] / MASS_SU2UU);
    }

    tri_params->rolling_coeff_s2m_SU = (float)rolling_coeff_s2m_UU;

    double* meshRot = new double[4];
    memset(meshRot, 0.0, sizeof(meshRot));
    meshRot[0] = 1.0;
    for (unsigned int fam = 0; fam < meshSoup->numTriangleFamilies; fam++) {
        generate_rot_matrix<float>(meshRot, tri_params->fam_frame_broad[fam].rot_mat);
        tri_params->fam_frame_broad[fam].pos[0] = (float)0.0;
        tri_params->fam_frame_broad[fam].pos[1] = (float)0.0;
        tri_params->fam_frame_broad[fam].pos[2] = (float)0.0;

        generate_rot_matrix<double>(meshRot, tri_params->fam_frame_narrow[fam].rot_mat);
        tri_params->fam_frame_narrow[fam].pos[0] = (double)0.0;
        tri_params->fam_frame_narrow[fam].pos[1] = (double)0.0;
        tri_params->fam_frame_narrow[fam].pos[2] = (double)0.0;
    }

    TRACK_VECTOR_RESIZE(SD_numTrianglesTouching, nSDs, "SD_numTrianglesTouching", 0);
    TRACK_VECTOR_RESIZE(SD_TriangleCompositeOffsets, nSDs, "SD_TriangleCompositeOffsets", 0);

    // TODO do we have a good heuristic???
    // this gets resized on-the-fly every timestep
    TRACK_VECTOR_RESIZE(triangles_in_SD_composite, 0, "triangles_in_SD_composite", 0);
}
void ChSystemGranularSMC_trimesh::initialize() {
    initializeSpheres();
    initializeTriangles();
}

// p = pos + rot_mat * p
void ChSystemGranularSMC_trimesh::ApplyFrameTransform(float3& p, float* pos, float* rot_mat) {
    float3 result;

    // Apply rotation matrix to point
    result.x = pos[0] + rot_mat[0] * p.x + rot_mat[1] * p.y + rot_mat[2] * p.z;
    result.y = pos[1] + rot_mat[3] * p.x + rot_mat[4] * p.y + rot_mat[5] * p.z;
    result.z = pos[2] + rot_mat[6] * p.x + rot_mat[7] * p.y + rot_mat[8] * p.z;

    // overwrite p only at the end
    p = result;
}

void ChSystemGranularSMC_trimesh::write_meshes(std::string filename) {
    if (file_write_mode == GRAN_OUTPUT_MODE::NONE) {
        return;
    }

    printf("Writing meshes\n");
    std::ofstream outfile(filename + "_mesh.vtk", std::ios::out);
    std::ostringstream ostream;
    ostream << "# vtk DataFile Version 1.0\n";
    ostream << "Unstructured Grid Example\n";
    ostream << "ASCII\n";
    ostream << "\n\n";

    ostream << "DATASET UNSTRUCTURED_GRID\n";
    ostream << "POINTS " << meshSoup->nTrianglesInSoup * 3 << " float\n";

    // Write all vertices
    for (unsigned int tri_i = 0; tri_i < meshSoup->nTrianglesInSoup; tri_i++) {
        float3 p1 = make_float3(meshSoup->node1[tri_i].x, meshSoup->node1[tri_i].y, meshSoup->node1[tri_i].z);
        float3 p2 = make_float3(meshSoup->node2[tri_i].x, meshSoup->node2[tri_i].y, meshSoup->node2[tri_i].z);
        float3 p3 = make_float3(meshSoup->node3[tri_i].x, meshSoup->node3[tri_i].y, meshSoup->node3[tri_i].z);

        unsigned int fam = meshSoup->triangleFamily_ID[tri_i];
        ApplyFrameTransform(p1, tri_params->fam_frame_broad[fam].pos, tri_params->fam_frame_broad[fam].rot_mat);
        ApplyFrameTransform(p2, tri_params->fam_frame_broad[fam].pos, tri_params->fam_frame_broad[fam].rot_mat);
        ApplyFrameTransform(p3, tri_params->fam_frame_broad[fam].pos, tri_params->fam_frame_broad[fam].rot_mat);

        ostream << p1.x << " " << p1.y << " " << p1.z << "\n";
        ostream << p2.x << " " << p2.y << " " << p2.z << "\n";
        ostream << p3.x << " " << p3.y << " " << p3.z << "\n";
    }

    ostream << "\n\n";
    ostream << "CELLS " << meshSoup->nTrianglesInSoup << " " << 4 * meshSoup->nTrianglesInSoup << "\n";
    for (unsigned int tri_i = 0; tri_i < meshSoup->nTrianglesInSoup; tri_i++) {
        ostream << "3 " << 3 * tri_i << " " << 3 * tri_i + 1 << " " << 3 * tri_i + 2 << "\n";
    }

    ostream << "\n\n";
    ostream << "CELL_TYPES " << meshSoup->nTrianglesInSoup << "\n";
    for (unsigned int tri_i = 0; tri_i < meshSoup->nTrianglesInSoup; tri_i++) {
        ostream << "9\n";
    }

    outfile << ostream.str();
}

void ChSystemGranularSMC_trimesh::cleanupTriMesh() {
    cudaFree(meshSoup->triangleFamily_ID);
    cudaFree(meshSoup->familyMass_SU);

    cudaFree(meshSoup->node1);
    cudaFree(meshSoup->node2);
    cudaFree(meshSoup->node3);

    cudaFree(meshSoup->vel);
    cudaFree(meshSoup->omega);

    cudaFree(meshSoup->generalizedForcesPerFamily);
    cudaFree(tri_params->fam_frame_broad);
    cudaFree(tri_params->fam_frame_narrow);
    cudaFree(meshSoup);
    cudaFree(tri_params);
}

void ChSystemGranularSMC_trimesh::collectGeneralizedForcesOnMeshSoup(float* genForcesOnSoup) {
    // pull directly from unified memory
    for (unsigned int i = 0; i < 6 * meshSoup->numTriangleFamilies; i += 6) {
        // Divide by C_F to go from SU to UU
        genForcesOnSoup[i + 0] = (float)(meshSoup->generalizedForcesPerFamily[i + 0] * FORCE_SU2UU);
        genForcesOnSoup[i + 1] = (float)(meshSoup->generalizedForcesPerFamily[i + 1] * FORCE_SU2UU);
        genForcesOnSoup[i + 2] = (float)(meshSoup->generalizedForcesPerFamily[i + 2] * FORCE_SU2UU);

        // Divide by C_TAU to go from SU to UU
        genForcesOnSoup[i + 3] = (float)(meshSoup->generalizedForcesPerFamily[i + 3] * TORQUE_SU2UU);
        genForcesOnSoup[i + 4] = (float)(meshSoup->generalizedForcesPerFamily[i + 4] * TORQUE_SU2UU);
        genForcesOnSoup[i + 5] = (float)(meshSoup->generalizedForcesPerFamily[i + 5] * TORQUE_SU2UU);
    }
}

void ChSystemGranularSMC_trimesh::meshSoup_applyRigidBodyMotion(double* position_orientation_data, float* vel) {
    // Set both broadphase and narrowphase frames for each family
    for (unsigned int fam = 0; fam < meshSoup->numTriangleFamilies; fam++) {
        generate_rot_matrix<float>(position_orientation_data + 7 * fam + 3, tri_params->fam_frame_broad[fam].rot_mat);
        tri_params->fam_frame_broad[fam].pos[0] = (float)position_orientation_data[7 * fam + 0];
        tri_params->fam_frame_broad[fam].pos[1] = (float)position_orientation_data[7 * fam + 1];
        tri_params->fam_frame_broad[fam].pos[2] = (float)position_orientation_data[7 * fam + 2];

        generate_rot_matrix<double>(position_orientation_data + 7 * fam + 3, tri_params->fam_frame_narrow[fam].rot_mat);
        tri_params->fam_frame_narrow[fam].pos[0] = position_orientation_data[7 * fam + 0];
        tri_params->fam_frame_narrow[fam].pos[1] = position_orientation_data[7 * fam + 1];
        tri_params->fam_frame_narrow[fam].pos[2] = position_orientation_data[7 * fam + 2];

        // Set linear and angular velocity
        const float C_V = (float)(TIME_SU2UU / LENGTH_SU2UU);
        meshSoup->vel[fam] = make_float3(C_V * vel[6 * fam + 0], C_V * vel[6 * fam + 1], C_V * vel[6 * fam + 2]);
        const float C_O = (float)TIME_SU2UU;
        meshSoup->omega[fam] = make_float3(C_O * vel[6 * fam + 3], C_O * vel[6 * fam + 4], C_O * vel[6 * fam + 5]);
    }
}

template <typename T>
void ChSystemGranularSMC_trimesh::generate_rot_matrix(double* ep, T* rot_mat) {
    rot_mat[0] = (T)(2 * (ep[0] * ep[0] + ep[1] * ep[1] - 0.5));
    rot_mat[1] = (T)(2 * (ep[1] * ep[2] - ep[0] * ep[3]));
    rot_mat[2] = (T)(2 * (ep[1] * ep[3] + ep[0] * ep[2]));
    rot_mat[3] = (T)(2 * (ep[1] * ep[2] + ep[0] * ep[3]));
    rot_mat[4] = (T)(2 * (ep[0] * ep[0] + ep[2] * ep[2] - 0.5));
    rot_mat[5] = (T)(2 * (ep[2] * ep[3] - ep[0] * ep[1]));
    rot_mat[6] = (T)(2 * (ep[1] * ep[3] - ep[0] * ep[2]));
    rot_mat[7] = (T)(2 * (ep[2] * ep[3] + ep[0] * ep[1]));
    rot_mat[8] = (T)(2 * (ep[0] * ep[0] + ep[3] * ep[3] - 0.5));
}

}  // namespace granular
}  // namespace chrono
