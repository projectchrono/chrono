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
#ifdef _WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif
#include "chrono/physics/ChGlobal.h"
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

    tri_params->K_n_s2m_SU = K_n_s2m_UU / K_SU2UU;
    tri_params->K_t_s2m_SU = K_t_s2m_UU / K_SU2UU;

    tri_params->Gamma_n_s2m_SU = Gamma_n_s2m_UU / GAMMA_SU2UU;
    tri_params->Gamma_t_s2m_SU = Gamma_t_s2m_UU / GAMMA_SU2UU;

    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);
    tri_params->adhesionAcc_s2m = adhesion_s2m_over_gravity * magGravAcc;

    for (unsigned int fam = 0; fam < meshSoup->numTriangleFamilies; fam++) {
        meshSoup->familyMass_SU[fam] = meshSoup->familyMass_SU[fam] / MASS_SU2UU;
        meshSoup->inflation_radii[fam] = meshSoup->inflation_radii[fam] / LENGTH_SU2UU;
    }

    double rolling_scalingFactor = 1.;
    if (gran_params->rolling_mode == GRAN_ROLLING_MODE::VISCOUS) {
        rolling_scalingFactor = 1. / TIME_SU2UU;
    }
    tri_params->rolling_coeff_s2m_SU = rolling_scalingFactor * rolling_coeff_s2m_UU;

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

void ChSystemGranularSMC_trimesh::load_meshes(std::vector<std::string> objfilenames,
                                              std::vector<ChMatrix33<float>> rotscale,
                                              std::vector<float3> translations,
                                              std::vector<float> masses,
                                              std::vector<bool> inflated,
                                              std::vector<float> inflation_radii) {
    unsigned int size = objfilenames.size();
    if (size != rotscale.size() || size != translations.size() || size != masses.size() || size != inflated.size() ||
        size != inflation_radii.size()) {
        GRANULAR_ERROR("Mesh loading vectors must all have same size\n");
    }

    if (size == 0) {
        printf("WARNING: No meshes provided!\n");
    }

    unsigned int nTriangles = 0;
    unsigned int numTriangleFamilies = 0;
    std::vector<ChTriangleMeshConnected> all_meshes;
    for (unsigned int i = 0; i < objfilenames.size(); i++) {
        INFO_PRINTF("Importing %s...\n", objfilenames[i].c_str());
        all_meshes.push_back(ChTriangleMeshConnected());
        ChTriangleMeshConnected& mesh = all_meshes[all_meshes.size() - 1];

        mesh.LoadWavefrontMesh(objfilenames[i], true, false);

        // Apply displacement
        ChVector<> displ(translations[i].x, translations[i].y, translations[i].z);

        // Apply scaling and then rotation
        mesh.Transform(displ, rotscale[i]);

        unsigned int num_triangles_curr = mesh.getNumTriangles();

        if (num_triangles_curr == 0) {
            GRANULAR_ERROR("ERROR! Mesh %s has no triangles in it! Exiting!\n", objfilenames[i].c_str());
        }

        nTriangles += num_triangles_curr;
        numTriangleFamilies++;
    }

    INFO_PRINTF("nTriangles is %u\n", nTriangles);
    INFO_PRINTF("nTriangleFamiliesInSoup is %u\n", numTriangleFamilies);

    // Allocate memory to store mesh soup in unified memory
    INFO_PRINTF("Allocating mesh unified memory\n");
    setupTriMesh(all_meshes, nTriangles, masses, inflated, inflation_radii);
    INFO_PRINTF("Done allocating mesh unified memory\n");
}

// result = rot_mat * p + pos
template <class T>
ChVector<T> ChSystemGranularSMC_trimesh::ApplyFrameTransform(ChVector<T>& p, T* pos, T* rot_mat) {
    ChVector<T> result;

    // Apply rotation matrix to point
    result[0] = rot_mat[0] * p[0] + rot_mat[1] * p[1] + rot_mat[2] * p[2];
    result[1] = rot_mat[3] * p[0] + rot_mat[4] * p[1] + rot_mat[5] * p[2];
    result[2] = rot_mat[6] * p[0] + rot_mat[7] * p[1] + rot_mat[8] * p[2];

    // Apply translation
    result[0] += pos[0];
    result[1] += pos[1];
    result[2] += pos[2];

    return result;
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
        ChVector<float> p1(meshSoup->node1[tri_i].x, meshSoup->node1[tri_i].y, meshSoup->node1[tri_i].z);
        ChVector<float> p2(meshSoup->node2[tri_i].x, meshSoup->node2[tri_i].y, meshSoup->node2[tri_i].z);
        ChVector<float> p3(meshSoup->node3[tri_i].x, meshSoup->node3[tri_i].y, meshSoup->node3[tri_i].z);

        unsigned int fam = meshSoup->triangleFamily_ID[tri_i];
        p1 = ApplyFrameTransform<float>(p1, tri_params->fam_frame_broad[fam].pos,
                                        tri_params->fam_frame_broad[fam].rot_mat);
        p2 = ApplyFrameTransform<float>(p2, tri_params->fam_frame_broad[fam].pos,
                                        tri_params->fam_frame_broad[fam].rot_mat);
        p3 = ApplyFrameTransform<float>(p3, tri_params->fam_frame_broad[fam].pos,
                                        tri_params->fam_frame_broad[fam].rot_mat);

        ostream << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
        ostream << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
        ostream << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
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
    cudaFree(meshSoup->inflated);
    cudaFree(meshSoup->inflation_radii);

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

void ChSystemGranularSMC_trimesh::setupTriMesh(const std::vector<ChTriangleMeshConnected>& all_meshes,
                                               unsigned int nTriangles,
                                               std::vector<float> masses,
                                               std::vector<bool> inflated,
                                               std::vector<float> inflation_radii) {
    meshSoup->nTrianglesInSoup = nTriangles;

    if (nTriangles != 0) {
        // Allocate all of the requisite pointers
        gpuErrchk(
            cudaMallocManaged(&meshSoup->triangleFamily_ID, nTriangles * sizeof(unsigned int), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup->node1, nTriangles * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup->node2, nTriangles * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup->node3, nTriangles * sizeof(float3), cudaMemAttachGlobal));
    }

    INFO_PRINTF("Done allocating nodes for %d triangles\n", nTriangles);

    // Setup the clean copy of the mesh soup from the obj file data
    unsigned int family = 0;
    unsigned int tri_i = 0;
    // for each obj file data set
    for (auto mesh : all_meshes) {
        int n_triangles_mesh = mesh.getNumTriangles();
        for (int i = 0; i < n_triangles_mesh; i++) {
            geometry::ChTriangle tri = mesh.getTriangle(i);

            meshSoup->node1[tri_i] = make_float3(tri.p1.x(), tri.p1.y(), tri.p1.z());
            meshSoup->node2[tri_i] = make_float3(tri.p2.x(), tri.p2.y(), tri.p2.z());
            meshSoup->node3[tri_i] = make_float3(tri.p3.x(), tri.p3.y(), tri.p3.z());

            meshSoup->triangleFamily_ID[tri_i] = family;

            // Normal of a single vertex... Should still work
            int normal_i = mesh.m_face_n_indices.at(i).x();  // normals at each vertex of this triangle
            ChVector<double> normal = mesh.m_normals[normal_i];

            // Generate normal using RHR from nodes 1, 2, and 3
            ChVector<double> AB = tri.p2 - tri.p1;
            ChVector<double> AC = tri.p3 - tri.p1;
            ChVector<double> cross;
            cross.Cross(AB, AC);

            // If the normal created by a RHR traversal is not correct, switch two vertices
            if (cross.Dot(normal) < 0) {
                std::swap(meshSoup->node2[tri_i], meshSoup->node3[tri_i]);
            }
            tri_i++;
        }
        family++;
        INFO_PRINTF("Done writing family %d\n", family);
    }

    meshSoup->numTriangleFamilies = family;

    if (meshSoup->nTrianglesInSoup != 0) {
        gpuErrchk(cudaMallocManaged(&meshSoup->familyMass_SU, family * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup->inflated, family * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup->inflation_radii, family * sizeof(float), cudaMemAttachGlobal));

        for (unsigned int i = 0; i < family; i++) {
            // NOTE The SU conversion is done in initialize after the scaling is determined
            meshSoup->familyMass_SU[i] = masses[i];
            meshSoup->inflated[i] = inflated[i];
            meshSoup->inflation_radii[i] = inflation_radii[i];
        }

        gpuErrchk(cudaMallocManaged(&meshSoup->generalizedForcesPerFamily,
                                    6 * meshSoup->numTriangleFamilies * sizeof(float), cudaMemAttachGlobal));
        // Allocate memory for the float and double frames
        gpuErrchk(cudaMallocManaged(&tri_params->fam_frame_broad,
                                    meshSoup->numTriangleFamilies * sizeof(ChGranMeshFamilyFrame<float>),
                                    cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&tri_params->fam_frame_narrow,
                                    meshSoup->numTriangleFamilies * sizeof(ChGranMeshFamilyFrame<double>),
                                    cudaMemAttachGlobal));

        // Allocate memory for linear and angular velocity
        gpuErrchk(
            cudaMallocManaged(&meshSoup->vel, meshSoup->numTriangleFamilies * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(
            cudaMallocManaged(&meshSoup->omega, meshSoup->numTriangleFamilies * sizeof(float3), cudaMemAttachGlobal));

        for (unsigned int i = 0; i < family; i++) {
            meshSoup->vel[i] = make_float3(0, 0, 0);
            meshSoup->omega[i] = make_float3(0, 0, 0);
        }
    }
}

void ChSystemGranularSMC_trimesh::collectGeneralizedForcesOnMeshSoup(float* genForcesOnSoup) {
    // pull directly from unified memory
    for (unsigned int i = 0; i < 6 * meshSoup->numTriangleFamilies; i += 6) {
        // Divide by C_F to go from SU to UU
        genForcesOnSoup[i + 0] = meshSoup->generalizedForcesPerFamily[i + 0] * FORCE_SU2UU;
        genForcesOnSoup[i + 1] = meshSoup->generalizedForcesPerFamily[i + 1] * FORCE_SU2UU;
        genForcesOnSoup[i + 2] = meshSoup->generalizedForcesPerFamily[i + 2] * FORCE_SU2UU;

        // Divide by C_TAU to go from SU to UU
        genForcesOnSoup[i + 3] = meshSoup->generalizedForcesPerFamily[i + 3] * TORQUE_SU2UU;
        genForcesOnSoup[i + 4] = meshSoup->generalizedForcesPerFamily[i + 4] * TORQUE_SU2UU;
        genForcesOnSoup[i + 5] = meshSoup->generalizedForcesPerFamily[i + 5] * TORQUE_SU2UU;
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
        const float C_V = TIME_SU2UU / LENGTH_SU2UU;
        meshSoup->vel[fam] = make_float3(C_V * vel[6 * fam + 0], C_V * vel[6 * fam + 1], C_V * vel[6 * fam + 2]);
        const float C_O = TIME_SU2UU;
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