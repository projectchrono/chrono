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

#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include "chrono/physics/ChGlobal.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "ChGranularTriMesh.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

namespace chrono {
namespace granular {

double ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::get_max_K() {
    return std::max(std::max(YoungModulus_SPH2SPH, YoungModulus_SPH2WALL), YoungModulus_SPH2MESH);
}

ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(
    float radiusSPH,
    float density)
    : ChSystemGranularMonodisperse_SMC_Frictionless(radiusSPH, density),
      problemSetupFinished(false),
      timeToWhichDEsHaveBeenPropagated(0.f) {}

ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh() {
    // work to do here
    cleanupTriMesh_DEVICE();
}

void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::load_meshes(std::vector<std::string> objfilenames,
                                                                        std::vector<float3> scalings) {
    if (objfilenames.size() != scalings.size()) {
        GRANULAR_ERROR("Vectors of obj files and scalings must have same size\n");
    }

    unsigned int nTriangles = 0;
    unsigned int nFamiliesInSoup = 0;
    std::vector<geometry::ChTriangleMeshConnected> all_meshes;
    for (unsigned int i = 0; i < objfilenames.size(); i++) {
        printf("importing %s\n", objfilenames[i].c_str());
        all_meshes.push_back(geometry::ChTriangleMeshConnected());
        geometry::ChTriangleMeshConnected& mesh = all_meshes[all_meshes.size() - 1];

        mesh.LoadWavefrontMesh(GetChronoDataFile(objfilenames[i]), true, false);
        mesh.Transform({0, 0, 0}, ChMatrix33<>(ChVector<>(scalings[i].x, scalings[i].y, scalings[i].z)));

        nTriangles += mesh.getNumTriangles();
        nFamiliesInSoup++;
    }

    printf("nTriangles is %u\n", nTriangles);
    printf("nTriangleFamiliesInSoup is %u\n", nFamiliesInSoup);

    // Allocate triangle collision parameters
    gpuErrchk(cudaMallocManaged(&tri_params, sizeof(GranParamsHolder_trimesh), cudaMemAttachGlobal));

    // Allocate memory to store mesh soup in unified memory
    printf("Allocating mesh unified memory\n");
    setupTriMesh_DEVICE(all_meshes, nTriangles);
    printf("Done allocating mesh unified memory\n");

    // Allocate triangle collision memory
    BUCKET_countsOfTrianglesTouching.resize(TRIANGLEBUCKET_COUNT);
    triangles_in_BUCKET_composite.resize(TRIANGLEBUCKET_COUNT * MAX_TRIANGLE_COUNT_PER_BUCKET);
}

template <class T>
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::ApplyFrameTransform(ChVector<T>& p, T* pos, T* rot_mat) {
    // Apply roation matrix to point
    p[0] = rot_mat[0] * p[0] + rot_mat[1] * p[1] + rot_mat[2] * p[2];
    p[1] = rot_mat[3] * p[0] + rot_mat[4] * p[1] + rot_mat[5] * p[2];
    p[2] = rot_mat[6] * p[0] + rot_mat[7] * p[1] + rot_mat[8] * p[2];

    // Apply translation
    p[0] += pos[0];
    p[1] += pos[1];
    p[2] += pos[2];
}

void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::write_meshes(std::string filename) {
    printf("Writing meshes\n");
    std::ofstream outfile(filename + "_mesh.vtk", std::ios::out);
    std::ostringstream ostream;
    ostream << "# vtk DataFile Version 1.0\n";
    ostream << "Unstructured Grid Example\n";
    ostream << "ASCII\n";
    ostream << "\n\n";

    ostream << "DATASET UNSTRUCTURED_GRID\n";
    ostream << "POINTS " << meshSoup_DEVICE->nTrianglesInSoup * 3 << " float\n";

    // Write all vertices
    for (unsigned int tri_i = 0; tri_i < meshSoup_DEVICE->nTrianglesInSoup; tri_i++) {
        ChVector<float> p1(meshSoup_DEVICE->node1_X[tri_i], meshSoup_DEVICE->node1_Y[tri_i],
                           meshSoup_DEVICE->node1_Z[tri_i]);
        ChVector<float> p2(meshSoup_DEVICE->node2_X[tri_i], meshSoup_DEVICE->node2_Y[tri_i],
                           meshSoup_DEVICE->node2_Z[tri_i]);
        ChVector<float> p3(meshSoup_DEVICE->node3_X[tri_i], meshSoup_DEVICE->node3_Y[tri_i],
                           meshSoup_DEVICE->node3_Z[tri_i]);

        ApplyFrameTransform<float>(p1, tri_params->fam_frame_broad->pos, tri_params->fam_frame_broad->rot_mat);
        ApplyFrameTransform<float>(p2, tri_params->fam_frame_broad->pos, tri_params->fam_frame_broad->rot_mat);
        ApplyFrameTransform<float>(p3, tri_params->fam_frame_broad->pos, tri_params->fam_frame_broad->rot_mat);

        ostream << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
        ostream << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
        ostream << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
    }

    ostream << "\n\n";
    ostream << "CELLS " << meshSoup_DEVICE->nTrianglesInSoup << " " << 4 * meshSoup_DEVICE->nTrianglesInSoup << "\n";
    for (unsigned int tri_i = 0; tri_i < meshSoup_DEVICE->nTrianglesInSoup; tri_i++) {
        ostream << "3 " << 3 * tri_i << " " << 3 * tri_i + 1 << " " << 3 * tri_i + 2 << "\n";
    }

    ostream << "\n\n";
    ostream << "CELL_TYPES " << meshSoup_DEVICE->nTrianglesInSoup << "\n";
    for (unsigned int tri_i = 0; tri_i < meshSoup_DEVICE->nTrianglesInSoup; tri_i++) {
        ostream << "9\n";
    }

    outfile << ostream.str();
}

void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::cleanupTriMesh_DEVICE() {
    cudaFree(meshSoup_DEVICE->triangleFamily_ID);

    cudaFree(meshSoup_DEVICE->node1_X);
    cudaFree(meshSoup_DEVICE->node1_Y);
    cudaFree(meshSoup_DEVICE->node1_Z);

    cudaFree(meshSoup_DEVICE->node2_X);
    cudaFree(meshSoup_DEVICE->node2_Y);
    cudaFree(meshSoup_DEVICE->node2_Z);

    cudaFree(meshSoup_DEVICE->node3_X);
    cudaFree(meshSoup_DEVICE->node3_Y);
    cudaFree(meshSoup_DEVICE->node3_Z);

    cudaFree(meshSoup_DEVICE->node1_XDOT);
    cudaFree(meshSoup_DEVICE->node1_YDOT);
    cudaFree(meshSoup_DEVICE->node1_ZDOT);

    cudaFree(meshSoup_DEVICE->node2_XDOT);
    cudaFree(meshSoup_DEVICE->node2_YDOT);
    cudaFree(meshSoup_DEVICE->node2_ZDOT);

    cudaFree(meshSoup_DEVICE->node3_XDOT);
    cudaFree(meshSoup_DEVICE->node3_YDOT);
    cudaFree(meshSoup_DEVICE->node3_ZDOT);

    cudaFree(meshSoup_DEVICE->generalizedForcesPerFamily);
}

void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::setupTriMesh_DEVICE(
    const std::vector<geometry::ChTriangleMeshConnected>& all_meshes,
    unsigned int nTriangles) {
    // Allocate the device soup storage
    gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE, sizeof(ChTriangleSoup<float>), cudaMemAttachGlobal));

    meshSoup_DEVICE->nTrianglesInSoup = nTriangles;

    if (nTriangles != 0) {
        // Allocate all of the requisite pointers
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->triangleFamily_ID, nTriangles * sizeof(unsigned int),
                                    cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node1_X, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node1_Y, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node1_Z, nTriangles * sizeof(float), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node2_X, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node2_Y, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node2_Z, nTriangles * sizeof(float), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node3_X, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node3_Y, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node3_Z, nTriangles * sizeof(float), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node1_XDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node1_YDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node1_ZDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node2_XDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node2_YDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node2_ZDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node3_XDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node3_YDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&meshSoup_DEVICE->node3_ZDOT, nTriangles * sizeof(float), cudaMemAttachGlobal));
    }
    printf("Done allocating nodes for %d triangles\n", nTriangles);

    // Setup the clean copy of the mesh soup from the obj file data
    unsigned int family = 0;
    unsigned int tri_i = 0;
    // for each obj file data set
    for (auto mesh : all_meshes) {
        int n_triangles_mesh = mesh.getNumTriangles();
        for (int i = 0; i < n_triangles_mesh; i++) {
            geometry::ChTriangle tri = mesh.getTriangle(i);

            meshSoup_DEVICE->node1_X[tri_i] = tri.p1.x();
            meshSoup_DEVICE->node1_Y[tri_i] = tri.p1.y();
            meshSoup_DEVICE->node1_Z[tri_i] = tri.p1.z();

            meshSoup_DEVICE->node2_X[tri_i] = tri.p2.x();
            meshSoup_DEVICE->node2_Y[tri_i] = tri.p2.y();
            meshSoup_DEVICE->node2_Z[tri_i] = tri.p2.z();

            meshSoup_DEVICE->node3_X[tri_i] = tri.p3.x();
            meshSoup_DEVICE->node3_Y[tri_i] = tri.p3.y();
            meshSoup_DEVICE->node3_Z[tri_i] = tri.p3.z();

            meshSoup_DEVICE->triangleFamily_ID[tri_i] = family;

            // TODO: test
            // Normal of a single vertex... Should still work
            int normal_i = mesh.m_face_n_indices[i].x();  // normals at each vertex of this triangle
            ChVector<double> normal = mesh.m_normals[normal_i];

            // Generate normal using RHR from nodes 1, 2, and 3
            ChVector<double> AB = tri.p2 - tri.p1;
            ChVector<double> AC = tri.p3 - tri.p1;
            ChVector<double> cross;
            cross.Cross(AB, AC);

            // If the normal created by a RHR traversal is not correct, switch two vertices
            if (cross.Dot(normal) < 0) {
                std::swap(meshSoup_DEVICE->node2_X[tri_i], meshSoup_DEVICE->node3_X[tri_i]);
                std::swap(meshSoup_DEVICE->node2_Y[tri_i], meshSoup_DEVICE->node3_Y[tri_i]);
                std::swap(meshSoup_DEVICE->node2_Z[tri_i], meshSoup_DEVICE->node3_Z[tri_i]);
            }
            tri_i++;
        }
        family++;
        printf("Done writing family %d\n", family);
    }

    meshSoup_DEVICE->nFamiliesInSoup = family;

    if (meshSoup_DEVICE->nTrianglesInSoup != 0) {
        // Allocate memory for the float and double frames
        gpuErrchk(cudaMallocManaged(&tri_params->fam_frame_broad,
                                    meshSoup_DEVICE->nFamiliesInSoup * sizeof(ChFamilyFrame<float>),
                                    cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&tri_params->fam_frame_narrow,
                                    meshSoup_DEVICE->nFamiliesInSoup * sizeof(ChFamilyFrame<double>),
                                    cudaMemAttachGlobal));
    }
}

/**
* \brief Collects the forces that each mesh feels as a result of their interaction with the DEs.
*
* The generalized forces felt by each family in the triangle soup are copied from the device into the host array
that is
* provided to this function. Each generalized force acting on a family in the soup has six components: three forces
&
* three torques.
*
* The forces measured are based on the position of the DEs as they are at the beginning of this function call,
before
* the DEs are moved forward in time. In other words, the forces are associated with the configuration of the DE
system
* from time timeToWhichDEsHaveBeenPropagated upon entrying this function.
* The logic is this: when the time integration is carried out, the forces are measured and saved in
* meshSoup_DEVICE->nFamiliesInSoup. Then, the numerical integration takes places and the state of DEs is update
along
* with the value of timeToWhichDEsHaveBeenPropagated.
*
* \param [in] crntTime The time at which the force is computed
* \param [out] genForcesOnSoup Array that stores the generalized forces on the meshes in the soup

* \return nothing
*
* \attention The size of genForcesOnSoup should be 6 * nFamiliesInSoup
*/
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::collectGeneralizedForcesOnMeshSoup(float crntTime,
                                                                                               float* genForcesOnSoup) {
    if (!problemSetupFinished) {
        setupSimulation();
        problemSetupFinished = true;
    }

    // update the time that the DE system has reached
    timeToWhichDEsHaveBeenPropagated = crntTime;

    // Values in meshSoup_DEVICE are legit and ready to be loaded in user provided array.
    gpuErrchk(cudaMemcpy(genForcesOnSoup, meshSoup_DEVICE->generalizedForcesPerFamily,
                         6 * meshSoup_DEVICE->nFamiliesInSoup * sizeof(float), cudaMemcpyDeviceToHost));
}

/**
 * \brief Function sets up data structures, allocates space on the device, generates the spheres, etc.
 *
 */
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::setupSimulation() {
    switch_to_SimUnits();
    generate_DEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copy_const_data_to_device();
    gpuErrchk(cudaDeviceSynchronize());

    // Seed arrays that are populated by the kernel call
    // Set all the offsets to zero
    gpuErrchk(cudaMemset(SD_NumOf_DEs_Touching.data(), 0, nSDs * sizeof(unsigned int)));
    // For each SD, all the spheres touching that SD should have their ID be NULL_GRANULAR_ID
    gpuErrchk(cudaMemset(DEs_in_SD_composite.data(), NULL_GRANULAR_ID,
                         MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));

    // Figure our the number of blocks that need to be launched to cover the box
    printf("doing priming!\n");
    printf("max possible composite offset is %zu\n", (size_t)nSDs * MAX_COUNT_OF_DEs_PER_SD);
}

void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::meshSoup_applyRigidBodyMotion(
    double* position_orientation_data) {
    // Set both broadphase and narrowphase frames for each family
    for (unsigned int fam = 0; fam < meshSoup_DEVICE->nFamiliesInSoup; fam++) {
        generate_rot_matrix<float>(position_orientation_data + 7 * fam + 3, tri_params->fam_frame_broad[fam].rot_mat);
        tri_params->fam_frame_broad[fam].pos[0] = (float)position_orientation_data[7 * fam + 0];
        tri_params->fam_frame_broad[fam].pos[1] = (float)position_orientation_data[7 * fam + 1];
        tri_params->fam_frame_broad[fam].pos[2] = (float)position_orientation_data[7 * fam + 2];

        generate_rot_matrix<double>(position_orientation_data + 7 * fam + 3, tri_params->fam_frame_narrow[fam].rot_mat);
        tri_params->fam_frame_narrow[fam].pos[0] = position_orientation_data[7 * fam + 0];
        tri_params->fam_frame_narrow[fam].pos[1] = position_orientation_data[7 * fam + 1];
        tri_params->fam_frame_narrow[fam].pos[2] = position_orientation_data[7 * fam + 2];
    }
}

template <typename T>
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::generate_rot_matrix(double* ep, T* rot_mat) {
    rot_mat[0] = ep[0] * ep[0] + ep[1] * ep[1] - ep[2] * ep[2] - ep[3] * ep[3];
    rot_mat[1] = 2 * (ep[1] * ep[2] + ep[0] * ep[3]);
    rot_mat[2] = 2 * (ep[1] * ep[3] - ep[0] * ep[2]);

    rot_mat[3] = 2 * (ep[1] * ep[2] - ep[0] * ep[3]);
    rot_mat[4] = ep[0] * ep[0] - ep[1] * ep[1] + ep[2] * ep[2] - ep[3] * ep[3];
    rot_mat[5] = 2 * (ep[2] * ep[3] + ep[0] * ep[1]);

    rot_mat[6] = 2 * (ep[1] * ep[3] + ep[0] * ep[2]);
    rot_mat[7] = 2 * (ep[2] * ep[3] - ep[0] * ep[1]);
    rot_mat[8] = ep[0] * ep[0] - ep[1] * ep[1] - ep[2] * ep[2] + ep[3] * ep[3];
}

}  // namespace granular
}  // namespace chrono