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
// Authors: Dan Negrut
// =============================================================================
/*! \file */

#include <vector>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "ChGranularTriMesh.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

namespace chrono {
namespace granular {

double ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::get_max_K() {
    return std::max(std::max(YoungModulus_SPH2SPH, YoungModulus_SPH2WALL), YoungModulus_SPH2MESH);
}

ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(
    float radiusSPH,
    float density,
    std::string meshFileName)
    : ChSystemGranularMonodisperse_SMC_Frictionless(radiusSPH, density),
      problemSetupFinished(false),
      timeToWhichDEsHaveBeenPropagated(0.f) {
    setupSoup_HOST_DEVICE(meshFileName.c_str());
}

ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh() {
    // work to do here
    cleanupSoup_DEVICE();
    cleanupSoup_HOST();
}

/** \brief Method reads in a mesh soup; indirectly allocates memory on HOST and DEVICE to store mesh soup.
 *
 * \param mesh_filename Contains the name of the file that stores the information about the mesh soup
 * \return nothing
 *
 * Given a file name, this function reads in from that file a mesh soup. This mesh soup is used to allocate memory on
 * the HOST and DEVICE. Finally, the HOST mesh soup is initialize with the mesh soup that is read in.
 *
 * \attention The mesh soup, provided in the input file, should be in obj format.
 */
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::setupSoup_HOST_DEVICE(const char* mesh_filename) {
    std::vector<tinyobj::shape_t> shapes;

    // The mesh soup stored in an obj file
    tinyobj::LoadObj(shapes, mesh_filename);

    unsigned int nTriangles = 0;
    for (auto shape : shapes)
        nTriangles += shape.mesh.indices.size() / 3;

    // Allocate memory to store mesh soup; done both on the HOST and DEVICE sides
    setupSoup_HOST(shapes, nTriangles);
    setupSoup_DEVICE(nTriangles);
}

/**
On the HOST sice, allocate memory to hang on to the mesh soup. The HOST mesh soup is initialized with values provided in
the input obj file.
*/
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::setupSoup_HOST(const std::vector<tinyobj::shape_t>& shapes,
                                                                           unsigned int nTriangles) {
    /// Set up the clean HOST mesh soup
    meshSoup_HOST.nTrianglesInSoup = nTriangles;

    meshSoup_HOST.triangleFamily_ID = new unsigned int[nTriangles];

    meshSoup_HOST.node1_X = new float[nTriangles];
    meshSoup_HOST.node1_Y = new float[nTriangles];
    meshSoup_HOST.node1_Z = new float[nTriangles];

    meshSoup_HOST.node2_X = new float[nTriangles];
    meshSoup_HOST.node2_Y = new float[nTriangles];
    meshSoup_HOST.node2_Z = new float[nTriangles];

    meshSoup_HOST.node3_X = new float[nTriangles];
    meshSoup_HOST.node3_Y = new float[nTriangles];
    meshSoup_HOST.node3_Z = new float[nTriangles];

    meshSoup_HOST.node1_XDOT = new float[nTriangles];
    meshSoup_HOST.node1_YDOT = new float[nTriangles];
    meshSoup_HOST.node1_ZDOT = new float[nTriangles];

    meshSoup_HOST.node2_XDOT = new float[nTriangles];
    meshSoup_HOST.node2_YDOT = new float[nTriangles];
    meshSoup_HOST.node2_ZDOT = new float[nTriangles];

    meshSoup_HOST.node3_XDOT = new float[nTriangles];
    meshSoup_HOST.node3_YDOT = new float[nTriangles];
    meshSoup_HOST.node3_ZDOT = new float[nTriangles];

    /// Set up the working HOST mesh soup
    meshSoupWorking_HOST.nTrianglesInSoup = nTriangles;

    meshSoupWorking_HOST.triangleFamily_ID = new unsigned int[nTriangles];

    meshSoupWorking_HOST.node1_X = new float[nTriangles];
    meshSoupWorking_HOST.node1_Y = new float[nTriangles];
    meshSoupWorking_HOST.node1_Z = new float[nTriangles];

    meshSoupWorking_HOST.node2_X = new float[nTriangles];
    meshSoupWorking_HOST.node2_Y = new float[nTriangles];
    meshSoupWorking_HOST.node2_Z = new float[nTriangles];

    meshSoupWorking_HOST.node3_X = new float[nTriangles];
    meshSoupWorking_HOST.node3_Y = new float[nTriangles];
    meshSoupWorking_HOST.node3_Z = new float[nTriangles];

    meshSoupWorking_HOST.node1_XDOT = new float[nTriangles];
    meshSoupWorking_HOST.node1_YDOT = new float[nTriangles];
    meshSoupWorking_HOST.node1_ZDOT = new float[nTriangles];

    meshSoupWorking_HOST.node2_XDOT = new float[nTriangles];
    meshSoupWorking_HOST.node2_YDOT = new float[nTriangles];
    meshSoupWorking_HOST.node2_ZDOT = new float[nTriangles];

    meshSoupWorking_HOST.node3_XDOT = new float[nTriangles];
    meshSoupWorking_HOST.node3_YDOT = new float[nTriangles];
    meshSoupWorking_HOST.node3_ZDOT = new float[nTriangles];

    // Set up mesh from the input file
    size_t tri_index = 0;
    for (auto shape : shapes) {
        std::vector<unsigned int>& indices = shape.mesh.indices;
        std::vector<float>& positions = shape.mesh.positions;
        std::vector<float>& normals = shape.mesh.normals;

        // Grab three indices which indicate the vertices of a triangle
        for (size_t i = 0; i < indices.size(); i += 9, tri_index++) {
            meshSoup_HOST.node1_X[tri_index] = positions[indices[i + 0]];
            meshSoup_HOST.node1_Y[tri_index] = positions[indices[i + 1]];
            meshSoup_HOST.node1_Z[tri_index] = positions[indices[i + 2]];

            meshSoup_HOST.node2_X[tri_index] = positions[indices[i + 3]];
            meshSoup_HOST.node2_Y[tri_index] = positions[indices[i + 4]];
            meshSoup_HOST.node2_Z[tri_index] = positions[indices[i + 5]];

            meshSoup_HOST.node3_X[tri_index] = positions[indices[i + 6]];
            meshSoup_HOST.node3_Y[tri_index] = positions[indices[i + 7]];
            meshSoup_HOST.node3_Z[tri_index] = positions[indices[i + 8]];

            // Normal of a vertex... Should still work
            float norm_vert[3] = {0};
            norm_vert[0] = normals[indices[i + 0]];
            norm_vert[1] = normals[indices[i + 1]];
            norm_vert[2] = normals[indices[i + 2]];

            // Generate normal using RHR from nodes 1, 2, and 3
            float AB[3];
            AB[0] = positions[indices[i + 3]] - positions[indices[i + 0]];
            AB[1] = positions[indices[i + 4]] - positions[indices[i + 1]];
            AB[2] = positions[indices[i + 5]] - positions[indices[i + 2]];

            float AC[3];
            AC[0] = positions[indices[i + 6]] - positions[indices[i + 0]];
            AC[1] = positions[indices[i + 7]] - positions[indices[i + 1]];
            AC[2] = positions[indices[i + 8]] - positions[indices[i + 2]];

            float cross[3];
            cross[0] = AB[1] * AC[2] - AB[2] * AC[1];
            cross[1] = -(AB[0] * AC[2] - AB[2] * AC[0]);
            cross[2] = AB[0] * AC[1] - AB[1] * AC[0];

            // If the normal created by a RHR traversal is not correct, switch two vertices
            if (norm_vert[0] * cross[0] + norm_vert[1] * cross[1] + norm_vert[2] * cross[2] < 0) {
                // GRANULAR_ERROR("Input mesh has inside-out elements.")
                std::swap(meshSoup_HOST.node2_X[tri_index], meshSoup_HOST.node3_X[tri_index]);
                std::swap(meshSoup_HOST.node2_Y[tri_index], meshSoup_HOST.node3_Y[tri_index]);
                std::swap(meshSoup_HOST.node2_Z[tri_index], meshSoup_HOST.node3_Z[tri_index]);
            }
        }
    }
}

void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::cleanupSoup_HOST() {
    delete[] meshSoup_HOST.triangleFamily_ID;

    delete[] meshSoup_HOST.node1_X;
    delete[] meshSoup_HOST.node1_Y;
    delete[] meshSoup_HOST.node1_Z;

    delete[] meshSoup_HOST.node2_X;
    delete[] meshSoup_HOST.node2_Y;
    delete[] meshSoup_HOST.node2_Z;

    delete[] meshSoup_HOST.node3_X;
    delete[] meshSoup_HOST.node3_Y;
    delete[] meshSoup_HOST.node3_Z;

    delete[] meshSoup_HOST.node1_XDOT;
    delete[] meshSoup_HOST.node1_YDOT;
    delete[] meshSoup_HOST.node1_ZDOT;

    delete[] meshSoup_HOST.node2_XDOT;
    delete[] meshSoup_HOST.node2_YDOT;
    delete[] meshSoup_HOST.node2_ZDOT;

    delete[] meshSoup_HOST.node3_XDOT;
    delete[] meshSoup_HOST.node3_YDOT;
    delete[] meshSoup_HOST.node3_ZDOT;

    delete[] meshSoupWorking_HOST.triangleFamily_ID;

    delete[] meshSoupWorking_HOST.node1_X;
    delete[] meshSoupWorking_HOST.node1_Y;
    delete[] meshSoupWorking_HOST.node1_Z;

    delete[] meshSoupWorking_HOST.node2_X;
    delete[] meshSoupWorking_HOST.node2_Y;
    delete[] meshSoupWorking_HOST.node2_Z;

    delete[] meshSoupWorking_HOST.node3_X;
    delete[] meshSoupWorking_HOST.node3_Y;
    delete[] meshSoupWorking_HOST.node3_Z;

    delete[] meshSoupWorking_HOST.node1_XDOT;
    delete[] meshSoupWorking_HOST.node1_YDOT;
    delete[] meshSoupWorking_HOST.node1_ZDOT;

    delete[] meshSoupWorking_HOST.node2_XDOT;
    delete[] meshSoupWorking_HOST.node2_YDOT;
    delete[] meshSoupWorking_HOST.node2_ZDOT;

    delete[] meshSoupWorking_HOST.node3_XDOT;
    delete[] meshSoupWorking_HOST.node3_YDOT;
    delete[] meshSoupWorking_HOST.node3_ZDOT;
}

// void LoadSoup(ChTriangleSoup& original_soup,
//    ChTriangleSoup& tri_soup,
//    unsigned int nTriangles,
//    double x,
//    double y,
//    double z) {
//    // Offset mesh by the position
//    for (unsigned int i = 0; i < nTriangles; i++) {
//        tri_soup.node1_X[i] = original_soup.node1_X[i] + x;
//        tri_soup.node2_X[i] = original_soup.node2_X[i] + x;
//        tri_soup.node3_X[i] = original_soup.node3_X[i] + x;
//
//        tri_soup.node1_Y[i] = original_soup.node1_Y[i] + y;
//        tri_soup.node2_Y[i] = original_soup.node2_Y[i] + y;
//        tri_soup.node3_Y[i] = original_soup.node3_Y[i] + y;
//
//        tri_soup.node1_Z[i] = original_soup.node1_Z[i] + z;
//        tri_soup.node2_Z[i] = original_soup.node2_Z[i] + z;
//        tri_soup.node3_Z[i] = original_soup.node3_Z[i] + z;
//    }
//}

/** A new location of the triangles in the mesh soup is provided upon input. Upon output, a set of generalized forces
 * that the material impresses on each family of the mesh soup is computed.
 */
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::update_DMeshSoup_Location() {
    cudaMemcpy(meshSoup_DEVICE.node1_X, meshSoupWorking_HOST.node1_X, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node1_Y, meshSoupWorking_HOST.node1_Y, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node1_Z, meshSoupWorking_HOST.node1_Z, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);

    cudaMemcpy(meshSoup_DEVICE.node2_X, meshSoupWorking_HOST.node2_X, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node2_Y, meshSoupWorking_HOST.node2_Y, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node2_Z, meshSoupWorking_HOST.node2_Z, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);

    cudaMemcpy(meshSoup_DEVICE.node3_X, meshSoupWorking_HOST.node3_X, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node3_Y, meshSoupWorking_HOST.node3_Y, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node3_Z, meshSoupWorking_HOST.node3_Z, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int),
               cudaMemcpyHostToDevice);
}

/**
* \brief Collects the forces that each mesh feels as a result of their interaction with the DEs.
*
* The generalized forces felt by each family in the triangle soup are copied from the device into the host array that is
* provided to this function. Each generalized force acting on a family in the soup has six components: three forces &
* three torques.
*
* The forces measured are based on the position of the DEs as they are at the beginning of this function call, before
* the DEs are moved forward in time. In other words, the forces are associated with the configuration of the DE system
* from time timeToWhichDEsHaveBeenPropagated upon entrying this function.
* The logic is this: when the time integration is carried out, the forces are measured and saved in
* meshSoup_DEVICE.nFamiliesInSoup. Then, the numerical integration takes places and the state of DEs is update along
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
    gpuErrchk(cudaMemcpy(genForcesOnSoup, meshSoup_DEVICE.generalizedForcesPerFamily,
                         6 * meshSoup_DEVICE.nFamiliesInSoup * sizeof(float), cudaMemcpyDeviceToHost));
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
    copy_triangle_data_to_device();
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

/** \brief Applies a translation and rotation to each point of each soup family. Works for rigid meshes only.
 *
 * The location of each vertex of each triangle is stored in double precision. This function does two things:
 * - gets the location and the orientation of the each mesh's RF
 * - figures out the location of each vertex of the mesh soup
 *
 * \param [in] position_orientation_data
 * \return nothing
 *
 * This is the function that computes the location of each vertex in each mesh. To that end, it converts double
 * precision data to int. The int values are stored in the meshSoup_DEVICE, which is an input for the GPU kernel.
 *
 * \attention The number of entries in the array is 7 * nFamiliesInSoup.
 * \attention First three entries are the location of the mesh reference frame wrt global ref frame.
 * \attention The next four entries provide the orientation of the mesh wrt global ref frame (Euler params).
 */

/**
 * meshSoup_HOST holds a clean mesh soup in the global reference frame
 * Outputs transformed mesh soup to meshSoupWorking_HOST
 * The number of entries in the array is 7 * nFamiliesInSoup
 * First three entries are the location of the mesh reference frame wrt global ref frame.
 * The next four entries provide the orientation of the mesh wrt global ref frame (Euler params).
 */
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::meshSoup_applyRigidBodyMotion(
    float crntTime,
    double* position_orientation_data) {
    // Assemble displacements and rotations for each family transform
    std::vector<ChVector<>> pos;
    std::vector<ChQuaternion<>> rot;
    for (size_t fam = 0; fam < meshSoup_HOST.nFamiliesInSoup; fam++) {
        size_t fam_i = 7 * fam;
        pos.push_back(ChVector<>(position_orientation_data[fam_i], position_orientation_data[fam_i + 1],
                                 position_orientation_data[fam_i + 2]));
        rot.push_back(ChQuaternion<>(position_orientation_data[fam_i + 3], position_orientation_data[fam_i + 4],
                                     position_orientation_data[fam_i + 5], position_orientation_data[fam_i + 6]));
    }

    for (size_t i = 0; i < meshSoup_HOST.nTrianglesInSoup; i++) {
        // Assemble nodes in chrono structures
        unsigned int fam = meshSoup_HOST.triangleFamily_ID[i];
        ChVector<> node1(meshSoup_HOST.node1_X[i], meshSoup_HOST.node1_Y[i], meshSoup_HOST.node1_Z[i]);
        ChVector<> node2(meshSoup_HOST.node2_X[i], meshSoup_HOST.node2_Y[i], meshSoup_HOST.node2_Z[i]);
        ChVector<> node3(meshSoup_HOST.node3_X[i], meshSoup_HOST.node3_Y[i], meshSoup_HOST.node3_Z[i]);

        // Apply the appropriate transform for this family
        node1 = rot[fam].Rotate(node1) + pos[fam];  // TODO units for pos need to be in SU
        node2 = rot[fam].Rotate(node2) + pos[fam];  // TODO units for pos need to be in SU
        node3 = rot[fam].Rotate(node3) + pos[fam];  // TODO units for pos need to be in SU

        // Store the transformed mesh in the working host copy
        meshSoupWorking_HOST.node1_X[i] = node1.x();
        meshSoupWorking_HOST.node1_Y[i] = node1.y();
        meshSoupWorking_HOST.node1_Z[i] = node1.z();

        meshSoupWorking_HOST.node2_X[i] = node2.x();
        meshSoupWorking_HOST.node2_Y[i] = node2.y();
        meshSoupWorking_HOST.node2_Z[i] = node2.z();

        meshSoupWorking_HOST.node3_X[i] = node3.x();
        meshSoupWorking_HOST.node3_Y[i] = node3.y();
        meshSoupWorking_HOST.node3_Z[i] = node3.z();
    }
}
}  // namespace granular
}  // namespace chrono
