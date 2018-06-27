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

#include "ChGranularTriMesh.h"
#include <vector>

/**
This method defines the mass, time, length Simulation Units. It also sets several other constants that enter the scaling
of various physical quantities set by the user.
*/
// void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::switch_to_SimUnits() {
//    double massSphere = 4. / 3. * M_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density;
//    MASS_UNIT = massSphere;
//    double K_stiffness = (YoungModulus_SPH2SPH > YoungModulus_SPH2WALL ? YoungModulus_SPH2SPH :
//    YoungModulus_SPH2WALL); if (K_stiffness < YoungModulus_SPH2MESH)
//        K_stiffness = YoungModulus_SPH2MESH;
//
//    TIME_UNIT = sqrt(massSphere / (PSI_h * K_stiffness)) / PSI_T;
//
//    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);
//    LENGTH_UNIT = massSphere * magGravAcc / (PSI_L * K_stiffness);
//
//    sphereRadius_SU = sphere_radius / LENGTH_UNIT;
//
//    float scalingFactor = ((float)PSI_L) / (PSI_T * PSI_T * PSI_h);
//    gravity_X_SU = scalingFactor * X_accGrav / magGravAcc;
//    gravity_Y_SU = scalingFactor * Y_accGrav / magGravAcc;
//    gravity_Z_SU = scalingFactor * Z_accGrav / magGravAcc;
//
//    /// SU values for normal stiffnesses for s2s (sphere to sphere), s2w (sphere2wall), and s2m (sphere2mesh)
//    scalingFactor = (1.f / (1.f * PSI_T * PSI_T * PSI_h));
//    K_n_s2s_SU = scalingFactor * (YoungModulus_SPH2SPH / K_stiffness);
//    K_n_s2w_SU = scalingFactor * (YoungModulus_SPH2WALL / K_stiffness);
//    K_n_s2m_SU = scalingFactor * (YoungModulus_SPH2MESH / K_stiffness);
//
//    // TODO Make this legit, from user input
//    Gamma_n_SU = .005;
//
//    // Handy debug output
//    printf("SU gravity is %f, %f, %f\n", gravity_X_SU, gravity_Y_SU, gravity_Z_SU);
//    printf("SU mass is %f\n", MASS_UNIT);
//    printf("SU radius is %u\n", sphereRadius_SU);
//}

chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(float radiusSPH, float density, const char* meshFileName)
    : granMat(radiusSPH, density) {
    setupSoup_HOST_DEVICE(meshFileName);
}

chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::
    ~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh() {
    // work to do here
    CleanupSoup_DEVICE();
    CleanupSoup_HOST();
;
}

/**
Given a file name, this function reads in from that file a mesh soup. This mesh soup is used to allocate memory on the
HOST and DEVICE. Finally, the HOST mesh soup is initialize with mesh soup that is read in.
*/
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::setupSoup_HOST_DEVICE(
    const char* mesh_filename) {
    std::vector<tinyobj::shape_t> shapes;

    /// read in the mesh soup; stored in an obj file
    tinyobj::LoadObj(shapes, mesh_filename);

    unsigned int nTriangles = 0;
    for (auto shape : shapes)
        nTriangles += shape.mesh.indices.size() / 3;

    /// Get the soup ready on the HOST side
    SetupSoup_HOST(shapes, nTriangles);
    SetupSoup_DEVICE(nTriangles);
}


/**
On the HOST sice, allocate memory to hang on to the mesh soup. The HOST mesh soup is initialized with values provided in
the input obj file.
*/
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::SetupSoup_HOST(
    const std::vector<tinyobj::shape_t>& shapes,
    unsigned int nTriangles) {
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
                GRANULAR_ERROR("Input mesh has inside-out elements.")
                // std::swap(original_soup.node2_X[tri_index], original_soup.node3_X[tri_index]);
                // std::swap(original_soup.node2_Y[tri_index], original_soup.node3_Y[tri_index]);
                // std::swap(original_soup.node2_Z[tri_index], original_soup.node3_Z[tri_index]);
            }
        }

        /// Set up the HOST mesh soup
        meshSoup_HOST.nTrianglesInSoup = nTriangles;

        meshSoup_HOST.triangleFamily_ID = new unsigned int[nTriangles];

        meshSoup_HOST.node1_X = new int[nTriangles];
        meshSoup_HOST.node1_Y = new int[nTriangles];
        meshSoup_HOST.node1_Z = new int[nTriangles];

        meshSoup_HOST.node2_X = new int[nTriangles];
        meshSoup_HOST.node2_Y = new int[nTriangles];
        meshSoup_HOST.node2_Z = new int[nTriangles];

        meshSoup_HOST.node3_X = new int[nTriangles];
        meshSoup_HOST.node3_Y = new int[nTriangles];
        meshSoup_HOST.node3_Z = new int[nTriangles];

        meshSoup_HOST.node1_XDOT = new float[nTriangles];
        meshSoup_HOST.node1_YDOT = new float[nTriangles];
        meshSoup_HOST.node1_ZDOT = new float[nTriangles];

        meshSoup_HOST.node2_XDOT = new float[nTriangles];
        meshSoup_HOST.node2_YDOT = new float[nTriangles];
        meshSoup_HOST.node2_ZDOT = new float[nTriangles];

        meshSoup_HOST.node3_XDOT = new float[nTriangles];
        meshSoup_HOST.node3_YDOT = new float[nTriangles];
        meshSoup_HOST.node3_ZDOT = new float[nTriangles];
    }
}

void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::CleanupSoup_HOST() {
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
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::update_DMeshSoup_Location() {
    cudaMemcpy(meshSoup_DEVICE.node1_X, meshSoup_HOST.node1_X, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node1_Y, meshSoup_HOST.node1_Y, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node1_Z, meshSoup_HOST.node1_Z, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);

    cudaMemcpy(meshSoup_DEVICE.node2_X, meshSoup_HOST.node2_X, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node2_Y, meshSoup_HOST.node2_Y, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node2_Z, meshSoup_HOST.node2_Z, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);

    cudaMemcpy(meshSoup_DEVICE.node3_X, meshSoup_HOST.node3_X, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node3_Y, meshSoup_HOST.node3_Y, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(meshSoup_DEVICE.node3_Z, meshSoup_HOST.node3_Z, meshSoup_DEVICE.nTrianglesInSoup * sizeof(int), cudaMemcpyHostToDevice);
}

/**
The generalized forces felt by each family in the triangle soup are copied from the device into the host array that is provided to this function.
Each generalized force acting on a family in the soup has six components: three forces & three torques.
*/
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::collectGeneralizedForcesOnMeshSoup(float crntTime, float* genForcesOnSoup) {
    ;
}
