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
// Authors: Nic Olsen, Dan Negrut
// =============================================================================

#include <string>
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"

ChGranularChronoTriMeshAPI::ChGranularChronoTriMeshAPI(float sphere_rad, float density, float3 boxDims) {
    pGranSystemSMC_TriMesh = new chrono::granular::ChSystemGranularSMC_trimesh(sphere_rad, density, boxDims);
}

void ChGranularChronoTriMeshAPI::load_meshes(std::vector<std::string> objfilenames,
                                             std::vector<chrono::ChMatrix33<float>> rotscale,
                                             std::vector<float3> translations,
                                             std::vector<float> masses) {
    unsigned int size = (unsigned int)objfilenames.size();
    if (size != rotscale.size() || size != translations.size() || size != masses.size()) {
        GRANULAR_ERROR("Mesh loading vectors must all have same size\n");
    }

    if (size == 0) {
        printf("WARNING: No meshes provided!\n");
    }

    unsigned int nTriangles = 0;
    unsigned int numTriangleFamilies = 0;
    std::vector<chrono::geometry::ChTriangleMeshConnected> all_meshes;
    for (unsigned int i = 0; i < objfilenames.size(); i++) {
        // INFO_PRINTF("Importing %s...\n", objfilenames[i].c_str()); <--- work on this later
        all_meshes.push_back(chrono::geometry::ChTriangleMeshConnected());
        chrono::geometry::ChTriangleMeshConnected& mesh = all_meshes[all_meshes.size() - 1];

        bool readin_flag = mesh.LoadWavefrontMesh(objfilenames[i], true, false);
        if (!readin_flag) {
            GRANULAR_ERROR("ERROR! Mesh %s failed to load in! Exiting!\n", objfilenames[i].c_str());
        }
        
        // Apply displacement
        chrono::ChVector<> displ(translations[i].x, translations[i].y, translations[i].z);

        // Apply scaling and then rotation
        mesh.Transform(displ, rotscale[i].cast<double>());

        unsigned int num_triangles_curr = mesh.getNumTriangles();

        if (num_triangles_curr == 0) {
            printf("WARNING: Mesh %s has no triangles in it!\n", objfilenames[i].c_str());
        }

        nTriangles += num_triangles_curr;
        numTriangleFamilies++;
    }

    // work on this later: INFO_PRINTF("nTriangles is %u\n", nTriangles);
    // work on this later: INFO_PRINTF("nTriangleFamiliesInSoup is %u\n", numTriangleFamilies);

    // Allocate memory to store mesh soup in unified memory
    // work on this later: INFO_PRINTF("Allocating mesh unified memory\n");
    setupTriMesh(all_meshes, nTriangles, masses);
    // work on this later: INFO_PRINTF("Done allocating mesh unified memory\n");
}

void ChGranularChronoTriMeshAPI::setupTriMesh(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                                              unsigned int nTriangles,
                                              std::vector<float> masses) {
    chrono::granular::ChTriangleSoup<float3>* pMeshSoup = pGranSystemSMC_TriMesh->getMeshSoup();
    pMeshSoup->nTrianglesInSoup = nTriangles;

    if (nTriangles != 0) {
        // Allocate all of the requisite pointers
        gpuErrchk(
            cudaMallocManaged(&pMeshSoup->triangleFamily_ID, nTriangles * sizeof(unsigned int), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&pMeshSoup->node1, nTriangles * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&pMeshSoup->node2, nTriangles * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&pMeshSoup->node3, nTriangles * sizeof(float3), cudaMemAttachGlobal));
    }

    MESH_INFO_PRINTF("Done allocating nodes for %d triangles\n", nTriangles);

    // Setup the clean copy of the mesh soup from the obj file data
    unsigned int family = 0;
    unsigned int tri_i = 0;
    // for each obj file data set
    for (auto mesh : all_meshes) {
        int n_triangles_mesh = mesh.getNumTriangles();
        for (int i = 0; i < n_triangles_mesh; i++) {
            chrono::geometry::ChTriangle tri = mesh.getTriangle(i);

            pMeshSoup->node1[tri_i] = make_float3((float)tri.p1.x(), (float)tri.p1.y(), (float)tri.p1.z());
            pMeshSoup->node2[tri_i] = make_float3((float)tri.p2.x(), (float)tri.p2.y(), (float)tri.p2.z());
            pMeshSoup->node3[tri_i] = make_float3((float)tri.p3.x(), (float)tri.p3.y(), (float)tri.p3.z());

            pMeshSoup->triangleFamily_ID[tri_i] = family;

            // Normal of a single vertex... Should still work
            int normal_i = mesh.m_face_n_indices.at(i).x();  // normals at each vertex of this triangle
            chrono::ChVector<double> normal = mesh.m_normals[normal_i];

            // Generate normal using RHR from nodes 1, 2, and 3
            chrono::ChVector<double> AB = tri.p2 - tri.p1;
            chrono::ChVector<double> AC = tri.p3 - tri.p1;
            chrono::ChVector<double> cross;
            cross.Cross(AB, AC);

            // If the normal created by a RHR traversal is not correct, switch two vertices
            if (cross.Dot(normal) < 0) {
                std::swap(pMeshSoup->node2[tri_i], pMeshSoup->node3[tri_i]);
            }
            tri_i++;
        }
        family++;
        MESH_INFO_PRINTF("Done writing family %d\n", family);
    }

    pMeshSoup->numTriangleFamilies = family;

    if (pMeshSoup->nTrianglesInSoup != 0) {
        gpuErrchk(cudaMallocManaged(&pMeshSoup->familyMass_SU, family * sizeof(float), cudaMemAttachGlobal));

        for (unsigned int i = 0; i < family; i++) {
            // NOTE The SU conversion is done in initialize after the scaling is determined
            pMeshSoup->familyMass_SU[i] = masses[i];
        }

        gpuErrchk(cudaMallocManaged(&pMeshSoup->generalizedForcesPerFamily,
                                    6 * pMeshSoup->numTriangleFamilies * sizeof(float), cudaMemAttachGlobal));
        // Allocate memory for the float and double frames
        gpuErrchk(
            cudaMallocManaged(&pGranSystemSMC_TriMesh->getTriParams()->fam_frame_broad,
                              pMeshSoup->numTriangleFamilies * sizeof(chrono::granular::ChGranMeshFamilyFrame<float>),
                              cudaMemAttachGlobal));
        gpuErrchk(
            cudaMallocManaged(&pGranSystemSMC_TriMesh->getTriParams()->fam_frame_narrow,
                              pMeshSoup->numTriangleFamilies * sizeof(chrono::granular::ChGranMeshFamilyFrame<double>),
                              cudaMemAttachGlobal));

        // Allocate memory for linear and angular velocity
        gpuErrchk(
            cudaMallocManaged(&pMeshSoup->vel, pMeshSoup->numTriangleFamilies * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(
            cudaMallocManaged(&pMeshSoup->omega, pMeshSoup->numTriangleFamilies * sizeof(float3), cudaMemAttachGlobal));

        for (unsigned int i = 0; i < family; i++) {
            pMeshSoup->vel[i] = make_float3(0, 0, 0);
            pMeshSoup->omega[i] = make_float3(0, 0, 0);
        }
    }
}

// initialize particle positions, velocity and angular velocity in user units
void ChGranularSMC_API::setElemsPositions(const std::vector<chrono::ChVector<float>>& points,
                                          const std::vector<chrono::ChVector<float>>& vels,
                                          const std::vector<chrono::ChVector<float>>& ang_vel) {
    std::vector<float3> pointsFloat3;
    std::vector<float3> velsFloat3;
    std::vector<float3> angVelsFloat3;
    convertChVector2Float3Vec(points, pointsFloat3);
    convertChVector2Float3Vec(vels, velsFloat3);
    convertChVector2Float3Vec(ang_vel, angVelsFloat3);
    gran_sys->setParticlePositions(pointsFloat3, velsFloat3, angVelsFloat3);
}

void ChGranularChronoTriMeshAPI::setElemsPositions(const std::vector<chrono::ChVector<float>>& points,
                                                   const std::vector<chrono::ChVector<float>>& vels,
                                                   const std::vector<chrono::ChVector<float>>& ang_vel) {
    std::vector<float3> pointsFloat3;
    std::vector<float3> velsFloat3;
    std::vector<float3> angVelsFloat3;
    convertChVector2Float3Vec(points, pointsFloat3);
    convertChVector2Float3Vec(vels, velsFloat3);
    convertChVector2Float3Vec(ang_vel, angVelsFloat3);
    pGranSystemSMC_TriMesh->setParticlePositions(pointsFloat3, velsFloat3, angVelsFloat3);
}

// return particle position
chrono::ChVector<float> ChGranularSMC_API::getPosition(int nSphere){
    float3 pos = gran_sys->getPosition(nSphere);
    chrono::ChVector<float> pos_vec(pos.x, pos.y, pos.z); 
    return pos_vec;
}

chrono::ChVector<float> ChGranularChronoTriMeshAPI::getPosition(int nSphere){
    float3 pos = pGranSystemSMC_TriMesh->getPosition(nSphere);
    chrono::ChVector<float> pos_vec(pos.x, pos.y, pos.z); 
    return pos_vec;
}

// return particle velocity
chrono::ChVector<float> ChGranularSMC_API::getVelo(int nSphere){
    float3 velo = gran_sys->getVelocity(nSphere);
    chrono::ChVector<float> velo_vec(velo.x, velo.y, velo.z); 
    return velo_vec;
}

chrono::ChVector<float> ChGranularChronoTriMeshAPI::getVelo(int nSphere){
    float3 velo = pGranSystemSMC_TriMesh->getVelocity(nSphere);
    chrono::ChVector<float> velo_vec(velo.x, velo.y, velo.z); 
    return velo_vec;
}

// return particle angular velocity
chrono::ChVector<float> ChGranularSMC_API::getAngularVelo(int nSphere){
    float3 omega = gran_sys->getAngularVelocity(nSphere);
    chrono::ChVector<float> omega_vec(omega.x, omega.y, omega.z); 
    return omega_vec;
}

chrono::ChVector<float> ChGranularChronoTriMeshAPI::getAngularVelo(int nSphere){
    float3 omega = pGranSystemSMC_TriMesh->getAngularVelocity(nSphere);
    chrono::ChVector<float> omega_vec(omega.x, omega.y, omega.z); 
    return omega_vec;
}

// return BC plane position
chrono::ChVector<float> ChGranularSMC_API::getBCPlanePos(size_t plane_id){
    //todo: throw an error if BC not a plane type
    float3 pos = gran_sys->Get_BC_Plane_Position(plane_id);
    return chrono::ChVector<float> (pos.x, pos.y, pos.z);
}

// return number of sphere-to-sphere contact
int ChGranularSMC_API::getNumContacts(){
    return gran_sys->getNumContacts();
}
