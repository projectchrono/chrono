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
// Authors: Nic Olsen, Dan Negrut, Radu Serban
// =============================================================================

#include <string>

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuUtilities.h"

namespace chrono {
namespace gpu {

#define MESH_INFO_PRINTF(...)                                         \
    if (mesh_verbosity == ChSystemGpuMesh::MeshVerbosity::INFO) { \
        printf(__VA_ARGS__);                                          \
    }

static void convertChVector2Float3Vec(const std::vector<ChVector<float>>& points,
                                      std::vector<float3>& pointsFloat3) {
    size_t nPoints = points.size();
    pointsFloat3.resize(nPoints);
    for (size_t index = 0; index < nPoints; index++) {
        pointsFloat3.at(index).x = points.at(index)[0];
        pointsFloat3.at(index).y = points.at(index)[1];
        pointsFloat3.at(index).z = points.at(index)[2];
    }
}

// -----------------------------------------------------------------------------

ChSystemGpuMesh::ChSystemGpuMesh(float sphere_rad, float density, float3 boxDims)
    : mesh_verbosity(MeshVerbosity::QUIET) {
    m_sys_trimesh = new ChSystemGpuMesh_impl(sphere_rad, density, boxDims);
}

ChSystemGpuMesh::~ChSystemGpuMesh() {
    delete m_sys_trimesh;
}

ChSystemGpu::ChSystemGpu(float sphere_rad, float density, float3 boxDims) {
    m_sys = new ChSystemGpu_impl(sphere_rad, density, boxDims);
}

ChSystemGpu::~ChSystemGpu() {
    delete m_sys;
}

// -----------------------------------------------------------------------------

void ChSystemGpuMesh::LoadMeshes(std::vector<std::string> objfilenames,
                                      std::vector<ChMatrix33<float>> rotscale,
                                      std::vector<float3> translations,
                                      std::vector<float> masses) {
    unsigned int size = (unsigned int)objfilenames.size();
    if (size != rotscale.size() || size != translations.size() || size != masses.size()) {
        CHGPU_ERROR("Mesh loading vectors must all have same size\n");
    }

    if (size == 0) {
        printf("WARNING: No meshes provided!\n");
    }

    unsigned int nTriangles = 0;
    unsigned int numTriangleFamilies = 0;
    std::vector<geometry::ChTriangleMeshConnected> all_meshes;
    for (unsigned int i = 0; i < size; i++) {
        // INFO_PRINTF("Importing %s...\n", objfilenames[i].c_str()); <--- work on this later
        all_meshes.push_back(geometry::ChTriangleMeshConnected());
        geometry::ChTriangleMeshConnected& mesh = all_meshes[all_meshes.size() - 1];

        bool readin_flag = mesh.LoadWavefrontMesh(objfilenames[i], true, false);
        if (!readin_flag) {
            CHGPU_ERROR("ERROR! Mesh %s failed to load in! Exiting!\n", objfilenames[i].c_str());
        }

        // Apply displacement
        ChVector<> displ(translations[i].x, translations[i].y, translations[i].z);

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
    SetMeshes(all_meshes, masses);
    // work on this later: INFO_PRINTF("Done allocating mesh unified memory\n");
}

void ChSystemGpuMesh::SetMeshes(const std::vector<geometry::ChTriangleMeshConnected>& all_meshes,
                                     std::vector<float> masses) {
    int nTriangles = 0;
    for (const auto& mesh : all_meshes)
        nTriangles += mesh.getNumTriangles();

    ChSystemGpuMesh_impl::TriangleSoup* pMeshSoup = m_sys_trimesh->getMeshSoup();
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
    for (const auto& mesh : all_meshes) {
        for (int i = 0; i < mesh.getNumTriangles(); i++) {
            geometry::ChTriangle tri = mesh.getTriangle(i);

            pMeshSoup->node1[tri_i] = make_float3((float)tri.p1.x(), (float)tri.p1.y(), (float)tri.p1.z());
            pMeshSoup->node2[tri_i] = make_float3((float)tri.p2.x(), (float)tri.p2.y(), (float)tri.p2.z());
            pMeshSoup->node3[tri_i] = make_float3((float)tri.p3.x(), (float)tri.p3.y(), (float)tri.p3.z());

            pMeshSoup->triangleFamily_ID[tri_i] = family;

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
        gpuErrchk(cudaMallocManaged(&m_sys_trimesh->getTriParams()->fam_frame_broad,
                                    pMeshSoup->numTriangleFamilies * sizeof(ChSystemGpuMesh_impl::MeshFrame<float>),
                                    cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&m_sys_trimesh->getTriParams()->fam_frame_narrow,
                                    pMeshSoup->numTriangleFamilies * sizeof(ChSystemGpuMesh_impl::MeshFrame<double>),
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

// -----------------------------------------------------------------------------

// initialize particle positions, velocity and angular velocity in user units
void ChSystemGpu::SetParticlePositions(const std::vector<ChVector<float>>& points,
                                     const std::vector<ChVector<float>>& vels,
                                     const std::vector<ChVector<float>>& ang_vel) {
    std::vector<float3> pointsFloat3;
    std::vector<float3> velsFloat3;
    std::vector<float3> angVelsFloat3;
    convertChVector2Float3Vec(points, pointsFloat3);
    convertChVector2Float3Vec(vels, velsFloat3);
    convertChVector2Float3Vec(ang_vel, angVelsFloat3);
    m_sys->SetParticlePositions(pointsFloat3, velsFloat3, angVelsFloat3);
}

void ChSystemGpuMesh::SetParticlePositions(const std::vector<ChVector<float>>& points,
                                            const std::vector<ChVector<float>>& vels,
                                            const std::vector<ChVector<float>>& ang_vel) {
    std::vector<float3> pointsFloat3;
    std::vector<float3> velsFloat3;
    std::vector<float3> angVelsFloat3;
    convertChVector2Float3Vec(points, pointsFloat3);
    convertChVector2Float3Vec(vels, velsFloat3);
    convertChVector2Float3Vec(ang_vel, angVelsFloat3);
    m_sys_trimesh->SetParticlePositions(pointsFloat3, velsFloat3, angVelsFloat3);
}

// -----------------------------------------------------------------------------

ChVector<float> ChSystemGpu::GetParticlePosition(int nSphere) const {
    float3 pos = m_sys->GetParticlePosition(nSphere);
    return ChVector<float>(pos.x, pos.y, pos.z);
}

ChVector<float> ChSystemGpuMesh::GetParticlePosition(int nSphere) const {
    float3 pos = m_sys_trimesh->GetParticlePosition(nSphere);
    return ChVector<float>(pos.x, pos.y, pos.z);
}

// -----------------------------------------------------------------------------

ChVector<float> ChSystemGpu::GetParticleVelocity(int nSphere) const {
    float3 vel = m_sys->GetParticleLinVelocity(nSphere);
    return ChVector<float>(vel.x, vel.y, vel.z);
}

ChVector<float> ChSystemGpuMesh::GetParticleVelocity(int nSphere) const {
    float3 vel = m_sys_trimesh->GetParticleLinVelocity(nSphere);
    return ChVector<float>(vel.x, vel.y, vel.z);
}

// -----------------------------------------------------------------------------

ChVector<float> ChSystemGpu::GetParticleAngVelocity(int nSphere) const {
    float3 omega = m_sys->GetParticleAngVelocity(nSphere);
    return ChVector<float>(omega.x, omega.y, omega.z);
}

ChVector<float> ChSystemGpuMesh::GetParticleAngVelocity(int nSphere) const {
    float3 omega = m_sys_trimesh->GetParticleAngVelocity(nSphere);
    return ChVector<float>(omega.x, omega.y, omega.z);
}

// -----------------------------------------------------------------------------

ChVector<float> ChSystemGpu::GetBCplanePosition(size_t plane_id) const {
    // todo: throw an error if BC not a plane type
    float3 pos = m_sys->GetBCplanePosition(plane_id);
    return ChVector<float>(pos.x, pos.y, pos.z);
}

int ChSystemGpu::GetNumContacts() const {
    return m_sys->GetNumContacts();
}

}  // namespace gpu
}  // namespace chrono
