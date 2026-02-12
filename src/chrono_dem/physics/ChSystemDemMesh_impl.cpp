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
// Authors: Conlain Kelly, Nic Olsen, Ruochun Zhang, Dan Negrut, Radu Serban
// =============================================================================

#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <cmath>

#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono_dem/physics/ChSystemDemMesh_impl.h"
#include "chrono_dem/utils/ChDemUtilities.h"

namespace chrono {
namespace dem {

ChSystemDemMesh_impl::ChSystemDemMesh_impl(float sphere_rad, float density, float3 boxDims, float3 O)
    : ChSystemDem_impl(sphere_rad, density, boxDims, O),
      K_n_s2m_UU(0),
      K_t_s2m_UU(0),
      Gamma_n_s2m_UU(0),
      Gamma_t_s2m_UU(0),
      YoungsModulus_mesh_UU(0.0),
      COR_mesh_UU(0.0),
      use_mat_based(false),
      PoissonRatio_mesh_UU(0.0),
      rolling_coeff_s2m_UU(0),
      spinning_coeff_s2m_UU(0),
      adhesion_s2m_over_gravity(0) {
    // Allocate triangle collision parameters
    demErrchk(cudaMallocManaged(&tri_params, sizeof(MeshParams), cudaMemAttachGlobal));

    // Allocate the device soup storage
    demErrchk(cudaMallocManaged(&meshSoup, sizeof(TriangleSoup), cudaMemAttachGlobal));
    // start with no triangles
    meshSoup->nTrianglesInSoup = 0;
    meshSoup->numTriangleFamilies = 0;

    tri_params->static_friction_coeff_s2m = 0;
}

ChSystemDemMesh_impl::~ChSystemDemMesh_impl() {
    // work to do here
    cleanupTriMesh();
}
double ChSystemDemMesh_impl::get_max_K() const {
    double maxK;
    if (tri_params->use_mat_based == true) {
        // material pparameter sigma for different surface
        // see reference eq 2.13, 2.14 in Book Contact Force Model, Flores and Lankarani
        double sigma_sphere = (1 - std::pow(PoissonRatio_sphere_UU, 2)) / YoungsModulus_sphere_UU;
        double sigma_wall = (1 - std::pow(PoissonRatio_wall_UU, 2)) / YoungsModulus_wall_UU;
        double sigma_mesh = (1 - std::pow(PoissonRatio_mesh_UU, 2)) / YoungsModulus_mesh_UU;

        maxK = 4.0 / (3.0 * (sigma_sphere + std::min(std::min(sigma_sphere, sigma_wall), sigma_mesh))) *
               std::sqrt(sphere_radius_UU);
        INFO_PRINTF("Use material based contact force model, maximum effective stiffnes is %e\n", maxK);
        return maxK;

    } else {
        maxK = std::max(std::max(K_n_s2s_UU, K_n_s2w_UU), K_n_s2m_UU);
        INFO_PRINTF("Use user defined contact force model, maximum effective stiffnes is %e\n", maxK);
        return maxK;
    }
}

void ChSystemDemMesh_impl::combineMaterialSurface() {
    // effective youngs modulus and shear modulus in user units
    double E_eff_s2m_uu, G_eff_s2m_uu;

    materialPropertyCombine(YoungsModulus_sphere_UU, YoungsModulus_mesh_UU, PoissonRatio_sphere_UU,
                            PoissonRatio_mesh_UU, E_eff_s2m_uu, G_eff_s2m_uu);

    // unit conversion of youngs modulus
    double E_SU2UU = this->MASS_SU2UU / (this->LENGTH_SU2UU * this->TIME_SU2UU * this->TIME_SU2UU);

    // convert effective youngs modulus and shear modulus to simulation units
    tri_params->E_eff_s2m_SU = (float)E_eff_s2m_uu / E_SU2UU;
    tri_params->G_eff_s2m_SU = (float)G_eff_s2m_uu / E_SU2UU;

    // assign coefficient of restitution in simulation units
    tri_params->COR_s2m_SU = std::min(COR_sphere_UU, COR_mesh_UU);
}

void ChSystemDemMesh_impl::initializeTriangles() {
    tri_params->use_mat_based = use_mat_based;

    if (use_mat_based == false) {
        double K_SU2UU = MASS_SU2UU / (TIME_SU2UU * TIME_SU2UU);
        double GAMMA_SU2UU = 1. / TIME_SU2UU;

        tri_params->K_n_s2m_SU = (float)(K_n_s2m_UU / K_SU2UU);
        tri_params->K_t_s2m_SU = (float)(K_t_s2m_UU / K_SU2UU);

        tri_params->Gamma_n_s2m_SU = (float)(Gamma_n_s2m_UU / GAMMA_SU2UU);
        tri_params->Gamma_t_s2m_SU = (float)(Gamma_t_s2m_UU / GAMMA_SU2UU);

    } else {
        combineMaterialSurface();
    }

    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);
    tri_params->adhesionAcc_s2m = (float)(adhesion_s2m_over_gravity * magGravAcc);

    for (unsigned int fam = 0; fam < meshSoup->numTriangleFamilies; fam++) {
        meshSoup->familyMass_SU[fam] = (float)(meshSoup->familyMass_SU[fam] / MASS_SU2UU);
    }

    tri_params->rolling_coeff_s2m_SU = (float)rolling_coeff_s2m_UU;

    const double meshRot[4] = {1., 0., 0., 0.};
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
    TRACK_VECTOR_RESIZE(SD_TrianglesCompositeOffsets, nSDs, "SD_TrianglesCompositeOffsets", 0);

    TRACK_VECTOR_RESIZE(Triangle_NumSDsTouching, meshSoup->nTrianglesInSoup, "Triangle_NumSDsTouching", 0);
    TRACK_VECTOR_RESIZE(Triangle_SDsCompositeOffsets, meshSoup->nTrianglesInSoup, "Triangle_SDsCompositeOffsets", 0);

    // TODO do we have a good heuristic???
    // this gets resized on-the-fly every timestep
    TRACK_VECTOR_RESIZE(SD_trianglesInEachSD_composite, 0, "SD_trianglesInEachSD_composite", 0);
}

// p = pos + rot_mat * p
void ChSystemDemMesh_impl::ApplyFrameTransform(float3& p, float* pos, float* rot_mat) {
    float3 result;

    // Apply rotation matrix to point
    result.x = pos[0] + rot_mat[0] * p.x + rot_mat[1] * p.y + rot_mat[2] * p.z;
    result.y = pos[1] + rot_mat[3] * p.x + rot_mat[4] * p.y + rot_mat[5] * p.z;
    result.z = pos[2] + rot_mat[6] * p.x + rot_mat[7] * p.y + rot_mat[8] * p.z;

    // overwrite p only at the end
    p = result;
}

void ChSystemDemMesh_impl::cleanupTriMesh() {
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

void ChSystemDemMesh_impl::ApplyMeshMotion(unsigned int mesh_id,
                                           const double* pos,
                                           const double* rot,
                                           const double* lin_vel,
                                           const double* ang_vel) {
    // Set position and orientation
    tri_params->fam_frame_broad[mesh_id].pos[0] = (float)pos[0];
    tri_params->fam_frame_broad[mesh_id].pos[1] = (float)pos[1];
    tri_params->fam_frame_broad[mesh_id].pos[2] = (float)pos[2];
    generate_rot_matrix<float>(rot, tri_params->fam_frame_broad[mesh_id].rot_mat);

    tri_params->fam_frame_narrow[mesh_id].pos[0] = pos[0];
    tri_params->fam_frame_narrow[mesh_id].pos[1] = pos[1];
    tri_params->fam_frame_narrow[mesh_id].pos[2] = pos[2];
    generate_rot_matrix<double>(rot, tri_params->fam_frame_narrow[mesh_id].rot_mat);

    // Set linear and angular velocity
    const float C_V = (float)(TIME_SU2UU / LENGTH_SU2UU);
    meshSoup->vel[mesh_id] = make_float3(C_V * (float)lin_vel[0], C_V * (float)lin_vel[1], C_V * (float)lin_vel[2]);

    const float C_O = (float)TIME_SU2UU;
    meshSoup->omega[mesh_id] = make_float3(C_O * (float)ang_vel[0], C_O * (float)ang_vel[1], C_O * (float)ang_vel[2]);
}

template <typename T>
void ChSystemDemMesh_impl::generate_rot_matrix(const double* ep, T* rot_mat) {
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

}  // namespace dem
}  // namespace chrono
