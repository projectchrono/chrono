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

#pragma once

//#include "chrono_granular/ChGranularDefines.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularUtilities.h"

namespace chrono {
namespace granular {

/**
 *\brief Template class used as a place holder for arrays associated with a mesh. No memory
 * allocation of freeing done by objects of this class. All its members are public.
 *
 *\attention  The order of the nodes in a triangle defines the positive face of the triangle; right-hand rule used.
 *\attention Some other agent needs to allocate/deallocate memory pointed to by variables in this class
 *
 */
template <class T3>
struct ChTriangleSoup {
    unsigned int nTrianglesInSoup;    //!< total number of triangles in the soup
    unsigned int nFamiliesInSoup;     //!< indicates how many meshes are squashed together in this soup
    unsigned int* triangleFamily_ID;  //!< each entry says what family that triagnle belongs to; size: nTrianglesInSoup

    float* familyMass_SU;  //!< entry i is the SU mass of family i

    T3* node1;  //!< Position in local reference frame of node 1
    T3* node2;  //!< Position in local reference frame of node 2
    T3* node3;  //!< Position in local reference frame of node 3

    T3* vel;    //!< entry i is the linear velocity of family i (rigid body motion)
    T3* omega;  //!< entry i is the angular velocity of family i (rigid body motion)

    float* generalizedForcesPerFamily;  //!< Generalized forces acting on each family. Expressed
                                        //!< in the global reference frame. Size: 6 * MAX_TRIANGLE_FAMILIES.
};

template <class T>
struct ChFamilyFrame {
    T pos[3];
    T rot_mat[9];
};

/** \brief Class implements functionality required to handle the interaction between a mesh soup and granular material.
 *
 * Mesh soup: a collection of meshes that each has a certain number of triangle elements. For instance, the meshes
 * associated with the four wheels of a rover operating on granular material would be smashed into one soup having four
 * mesh families.
 *
 * Assumptions: Mono-disperse setup, one radius for all spheres. There is no friction. There can be adehsion.
 * The granular material interacts through an implement that is defined via a triangular mesh.
 */
class CH_GRANULAR_API ChSystemGranular_MonodisperseSMC_trimesh : public ChSystemGranular_MonodisperseSMC {
  public:
    ChSystemGranular_MonodisperseSMC_trimesh(float radiusSPH, float density);
    virtual ~ChSystemGranular_MonodisperseSMC_trimesh();

    void set_K_n_SPH2MESH(double someValue) { K_n_s2m_UU = someValue; }
    void set_Gamma_n_SPH2MESH(double someValue) { Gamma_n_s2m_UU = someValue; }

    void set_K_t_SPH2MESH(double someValue) { K_t_s2m_UU = someValue; }
    void set_Gamma_t_SPH2MESH(double someValue) { Gamma_t_s2m_UU = someValue; }

    unsigned int nMeshesInSoup() const { return meshSoup_DEVICE->nFamiliesInSoup; }

    /**
    * \brief Collects the forces that each mesh feels as a result of their interaction with the DEs.
    * Each generalized force acting on a family in the soup has six components: three forces &
    * three torques.
    *
    * \param [in] crntTime The time at which the force is computed
    * \param [out] genForcesOnSoup Array that stores the generalized forces on the meshes in the soup

    * \return nothing
    *
    * \attention The size of genForcesOnSoup should be 6 * nFamiliesInSoup
    */
    void collectGeneralizedForcesOnMeshSoup(float* genForcesOnSoup);

    /// position_orientation_data should have 7 entries for each family: 3 pos, 4 orientation
    /// vel should have 6 entries for each family: 3 linear velocity, 3 angular velocity
    void meshSoup_applyRigidBodyMotion(double* position_orientation_data, float* vel);

    virtual double advance_simulation(float duration) override;
    /// Extra parameters needed for triangle-sphere contact
    struct ChGranParams_trimesh {
        float Gamma_n_s2m_SU;  //!< sphere-to-mesh contact damping coefficient, expressed in SU
        float Gamma_t_s2m_SU;
        float Kn_s2m_SU;  //!< normal stiffness coefficient, expressed in SU: sphere-to-mesh
        float Kt_s2m_SU;
        float adhesion_ratio_s2m;            //!< Ratio of adhesion force to sphere weight
        unsigned int num_triangle_families;  /// Number of triangle families
        ChFamilyFrame<float>* fam_frame_broad;
        ChFamilyFrame<double>* fam_frame_narrow;
    };

    virtual void initialize() override;
    void load_meshes(std::vector<std::string> objfilenames,
                     std::vector<float3> scalefactors,
                     std::vector<float> masses);
    void write_meshes(std::string outfilename);

    /// Set the ratio of adhesion force to sphere weight for sphere to mesh
    void set_Adhesion_ratio_S2M(float someValue) { adhesion_s2m_over_gravity = someValue; }

    /// Enable mesh contact
    void enableMeshCollision() { mesh_collision_enabled = true; }
    /// Disable mesh contact
    void disableMeshCollision() { mesh_collision_enabled = false; }

  protected:
    /// Create a helper to do triangle initialization
    void initializeTriangles();
    ChGranParams_trimesh* tri_params;

    /// clean copy of mesh soup interacting with granular material
    // store a pointer since we use unified memory for this
    // Stored in UU
    ChTriangleSoup<float3>* meshSoup_DEVICE;

    double K_n_s2m_UU;  //!< the stiffness associated w/ contact between a mesh element and gran material
    double K_n_s2m_SU;  //!< size of the normal stiffness (SU) for sphere-to-mesh contact

    double Gamma_n_s2m_UU;
    double Gamma_n_s2m_SU;

    double K_t_s2m_UU;
    double K_t_s2m_SU;

    double Gamma_t_s2m_UU;
    double Gamma_t_s2m_SU;

    float adhesion_s2m_over_gravity;

    bool mesh_collision_enabled = true;

    /// Number of triangles touching each bucket
    std::vector<unsigned int, cudallocator<unsigned int>> BUCKET_countsOfTrianglesTouching;
    std::vector<unsigned int, cudallocator<unsigned int>> triangles_in_BUCKET_composite;
    // Number of triangles touching each SD
    std::vector<unsigned int, cudallocator<unsigned int>> SD_isTouchingTriangle;
    // Function members
    void copyTriangleDataToDevice();
    void resetTriangleBroadphaseInformation();
    void resetTriangleForces();

    void setupTriMesh_DEVICE(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                             unsigned int nTriangles,
                             std::vector<float> masses);
    void cleanupTriMesh_DEVICE();

    // void initialize();

    virtual double get_max_K() override;

    template <typename T>
    void generate_rot_matrix(double* ep, T* rot_mat);

    template <class T>
    ChVector<T> ApplyFrameTransform(ChVector<T>& p, T* pos, T* rot_mat);
};

}  // namespace granular
}  // namespace chrono