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
template <class T>
struct ChTriangleSoup {
    unsigned int nTrianglesInSoup;    //!< total number of triangles in the soup
    unsigned int nFamiliesInSoup;     //!< indicates how many meshes are squashed together in this soup
    unsigned int* triangleFamily_ID;  //!< each entry says what family that triagnle belongs to; size: nTrianglesInSoup

    T* node1_X;  //!< X position in global reference frame of node 1
    T* node1_Y;  //!< Y position in global reference frame of node 1
    T* node1_Z;  //!< Z position in global reference frame of node 1

    T* node2_X;  //!< X position in global reference frame of node 2
    T* node2_Y;  //!< Y position in global reference frame of node 2
    T* node2_Z;  //!< Z position in global reference frame of node 2

    T* node3_X;  //!< X position in global reference frame of node 3
    T* node3_Y;  //!< Y position in global reference frame of node 3
    T* node3_Z;  //!< Z position in global reference frame of node 3

    float* node1_XDOT;  //!< X velocity in global reference frame of node 1
    float* node1_YDOT;  //!< Y velocity in global reference frame of node 1
    float* node1_ZDOT;  //!< Z velocity in global reference frame of node 1

    float* node2_XDOT;  //!< X velocity in global reference frame of node 2
    float* node2_YDOT;  //!< Y velocity in global reference frame of node 2
    float* node2_ZDOT;  //!< Z velocity in global reference frame of node 2

    float* node3_XDOT;  //!< X velocity in global reference frame of node 3
    float* node3_YDOT;  //!< Y velocity in global reference frame of node 3
    float* node3_ZDOT;  //!< Z velocity in global reference frame of node 3

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
class CH_GRANULAR_API ChSystemGranularMonodisperse_SMC_Frictionless_trimesh
    : public ChSystemGranularMonodisperse_SMC_Frictionless {
  public:
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(float radiusSPH, float density)
        : ChSystemGranularMonodisperse_SMC_Frictionless(radiusSPH, density), K_n_s2m_UU(0), Gamma_n_s2m_UU(0) {}
    virtual ~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh();

    void set_K_n_SPH2MESH(double someValue) { K_n_s2m_UU = someValue; }
    void set_Gamma_n_SPH2MESH(double someValue) { Gamma_n_s2m_UU = someValue; }

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
    void meshSoup_applyRigidBodyMotion(double* position_orientation_data);

    virtual double advance_simulation(float duration) override;
    /// Extra parameters needed for triangle-sphere contact
    struct GranParamsHolder_trimesh {
        float Gamma_n_s2m_SU;                //!< sphere-to-mesh contact damping coefficient, expressed in SU
        float Kn_s2m_SU;                     //!< normal stiffness coefficient, expressed in SU: sphere-to-mesh
        unsigned int num_triangle_families;  /// Number of triangle families
        ChFamilyFrame<float>* fam_frame_broad;
        ChFamilyFrame<double>* fam_frame_narrow;
    };

    virtual void initialize() override;
    void load_meshes(std::vector<std::string> objfilenames, std::vector<float3> scalefactors);
    void write_meshes(std::string outfilename);

    /// Enable mesh contact
    void enableMeshCollision() { mesh_collision_enabled = true; }
    /// Disable mesh contact
    void disableMeshCollision() { mesh_collision_enabled = false; }

  private:
    GranParamsHolder_trimesh* tri_params;

    /// clean copy of mesh soup interacting with granular material
    // store a pointer since we use unified memory for this
    // Stored in UU
    ChTriangleSoup<float>* meshSoup_DEVICE;

    double K_n_s2m_UU;  //!< the stiffness associated w/ contact between a mesh element and gran material
    double K_n_s2m_SU;  //!< size of the normal stiffness (SU) for sphere-to-mesh contact

    double Gamma_n_s2m_UU;
    double Gamma_n_s2m_SU;

    bool mesh_collision_enabled = true;

    /// Number of triangles touching each bucket
    std::vector<unsigned int, cudallocator<unsigned int>> BUCKET_countsOfTrianglesTouching;
    std::vector<unsigned int, cudallocator<unsigned int>> triangles_in_BUCKET_composite;
    // Number of triangles touching each SD
    std::vector<unsigned int, cudallocator<unsigned int>> SD_isTouchingTriangle;
    // Function members
    void copy_triangle_data_to_device();
    void resetTriangleBroadphaseInformation();
    void resetTriangleForces();

    void setupTriMesh_DEVICE(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                             unsigned int nTriangles);
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