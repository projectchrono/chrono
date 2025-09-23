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

#pragma once

#include "chrono_dem/physics/ChSystemDem_impl.h"

namespace chrono {
namespace dem {

// Underlying implementation of the Chrono::Dem mesh system.
// Implements functionality required to handle the interaction between a mesh soup and granular material.
//
// Mesh soup: a collection of meshes that each has a certain number of triangle elements. For instance, the meshes
// associated with the four wheels of a rover operating on granular material would be smashed into one soup having four
// mesh families.
class ChSystemDemMesh_impl : public ChSystemDem_impl {
  public:
    virtual ~ChSystemDemMesh_impl();

  protected:
    /// Position and rotation matrix defining the frame of a triangle mesh.
    template <class T>
    struct MeshFrame {
        // TODO optimize rotations
        T pos[3];
        T rot_mat[9];
    };

    /// Extra parameters needed for triangle-sphere contact.
    struct MeshParams {
        /// Sphere-to-mesh normal stiffness, expressed in SU (Hertzian spring)
        float K_n_s2m_SU;
        /// Sphere-to-mesh tangent stiffness, expressed in SU (Hertzian spring)
        float K_t_s2m_SU;

        /// Sphere-to-mesh normal contact damping coefficient, expressed in SU
        float Gamma_n_s2m_SU;
        /// Sphere-to-mesh tangent contact damping coefficient, expressed in SU
        float Gamma_t_s2m_SU;

        /// material based property
        bool use_mat_based;

        /// effective sphere-to-mesh youngs modulus, expressed in SU
        float E_eff_s2m_SU;

        /// effective sphere-to-mesh shear modulus, expressed in SU
        float G_eff_s2m_SU;

        /// effective sphere-to-mesh coefficient of restitution, expressed in SU
        float COR_s2m_SU;

        /// Acceleration caused by adhesion force (constant adhesion model)
        float adhesionAcc_s2m;

        /// Ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
        float static_friction_coeff_s2m;

        /// Coefficient of rolling resistance. Units and effect depend on the system rolling resistance model
        float rolling_coeff_s2m_SU;

        /// Coefficient of spinning resistance. Units and effect depend on the system spinning resistance model
        float spinning_coeff_s2m_SU;

        /// Number of triangle families
        unsigned int num_triangle_families;

        /// Reference frames of the triangle families in single precision
        MeshFrame<float>* fam_frame_broad;

        /// Reference frames of the triangle families in double precision
        MeshFrame<double>* fam_frame_narrow;
    };

    /// Structure used to hold pointers for mesh arrays.
    /// Note: The order of the nodes in a triangle defines the positive face of the triangle; right-hand rule used.
    struct TriangleSoup {
        /// Total number of triangles in the soup
        unsigned int nTrianglesInSoup;
        /// Indicates how many meshes are squashed together in this soup
        unsigned int numTriangleFamilies;
        /// Each entry says what family that triagnle belongs to; size: nTrianglesInSoup
        unsigned int* triangleFamily_ID;

        /// Entry i is the SU mass of family i
        float* familyMass_SU;

        /// Position in local reference frame of triangle vertex 1
        float3* node1;
        /// Position in local reference frame of triangle vertex 2
        float3* node2;
        /// Position in local reference frame of triangle vertex 3
        float3* node3;

        /// Entry i is the linear velocity of family i (rigid body motion)
        float3* vel;
        /// Entry i is the angular velocity of family i (rigid body motion)
        float3* omega;

        /// Generalized forces acting on each family. Expressed
        /// in the global reference frame. Size: 6 * numTriangleFamilies.
        float* generalizedForcesPerFamily;
    };

    TriangleSoup* getMeshSoup() { return meshSoup; }
    MeshParams* getTriParams() { return tri_params; }

    // The system is not default-constructible
    ChSystemDemMesh_impl() = delete;

    /// Construct Chrono::Dem system with given sphere radius, density, big domain dimensions and frame origin.
    ChSystemDemMesh_impl(float sphere_rad, float density, float3 boxDims, float3 O);

    /// combine material properties of two types to get effective ones
    void combineMaterialSurface();

    /// Apply rigid body motion to specified mesh.
    void ApplyMeshMotion(unsigned int mesh_id,
                         const double* pos,
                         const double* rot,
                         const double* lin_vel,
                         const double* ang_vel);

    /// Initialize trimeshes before starting simulation (typically called by initialize).
    void initializeTriangles();

    /// Clean up data structures associated with triangle mesh
    void cleanupTriMesh();

    /// Broadphase CD for triangles
    void runTriangleBroadphase();

    virtual double get_max_K() const override;

    template <typename T>
    void generate_rot_matrix(const double* ep, T* rot_mat);

    static void ApplyFrameTransform(float3& p, float* pos, float* rot_mat);

    /// Advance simulation by duration in user units, return actual duration elapsed.
    /// Requires initialize() to have been called.
    virtual double AdvanceSimulation(float duration) override;

    /// Set of simulation parameters related to triangle data
    MeshParams* tri_params;

    /// Clean copy of mesh soup interacting with granular material in unified memory. Stored in UU
    TriangleSoup* meshSoup;

    /// Sphere-to-mesh normal contact stiffness, in user units (Hertzian spring)
    double K_n_s2m_UU;
    /// Sphere-to-mesh normal damping coefficient, in user units
    double Gamma_n_s2m_UU;

    /// Sphere-to-mesh tangent contact stiffness, in user units (Hertzian spring)
    double K_t_s2m_UU;
    /// Sphere-to-mesh tangent damping coefficient, in user units
    double Gamma_t_s2m_UU;

    /// flag for using material based property
    bool use_mat_based = false;

    /// youngs modulus of mesh
    double YoungsModulus_mesh_UU;

    /// Poisson ratio of mesh
    double PoissonRatio_mesh_UU;

    /// coefficient of restitution of mesh
    double COR_mesh_UU;

    /// Rolling friction coefficient for sphere-to-mesh -- units and effect depend on rolling resistance model
    float rolling_coeff_s2m_UU;

    /// Spinning friction coefficient for sphere-to-mesh -- units and effect depend on spinning resistance model
    float spinning_coeff_s2m_UU;

    /// Ratio of sphere-to-mesh adhesion to gravity (constant adhesion model)
    float adhesion_s2m_over_gravity;

    /// Enable or disable collision between spheres and meshes
    bool mesh_collision_enabled = true;

    /// stores list of triangles touching each SD; goes SD by SD; size can change during simulation
    std::vector<unsigned int, cudallocator<unsigned int>> SD_trianglesInEachSD_composite;

    /// the count of triangles touching each SD; size of vector should be # of SDs
    std::vector<unsigned int, cudallocator<unsigned int>> SD_numTrianglesTouching;

    /// offsets in the composite array for each SD; i.e., offset where each SD starts storing its triangles.
    /// Size of vector should be # of SDs.
    std::vector<unsigned int, cudallocator<unsigned int>> SD_TrianglesCompositeOffsets;

    /// Number of SDs that each triangle touches; stored triangle by triangle.
    /// Nonessential array, only needed to carry out broadphase for mesh
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_NumSDsTouching;

    /// Helper array that stores the prefix scan output done on Triangle_NumSDsTouching.
    /// Nonessential array, only needed to carry out broadphase for mesh
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsCompositeOffsets;

    /// for each triangle, lists the collection of SDs that triangle touches; e.g., triangle 0 touches SDs 23, 32, 9,
    /// 199; triangle 0 touchs SDs 23, 33, 109; triangle 2 touches SDs 991; triangle 3 touches 43, 23, etc.
    /// Nonessential array, only needed to carry out broadphase for mesh
    std::vector<unsigned int, cudallocator<unsigned int>> SDsTouchedByEachTriangle_composite_out;

    /// TriangleIDS_ByMultiplicity_out is mirroring the SDsTouchedByEachTriangle_composite_out vector. Its entries is
    /// the list of triangle IDs, with the right multiplicity. It's used for a sort by key operation needed to figure
    /// out what triangles are stored in each SD. Thus, for the example above, the entries would be
    /// 0,0,0,0,1,1,1,2,3,3,etc. Nonessential array, only needed to carry out broadphase for mesh
    std::vector<unsigned int, cudallocator<unsigned int>> TriangleIDS_ByMultiplicity_out;

    /// dummy vector used in the broadphase done for the mesh, to understand what SD contains what triangles
    std::vector<unsigned int, cudallocator<unsigned int>> SDsTouchedByEachTriangle_composite;

    /// dummy vector used in the broadphase done for the mesh, to understand what SD contains what triangles
    std::vector<unsigned int, cudallocator<unsigned int>> TriangleIDS_ByMultiplicity;

  public:
    /// Get nicer handles to pointer names, enforce const-ness on the mesh params
    typedef const chrono::dem::ChSystemDemMesh_impl::MeshParams* MeshParamsPtr;

    /// Get nicer handles to pointer names, enforce const-ness on the mesh params
    typedef chrono::dem::ChSystemDemMesh_impl::TriangleSoup* TriangleSoupPtr;

    friend class ChSystemDemMesh;
};

}  // namespace dem
}  // namespace chrono
