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

#pragma once

//#include "chrono_granular/ChGranularDefines.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono_granular/physics/ChGranular.h"

namespace chrono {
namespace granular {

/// @addtogroup granular_physics
/// @{

///
/// Class used to hold pointers for mesh arrays.
///
///\attention The order of the nodes in a triangle defines the positive face of the triangle; right-hand rule used.

// TODO is a template necessary
template <class T3>
struct ChTriangleSoup {
    /// Total number of triangles in the soup
    unsigned int nTrianglesInSoup;
    /// Indicates how many meshes are squashed together in this soup
    unsigned int numTriangleFamilies;
    /// Each entry says what family that triagnle belongs to; size: nTrianglesInSoup
    unsigned int* triangleFamily_ID;

    /// Entry i is the SU mass of family i
    float* familyMass_SU;

    /// Entry i true indicates that family i is inflated
    bool* inflated;
    /// Entry i is the SU radius of inflation of family i
    float* inflation_radii;

    /// Position in local reference frame of triangle vertex 1
    T3* node1;
    /// Position in local reference frame of triangle vertex 2
    T3* node2;
    /// Position in local reference frame of triangle vertex 3
    T3* node3;

    /// Entry i is the linear velocity of family i (rigid body motion)
    T3* vel;
    /// Entry i is the angular velocity of family i (rigid body motion)
    T3* omega;

    /// Generalized forces acting on each family. Expressed
    /// in the global reference frame. Size: 6 * getNumTriangleFamilies.
    float* generalizedForcesPerFamily;
};

// TODO optimize rotations
/// Position and rotation matrix defining the frame of a triangle mesh
template <class T>
struct ChGranMeshFamilyFrame {
    T pos[3];
    T rot_mat[9];
};

/// Extra parameters needed for triangle-sphere contact
struct ChGranParams_trimesh {
    /// Sphere-to-mesh normal stiffness, expressed in SU (Hertzian spring)
    float K_n_s2m_SU;
    /// Sphere-to-mesh tangent stiffness, expressed in SU (Hertzian spring)
    float K_t_s2m_SU;

    /// Sphere-to-mesh normal contact damping coefficient, expressed in SU
    float Gamma_n_s2m_SU;
    /// Sphere-to-mesh tangent contact damping coefficient, expressed in SU
    float Gamma_t_s2m_SU;

    /// Acceleration caused by adhesion force (constant adhesion model)
    float adhesionAcc_s2m;

    /// Ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
    float static_friction_coeff_s2m;

    /// Coefficient of rolling resistance. Units and effect depend on the system rolling resistance model
    float rolling_coeff_s2m_SU;

    /// Number of triangle families
    unsigned int num_triangle_families;

    /// Reference frames of the triangle families in single precision
    ChGranMeshFamilyFrame<float>* fam_frame_broad;

    /// Reference frames of the triangle families in double precision
    ChGranMeshFamilyFrame<double>* fam_frame_narrow;
};

/// Class implements functionality required to handle the interaction between a mesh soup and granular material.
///
/// Mesh soup: a collection of meshes that each has a certain number of triangle elements. For instance, the meshes
/// associated with the four wheels of a rover operating on granular material would be smashed into one soup having four
/// mesh families.
class CH_GRANULAR_API ChSystemGranularSMC_trimesh : public ChSystemGranularSMC {
  public:
    // we do not want the system to be default-constructible
    ChSystemGranularSMC_trimesh() = delete;
    ChSystemGranularSMC_trimesh(float sphere_rad, float density, float3 boxDims);
    virtual ~ChSystemGranularSMC_trimesh();

    void set_K_n_SPH2MESH(double someValue) { K_n_s2m_UU = someValue; }
    void set_Gamma_n_SPH2MESH(double someValue) { Gamma_n_s2m_UU = someValue; }

    void set_K_t_SPH2MESH(double someValue) { K_t_s2m_UU = someValue; }
    void set_Gamma_t_SPH2MESH(double someValue) { Gamma_t_s2m_UU = someValue; }

    unsigned int getNumTriangleFamilies() const { return meshSoup->numTriangleFamilies; }

    /// Collect forces exerted on meshes by granular system
    /// Each generalized force is 3 forces (x,y,z) and 3 torques (x,y,z)
    /// Forces are genForcesOnSoup in order of triangle family
    /// genForcesOnSoup should have 6 entries for each family
    void collectGeneralizedForcesOnMeshSoup(float* genForcesOnSoup);

    /// position_orientation_data should have 7 entries for each family: 3 pos, 4 orientation
    /// vel should have 6 entries for each family: 3 linear velocity, 3 angular velocity
    void meshSoup_applyRigidBodyMotion(double* position_orientation_data, float* vel);

    virtual double advance_simulation(float duration) override;
    // override of parent initialize function
    virtual void initialize() override;
    /// Load triangle meshes into granular system. MUST happen before initialize is called
    void load_meshes(std::vector<std::string> objfilenames,
                     std::vector<ChMatrix33<float>> rotscale,
                     std::vector<float3> translations,
                     std::vector<float> masses,
                     std::vector<bool> inflated,
                     std::vector<float> inflation_radii);

    /// Write visualization files for triangle meshes with current positions
    void write_meshes(std::string outfilename);

    /// Set the ratio of adhesion force to sphere weight for sphere to mesh
    void set_Adhesion_ratio_S2M(float someValue) { adhesion_s2m_over_gravity = someValue; }

    // these quantities are unitless anyways
    void set_static_friction_coeff_SPH2MESH(float mu) { tri_params->static_friction_coeff_s2m = mu; }
    // set internally and convert later
    void set_rolling_coeff_SPH2MESH(float mu) { rolling_coeff_s2s_UU = mu; }

    /// Enable mesh contact
    void enableMeshCollision() { mesh_collision_enabled = true; }
    /// Disable mesh contact
    void disableMeshCollision() { mesh_collision_enabled = false; }

  protected:
    /// Create a helper to do triangle initialization
    virtual void initializeTriangles();
    /// Set of simulation parameters related to triangle data
    ChGranParams_trimesh* tri_params;

    /// Clean copy of mesh soup interacting with granular material in unified memory. Stored in UU
    ChTriangleSoup<float3>* meshSoup;

    /// Sphere-to-mesh normal contact stiffness, in user units (Hertzian spring)
    double K_n_s2m_UU;
    /// Sphere-to-mesh normal damping coefficient, in user units
    double Gamma_n_s2m_UU;

    /// Sphere-to-mesh tangent contact stiffness, in user units (Hertzian spring)
    double K_t_s2m_UU;
    /// Sphere-to-mesh tangent damping coefficient, in user units
    double Gamma_t_s2m_UU;

    /// Rolling friction coefficient for sphere-to-mesh -- units and effect depend on rolling resistance model
    double rolling_coeff_s2m_UU;

    /// Ratio of sphere-to-mesh adhesion to gravity (constant adhesion model)
    float adhesion_s2m_over_gravity;

    /// Enable or disable collision between spheres and meshes
    bool mesh_collision_enabled = true;

    /// Number of triangles touching each subdomain
    std::vector<unsigned int, cudallocator<unsigned int>> triangles_in_SD_composite;

    /// Number of triangles touching each subdomain
    std::vector<unsigned int, cudallocator<unsigned int>> SD_numTrianglesTouching;

    /// Big array of triangle offsets for each subdomain
    std::vector<unsigned int, cudallocator<unsigned int>> SD_TriangleCompositeOffsets;

    /// Reset information used for triangle broadphase collision detection
    void resetTriangleBroadphaseInformation();
    /// Reset computed forces and torques on each triangle family
    void resetTriangleForces();

    /// Setup data structures associated with triangle mesh
    void setupTriMesh(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                      unsigned int nTriangles,
                      std::vector<float> masses,
                      std::vector<bool> inflated,
                      std::vector<float> inflation_radii);

    /// Clean up data structures associated with triangle mesh
    void cleanupTriMesh();

    /// Broadphase CD for triangles
    void runTriangleBroadphase();

    virtual double get_max_K() const override;

    template <typename T>
    void generate_rot_matrix(double* ep, T* rot_mat);

    template <class T>
    ChVector<T> ApplyFrameTransform(ChVector<T>& p, T* pos, T* rot_mat);
};
/// @} granular_physics

}  // namespace granular
}  // namespace chrono