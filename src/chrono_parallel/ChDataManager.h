// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: This class contains manages all data associated with a parallel
// System. Rather than passing in individual data parameters to different parts
// of the code like the collision detection and the solver, passing a pointer to
// a data manager is more convenient from a development perspective.
//
// =============================================================================

#pragma once

#include <memory>

#include "chrono/physics/ChContactContainer.h"

// Chrono::Parallel headers
#include "chrono_parallel/ChTimerParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChSettings.h"
#include "chrono_parallel/ChMeasures.h"
#include "chrono_parallel/math/matrix.h"
#include "chrono_parallel/math/sse.h"

// ATTENTION: It is important for these to be included after sse.h!
// Blaze Includes
#include <blaze/system/Version.h>
#include <blaze/math/CompressedMatrix.h>
#include <blaze/math/DynamicVector.h>
#if BLAZE_MAJOR_VERSION == 2
#include <blaze/math/DenseSubvector.h>
#elif BLAZE_MAJOR_VERSION == 3
#include <blaze/math/Subvector.h>
#endif

using blaze::CompressedMatrix;
using blaze::DynamicVector;
using blaze::Submatrix;
using blaze::Subvector;
using custom_vector;

namespace chrono {

class ChBody;
class ChLink;
class ChPhysicsItem;
class real2;
class real3;
class real4;
class vec3;
class Ch3DOFContainer;
class ChFEAContainer;
class ChFluidContainer;
class ChMPMContainer;
class ChFLIPContainer;
class ChConstraintRigidRigid;
class ChConstraintBilateral;
template <typename T>
class ChMaterialCompositionStrategy;

namespace collision {
class ChCBroadphase;           // forward declaration
class ChCNarrowphaseDispatch;  // forward declaration
class ChCAABBGenerator;        // forward declaration
}

#if BLAZE_MAJOR_VERSION == 2
typedef blaze::SparseSubmatrix<CompressedMatrix<real>> SubMatrixType;
typedef blaze::DenseSubvector<DynamicVector<real>> SubVectorType;
typedef blaze::SparseSubmatrix<const CompressedMatrix<real>> ConstSubMatrixType;
typedef blaze::DenseSubvector<const DynamicVector<real>> ConstSubVectorType;
#elif BLAZE_MAJOR_VERSION == 3
typedef blaze::Submatrix<CompressedMatrix<real>> SubMatrixType;
typedef blaze::Subvector<DynamicVector<real>> SubVectorType;
typedef blaze::Submatrix<const CompressedMatrix<real>> ConstSubMatrixType;
typedef blaze::Subvector<const DynamicVector<real>> ConstSubVectorType;
#endif

// These defines are used in the submatrix calls below to keep them concise
// They aren't names to be easy to understand, but for length
#define _num_dof_ data_manager->num_dof
#define _num_rigid_dof_ data_manager->num_rigid_bodies * 6
#define _num_shaft_dof_ data_manager->num_shafts
#define _num_motor_dof_ data_manager->num_motors
#define _num_fluid_dof_ data_manager->num_fluid_bodies * 3
#define _num_bil_ data_manager->num_bilaterals
#define _num_uni_ data_manager->num_unilaterals
#define _num_r_c_ data_manager->num_rigid_contacts
#define _num_rf_c_ data_manager->num_rigid_fluid_contacts
#define _num_fluid_ data_manager->num_fluid_bodies

// 0
//_num_rigid_dof_
//_num_shaft_dof_
//_num_motor_dof_
//_num_fluid_dof_

// 0
//_num_r_c_
// 2*_num_r_c_
// 3*_num_r_c_
//_num_bil_
//_num_rf_c_
// 2*_num_rf_c_
//_num_fluid_
// 3*_num_fluid_
#define _D_ data_manager->host_data.D
#define _M_invD_ data_manager->host_data.M_invD
#define _D_T_ data_manager->host_data.D_T
#define _E_ data_manager->host_data.E
#define _gamma_ data_manager->host_data.gamma

#define _DN_ submatrix(_D_, 0, 0, _num_rigid_dof_, 1 * _num_r_c_)
#define _DT_ submatrix(_D_, 0, _num_r_c_, _num_rigid_dof_, 2 * _num_r_c_)
#define _DS_ submatrix(_D_, 0, 3 * _num_r_c_, _num_rigid_dof_, 3 * _num_r_c_)
// D Bilateral
#define _DB_ submatrix(_D_, 0, _num_uni_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, _num_bil_)
// D Rigid Fluid
#define _DRFN_ submatrix(_D_, 0, _num_uni_ + _num_bil_, _num_dof_, _num_rf_c_)
#define _DRFT_ submatrix(_D_, 0, _num_uni_ + _num_bil_ + _num_rf_c_, _num_dof_, 2 * _num_rf_c_)
// D fluid fluid density
#define _DFFD_                                                                                                  \
    submatrix(_D_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, _num_uni_ + _num_bil_ + 3 * _num_rf_c_, \
              _num_fluid_dof_, _num_fluid_)
// D fluid fluid viscosity
#define _DFFV_                                                          \
    submatrix(_D_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, \
              _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_, _num_fluid_dof_, 3 * _num_fluid_)
//======
#define _MINVDN_ submatrix(_M_invD_, 0, 0, _num_rigid_dof_, 1 * _num_r_c_)
#define _MINVDT_ submatrix(_M_invD_, 0, _num_r_c_, _num_rigid_dof_, 2 * _num_r_c_)
#define _MINVDS_ submatrix(_M_invD_, 0, 3 * _num_r_c_, _num_rigid_dof_, 3 * _num_r_c_)
// Bilateral
#define _MINVDB_ submatrix(_M_invD_, 0, _num_uni_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, _num_bil_)
// Rigid Fluid
#define _MINVDRFN_ submatrix(_M_invD_, 0, _num_uni_ + _num_bil_, _num_dof_, _num_rf_c_)
#define _MINVDRFT_ submatrix(_M_invD_, 0, _num_uni_ + _num_bil_ + _num_rf_c_, _num_dof_, 2 * _num_rf_c_)
// Density
#define _MINVDFFD_                                                                                                   \
    submatrix(_M_invD_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, _num_uni_ + _num_bil_ + 3 * _num_rf_c_, \
              _num_fluid_dof_, _num_fluid_)
// Viscosity
#define _MINVDFFV_                                                           \
    submatrix(_M_invD_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, \
              _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_, _num_fluid_dof_, 3 * _num_fluid_)
//======
#define _DNT_ submatrix(_D_T_, 0, 0, _num_r_c_, _num_rigid_dof_)
#define _DTT_ submatrix(_D_T_, _num_r_c_, 0, 2 * _num_r_c_, _num_rigid_dof_)
#define _DST_ submatrix(_D_T_, 3 * _num_r_c_, 0, 3 * _num_r_c_, _num_rigid_dof_)
// Bilateral
#define _DBT_ submatrix(_D_T_, _num_uni_, 0, _num_bil_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_)
// Rigid Fluid
#define _DRFNT_ submatrix(_D_T_, _num_uni_ + _num_bil_, 0, _num_rf_c_, _num_dof_)
#define _DRFTT_ submatrix(_D_T_, _num_uni_ + _num_bil_ + _num_rf_c_, 0, 2 * _num_rf_c_, _num_dof_)
// Density
#define _DFFDT_                                                                                                   \
    submatrix(_D_T_, _num_uni_ + _num_bil_ + 3 * _num_rf_c_, _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, \
              _num_fluid_, _num_fluid_dof_)
// Viscosity
#define _DFFVT_                                                            \
    submatrix(_D_T_, _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_, \
              _num_rigid_dof_ + _num_shaft_dof_ + _num_motor_dof_, 3 * _num_fluid_, _num_fluid_dof_)
//======
#define _EN_ subvector(_E_, 0, _num_r_c_)
#define _ET_ subvector(_E_, _num_r_c_, 2 * _num_r_c_)
#define _ES_ subvector(_E_, 3 * _num_r_c_, 3 * _num_r_c_)
// Bilateral
#define _EB_ subvector(_E_, _num_uni_, _num_bil_)
// Rigid Fluid
#define _ERFN_ subvector(_E_, _num_uni_ + _num_bil_, _num_rf_c_)
#define _ERFT_ subvector(_E_, _num_uni_ + _num_bil_ + _num_rf_c_, 2 * _num_rf_c_)
// Density
#define _EFFD_ subvector(_E_, _num_uni_ + _num_bil_ + 3 * _num_rf_c_, _num_fluid_)
// Viscosity
#define _EFFV_ subvector(_E_, _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_, 3 * _num_fluid_)

////======
//#define _GAMMAN_ subvector(_gamma_, 0, _num_r_c_)
//#define _GAMMAT_ submatrix(_gamma_, _num_r_c_, 2 * _num_r_c_)
//#define _GAMMAS_ submatrix(_gamma_, 3 * _num_r_c_, 3 * _num_r_c_)
//// Bilateral
//#define _GAMMAB_ submatrix(_gamma_, _num_uni_,  _num_bil_)
//// Rigid Fluid
//#define _GAMMARFN_ submatrix(_gamma_, _num_uni_ + _num_bil_ _num_rf_c_)
//#define _GAMMARFT_ submatrix(_gamma_, _num_uni_ + _num_bil_ + _num_rf_c_, 2 * _num_rf_c_)
//// Density
//#define _GAMMAFFD_ submatrix(_gamma_,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_, _num_fluid_)
//// Viscosity
//#define _GAMMAFFV_ submatrix(_gamma_,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_,  3 * _num_fluid_)

// The maximum number of shear history contacts per smaller body (SMC)
#define max_shear 20

/// @addtogroup parallel_module
/// @{

/// Structure of arrays containing contact shape information.
struct shape_container {
    custom_vector<short2> fam_rigid;      ///< Family information
    custom_vector<uint> id_rigid;         ///< Body identifier for each shape
    custom_vector<int> typ_rigid;         ///< Shape type
    custom_vector<int> start_rigid;       ///< Index for shape start
    custom_vector<int> length_rigid;      ///< Usually 1, except for convex
    custom_vector<quaternion> ObR_rigid;  ///< Shape rotation
    custom_vector<real3> ObA_rigid;       ///< Position of shape

    custom_vector<real> sphere_rigid;
    custom_vector<real3> box_like_rigid;
    custom_vector<real3> triangle_rigid;
    custom_vector<real2> capsule_rigid;
    custom_vector<real4> rbox_like_rigid;
    custom_vector<real3> convex_rigid;
    custom_vector<int> tetrahedron_rigid;

    custom_vector<real3> triangle_global;
    custom_vector<real3> obj_data_A_global;
    custom_vector<quaternion> obj_data_R_global;
};

/// Structure of arrays containing simulation data.
struct host_container {
    // Collision data

    custom_vector<real3> aabb_min;  ///< List of bounding boxes minimum point
    custom_vector<real3> aabb_max;  ///< List of bounding boxes maximum point

    custom_vector<real3> aabb_min_tet;  ///< List of bounding boxes minimum point for tets
    custom_vector<real3> aabb_max_tet;  ///< List of bounding boxes maximum point for tets

    custom_vector<long long> contact_pairs;  ///< Contact pairs (encoded in a single long log)

    // Contact data
    custom_vector<real3> norm_rigid_rigid;
    custom_vector<real3> cpta_rigid_rigid;
    custom_vector<real3> cptb_rigid_rigid;
    custom_vector<real> dpth_rigid_rigid;
    custom_vector<real> erad_rigid_rigid;
    custom_vector<vec2> bids_rigid_rigid;

    custom_vector<real3> norm_rigid_fluid;
    custom_vector<real3> cpta_rigid_fluid;
    custom_vector<real> dpth_rigid_fluid;
    custom_vector<int> neighbor_rigid_fluid;
    // custom_vector<vec2> bids_rigid_fluid;
    custom_vector<int> c_counts_rigid_fluid;

    // custom_vector<vec2> bids_fluid_fluid;
    // each particle has a finite number of neighbors preallocated
    custom_vector<int> neighbor_3dof_3dof;
    custom_vector<int> c_counts_3dof_3dof;
    custom_vector<int> particle_indices_3dof;
    // custom_vector<int> fluid_contact_index;
    // custom_vector<long long> bids_fluid_fluid;
    custom_vector<int> reverse_mapping_3dof;

    custom_vector<real3> norm_rigid_tet;
    custom_vector<real3> cpta_rigid_tet;
    custom_vector<real3> cptb_rigid_tet;
    custom_vector<real> dpth_rigid_tet;
    custom_vector<int> neighbor_rigid_tet;
    custom_vector<real4> face_rigid_tet;
    custom_vector<int> c_counts_rigid_tet;
    // contact with nodes
    custom_vector<real3> norm_rigid_tet_node;
    custom_vector<real3> cpta_rigid_tet_node;
    custom_vector<real> dpth_rigid_tet_node;
    custom_vector<int> neighbor_rigid_tet_node;
    custom_vector<int> c_counts_rigid_tet_node;

    custom_vector<real3> norm_marker_tet;
    custom_vector<real3> cpta_marker_tet;
    custom_vector<real3> cptb_marker_tet;
    custom_vector<real> dpth_marker_tet;
    custom_vector<int> neighbor_marker_tet;
    custom_vector<real4> face_marker_tet;
    custom_vector<int> c_counts_marker_tet;

    // Contact forces (SMC)
    // These vectors hold the total contact force and torque, respectively,
    // for bodies that are involved in at least one contact.
    custom_vector<real3> ct_body_force;   ///< Total contact force on bodies
    custom_vector<real3> ct_body_torque;  ///< Total contact torque on these bodies

    // Contact shear history (SMC)
    custom_vector<vec3> shear_neigh;  ///< Neighbor list of contacting bodies and shapes
    custom_vector<real3> shear_disp;  ///< Accumulated shear displacement for each neighbor

    /// Mapping from all bodies in the system to bodies involved in a contact.
    /// For bodies that are currently not in contact, the mapping entry is -1.
    /// Otherwise, the mapping holds the appropriate index in the vectors above.
    custom_vector<int> ct_body_map;

    /// This vector holds the friction information as a triplet:
    /// x - Sliding friction,
    /// y - Rolling friction,
    /// z - Spinning Friction.
    /// This is precomputed at every timestep for all contacts in parallel.
    /// Improves performance and reduces conditionals later on.
    custom_vector<real3> fric_rigid_rigid;
    /// Holds the cohesion value for each contact.
    /// Similar to friction this is precomputed for all contacts in parallel.
    custom_vector<real> coh_rigid_rigid;
    /// Precomputed compliance values for all contacts.
    custom_vector<real4> compliance_rigid_rigid;

    // Object data
    custom_vector<real3> pos_rigid;
    custom_vector<quaternion> rot_rigid;
    custom_vector<char> active_rigid;
    custom_vector<char> collide_rigid;
    custom_vector<real> mass_rigid;

    // Information for 3dof nodes
    custom_vector<real3> pos_3dof;
    custom_vector<real3> sorted_pos_3dof;
    custom_vector<real3> vel_3dof;
    custom_vector<real3> sorted_vel_3dof;

    // Information for FEM nodes
    custom_vector<real3> pos_node_fea;
    custom_vector<real3> vel_node_fea;
    custom_vector<real> mass_node_fea;
    custom_vector<uvec4> tet_indices;

    custom_vector<uvec4> boundary_triangles_fea;
    custom_vector<uint> boundary_node_fea;
    custom_vector<uint> boundary_element_fea;
    custom_vector<short2> boundary_family_fea;

    /// Bilateral constraint type (all supported constraints).
    custom_vector<int> bilateral_type;

    /// Keeps track of active bilateral constraints.
    custom_vector<int> bilateral_mapping;

    // Shaft data
    custom_vector<real> shaft_rot;     ///< shaft rotation angles
    custom_vector<real> shaft_inr;     ///< shaft inverse inertias
    custom_vector<char> shaft_active;  ///< shaft active (not sleeping nor fixed) flags

    // Material properties (NSC)
    custom_vector<real3> fric_data;        ///< friction information (sliding, rolling, spinning)
    custom_vector<real> cohesion_data;     ///< constant cohesion forces (NSC and SMC)
    custom_vector<real4> compliance_data;  ///< compliance (NSC only)

    // Material properties (SMC)
    custom_vector<real2> elastic_moduli;       ///< Young's modulus and Poisson ratio (SMC only)
    custom_vector<real> mu;                    ///< Coefficient of friction (SMC only)
    custom_vector<real> cr;                    ///< Coefficient of restitution (SMC only)
    custom_vector<real4> smc_coeffs;           ///< Stiffness and damping coefficients (SMC only)
    custom_vector<real> adhesionMultDMT_data;  ///< Adhesion multipliers used in DMT model (SMC only)
    // Derjaguin-Muller-Toporov (DMT) model:
    // adhesion = adhesionMult * Sqrt(R_eff). Given the surface energy, w,
    //    adhesionMult = 2 * CH_C_PI * w * Sqrt(R_eff).
    // Given the equilibrium penetration distance, y_eq,
    //    adhesionMult = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)

    /// This matrix, if used will hold D^TxM^-1xD in sparse form.
    CompressedMatrix<real> Nshur;
    /// The D Matrix hold the Jacobian for the entire system.
    CompressedMatrix<real> D;
    /// D_T is the transpose of the D matrix, note that D_T is actually computed
    /// first and D is taken as the transpose. This is due to the way that blaze
    /// handles sparse matrix allocation, it is easier to do it on a per row basis.
    CompressedMatrix<real> D_T;
    /// M_inv is the inverse mass matrix, This matrix, if holding the full inertia
    /// tensor is block diagonal.
    CompressedMatrix<real> M_inv, M;
    /// Minv_D holds M_inv multiplied by D, this is done as a preprocessing step
    /// so that later, when the full matrix vector product is needed it can be
    /// performed in two steps, first R = Minv_D*x, and then D_T*R where R is just
    /// a temporary variable used here for illustrative purposes. In reality the
    /// entire operation happens inline without a temp variable.
    CompressedMatrix<real> M_invD;

    DynamicVector<real> R_full;  ///< The right hand side of the system
    DynamicVector<real> R;       ///< The rhs of the system, changes during solve
    DynamicVector<real> b;       ///< Correction terms
    DynamicVector<real> s;
    DynamicVector<real> M_invk;  ///< Result of M_inv multiplied by vector of forces
    DynamicVector<real> gamma;   ///< The unknowns we are solving for
    DynamicVector<real> v;       ///< This vector holds the velocities for all objects
    DynamicVector<real> hf;      ///< This vector holds h*forces, h is time step
    /// While E is the compliance matrix, in reality it is completely diagonal
    /// therefore it is stored in a vector for performance reasons.
    DynamicVector<real> E;

    // Contact forces (NSC)
    DynamicVector<real> Fc;

    //========Broadphase Data========

    custom_vector<uint> bin_intersections;
    custom_vector<uint> bin_number;
    custom_vector<uint> bin_number_out;
    custom_vector<uint> bin_aabb_number;
    custom_vector<uint> bin_start_index;
    custom_vector<uint> bin_num_contact;
};

/// Global data manager for Chrono::Parallel.
class CH_PARALLEL_API ChParallelDataManager {
  public:
    ChParallelDataManager();
    ~ChParallelDataManager();

    /// Structure that contains the data on the host, the naming convention is
    /// from when the code supported the GPU (host vs device).
    host_container host_data;
    shape_container shape_data;
    /// This pointer is used by the bilarerals for computing the jacobian and other terms.
    std::shared_ptr<ChSystemDescriptor> system_descriptor;

    std::shared_ptr<Ch3DOFContainer> node_container;
    std::shared_ptr<Ch3DOFContainer> fea_container;

    ChConstraintRigidRigid* rigid_rigid;
    ChConstraintBilateral* bilateral;

    collision::ChCBroadphase* broadphase;
    collision::ChCNarrowphaseDispatch* narrowphase;
    collision::ChCAABBGenerator* aabb_generator;

    // These pointers are used to compute the mass matrix instead of filling a
    // a temporary data structure
    std::vector<std::shared_ptr<ChBody>>* body_list;                  ///< List of bodies
    std::vector<std::shared_ptr<ChLinkBase>>* link_list;              ///< List of bilaterals
    std::vector<std::shared_ptr<ChPhysicsItem>>* other_physics_list;  ///< List to other items

    // Indexing variables
    uint num_rigid_bodies;             ///< The number of rigid bodies in a system
    uint num_fluid_bodies;             ///< The number of fluid bodies in the system
    uint num_shafts;                   ///< The number of shafts in a system
    uint num_motors;                   ///< The number of motor links with 1 state variable
    uint num_linmotors;                ///< The number of linear speed motors
    uint num_rotmotors;                ///< The number of rotation speed motors
    uint num_dof;                      ///< The number of degrees of freedom in the system
    uint num_rigid_shapes;             ///< The number of collision models in a system
    uint num_rigid_contacts;           ///< The number of contacts between rigid bodies in a system
    uint num_rigid_fluid_contacts;     ///< The number of contacts between rigid and fluid objects
    uint num_fluid_contacts;           ///< The number of contacts between fluid objects
    uint num_unilaterals;              ///< The number of contact constraints
    uint num_bilaterals;               ///< The number of bilateral constraints
    uint num_constraints;              ///< Total number of constraints
    uint num_fea_nodes;                ///< Total number of FEM nodes
    uint num_fea_tets;                 ///< Total number of FEM nodes
    uint num_rigid_tet_contacts;       ///< The number of contacts between tetrahedron and rigid bodies
    uint num_marker_tet_contacts;      ///< The number of contacts between tetrahedron and fluid markers
    uint num_rigid_tet_node_contacts;  ///< The number of contacts between tetrahedron nodes and rigid bodies
    uint nnz_bilaterals;               ///< The number of non-zero entries in the bilateral Jacobian

    /// Flag indicating whether or not the contact forces are current (NSC only).
    bool Fc_current;
    /// This object hold all of the timers for the system.
    ChTimerParallel system_timer;
    /// Structure that contains all settings for the system, collision detection and the solver.
    settings_container settings;
    measures_container measures;

    /// Material composition strategy.
    std::unique_ptr<ChMaterialCompositionStrategy<real>> composition_strategy;

    /// User-provided callback for overriding coposite material properties.
    ChContactContainer::AddContactCallback* add_contact_callback;

    /// Output a vector (one dimensional matrix) from blaze to a file.
    int OutputBlazeVector(DynamicVector<real> src, std::string filename);
    /// Output a sparse blaze matrix to a file.
    int OutputBlazeMatrix(CompressedMatrix<real> src, std::string filename);
    /// Convenience function that outputs all of the data associated for a system.
    /// This is useful when debugging.
    int ExportCurrentSystem(std::string output_dir);

    void PrintMatrix(CompressedMatrix<real> src);
};

/// @} parallel_module

}  // end namespace chrono
