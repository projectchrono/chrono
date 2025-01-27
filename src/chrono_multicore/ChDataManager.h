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
// Description: This class contains manages all data associated with a multicore
// system. Rather than passing in individual data parameters to different parts
// of the code like the collision detection and the solver, passing a pointer to
// a data manager is more convenient from a development perspective.
//
// =============================================================================

#pragma once

#include <memory>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/multicore_math/ChMulticoreMath.h"
#include "chrono/collision/multicore/ChCollisionData.h"

#include "chrono_multicore/ChTimerMulticore.h"
#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono_multicore/ChSettings.h"
#include "chrono_multicore/ChMeasures.h"

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

// Forward declarations

class ChBody;
class ChLink;
class ChPhysicsItem;
class real2;
class real3;
class real4;
class vec3;
class Ch3DOFContainer;
class ChFluidContainer;
class ChConstraintRigidRigid;
class ChConstraintBilateral;
class ChContactMaterialCompositionStrategy;

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
#define _num_r_c_ data_manager->cd_data->num_rigid_contacts
#define _num_rf_c_ data_manager->cd_data->num_rigid_fluid_contacts
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
// #define _GAMMAN_ subvector(_gamma_, 0, _num_r_c_)
// #define _GAMMAT_ submatrix(_gamma_, _num_r_c_, 2 * _num_r_c_)
// #define _GAMMAS_ submatrix(_gamma_, 3 * _num_r_c_, 3 * _num_r_c_)
//// Bilateral
// #define _GAMMAB_ submatrix(_gamma_, _num_uni_,  _num_bil_)
//// Rigid Fluid
// #define _GAMMARFN_ submatrix(_gamma_, _num_uni_ + _num_bil_ _num_rf_c_)
// #define _GAMMARFT_ submatrix(_gamma_, _num_uni_ + _num_bil_ + _num_rf_c_, 2 * _num_rf_c_)
//// Density
// #define _GAMMAFFD_ submatrix(_gamma_,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_, _num_fluid_)
//// Viscosity
// #define _GAMMAFFV_ submatrix(_gamma_,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_,  3 * _num_fluid_)

// The maximum number of shear history contacts per smaller body (SMC)
#define max_shear 20

/// @addtogroup multicore_module
/// @{

/// Structure of arrays containing simulation data.
struct host_container {
    // Contact forces (SMC)
    // These vectors hold the contact forces and torques for each individual contact. For each contact, the force and
    // torque are given at the body origin, expressed in the absolute frame. These vectors include the force and torque
    // for each of the two bodies involved in a contact.
    custom_vector<real3> ct_force;   ///< Contact forces per contact
    custom_vector<real3> ct_torque;  ///< Contact torques per contact
    // These vectors hold the resultant contact force and torque for each body in contact, accumulating over all
    // contacts that the body is involved in. The force and torque are given at the body origin, expresed in the
    // absolute frame. for bodies that are involved in at least one contact.
    custom_vector<real3> ct_body_force;   ///< Total contact force on bodies
    custom_vector<real3> ct_body_torque;  ///< Total contact torque on these bodies

    // Contact shear history (SMC)
    custom_vector<vec3> shear_neigh;          ///< Neighbor list of contacting bodies and shapes
    custom_vector<real3> shear_disp;          ///< Accumulated shear displacement for each neighbor
    custom_vector<real> contact_relvel_init;  ///< Initial relative normal velocity manitude per contact pair
    custom_vector<real> contact_duration;     ///< Accumulated contact duration, per contact pair

    /// Mapping from all bodies in the system to bodies involved in a contact.
    /// For bodies that are currently not in contact, the mapping entry is -1.
    /// Otherwise, the mapping holds the appropriate index in the vectors above.
    custom_vector<int> ct_body_map;

    /// This vector holds the friction information (composite material) as a triplet:
    /// x - Sliding friction,
    /// y - Rolling friction,
    /// z - Spinning Friction.
    /// This is precomputed at every timestep for all contacts in parallel.
    /// Improves performance and reduces conditionals later on.
    // Used for both NSC and SMC contacts.
    custom_vector<real3> fric_rigid_rigid;

    /// Holds the cohesion value (composite material) for each contact.
    /// Similar to friction this is precomputed for all contacts in parallel.
    /// Used for NSC only.
    custom_vector<real> coh_rigid_rigid;

    /// Precomputed compliance (composite material) values for all contacts.
    /// Used for NSC only.
    custom_vector<real4> compliance_rigid_rigid;

    // Precomputed composite material quantities (SMC)
    custom_vector<real2> modulus_rigid_rigid;   ///< E_eff and G_eff
    custom_vector<real3> adhesion_rigid_rigid;  ///< adhesion_eff, adhesionMultDMT_eff, and adhesionSPerko_eff
    custom_vector<real> cr_rigid_rigid;         ///< cr_eff (effective coefficient of restitution)
    custom_vector<real4> smc_rigid_rigid;       ///< kn, kt, gn, gt

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

    /// Bilateral constraint type (all supported constraints)
    custom_vector<int> bilateral_type;

    /// Keeps track of active bilateral constraints.
    custom_vector<int> bilateral_mapping;

    // Shaft data
    custom_vector<real> shaft_rot;     ///< shaft rotation angles
    custom_vector<real> shaft_inr;     ///< shaft inverse inertias
    custom_vector<char> shaft_active;  ///< shaft active (not sleeping nor fixed) flags

    // Material properties (NSC, only for fluid-rigid and FEA-rigid contacts)
    custom_vector<float> sliding_friction;  ///< sliding coefficients of friction
    custom_vector<float> cohesion;          ///< constant cohesion forces

    /// This matrix, if used will hold D^TxM^-1xD in sparse form.
    CompressedMatrix<real> Nschur;
    /// The D Matrix hold the Jacobian for the entire system.
    CompressedMatrix<real> D;
    /// D_T is the transpose of the D matrix, note that D_T is actually computed
    /// first and D is taken as the transpose. This is due to the way that blaze
    /// handles sparse matrix allocation, it is easier to do it on a per row basis.
    CompressedMatrix<real> D_T;
    /// Mass matrix; if holding the full inertia tensor, M is block diagonal.
    CompressedMatrix<real> M;
    /// M_inv is the inverse mass matrix; if holding the full inertia tensor, M_inv is block diagonal.
    CompressedMatrix<real> M_inv;
    /// M_invD holds M_inv multiplied by D. This is done as a preprocessing step
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
    DynamicVector<real> v;       ///< This vector holds the velocities for all objects
    DynamicVector<real> hf;      ///< This vector holds h*forces, h is time step

    /// Contact impulses. These are the unknowns solved for in the NSC formulation.
    /// Depending on the selected SolverMode, gamma is organized as follows (N is the number of rigid contacts):
    /// \li NORMAL [size(gamma) = N]\n
    ///     n1 n2 ... nN
    /// \li SLIDING [size(gamma) = 3N]\n
    ///     n1 n2 ... nN | u1 v1 u2 v2 ... uN vN
    /// \li SPINNING [size(gamma) = 6N]\n
    ///     n1 n2 ... nN | u1 v1 u2 v2 ... uN vN | tn1 tu1 tv1 tn2 tu2 tv2 ... tnN tuN tvN
    ///
    /// If there are any bilateral constraints, the corresponding impulses are stored at the end of `gamma`.
    DynamicVector<real> gamma;

    /// Compliance matrix elements.
    /// Note that E is a diagonal matrix and hence stored in a vector.
    DynamicVector<real> E;

    DynamicVector<real> Fc;  ///< Contact forces (NSC)
};

/// Global data manager for Chrono::Multicore.
class CH_MULTICORE_API ChMulticoreDataManager {
  public:
    ChMulticoreDataManager();
    ~ChMulticoreDataManager();

    host_container host_data;  ///< Structure of data arrays (state, contact, etc)

    /// Used by the bilarerals for computing the Jacobian and other terms.
    std::shared_ptr<ChSystemDescriptor> system_descriptor;

    std::shared_ptr<Ch3DOFContainer> node_container;  ///< container of 3-DOF particles

    ChConstraintRigidRigid* rigid_rigid;  ///< methods for unilateral constraints
    ChConstraintBilateral* bilateral;     ///< methods for bilateral constraints

    std::shared_ptr<ChCollisionData> cd_data;  ///< shared data for the Chrono collision system

    // These pointers are used to compute the mass matrix instead of filling a temporary data structure
    std::vector<std::shared_ptr<ChBody>>* body_list;                  ///< List of bodies
    std::vector<std::shared_ptr<ChLinkBase>>* link_list;              ///< List of bilaterals
    std::vector<std::shared_ptr<ChPhysicsItem>>* other_physics_list;  ///< List to other items

    // Indexing variables
    uint num_rigid_bodies;  ///< The number of rigid bodies in a system
    uint num_fluid_bodies;  ///< The number of fluid bodies in the system
    uint num_shafts;        ///< The number of shafts in a system
    uint num_motors;        ///< The number of motor links with 1 state variable
    uint num_linmotors;     ///< The number of linear speed motors
    uint num_rotmotors;     ///< The number of rotation speed motors
    uint num_dof;           ///< The number of degrees of freedom in the system
    uint num_unilaterals;   ///< The number of contact constraints
    uint num_bilaterals;    ///< The number of bilateral constraints
    uint num_constraints;   ///< Total number of constraints
    uint nnz_bilaterals;    ///< The number of non-zero entries in the bilateral Jacobian

    /// Flag indicating whether or not the contact forces are current (NSC only).
    bool Fc_current;
    /// Container for all timers for the system.
    ChTimerMulticore system_timer;
    /// Container for all settings for the system, collision detection, and solver.
    settings_container settings;
    /// Container for various statistics for collision detection and solver.
    measures_container measures;

    /// Material composition strategy.
    std::unique_ptr<ChContactMaterialCompositionStrategy> composition_strategy;

    /// User-provided callback for overriding composite material properties.
    std::shared_ptr<ChContactContainer::AddContactCallback> add_contact_callback;

    /// Output a vector (one dimensional matrix) from blaze to a file.
    int OutputBlazeVector(DynamicVector<real> src, std::string filename);
    /// Output a sparse blaze matrix to a file.
    int OutputBlazeMatrix(CompressedMatrix<real> src, std::string filename);
    /// Utility debugging function that outputs all of the data associated for a system.
    int ExportCurrentSystem(std::string output_dir);

    /// Print a sparse blaze matrix.
    void PrintMatrix(CompressedMatrix<real> src);
};

/// @} multicore_module

}  // end namespace chrono
