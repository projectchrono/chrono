// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This class contains manages all data associated with a parallel
// System. Rather than passing in individual data parameters to different parts
// of the code like the collision detection and the solver, passing a pointer to
// a data manager is more convenient from a development perspective.
// =============================================================================

#pragma once

// Chrono Includes
#include "lcp/ChLcpSystemDescriptor.h"
#include "physics/ChBody.h"
#include "physics/ChLinksAll.h"

// Chrono Parallel Includes
#include "chrono_parallel/ChTimerParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real4.h"
#include "chrono_parallel/math/mat33.h"
#include "chrono_parallel/math/other_types.h"
#include "chrono_parallel/ChSettings.h"
#include "chrono_parallel/ChMeasures.h"

// Thrust Includes
#include <thrust/host_vector.h>

// Blaze Includes
#include <blaze/math/CompressedMatrix.h>
#include <blaze/math/DynamicVector.h>
#include <blaze/math/DenseSubvector.h>

using blaze::CompressedMatrix;
using blaze::DynamicVector;
using blaze::SparseSubmatrix;
using blaze::submatrix;
using blaze::subvector;
using thrust::host_vector;

namespace chrono {

/// @addtogroup parallel_module
/// @{

typedef blaze::SparseSubmatrix<CompressedMatrix<real> > SubMatrixType;
typedef blaze::DenseSubvector<DynamicVector<real> > SubVectorType;

typedef blaze::SparseSubmatrix<const CompressedMatrix<real> > ConstSubMatrixType;
typedef blaze::DenseSubvector<const DynamicVector<real> > ConstSubVectorType;

// The maximum number of shear history contacts per smaller body (DEM)
#define max_shear 20

struct host_container {
    // Collision data
    host_vector<real3> ObA_rigid;       // Position of shape
    host_vector<real3> ObB_rigid;       // Size of shape (dims or convex data)
    host_vector<real3> ObC_rigid;       // Rounded size
    host_vector<real4> ObR_rigid;       // Shape rotation
    host_vector<short2> fam_rigid;      // Family information
    host_vector<int> typ_rigid;         // Shape type
    host_vector<real> margin_rigid;     // Inner collision margins
    host_vector<uint> id_rigid;         // Body identifier for each shape
    host_vector<real3> aabb_min_rigid;  // List of bounding boxes minimum point
    host_vector<real3> aabb_max_rigid;  // List of bounding boxes maximum point
    host_vector<real3> convex_data;     // list of convex points

    // Contact data
    host_vector<real3> norm_rigid_rigid;
    host_vector<real3> cpta_rigid_rigid;
    host_vector<real3> cptb_rigid_rigid;
    host_vector<real> dpth_rigid_rigid;
    host_vector<real> erad_rigid_rigid;
    host_vector<int2> bids_rigid_rigid;
    host_vector<long long> pair_rigid_rigid;

    host_vector<real3> norm_rigid_fluid;
    host_vector<real3> cpta_rigid_fluid;
    host_vector<real> dpth_rigid_fluid;
    host_vector<int2> bids_rigid_fluid;

    host_vector<int2> bids_fluid_fluid;

    // Contact forces (DEM)
    // These vectors hold the total contact force and torque, respectively,
    // for bodies that are involved in at least one contact.
    host_vector<real3> ct_body_force;   // Total contact force on bodies
    host_vector<real3> ct_body_torque;  // Total contact torque on these bodies

    // Contact shear history (DEM)
    host_vector<int3> shear_neigh;  // Neighbor list of contacting bodies and shapes
    host_vector<real3> shear_disp;  // Accumulated shear displacement for each neighbor

    // Mapping from all bodies in the system to bodies involved in a contact.
    // For bodies that are currently not in contact, the mapping entry is -1.
    // Otherwise, the mapping holds the appropriate index in the vectors above.
    host_vector<int> ct_body_map;

    // This vector holds the friction information as a triplet
    // x - Sliding friction
    // y - Rolling friction
    // z - Spinning Friction
    // This is precomputed at every timestep for all contacts in parallel
    // Improves performance and reduces conditionals later on
    host_vector<real3> fric_rigid_rigid;
    // Holds the cohesion value for each contact, similar to friction this is
    // precomputed for all contacts in parallel
    host_vector<real> coh_rigid_rigid;

    // Object data
    host_vector<real3> pos_rigid;
    host_vector<real4> rot_rigid;
    host_vector<bool> active_rigid;
    host_vector<bool> collide_rigid;
    host_vector<real> mass_rigid;

    host_vector<real3> pos_fluid;
    host_vector<real3> vel_fluid;
    host_vector<real> den_fluid;

    // Bilateral constraint type (all supported constraints)
    host_vector<int> bilateral_type;

    // keeps track of active bilateral constraints
    host_vector<int> bilateral_mapping;

    // Shaft data
    host_vector<real> shaft_rot;     // shaft rotation angles
    host_vector<real> shaft_inr;     // shaft inverse inertias
    host_vector<bool> shaft_active;  // shaft active (not sleeping nor fixed) flags

    // Material properties (DVI)
    host_vector<real3> fric_data;
    host_vector<real> cohesion_data;  // constant cohesion forces (DEM and DVI)
    host_vector<real4> compliance_data;

    // Material properties (DEM)
    host_vector<real2> elastic_moduli;    // Young's modulus and Poisson ratio
    host_vector<real> mu;                 // Coefficient of friction
    host_vector<real> cr;                 // Coefficient of restitution
    host_vector<real4> dem_coeffs;        // Stiffness and damping coefficients
    host_vector<real> adhesionMultDMT_data;  // adhesion multipliers used in Derjaguin, Muller and Toporov (DMT) model.
    // adhesion = adhesionMult * Sqrt(R_eff). Given the surface energy, w, adhesionMult = 2 * CH_C_PI * w * Sqrt(R_eff).
    // Given the equilibrium penetration distance, y_eq, adhesionMult = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)

    // For the variables below the convention is:
    //_n is normal
    //_t is tangential
    //_s is rolling and spinning
    //_b is bilateral
    //_T is transpose
    //_inv is inverse
    // This matrix, if used will hold D^TxM^-1xD in sparse form
    CompressedMatrix<real> Nshur;
    // The D Matrix hold the Jacobian for the entire system
    CompressedMatrix<real> D_n, D_t, D_s, D_b;
    // D_T is the transpose of the D matrix, note that D_T is actually computed
    // first and D is taken as the transpose. This is due to the way that blaze
    // handles sparse matrix allocation, it is easier to do it on a per row basis
    CompressedMatrix<real> D_n_T, D_t_T, D_s_T, D_b_T;
    // M_inv is the inverse mass matrix, This matrix, if holding the full inertia
    // tensor is block diagonal
    CompressedMatrix<real> M_inv;
    // Minv_D holds M_inv multiplied by D, this is done as a preprocessing step
    // so that later, when the full matrix vector product is needed it can be
    // performed in two steps, first R = Minv_D*x, and then D_T*R where R is just
    // a temporary variable used here for illustrative purposes. In reality the
    // entire operation happens inline without a temp variable.
    CompressedMatrix<real> M_invD_n, M_invD_t, M_invD_s, M_invD_b;

    DynamicVector<real> R_full;  // The right hand side of the system
    DynamicVector<real> R;       // The rhs of the system, changes during solve
    DynamicVector<real> b;       // Correction terms
    DynamicVector<real> s;
    DynamicVector<real> M_invk;  // result of M_inv multiplied by vector of forces
    DynamicVector<real> gamma;   // THe unknowns we are solving for
    DynamicVector<real> v;       // This vector holds the velocities for all objects
    DynamicVector<real> hf;      // This vector holds h*forces, h is time step
    DynamicVector<real> rhs_bilateral;
    // While E is the compliance matrix, in reality it is completely diagonal
    // therefore it is stored in a vector for performance reasons
    DynamicVector<real> E;

    // Contact forces (DVI)
    DynamicVector<real> Fc;
};

class CH_PARALLEL_API ChParallelDataManager {
  public:
    ChParallelDataManager();
    ~ChParallelDataManager();

    // Structure that contains the data on the host, the naming convention is
    // from when the code supported the GPU (host vs device)
    host_container host_data;

    // This pointer is used by the bilarerals for computing the jacobian and other
    // terms
    ChLcpSystemDescriptor* lcp_system_descriptor;

    // These pointers are used to compute the mass matrix instead of filling a
    // a temporary data structure
    std::vector<std::shared_ptr<ChBody> >* body_list;                  // List of bodies
    std::vector<std::shared_ptr<ChLink> >* link_list;                  // List of bilaterals
    std::vector<std::shared_ptr<ChPhysicsItem> >* other_physics_list;  // List to other items

    // Indexing variables
    uint num_rigid_bodies;          // The number of rigid bodies in a system
    uint num_fluid_bodies;          // The number of fluid bodies in the system
    uint num_shafts;                // The number of shafts in a system
    uint num_dof;                   // The number of degrees of freedom in the system
    uint num_rigid_shapes;          // The number of collision models in a system
    uint num_rigid_contacts;        // The number of contacts between rigid bodies in a system
    uint num_rigid_fluid_contacts;  // The number of contacts between rigid and fluid objects
    uint num_fluid_contacts;        // The number of contacts between fluid objects
    uint num_unilaterals;           // The number of contact constraints
    uint num_bilaterals;            // The number of bilateral constraints
    uint num_constraints;           // Total number of constraints
    uint nnz_bilaterals;            // The number of non-zero entries in the bilateral Jacobian

    // Flag indicating whether or not the contact forces are current (DVI only).
    bool Fc_current;
    // This object hold all of the timers for the system
    ChTimerParallel system_timer;
    // Structure that contains all settings for the system, collision detection
    // and the solver
    settings_container settings;
    measures_container measures;

    // Output a vector (one dimensional matrix) from blaze to a file
    int OutputBlazeVector(DynamicVector<real> src, std::string filename);
    // Output a sparse blaze matrix to a file
    int OutputBlazeMatrix(CompressedMatrix<real> src, std::string filename);
    // Convenience function that outputs all of the data associated for a system
    // This is useful when debugging
    int ExportCurrentSystem(std::string output_dir);
};

/// @} parallel_module
}

