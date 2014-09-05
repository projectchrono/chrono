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


#ifndef CH_DATAMANAGER_H
#define CH_DATAMANAGER_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChTimerParallel.h"
#include <blaze/math/CompressedMatrix.h>
using blaze::CompressedMatrix;
namespace chrono {

struct host_container {
   //collision data
   thrust::host_vector<real3> ObA_rigid;
   thrust::host_vector<real3> ObB_rigid;
   thrust::host_vector<real3> ObC_rigid;
   thrust::host_vector<real4> ObR_rigid;
   thrust::host_vector<int2> fam_rigid;
   thrust::host_vector<int> typ_rigid;
   thrust::host_vector<uint> id_rigid;
   thrust::host_vector<real3> aabb_rigid;

   //contact data
   thrust::host_vector<real3> norm_rigid_rigid;
   thrust::host_vector<real3> old_norm_rigid_rigid;
   thrust::host_vector<real3> cpta_rigid_rigid;
   thrust::host_vector<real3> cptb_rigid_rigid;
   thrust::host_vector<real> dpth_rigid_rigid;
   thrust::host_vector<real> erad_rigid_rigid;
   thrust::host_vector<int2> bids_rigid_rigid;
   thrust::host_vector<long long> pair_rigid_rigid;
   thrust::host_vector<long long> old_pair_rigid_rigid;

   thrust::host_vector<real3> fric_rigid_rigid;

   thrust::host_vector<real> gamma_data;
   thrust::host_vector<real> old_gamma_data;
   thrust::host_vector<real> dgm_data;

   //object data
   thrust::host_vector<real3> vel_data, vel_new_data;
   thrust::host_vector<real3> omg_data, omg_new_data;
   thrust::host_vector<real3> pos_data, pos_new_data;
   thrust::host_vector<real4> rot_data, rot_new_data;
   thrust::host_vector<real3> inr_data;
   thrust::host_vector<real3> frc_data;
   thrust::host_vector<real3> trq_data;
   thrust::host_vector<real3> acc_data;
   thrust::host_vector<bool> active_data;
   thrust::host_vector<bool> collide_data;
   thrust::host_vector<real> mass_data;

   thrust::host_vector<real3> lim_data;
   thrust::host_vector<real3> dem_data;
   thrust::host_vector<real3> gyr_data;
   thrust::host_vector<real> pressure_data;
   thrust::host_vector<real> bin_number;

   // Material properties (DVI)
   thrust::host_vector<real3> fric_data;
   thrust::host_vector<real> cohesion_data;
   thrust::host_vector<real4> compliance_data;

   // Material properties (DEM)
   thrust::host_vector<real2> elastic_moduli;   // Young's modulus and Poisson ratio
   thrust::host_vector<real> mu;               // Coefficient of friction
   thrust::host_vector<real> alpha;            // Dissipation factor (Hunt-Crossley)
   thrust::host_vector<real> cr;               // Coefficient of restitution

   thrust::host_vector<real> rhs_data;
   thrust::host_vector<real> diag;
   thrust::host_vector<real3> QXYZ_data, QUVW_data;

   thrust::host_vector<real3> JXYZA_bilateral, JXYZB_bilateral;
   thrust::host_vector<real3> JUVWA_bilateral, JUVWB_bilateral;
   thrust::host_vector<real> residual_bilateral;
   thrust::host_vector<real> correction_bilateral;
   thrust::host_vector<int2> bids_bilateral;
   thrust::host_vector<real> gamma_bilateral;


   CompressedMatrix<real> Nshur, D, D_T, M_inv, M_invD;


//			thrust::host_vector<real3> JXYZA_fluid_fluid, JXYZB_fluid_fluid;
//			thrust::host_vector<real3> JXYZA_rigid_fluid, JXYZB_rigid_fluid, JUVWB_rigid_fluid;
//
//			thrust::host_vector<real3> fluid_pos, fluid_vel, fluid_force;
//			thrust::host_vector<real> fluid_mass, fluid_density;
//			thrust::host_vector<real3> aabb_fluid;
};

class CH_PARALLEL_API ChParallelDataManager {
 public:
   ChParallelDataManager();
   ~ChParallelDataManager();

   host_container host_data;

   //Indexing variables
   uint num_bodies;
   uint num_models;
   uint num_contacts;
   uint old_num_contacts;
   uint num_unilaterals;
   uint num_bilaterals;
   uint num_constraints;

   //Collision variables
   real3 min_bounding_point, max_bounding_point;
   real collision_envelope;

   // Flag indicating whether or not effective contact radius is calculated
   bool erad_is_set;

   //Solver variables
   real step_size;
   real alpha;
   real contact_recovery_speed;

   ChTimerParallel system_timer;

   //real fluid_rad;
};
}

#endif
