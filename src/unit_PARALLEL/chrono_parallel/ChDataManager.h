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
#include "chrono_parallel/ChSettings.h"
#include <blaze/math/CompressedMatrix.h>

using blaze::CompressedMatrix;
using blaze::DynamicVector;
namespace chrono {

struct host_container {
   //collision data
   thrust::host_vector<real3> ObA_rigid;
   thrust::host_vector<real3> ObB_rigid;
   thrust::host_vector<real3> ObC_rigid;
   thrust::host_vector<real4> ObR_rigid;
   thrust::host_vector<short2> fam_rigid;
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

   //object data
   thrust::host_vector<real3> vel_data, vel_new_data;
   thrust::host_vector<real3> omg_data, omg_new_data;
   thrust::host_vector<real3> pos_data, pos_new_data;
   thrust::host_vector<real4> rot_data, rot_new_data;
   thrust::host_vector<real3> inr_data;
   thrust::host_vector<real3> frc_data;
   thrust::host_vector<real3> trq_data;
   thrust::host_vector<bool> active_data;
   thrust::host_vector<bool> collide_data;
   thrust::host_vector<real> mass_data;

   // shaft data
   thrust::host_vector<real> shaft_rot;    // shaft rotation angles
   thrust::host_vector<real> shaft_omg;    // shaft angular velocities
   thrust::host_vector<real> shaft_trq;    // shaft torques
   thrust::host_vector<real> shaft_inr;    // shaft inverse inertias
   thrust::host_vector<bool> shaft_active; // shaft active (not sleeping nor fixed) flags

   thrust::host_vector<real3> lim_data;
   thrust::host_vector<real3> gyr_data;
   thrust::host_vector<real> bin_number;

   // Material properties (DVI)
   thrust::host_vector<real3> fric_data;
   thrust::host_vector<real> cohesion_data;
   thrust::host_vector<real4> compliance_data;

   // Material properties (DEM)
   thrust::host_vector<real2> elastic_moduli;  // Young's modulus and Poisson ratio
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

   //This matrix, if used will hold D^TxM^-1xD in sparse form
   CompressedMatrix<real> Nshur;
   //The D Matrix hold the Jacobian for the entire system
   CompressedMatrix<real> D;
   //D_T is the transpose of the D matrix, note that D_T is actually computed
   //first and D is taken as the transpose. This is due to the way that blaze
   //handles sparse matrix alloction, it is easier to do it on a per row basis
   CompressedMatrix<real> D_T;
   //M_inv is the inver mass matrix, This matrix, if holding the full inertia
   //tensor is block diagonal
   CompressedMatrix<real> M_inv;
   //Minv_D holds M_inv multiplied by D, this is done as a preprocessing step
   //so that later, when the full matrix vector product is needed it can be
   //performed in two steps, first R = Minv_D*x, and then D_T*R where R is just
   //a temporary variable used here for illustrative purposes. In reality the
   //entire operation happens inline without a temp variable.
   CompressedMatrix<real> M_invD;

   DynamicVector<real> R; //The right hand side of the system
   DynamicVector<real> b; //Correction terms
   DynamicVector<real> M_invk; //result of M_inv multiplied by vector of forces
   DynamicVector<real> gamma; //THe unknowns we are solving for
   DynamicVector<real> v; //This vector holds the velocities for all objects
   DynamicVector<real> hf;//This vector holds h*forces, h is timestep
   DynamicVector<real> rhs_bilateral;
   //While E is the compliance matrix, in reality it is completely diagonal
   //therefore it is stored in a vector for performance reasons
   DynamicVector<real> E;
   //DynamicVector<real> gamma_bilateral;

};

class CH_PARALLEL_API ChParallelDataManager {
 public:
   ChParallelDataManager();
   ~ChParallelDataManager();

   host_container host_data;

   //Indexing variables
   uint num_bodies;			//The number of objects in a system
   uint num_models;			//The number of collision models in a system
   uint num_contacts;		//The number of contacts in a system
   uint old_num_contacts;	//The number of contacts during the previous step
   uint num_unilaterals;	//The number of contact constraints
   uint num_bilaterals;		//The number of bilateral constraints
   uint num_constraints;	//Total number of constraints
   uint num_shafts;			//The number of shafts in teh system

   // Flag indicating whether or not effective contact radius is calculated
   bool erad_is_set;
   //This object hold all of the timers for the system
   ChTimerParallel system_timer;
   //Structure that contains all settings for the system, collision detection
   //and the solver
   settings_container settings;

   //Output a vector (one dimensional matrix) from blaze to a file
   int OutputBlazeVector(blaze::DynamicVector<real> src, std::string filename);
   //Output a sparse blaze matrix to a file
   int OutputBlazeMatrix(blaze::CompressedMatrix<real> src,std::string filename);
   //Convenience funtion that outputs all of the data associted for a system
   //This is useful when debugging
   int ExportCurrentSystem(std::string output_dir);
};
}

#endif
