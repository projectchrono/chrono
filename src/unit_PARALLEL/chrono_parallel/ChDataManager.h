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
#include "parallel/ChOpenMP.h"

#include "core/ChFileutils.h"
#include "core/ChStream.h"

using blaze::CompressedMatrix;
namespace chrono {
struct collision_settings {
   collision_settings() {
      max_body_per_bin = 50;
      min_body_per_bin = 25;
      use_aabb_active = 0;
      collision_envelope = 0;
      bins_per_axis = I3(20, 20, 20);
      narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
      //edge_radius = 0.1;
   }

   //Collision variables
   real3 min_bounding_point, max_bounding_point;
   real collision_envelope;
   bool use_aabb_active;
   int min_body_per_bin;
   int max_body_per_bin;
   real3 aabb_min, aabb_max;
   int3 bins_per_axis;
   NARROWPHASETYPE narrowphase_algorithm;
   //real edge_radius;

};

struct solver_settings {

   solver_settings() {
      tolerance = 1e-4;
      tolerance_objective = 1e-6;
      collision_in_solver = false;
      update_rhs = false;
      verbose = false;
      test_objective = false;

      alpha = .2;
      contact_recovery_speed = .6;
      bilateral_clamp_speed = .6;
      clamp_bilaterals = true;
      perform_stabilization = false;
      collision_in_solver = false;

      max_iteration = 100;
      max_iteration_normal = 0;
      max_iteration_sliding = 100;
      max_iteration_spinning = 0;
      max_iteration_bilateral = 100;

      solver_type = APGD;
      solver_mode = SLIDING;
      step_size = .01;
   }

   SOLVERTYPE solver_type;
   SOLVERMODE solver_mode;
   real alpha;

   real contact_recovery_speed;
   real bilateral_clamp_speed;
   bool clamp_bilaterals;
   bool perform_stabilization;
   bool collision_in_solver;
   bool update_rhs;
   bool verbose;
   bool test_objective;

   uint max_iteration;
   uint max_iteration_normal;
   uint max_iteration_sliding;
   uint max_iteration_spinning;
   uint max_iteration_bilateral;

   //Solver variables
   real step_size;
   real tolerance;
   real tolerance_objective;

};

struct settings_container {

   settings_container() {
      min_threads = 1;
      max_threads = CHOMPfunctions::GetNumProcs();
      // Only perform thread tuning if max threads is greater than min_threads;
      // I don't really check to see if max_threads is > than min_threads
      // not sure if that is a huge issue
      perform_thread_tuning = ((min_threads == max_threads) ? false : true);
      perform_bin_tuning = true;
      system_type = SYSTEM_DVI;
      bin_perturb_size = 2; //increase or decrease the number of bins by 2
      bin_tuning_frequency = 20; //bins will be tuned every 20 frames
   }

   //CD Settings
   collision_settings collision;
   //Solver Settings
   solver_settings solver;
   //System Settings
   bool perform_thread_tuning;
   bool perform_bin_tuning;
   int bin_perturb_size;
   int bin_tuning_frequency;
   int min_threads;
   int max_threads;
   SYSTEMTYPE system_type;

};
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

   // Flag indicating whether or not effective contact radius is calculated
   bool erad_is_set;

   ChTimerParallel system_timer;

   settings_container settings;

   //real fluid_rad;

   int OutputBlazeVector(blaze::DynamicVector<real> src,
       std::string filename) {

     const char* numformat = "%.16g";
     ChStreamOutAsciiFile stream(filename.c_str());
     stream.SetNumFormat(numformat);

     for (int i = 0; i < src.size(); i++)
       stream << src[i] << "\n";

     return 0;
   }

   int OutputBlazeMatrix(blaze::CompressedMatrix<real> src,
       std::string filename) {

     const char* numformat = "%.16g";
     ChStreamOutAsciiFile stream(filename.c_str());
     stream.SetNumFormat(numformat);

     stream << src.rows() << " " << src.columns() << "\n";
     for (int i = 0; i < src.rows(); ++i) {
       for (CompressedMatrix<real>::Iterator it = src.begin(i); it != src.end(i);
           ++it) {
         stream << i << " " << it->index() << " " << it->value() << "\n";
       }
     }

     return 0;
   }

   int ExportCurrentSystem(std::string output_dir)
   {
     int offset = 0;
     if (settings.solver.solver_mode == NORMAL) {
       offset = num_contacts;
     } else if (settings.solver.solver_mode == SLIDING) {
       offset = 3*num_contacts;
     } else if (settings.solver.solver_mode == SPINNING) {
       offset = 6*num_contacts;
     }

     // fill in the vector for r (rhs vector)
     blaze::DynamicVector<real> r;
     r.resize(num_constraints);
     for (int i = 0; i < r.size(); i++) {
       r[i] = host_data.rhs_data[i];
     }

     //fill in b vector
     blaze::DynamicVector<real> b(num_constraints, 0.0);
     for (int i = 0; i < num_contacts; i++) {
       if (settings.solver.solver_mode == NORMAL) {
         b[i] = host_data.dpth_rigid_rigid[i]/settings.solver.step_size;
       } else if (settings.solver.solver_mode == SLIDING) {
         b[3*i  ] = host_data.dpth_rigid_rigid[i]/settings.solver.step_size;
         b[3*i+1] = 0.0;
         b[3*i+2] = 0.0;
       } else if (settings.solver.solver_mode == SPINNING) {
         b[6*i  ] = host_data.dpth_rigid_rigid[i]/settings.solver.step_size;
         b[6*i+1] = 0.0;
         b[6*i+2] = 0.0;
         b[6*i+3] = 0.0;
         b[6*i+4] = 0.0;
         b[6*i+5] = 0.0;
       }
     }
     for (int i = 0; i < host_data.correction_bilateral.size(); i++) {
       b[i+offset] = host_data.residual_bilateral[i];
     }

     //fill in the information for constraints and friction
     blaze::DynamicVector<real> fric(num_constraints, -2.0);
     for (int i = 0; i < num_contacts; i++) {
       if (settings.solver.solver_mode == NORMAL) {
         fric[i] = host_data.fric_rigid_rigid[i].x;
       } else if (settings.solver.solver_mode == SLIDING) {
         fric[3*i  ] = host_data.fric_rigid_rigid[i].x;
         fric[3*i+1] = -1;
         fric[3*i+2] = -1;
       } else if (settings.solver.solver_mode == SPINNING) {
         fric[6*i  ] = host_data.fric_rigid_rigid[i].x;
         fric[6*i+1] = -1;
         fric[6*i+2] = -1;
         fric[6*i+3] = -1;
         fric[6*i+4] = -1;
         fric[6*i+5] = -1;
       }
     }

     // output r
     const std::string filename = output_dir + "dump_r.dat";
     OutputBlazeVector(r, filename);

     // output b
     filename = output_dir + "dump_b.dat";
     OutputBlazeVector(b, filename);

     // output friction data
     filename = output_dir + "dump_fric.dat";
     OutputBlazeVector(fric, filename);

     // output D_T
     filename = output_dir + "dump_D.dat";
     OutputBlazeMatrix(host_data.D_T, filename);

     // output M_inv
     filename = output_dir + "dump_Minv.dat";
     OutputBlazeMatrix(host_data.M_inv, filename);

     return 0;
   }
};
}

#endif
