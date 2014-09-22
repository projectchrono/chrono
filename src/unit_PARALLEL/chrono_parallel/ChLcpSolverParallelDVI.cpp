#include "chrono_parallel/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

using namespace chrono;

void ChLcpSolverParallelDVI::RunTimeStep(real step) {
   data_container->settings.solver.step_size = step;
   if (data_container->settings.solver.solver_mode == NORMAL) {
      rigid_rigid.offset = 1;
      data_container->num_unilaterals = 1 * data_container->num_contacts;
   } else if (data_container->settings.solver.solver_mode == SLIDING) {
      rigid_rigid.offset = 3;
      data_container->num_unilaterals = 3 * data_container->num_contacts;
   } else if (data_container->settings.solver.solver_mode == SPINNING) {
      rigid_rigid.offset = 6;
      data_container->num_unilaterals = 6 * data_container->num_contacts;
   }
   data_container->num_constraints = data_container->num_unilaterals + data_container->num_bilaterals;

   Preprocess();

   data_container->host_data.rhs_data.resize(data_container->num_constraints);
   data_container->host_data.diag.resize(data_container->num_constraints);
   data_container->host_data.gamma_data.resize(data_container->num_constraints);

#pragma omp parallel for
   for (int i = 0; i < data_container->num_constraints; i++) {
      data_container->host_data.gamma_data[i] = 0;
   }
   if (warm_start) {
      RunWarmStartPreprocess();
   }
   data_container->system_timer.start("ChLcpSolverParallel_Setup");
   rigid_rigid.Setup(data_container);
   bilateral.Setup(data_container);

   solver->current_iteration = 0;
   solver->total_iteration = 0;
   solver->maxd_hist.clear();
   solver->maxdeltalambda_hist.clear();
   solver->iter_hist.clear();

   solver->rigid_rigid = &rigid_rigid;
   solver->bilateral = &bilateral;
   solver->Setup(data_container);
   data_container->system_timer.stop("ChLcpSolverParallel_Setup");
   if (data_container->settings.solver.collision_in_solver) {
      data_container->host_data.vel_new_data = data_container->host_data.vel_data;
      data_container->host_data.omg_new_data = data_container->host_data.omg_data;

      data_container->host_data.pos_new_data = data_container->host_data.pos_data;
      data_container->host_data.rot_new_data = data_container->host_data.rot_data;
   }

   //solve initial
   //solver.SetComplianceParameters(.2, 1e-3, 1e-3);
   //rigid_rigid.solve_sliding = true;
   data_container->system_timer.start("ChLcpSolverParallel_Jacobians");
   rigid_rigid.ComputeJacobians();

   bilateral.ComputeJacobians();
   data_container->system_timer.stop("ChLcpSolverParallel_Jacobians");
   data_container->system_timer.start("ChLcpSolverParallel_RHS");
   bilateral.ComputeRHS();
   data_container->system_timer.stop("ChLcpSolverParallel_RHS");
   custom_vector<real> rhs_bilateral(data_container->num_bilaterals, 0);
   thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->num_unilaterals, data_container->num_bilaterals, rhs_bilateral.begin());

   if (data_container->settings.solver.max_iteration_bilateral > 0) {
      data_container->system_timer.start("ChLcpSolverParallel_Stab");
      //thrust::copy_n(data_container->host_data.gamma_data.begin() + data_container->num_unilaterals, data_container->num_bilaterals, data_container->host_data.gamma_bilateral.begin());
      solver->SolveStab(data_container->settings.solver.max_iteration_bilateral, data_container->num_bilaterals, rhs_bilateral, data_container->host_data.gamma_bilateral);
      data_container->system_timer.stop("ChLcpSolverParallel_Stab");
      thrust::copy_n(data_container->host_data.gamma_bilateral.begin(), data_container->num_bilaterals,
                     data_container->host_data.gamma_data.begin() + data_container->num_unilaterals);
   }
   if (data_container->settings.solver.solver_type != APGD || data_container->settings.solver.solver_type == APGDRS) {
      //std::cout << "Compute N" << std::endl;
      ComputeN();
   }

   //solve normal
   if (data_container->settings.solver.solver_mode == NORMAL || data_container->settings.solver.solver_mode == SLIDING || data_container->settings.solver.solver_mode == SPINNING) {

      if (data_container->settings.solver.max_iteration_normal > 0) {
         solver->SetMaxIterations(data_container->settings.solver.max_iteration_normal);
         rigid_rigid.solve_sliding = false;
         rigid_rigid.solve_spinning = false;
         rigid_rigid.ComputeRHS();
         data_container->system_timer.start("ChLcpSolverParallel_Solve");
         solver->Solve();
         data_container->system_timer.stop("ChLcpSolverParallel_Solve");
      }
   }
   if (data_container->settings.solver.solver_mode != NORMAL) {
      if (data_container->settings.solver.max_iteration_sliding > 0) {
         solver->SetMaxIterations(data_container->settings.solver.max_iteration_sliding);
         rigid_rigid.solve_sliding = true;
         rigid_rigid.solve_spinning = false;
         rigid_rigid.ComputeRHS();
         data_container->system_timer.start("ChLcpSolverParallel_Solve");
         solver->Solve();
         data_container->system_timer.stop("ChLcpSolverParallel_Solve");
      }
   }
   if (data_container->settings.solver.solver_mode == SPINNING) {
      if (data_container->settings.solver.max_iteration_spinning > 0) {
         //cout<<"Solve Full"<<endl;
         solver->SetMaxIterations(data_container->settings.solver.max_iteration_spinning);
         rigid_rigid.solve_sliding = true;
         rigid_rigid.solve_spinning = true;
         rigid_rigid.ComputeRHS();
         data_container->system_timer.start("ChLcpSolverParallel_Solve");
         solver->Solve();
         data_container->system_timer.stop("ChLcpSolverParallel_Solve");
      }
   }
   thrust::copy_n(data_container->host_data.gamma_data.begin() + data_container->num_unilaterals, data_container->num_bilaterals,
                  data_container->host_data.gamma_bilateral.begin());
   //	for (int i = 0; i < data_container->num_bilaterals; i++) {
   //		data_container->host_data.gamma_bilateral[i] *= .5;
   //	}

   //	if (max_iter_bilateral > 0) {
   //		thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->num_unilaterals, data_container->num_bilaterals, rhs_bilateral.begin());
   //		thrust::copy_n(data_container->host_data.gamma_data.begin() + data_container->num_unilaterals, data_container->num_bilaterals,
   //				data_container->host_data.gamma_bilateral.begin());
   //		solver.SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, max_iter_bilateral);
   //		thrust::copy_n(data_container->host_data.gamma_bilateral.begin(), data_container->num_bilaterals,
   //				data_container->host_data.gamma_data.begin() + data_container->num_unilaterals);
   //	}
   solver->ComputeImpulses();

   tot_iterations = solver->GetIteration();
   residual = solver->GetResidual();
   //data_container->host_data.old_gamma_data = data_container->host_data.gamma_data;
   //rhs = data_container->host_data.rhs_data;
   //lambda = data_container->host_data.gam_data;

   for (int i = 0; i < solver->iter_hist.size(); i++) {
      AtIterationEnd(solver->maxd_hist[i], solver->maxdeltalambda_hist[i], solver->iter_hist[i]);
   }
   //if (warm_start) {
   //RunWarmStartPostProcess();
   //}

#if PRINT_LEVEL==2
   cout << "Solve Done: "<<residual << endl;
#endif
   //ChIntegratorGPU integrator;
   //integrator.IntegrateSemiImplicit(step, data_container->gpu_data);
}

void ChLcpSolverParallelDVI::RunWarmStartPostProcess() {
   if (data_container->num_contacts == 0) {
      return;
   }

   int3 num_bins_per_axis = I3(20, 40, 20);
   int l = num_bins_per_axis.x;
   int h = num_bins_per_axis.y;
   int w = num_bins_per_axis.z;
   uint N = data_container->num_contacts;

   thrust::host_vector<real3> points(N);
   thrust::host_vector<uint> counter((l + 1) * (w + 1) * (h + 1), 0);

   data_container->host_data.bin_number.resize((l + 1) * (w + 1) * (h + 1));
   thrust::fill(data_container->host_data.bin_number.begin(), data_container->host_data.bin_number.end(), 0);

   points = (data_container->host_data.cpta_rigid_rigid + data_container->host_data.cptb_rigid_rigid);
   points = .5 * points;

   real3 max_point = R3(-FLT_MAX);
   real3 min_point = R3(FLT_MAX);
   real3 origin;

   for (int i = 0; i < N; i++) {
      max_point.x = std::max(points[i].x, max_point.x);
      max_point.y = std::max(points[i].y, max_point.y);
      max_point.z = std::max(points[i].z, max_point.z);

      min_point.x = std::min(points[i].x, min_point.x);
      min_point.y = std::min(points[i].y, min_point.y);
      min_point.z = std::min(points[i].z, min_point.z);
   }

   origin = min_point;
   real3 bin_size_vec = (fabs(max_point - origin));
   bin_size_vec.x = bin_size_vec.x / real(l);
   bin_size_vec.y = bin_size_vec.y / real(h);
   bin_size_vec.z = bin_size_vec.z / real(w);

   for (int i = 0; i < N; i++) {
      points[i] = points[i] - origin;

      int3 temp;
      temp.x = floor(points[i].x / bin_size_vec.x);
      temp.y = floor(points[i].y / bin_size_vec.y);
      temp.z = floor(points[i].z / bin_size_vec.z);
      data_container->host_data.bin_number[temp.x + temp.y * w + temp.z * w * h] += data_container->host_data.gamma_data[i];
      counter[temp.x + temp.y * w + temp.z * w * h]++;
   }
   for (int i = 0; i < counter.size(); i++) {
      if (counter[i] > 0) {
         data_container->host_data.bin_number[i] = data_container->host_data.bin_number[i] / counter[i];
      }
   }
}

void ChLcpSolverParallelDVI::RunWarmStartPreprocess() {
   if (data_container->num_contacts == 0) {
      return;
   }

   if (data_container->host_data.old_pair_rigid_rigid.size() != 0) {

      //		cout << "SORTEDN: " << thrust::is_sorted(data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end())<<endl;
      //		cout << "SORTEDO: " << thrust::is_sorted(data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end())<<endl;
      //		return;

      thrust::host_vector<long long> res1(data_container->host_data.pair_rigid_rigid.size());

      uint numP = thrust::set_intersection(thrust::omp::par, data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end(),
                                           data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end(), res1.begin())
            - res1.begin();  //list of persistent contacts

      if (numP > 0) {
         res1.resize(numP);
         thrust::host_vector<uint> temporaryA(numP);
         thrust::host_vector<uint> temporaryB(numP);

         thrust::lower_bound(thrust::omp::par, data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end(), res1.begin(),
                             res1.end(), temporaryA.begin());     //return index of common new contact
         thrust::lower_bound(thrust::omp::par, data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end(), res1.begin(), res1.end(),
                             temporaryB.begin());     //return index of common new contact

         uint num_contacts = data_container->num_contacts;
         uint old_num_contacts = data_container->old_num_contacts;
#pragma omp parallel for
         for (int i = 0; i < numP; i++) {

            M33 contact_plane_old;

            {
               real3 U = data_container->host_data.old_norm_rigid_rigid[temporaryA[i]];

               if (U == R3(0, 0, 0)) {
                  U = R3(1, 0, 0);
               } else {
                  U = normalize(U);
               }
               U = normalize(U);
               real3 W = cross(U, R3(0, 1, 0));
               real mzlen = length(W);

               if (mzlen < 0.0001) {     // was near singularity? change singularity reference vector!
                  real3 mVsingular = R3(0, 1, 0);
                  if (mVsingular.x < 0.9) {
                     mVsingular = R3(0, 0, 1);
                  }
                  if (mVsingular.y < 0.9) {
                     mVsingular = R3(0, 1, 0);
                  }
                  if (mVsingular.z < 0.9) {
                     mVsingular = R3(1, 0, 0);
                  }

                  W = cross(U, mVsingular);
                  mzlen = length(W);
               }
               W = W * 1.0 / mzlen;
               real3 V = cross(W, U);

               contact_plane_old.U = U;
               contact_plane_old.V = V;
               contact_plane_old.W = W;
            }

            real3 old_gamma = R3(0), old_gamma_spinning = R3(0);
            old_gamma.x = data_container->host_data.old_gamma_data[temporaryA[i] + old_num_contacts * 0];
            old_gamma.y = data_container->host_data.old_gamma_data[temporaryA[i] + old_num_contacts * 1];
            old_gamma.z = data_container->host_data.old_gamma_data[temporaryA[i] + old_num_contacts * 2];

            old_gamma_spinning.x = data_container->host_data.old_gamma_data[temporaryA[i] + old_num_contacts * 3];
            old_gamma_spinning.y = data_container->host_data.old_gamma_data[temporaryA[i] + old_num_contacts * 4];
            old_gamma_spinning.z = data_container->host_data.old_gamma_data[temporaryA[i] + old_num_contacts * 5];

            real3 global_gamma = MatMult(contact_plane_old, old_gamma);
            real3 global_gamma_spin = MatMult(contact_plane_old, old_gamma_spinning);

            M33 contact_plane_new;

            {
               real3 U = data_container->host_data.norm_rigid_rigid[temporaryB[i]];

               if (U == R3(0, 0, 0)) {
                  U = R3(1, 0, 0);
               } else {
                  U = normalize(U);
               }
               U = normalize(U);
               real3 W = cross(U, R3(0, 1, 0));
               real mzlen = length(W);

               if (mzlen < 0.0001) {     // was near singularity? change singularity reference vector!
                  real3 mVsingular = R3(0, 1, 0);
                  if (mVsingular.x < 0.9) {
                     mVsingular = R3(0, 0, 1);
                  }
                  if (mVsingular.y < 0.9) {
                     mVsingular = R3(0, 1, 0);
                  }
                  if (mVsingular.z < 0.9) {
                     mVsingular = R3(1, 0, 0);
                  }

                  W = cross(U, mVsingular);
                  mzlen = length(W);
               }
               W = W * 1.0 / mzlen;
               real3 V = cross(W, U);

               contact_plane_new.U = U;
               contact_plane_new.V = V;
               contact_plane_new.W = W;
            }
            real3 new_gamma = MatTMult(contact_plane_new, global_gamma);
            real3 new_gamma_spin = MatTMult(contact_plane_new, global_gamma_spin);

            data_container->host_data.gamma_data[temporaryB[i] + num_contacts * 0] = new_gamma.x * .4;
            data_container->host_data.gamma_data[temporaryB[i] + num_contacts * 1] = new_gamma.y * .4;
            data_container->host_data.gamma_data[temporaryB[i] + num_contacts * 2] = new_gamma.z * .4;

            data_container->host_data.gamma_data[temporaryB[i] + num_contacts * 3] = new_gamma_spin.x * .4;
            data_container->host_data.gamma_data[temporaryB[i] + num_contacts * 4] = new_gamma_spin.y * .4;
            data_container->host_data.gamma_data[temporaryB[i] + num_contacts * 5] = new_gamma_spin.z * .4;

         }
      }
   }

   //
   //	num_bins_per_axis = I3(20,40,20);
   //	int l = num_bins_per_axis.x;
   //	int h = num_bins_per_axis.y;
   //	int w = num_bins_per_axis.z;
   //	uint N = data_container->num_contacts;
   //
   //	thrust::host_vector<real3> points(N);
   //
   //	points = (data_container->host_data.cpta_rigid_rigid + data_container->host_data.cptb_rigid_rigid);
   //	points = .5 * points;
   //
   //	real3 max_point = R3(-FLT_MAX);
   //	real3 min_point = R3(FLT_MAX);
   //
   //	for (int i = 0; i < N; i++) {
   //		max_point.x = max(points[i].x, max_point.x);
   //		max_point.y = max(points[i].y, max_point.y);
   //		max_point.z = max(points[i].z, max_point.z);
   //
   //		min_point.x = min(points[i].x, min_point.x);
   //		min_point.y = min(points[i].y, min_point.y);
   //		min_point.z = min(points[i].z, min_point.z);
   //	}
   //
   //	origin = min_point;
   //	bin_size_vec = (fabs(max_point - origin));
   //	bin_size_vec.x = bin_size_vec.x / real(l);
   //	bin_size_vec.y = bin_size_vec.y / real(h);
   //	bin_size_vec.z = bin_size_vec.z / real(w);
   //
   //	for (int i = 0; i < N; i++) {
   //		points[i] = points[i] - origin;
   //
   //		int3 temp;
   //		temp.x = floor(points[i].x / bin_size_vec.x);
   //		temp.y = floor(points[i].y / bin_size_vec.y);
   //		temp.z = floor(points[i].z / bin_size_vec.z);
   //		data_container->host_data.gamma_data[i] = data_container->host_data.bin_number[temp.x + temp.y * w + temp.z * w * h];
   //
   //	}

}

void ChLcpSolverParallelDVI::ComputeN() {
   if (data_container->num_constraints > 0) {
      data_container->host_data.D.reset();
      data_container->host_data.Nshur.reset();
      data_container->host_data.M_inv.reset();

      data_container->host_data.M_inv.resize(data_container->num_bodies * 6, data_container->num_bodies * 6);
      data_container->host_data.M_inv.reserve(data_container->num_bodies * 6);
      data_container->host_data.D.resize(data_container->num_bodies * 6, data_container->num_constraints);
      if (data_container->settings.solver.solver_mode == NORMAL) {
         data_container->host_data.D.reserve(data_container->num_contacts * 6 * 3 + data_container->num_bilaterals * 6 * 2);
      } else if (data_container->settings.solver.solver_mode == SLIDING) {
         data_container->host_data.D.reserve(data_container->num_contacts * 6 * 6 + data_container->num_bilaterals * 6 * 2);
      } else if (data_container->settings.solver.solver_mode == SPINNING) {
         data_container->host_data.D.reserve(data_container->num_contacts * 6 * 9 + data_container->num_bilaterals * 6 * 2);
      }

      rigid_rigid.Build_D(data_container->settings.solver.solver_mode);
      bilateral.Build_D();

      for (int i = 0; i < data_container->num_bodies; i++) {
         if (data_container->host_data.active_data[i]) {
            data_container->host_data.M_inv.append(i * 6 + 0, i * 6 + 0, data_container->host_data.mass_data[i]);
            data_container->host_data.M_inv.finalize(i * 6 + 0);
            data_container->host_data.M_inv.append(i * 6 + 1, i * 6 + 1, data_container->host_data.mass_data[i]);
            data_container->host_data.M_inv.finalize(i * 6 + 1);
            data_container->host_data.M_inv.append(i * 6 + 2, i * 6 + 2, data_container->host_data.mass_data[i]);
            data_container->host_data.M_inv.finalize(i * 6 + 2);
            data_container->host_data.M_inv.append(i * 6 + 3, i * 6 + 3, data_container->host_data.inr_data[i].x);
            data_container->host_data.M_inv.finalize(i * 6 + 3);
            data_container->host_data.M_inv.append(i * 6 + 4, i * 6 + 4, data_container->host_data.inr_data[i].y);
            data_container->host_data.M_inv.finalize(i * 6 + 4);
            data_container->host_data.M_inv.append(i * 6 + 5, i * 6 + 5, data_container->host_data.inr_data[i].z);
            data_container->host_data.M_inv.finalize(i * 6 + 5);
         }
      }
      data_container->host_data.D_T = trans(data_container->host_data.D);
      data_container->host_data.M_invD = data_container->host_data.M_inv * data_container->host_data.D;
      //data_container->host_data.Nshur = data_container->host_data.D_T * data_container->host_data.M_invD;
//      for (int i = 0; i < data_container->host_data.Nshur.rows(); i++) {
//         for (int j = 0; j < data_container->host_data.Nshur.columns(); j++) {
//            std::cout << data_container->host_data.Nshur(i, j) << std::endl;
//         }
//      }
//      std::cout<<"\n"<<std::endl;
//      for (int i = 0; i < data_container->host_data.D.rows(); i++) {
//         for (int j = 0; j < data_container->host_data.D.columns(); j++) {
//            std::cout <<"("<<i<<","<<j<<") = "<< data_container->host_data.D(i, j) << std::endl;
//         }
//      }


   }
}
