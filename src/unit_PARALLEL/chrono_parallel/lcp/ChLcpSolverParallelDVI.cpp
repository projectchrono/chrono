#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

using namespace chrono;

void ChLcpSolverParallelDVI::RunTimeStep(real step) {
   data_container->settings.step_size = step;
   if (data_container->settings.solver.solver_mode == NORMAL) {
      rigid_rigid.offset = 1;
      data_container->num_unilaterals = 1* data_container->num_contacts;
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

   ComputeN();


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

#if PRINT_LEVEL==2
   cout << "Solve Done: "<<residual << endl;
#endif
   //ChIntegratorGPU integrator;
   //integrator.IntegrateSemiImplicit(step, data_container->gpu_data);
}



void ChLcpSolverParallelDVI::ComputeN() {
   if (data_container->num_constraints > 0) {
      data_container->host_data.D_T.reset();
      data_container->host_data.Nshur.reset();
      data_container->host_data.M_inv.reset();

      data_container->host_data.M_inv.resize(data_container->num_bodies * 6, data_container->num_bodies * 6);
      data_container->host_data.M_inv.reserve(data_container->num_bodies * 6);
      data_container->host_data.D_T.resize(data_container->num_constraints, data_container->num_bodies * 6);
      if (data_container->settings.solver.solver_mode == NORMAL) {
         data_container->host_data.D_T.reserve(data_container->num_contacts * 6 * 2 + data_container->num_bilaterals * 6 * 2);
      } else if (data_container->settings.solver.solver_mode == SLIDING) {
         data_container->host_data.D_T.reserve(data_container->num_contacts * 6 * 6 + data_container->num_bilaterals * 6 * 2);
      } else if (data_container->settings.solver.solver_mode == SPINNING) {
         data_container->host_data.D_T.reserve(data_container->num_contacts * 6 * 9 + data_container->num_bilaterals * 6 * 2);
      }
      //std::cout << data_container->host_data.D_T.rows() << " " << data_container->host_data.D_T.columns() << std::endl;
      rigid_rigid.Build_D(data_container->settings.solver.solver_mode);
      bilateral.Build_D();

      // std::cout<<data_container->host_data.cohesion_data.size()<<std::endl;
      // std::cout<<data_container->num_bodies<<std::endl;

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
      data_container->host_data.D = trans(data_container->host_data.D_T);
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
