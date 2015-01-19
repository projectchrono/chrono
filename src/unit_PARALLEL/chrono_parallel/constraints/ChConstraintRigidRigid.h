#ifndef CHCONSTRAINT_RIGIDRIGID_H
#define CHCONSTRAINT_RIGIDRIGID_H

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

CH_PARALLEL_API
void Orthogonalize(real3& U, real3& V, real3& W);

CH_PARALLEL_API
void Compute_Jacobian(const real4& quat, const real3& U, const real3& V, const real3& W, const real3& point, real3& T1, real3& T2, real3& T3);

CH_PARALLEL_API
void Compute_Jacobian_Rolling(const real4& quat, const real3& U, const real3& V, const real3& W, real3& T1, real3& T2, real3& T3);

class CH_PARALLEL_API ChConstraintRigidRigid {
 public:
  ChConstraintRigidRigid() {
    solve_sliding = false;
    solve_spinning = false;
    offset = 3;
    inv_h = inv_hpa = inv_hhpa = 0;
  }

  ~ChConstraintRigidRigid() {}

  void Setup(ChParallelDataManager* data_container_)
  {
    data_container = data_container_;
    uint num_contacts = data_container->num_contacts;
    inv_h = 1 / data_container->settings.step_size;
    inv_hpa = 1 / (data_container->settings.step_size + data_container->settings.solver.alpha);
    inv_hhpa = inv_h * inv_hpa;

    if (num_contacts > 0) {
      contact_active_pairs.resize(int(num_contacts));
      contact_rotation.resize(int(num_contacts * 2));
      data_container->host_data.coh_rigid_rigid.resize(num_contacts);
      data_container->host_data.fric_rigid_rigid.resize(num_contacts);

#pragma omp parallel for
      for (int i = 0; i < num_contacts; i++) {
        int2 body = data_container->host_data.bids_rigid_rigid[i];
        uint b1 = body.x;
        uint b2 = body.y;

        contact_active_pairs[i] = bool2(data_container->host_data.active_data[b1], data_container->host_data.active_data[b2]);

        contact_rotation[i] = data_container->host_data.rot_data[b1];
        contact_rotation[i + num_contacts] = data_container->host_data.rot_data[b2];

        real coh = std::max((data_container->host_data.cohesion_data[b1] + data_container->host_data.cohesion_data[b2]) * .5, 0.0);
        data_container->host_data.coh_rigid_rigid[i] = coh;

        real3 f_a = data_container->host_data.fric_data[b1];
        real3 f_b = data_container->host_data.fric_data[b2];
        real3 mu;

        mu.x = (f_a.x == 0 || f_b.x == 0) ? 0 : (f_a.x + f_b.x) * .5;
        mu.y = (f_a.y == 0 || f_b.y == 0) ? 0 : (f_a.y + f_b.y) * .5;
        mu.z = (f_a.z == 0 || f_b.z == 0) ? 0 : (f_a.z + f_b.z) * .5;

        data_container->host_data.fric_rigid_rigid[i] = mu;
      }
    }
    solve_sliding = false;
    solve_spinning = false;
  }

  void host_Project_single(int index, int2* ids, real3* friction, real* cohesion, real* gamma);

  void host_Project(int2* ids, real3* friction, real* cohesion, real* gamma);

  void Project(real* gamma);
  void Project_NoPar(real* gamma);
  void Project_Single(int index, real* gamma);

  void func_Project(int& index, int2* ids, real3* fric, real* cohesion, real* gam);

  void func_Project_rolling(int& index, int2* ids, real3* fric, real* gam);

  void host_ComputeS(int2* ids, real3* mu, bool2* active, real3* norm, real3* vel, real3* omega, real3* ptA, real3* ptB, real4* rot, const real* rhs, real* b);

  void ComputeS(const custom_vector<real>& rhs, custom_vector<real3>& vel_data, custom_vector<real3>& omg_data, custom_vector<real>& b);

  void Build_D(SOLVERMODE solver_mode);
  void Build_b(SOLVERMODE solver_mode);
  void Build_E(SOLVERMODE solver_mode);
  void GenerateSparsity(SOLVERMODE solver_mode);

  bool solve_sliding;
  bool solve_spinning;
  int offset;

 protected:
  custom_vector<bool2> contact_active_pairs;
  custom_vector<real4> contact_rotation;

  real inv_h;
  real inv_hpa;
  real inv_hhpa;

  // Pointer to the system's data manager
  ChParallelDataManager *data_container;
};

}

#endif
