#ifndef CHSOLVERCG_H
#define CHSOLVERCG_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChOptimizationJacobi.h"
#include "ChSolverGPU.h"
#include "ChDataManager.h"
namespace chrono {
class ChApiGPU ChSolverCG: public ChSolverGPU {
    public:
        ChSolverCG();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
        void host_shurA(real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, int2 *ids, real *inv_mass, real3 *inv_inertia, bool *active, real3 *QXYZ, real3 *QUVW);
        void host_shurB(real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, int2 *ids, real *inv_mass, real3 *inv_inertia, bool *active, real3 *QXYZ, real3 *QUVW, real *AX);
        void host_RHS(int2 *ids, real *inv_mass, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *vel, real3 *omega, real *correction, real  step_size, real *rhs);
        void host_Project(real *gam, real *fric, int2 *ids);
        custom_vector<real> ShurProduct(const custom_vector<real> &x_t);
        uint number_of_constraints, number_of_contacts;

        custom_vector<int2> temp_bids;
        custom_vector<real> AX, rhs;
        real step_size;
        gpu_container *gpu_data;
};
}

#endif
