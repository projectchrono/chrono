#ifndef CHSOLVERGPU_H
#define CHSOLVERGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChDataManager.h"

namespace chrono {
class ChApiGPU ChSolverGPU {
    public:
        ChSolverGPU() {
            tolerance = 1e-6;
            epsilon = 1e-3;
            alpha = .2;
            compliance = 1e-4;
            complianceT = 1e-4;
        }
        void Setup();
        void Project(custom_vector<real> & gamma);
        void shurA(custom_vector<real> &x);
        void shurB(custom_vector<real> &x);

        void ComputeRHS();
        void ComputeImpulses();

        void host_shurA(int2 *ids, bool *active, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma,  real3 *QXYZ, real3 *QUVW);
        void host_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia,real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX);
        void host_RHS(int2 *ids, real *correction, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs);
        void host_Project(int2 *ids, real *friction, real *gamma);
        custom_vector<real> ShurProduct( custom_vector<real> &x_t);



        void Solve() {}

        int GetIteration() {
            return current_iteration;
        }
        real GetResidual() {
            return residual;
        }

    protected:

        real step_size;
        real lcp_omega_bilateral;
        real lcp_omega_contact;
        real lcp_contact_factor;
        real compliance;
        real complianceT;
        real alpha;
        uint number_of_bilaterals;
        uint number_of_contacts;
        uint number_of_objects;
        uint number_of_updates;
        uint number_of_constraints;

        int current_iteration, max_iteration;
        real residual, epsilon, tolerance;

        custom_vector<int2> temp_bids;
        custom_vector<real> AX, rhs, correction;
        gpu_container *gpu_data;
};

}

#endif
