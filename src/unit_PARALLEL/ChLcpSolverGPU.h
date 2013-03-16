#ifndef CHC_LCPITERATIVESOLVERGPU_H
#define CHC_LCPITERATIVESOLVERGPU_H

//////////////////////////////////////////////////
//
//   ChIterativeGPU.h
//
//   GPU LCP Solver
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChDataManager.h"
#include "lcp/ChLcpIterativeSolver.h"

namespace chrono {
    class ChApiGPU ChLcpSolverGPU: public ChLcpIterativeSolver {
        public:
            ChLcpSolverGPU() {
                lcp_contact_factor = 1;
                lcp_omega_bilateral = .1;
                tolerance=1e-7;
            }
            ~ChLcpSolverGPU() {
            }
            virtual double Solve(ChLcpSystemDescriptor &sysd, bool add_Mq_to_f = false) {
                return 0;
            }
            void host_ContactJacobians(real3* norm,
                    real3* ptA,
                    real3* ptB,
                    int2* ids,
                    real4* rot,
                    real3* pos,
                    real3* JXYZA,
                    real3* JXYZB,
                    real3* JUVWA,
                    real3* JUVWB);
            void host_process_contacts(real3* JXYZA,
                    real3* JXYZB,
                    real3* JUVWA,
                    real3* JUVWB,
                    real *contactDepth,
                    int2 *ids,
                    real3 *G,
                    real *dG,
                    real *mass,
                    real *fric,
                    real3 *inertia,
                    real4 *rot,
                    real3 *vel,
                    real3 *omega,
                    real3 *pos,
                    real3 *updateV,
                    real3 *updateO,
                    uint *offset);
            void host_Bilaterals(
                real4 *bilaterals,
                real *mass,
                real3 *inertia,
                real4 *rot,
                real3 *vel,
                real3 *omega,
                real3 *pos,
                real3 *updateV,
                real3 *updateO,
                uint *offset,
                real *dG);
            void host_addForces(bool* active, real *mass, real3 *inertia, real3 *forces, real3 *torques, real3 *vel, real3 *omega);
            void host_Reduce_Speeds(bool *active,real * mass, real3 *vel, real3 *omega, real3 *updateV, real3 *updateO, uint *d_body_num, uint *counter, real3 *fap);
            void host_ComputeGyro(real3 *omega, real3 *inertia, real3 *gyro, real3 *torque);
            void host_Offsets(int2 *ids, real4 *bilaterals, uint *Body);
            void host_Integrate_Timestep(bool *active, real3 *acc, real4 *rot, real3 *vel, real3 *omega, real3 *pos, real3 *lim);


            void RunTimeStep(real step, gpu_container &gpu_data);
            void Preprocess(gpu_container &gpu_data);
            void Iterate(gpu_container &gpu_data);
            void Reduce(gpu_container &gpu_data);
            void Integrate(gpu_container &gpu_data);
            real Max_DeltaGamma(custom_vector<real> &device_dgm_data);
            real Min_DeltaGamma(custom_vector<real> &device_dgm_data);
            real Avg_DeltaGamma(uint number_of_constraints, custom_vector<real> &device_dgm_data);
            real Total_KineticEnergy(gpu_container &gpu_data);

            uint GetIterations() {
                return iteration_number;
            }
            void SetCompliance(real c, real cT, real a) {
                compliance = c;
                complianceT = cT;
                alpha = a;
            }
            void SetContactFactor(real f) {
                lcp_contact_factor = f;
            }
            void SetOmegaBilateral(double mval) {
                if (mval > 0.) lcp_omega_bilateral = mval;
            }
            double GetOmegaSetOmegaBilateral() {
                return lcp_omega_bilateral;
            }
        private:
            unsigned int iteration_number;
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
            cudaEvent_t start, stop;
    };
}

#endif


