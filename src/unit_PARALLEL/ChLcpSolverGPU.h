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
                lcp_contact_factor = .6;
                lcp_omega_bilateral = .1;
            }
            ~ChLcpSolverGPU() {
            }
            virtual double Solve(ChLcpSystemDescriptor &sysd, bool add_Mq_to_f = false) {
                return 0;
            }

            void RunTimeStep(real step, gpu_container &gpu_data);
            void Preprocess(gpu_container &gpu_data);
            void Iterate(gpu_container &gpu_data);
            void Reduce(gpu_container &gpu_data);
            void Integrate(gpu_container &gpu_data);
            void WarmContact(const int &i);
            real Max_DeltaGamma(device_vector<real> &device_dgm_data);
            real Min_DeltaGamma(device_vector<real> &device_dgm_data);
            real Avg_DeltaGamma(uint number_of_constraints, device_vector<real> &device_dgm_data);
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


