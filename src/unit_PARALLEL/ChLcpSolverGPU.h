#ifndef CHC_LCPITERATIVESOLVERGPU_H
#define CHC_LCPITERATIVESOLVERGPU_H

//////////////////////////////////////////////////
//
//   ChIterativeGPU.h
//
//   GPU LCP Solver
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ChCuda.h"
#include "ChDataManager.h"
#include "lcp/ChLcpIterativeSolver.h"
#define HM_REAL float
#define HM_REAL3 float3
#define HM_REAL4 float4

namespace chrono {
class ChApiGPU ChLcpSolverGPU: public ChLcpIterativeSolver {
    public:
        ChLcpSolverGPU() {
            lcp_contact_factor = .6;
            lcp_omega_bilateral = .1;
        }
        ~ChLcpSolverGPU() {
        }
        virtual double Solve(ChLcpSystemDescriptor& sysd, bool add_Mq_to_f = false) {
            return 0;
        }

        void RunTimeStep(float step, gpu_container & gpu_data);
        void Preprocess(gpu_container & gpu_data);
        void Iterate(gpu_container & gpu_data);
        void Reduce(gpu_container & gpu_data);
        void Integrate(gpu_container & gpu_data);
        void WarmContact(const int & i);
        float Max_DeltaGamma(device_vector<float> &device_dgm_data);
        float Min_DeltaGamma(device_vector<float> &device_dgm_data);
        float Avg_DeltaGamma(uint number_of_constraints, device_vector<float> &device_dgm_data);
        float Total_KineticEnergy(gpu_container & gpu_data);

        uint GetIterations() {
            return iteration_number;
        }
        void SetCompliance(float c, float cT, float a) {
            compliance = c;
            complianceT = cT;
            alpha = a;
        }
        void SetContactFactor(float f) {
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
        float step_size;
        float lcp_omega_bilateral;
        float lcp_omega_contact;
        float lcp_contact_factor;
        float compliance;
        float complianceT;
        float alpha;
        uint number_of_bilaterals;
        uint number_of_contacts;
        uint number_of_objects;
        uint number_of_updates;
        uint number_of_constraints;
        cudaEvent_t start, stop;
};
}

#endif
