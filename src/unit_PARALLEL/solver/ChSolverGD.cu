#include "ChSolverGD.h"
using namespace chrono;


ChSolverCG::ChSolverGD() {
}

void ChSolverGD::Solve(real step, gpu_container &gpu_data_) {
    gpu_data = &gpu_data_;
    step_size = step;
    Setup();
    if (number_of_constraints > 0) {
        ComputeRHS();
        SolveGD(gpu_data->device_gam_data, rhs, 100);
        ComputeImpulses();
        gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
        gpu_data->device_omg_data += gpu_data->device_QUVW_data;
    }
}

uint ChSolverGD::SolveGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
    real eps = step_size;
    custom_vector<real> r;
    r = b - ShurProduct(x);
    real resold = 1, resnew, normb = Norm(b);
    if (normb == 0.0) {
        normb = 1;
    };
    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
        SEAXPY(eps, r, x, x); //x = x + eps *r;
        r = b - ShurProduct(x);
        resnew = Norm(x);
        residual = abs(resnew - resold);
        if (residual < tolerance) {
            break;
        }
        resold = resnew;
    }
    Project(x);
    return current_iteration;
}