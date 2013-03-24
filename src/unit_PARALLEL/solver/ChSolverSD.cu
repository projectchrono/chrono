#include "ChSolverSD.h"
using namespace chrono;


ChSolverCG::ChSolverSD() {
}

void ChSolverSD::Solve(real step, gpu_container &gpu_data_) {
    gpu_data = &gpu_data_;
    step_size = step;
    Setup();
    if (number_of_constraints > 0) {
        ComputeRHS();
        SolveSD(gpu_data->device_gam_data, rhs, 100);
        ComputeImpulses();
        gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
        gpu_data->device_omg_data += gpu_data->device_QUVW_data;
    }
}

uint ChSolverSD::SolveSD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
    custom_vector<real> r;
    r = b - ShurProduct(x);
    real resold = 1, resnew, normb = Norm(b), alpha;
    if (normb == 0.0) {
        normb = 1;
    }
    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
        alpha = Dot(r, r) / Dot(r, ShurProduct(r));
        SEAXPY(alpha, r, x, x); //x = x + eps *r;
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
