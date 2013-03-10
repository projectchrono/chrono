#include "ChSolverJacobi.h"
using namespace chrono;



ChSolverJacobi::ChSolverJacobi() {}

void ChSolverJacobi::Solve(ChOptimizationJacobi& optim) {
    custom_vector<real> dx;
    int n = optim.b.size();
    custom_vector<real> xnew = optim.x;
    optim.InvNDiag();

    for (current_iteration = 0; current_iteration < max_iteration; current_iteration++) {
        optim.inv_n_diag*(optim.b - optim.LinearOperator());
        dx = Abs(xnew - optim.x);
        real err = Norm(dx);
        real relerr = err / (Norm(xnew) + epsilon);
        optim.x = xnew;
        optim.Project();

        if ((err < tolerance) || (relerr < tolerance)) {
            return;
        }
    }
}
