
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChFEMContainer::ChFEMContainer(ChSystemParallelDVI* system) {}

ChFEMContainer::~ChFEMContainer() {}

void ChFEMContainer::Initialize() {
    X0.resize(num_tets);

    for (int i = 0; i < num_tets; i++) {
        int4 tet_index = tet_indices[i];

        real3 x0 = nodes[tet_index.x];
        real3 x1 = nodes[tet_index.y];
        real3 x2 = nodes[tet_index.z];
        real3 x3 = nodes[tet_index.w];

        real3 c1 = x1 - x0;
        real3 c2 = x2 - x0;
        real3 c3 = x3 - x0;
        Mat33 D = Mat33(c1, c2, c3);

        X0[i] = Inverse(D);
        V[i] = Determinant(D) / 6.0;

        //        real3 y[4];
        //        y[1] = X0[i].row(0);
        //        y[2] = X0[i].row(1);
        //        y[3] = X0[i].row(2);
        //        y[0] = -y[1] - y[2] - y[3];
    }

    //    ed = yDamping;
    //    nud = pDamping;
    //    real omnd = 1.0 - nud;
    //    real om2nd = 1.0 - 2 * nud;
    //    real fd = ed / (1.0 + nud) / om2nd;
    //    Mat33 Ed = fd * Mat33(omnd, nud, nud,   //
    //                          nud, omnd, nud,   //
    //                          nud, nud, omnd);  //
}

void ChFEMContainer::Build_D() {
    real e = youngs_modulus;
    real nu = poisson_ratio;

    const real mu = 0.5 * e / (1.0 + nu);  // 0.5?
    const real muInv = 1.0 / mu;

    real omn = 1.0 - nu;
    real om2n = 1.0 - 2 * nu;
    real s = e / (1.0 + nu);
    real f = s / om2n;
    Mat33 E = f * Mat33(omn, nu, nu,   //
                        nu, omn, nu,   //
                        nu, nu, omn);  //

    Mat33 Einv = Inverse(E);
}
}  // END_OF_NAMESPACE____

/////////////////////
