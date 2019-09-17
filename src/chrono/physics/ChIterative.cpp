// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "chrono/physics/ChIterative.h"

namespace chrono {

//: Transpose Free Quasi-Minimal Residual
//
//  Transpose free QMR. First solve Q_1 A Q_2 x = Q_1 b. Then,
//  return z which is Q_2 x. Here Q1 and Q2 are preconditioners.
//  Suppose M is about equal to A and M = M_1 * M_2, then
//  Q_1 = M_1^{-1} and Q_2 = M_2^{-1}
//  <p>
//  The residual holds |b - A * x_m| < sqrt{m+1} * tau_m.
//  The algorithm check the latter to see if convergence arrives instead of
//  checking real residual.
//  <p>
//<table align=center border=1>
// <tr><td> return value </td>   <td>   meaning </td> </tr>
// <tr><td>      0   </td><td>   convergence within maximum iterations </td> </tr>
// <tr><td>      1   </td><td>     no convergence after maximum iterations</td> </tr>
//  <tr><td>     2  </td><td>      breakdown in       tau </td> </tr>
//  <tr><td>     3  </td><td>      breakdown in       alpha </td> </tr>
//  <tr><td>     4  </td><td>      breakdown in       gamma</td> </tr>
//  <tr><td>     5  </td><td>      breakdown in       rho</td> </tr>
// </table>
//
//  <p>
//  See: R. W. Freund, A Transpose-Free Quasi-Minimal Residual algorithm for
//  non-Hermitian linear system. SIAM J. on Sci. Comp. 14(1993), pp. 470-482
//
//!category: itl,algorithms
//!component: function
//!definition: tfqmr.h
//!tparam: Matrix or multiplier for matrix free methods
//!tparam: Vector
//!tparam: Vector
//!tparam: Preconditioner -  Incomplete LU, Incomplete LU with threshold, SSOR or identity_preconditioner.
//!tparam: Iteration - Controls the stopping criteria
//

// template <class ChMatrix<>, class ChMatrix<>, class MatrixB, class Precond1, class Precond2, class Iteration>

int ch_iterative_TFQMR(ChVectorDynamic<>& x,
                       const ChVectorDynamic<>& b,
                       void (*SolveAX)(ChVectorDynamic<>& inX, ChVectorDynamic<>& outB, void* userdata),
                       void (*M1_solve)(ChVectorDynamic<>& eIn, ChVectorDynamic<>& eOut, void* userdata),
                       void (*M2_solve)(ChVectorDynamic<>& eIn, ChVectorDynamic<>& eOut, void* userdata),
                       double min_kappa,
                       int max_iterations,
                       int& iter,
                       int& error_code,
                       void* userdata) {
    // using namespace mtl;

    // typedef typename ChMatrix<>::value_type double;

    auto N = x.size();

    iter = 0;

    ChVectorDynamic<> tmp(N), tmpb(N), r0(N), v(N);

    double sigma, alpha, c, kappa, beta;

    ChVectorDynamic<> h(N);

    // x is initial value

    // 1. r0 = Q1 (b - A Q2 x)
    // M2.solve(x, r0);
    if (M2_solve) {
        M2_solve(x, r0, userdata);
    } else {
        r0 = x;
    }

    SolveAX(r0, tmpb, userdata);  //  +++++++  mult(A, r0, tmp);
    tmp = b - tmpb;

    // M1.solve(tmp, r0);
    if (M1_solve) {
        M1_solve(tmp, r0, userdata);
    } else {
        r0 = tmp;
    }

    // 2. w=y=r
    ChVectorDynamic<> w = r0;
    ChVectorDynamic<> y1 = r0;

    // 3. g=v=Q1AQ2y
    // M2.solve(y1, v);
    if (M2_solve) {
        M2_solve(y1, v, userdata);
    } else {
        v = y1;
    }

    SolveAX(v, tmp, userdata);  // ++++++++ mult(A, v, tmp);

    //  M1.solve(tmp, v);
    if (M1_solve) {
        M1_solve(tmp, v, userdata);
    } else {
        v = tmp;
    }

    ChVectorDynamic<> g = v;

    // 4. d=0
    ChVectorDynamic<> d(N);
    d.setZero();

    // 5. tau=||r||2
    double tau = r0.norm();

    // 6. theta=eta=0
    double theta = 0.0;
    double eta = 0.0;

    // 7. rtilde=r
    ChVectorDynamic<> rtilde = r0;

    // 8. rho=dot(rtilde,r)
    double rho = rtilde.dot(r0);
    double rho0 = rho;
    ChVectorDynamic<> y0(N);
    for (;;) {
        // 9. 10. 11.
        // sigma=dot(rtilde,v)
        // alpha=rho/sigma
        // y2k=y(2k-1)-alpha*v
        sigma = rtilde.dot(v);

        if (sigma == 0.) {
            error_code = 5; /*iter.fail(5, "tfqmr breakdown: sigma=0");*/
            break;
        }
        alpha = rho / sigma;

        y0 = y1 - alpha * v;

        // 12. h=Q1*A*Q2*y
        // M2.solve(y0, h);
        if (M2_solve) {
            M2_solve(y0, h, userdata);
        } else {
            y0 = h;
        }

        SolveAX(h, tmp, userdata);  // mult(A, h, tmp);

        // M1.solve(tmp, h);
        if (M1_solve) {
            M1_solve(tmp, h, userdata);
        } else {
            h = tmp;
        }

        // split the loop of "for m = 2k-1, 2k"

        // The first one
        // 13. w=w-alpha*Q1AQ2y0
        w -= alpha * g;

        // 18. d=y0+((theta0^2)*eta0/alpha)*d         //need check breakdown
        if (alpha == 0.) {
            error_code = 3; /*iter.fail(3, "tfqmr breakdown: alpha=0");*/
            break;
        }

        d = y1 + (theta * theta * eta / alpha) * d;

        // 14. theta=||w||_2/tau0       //need check breakdown
        if (tau == 0.) {
            error_code = 2; /*iter.fail(2, "tfqmr breakdown: tau=0"); */
            break;
        }
        theta = w.norm() / tau;

        // 15. c=1/sqrt(1+theta^2)
        c = 1. / std::sqrt(1. + theta * theta);

        // 16. tau=tau0*theta*c
        tau = tau * c * theta;

        // 17.  eta=(c^2)*alpha
        eta = c * c * alpha;

        // 19. x=x+eta*d
        x += eta * d;

        // 20. kappa=tau*sqrt(m+1)
        kappa = tau * std::sqrt(2. * (iter + 1));  // ALEX

        // 21. check stopping criterion
        if ((kappa < min_kappa) || (iter > max_iterations)) {
            // before return, transform x to the solution of Ax = b
            // M2.solve(x, tmp);
            if (M2_solve) {
                M2_solve(x, tmp, userdata);
            } else {
                tmp = x;
            }

            x = tmp;  // copy(tmp, x);

            if (kappa < min_kappa)  // ALEX
                error_code = 0;
            if (iter > max_iterations)
                error_code = 1;

            break;
        }

        // g = h;
        g = h;  // copy(h, g);

        //---THE SECOND ONE---

        // 13. w=w-alpha*Q1AQ2y0
         w -= alpha * g;

        // 18. d=y0+((theta0^2)*eta0/alpha)*d
        if (alpha == 0.) {
            error_code = 3; /*iter.fail(3,"tfqmr breakdown: alpha=0");*/
            break;
        }

        d = y0 + ( theta * theta * eta / alpha ) * d;

        // 14. theta=||w||_2/tau0
        if (tau == 0.) {
            error_code = 2; /*iter.fail(2, "tfqmr breakdown: tau=0");*/
            break;
        }
        theta = w.norm() / tau;

        // 15. c=1/sqrt(1+theta^2)
        c = 1. / std::sqrt(1. + theta * theta);

        // 16. tau=tau0*theta*c
        tau = tau * c * theta;

        // 17.  eta=(c^2)*alpha
        eta = c * c * alpha;

        // 19. x=x+eta*d
        x += eta * d;

        // 20. kappa=tau*sqrt(m+1)
        kappa = tau * sqrt(2. * (iter + 1) + 1.);  // ALEX

        // 21. check stopping criterion
        if ((kappa < min_kappa) || (iter > max_iterations)) {
            // M2.solve(x, tmp);
            if (M2_solve) {
                M2_solve(x, tmp, userdata);
            } else {
                tmp = x;
            }

            x = tmp;  // copy(tmp, x);

            if (kappa < min_kappa)  // ALEX
                error_code = 0;
            if (iter > max_iterations)
                error_code = 1;

            break;
        }

        // 22. rho=dot(rtilde,w)
        // 23. beta=rho/rho0                     //need check breakdown
        rho0 = rho;
        rho = rtilde.dot(w);
        if (rho0 == 0.) {
            error_code = 4; /*iter.fail(4, "tfqmr breakdown: beta=0");*/
            break;
        }
        beta = rho / rho0;

        // 24. y=w+beta*y0
        y1 = w + beta * y0;

        // 25. g=Q1AQ2y
        // g = Q1 * ( A * ( Q2 * y1) );
        // M2.solve(y1, g);
        if (M2_solve) {
            M2_solve(y1, g, userdata);
        } else {
            g = y1;
        }

        SolveAX(g, tmp, userdata);  // mult(A, g, tmp);

        // M1.solve(tmp, g);
        if (M1_solve) {
            M1_solve(tmp, g, userdata);
        } else {
            g = tmp;
        }

        // 26. v=Q1AQ2y+beta*(Q1AQ2y0+beta*v)
        v = g + beta * ( h + beta * v );

        ++iter;
    }

    return error_code;
}

void __easy_prodeval(ChVectorDynamic<>& inX, ChVectorDynamic<>& outB, void* userdata) {
    ChMatrixDynamic<>* mA = (ChMatrixDynamic<>*)userdata;
    outB = (*mA) * inX;  // b = A*x    residual
}

int ch_iterative_TFQMR_easy(ChMatrixDynamic<>& A, ChVectorDynamic<>& x, const ChVectorDynamic<>& b, double mkappa, int max_iterations) {
    if (A.cols() != A.rows())
        return 6;
    if (A.rows() != b.size())
        return 6;
    if (A.rows() != x.size())
        return 6;

    int merr, miters;

    return ch_iterative_TFQMR(x, b, __easy_prodeval, NULL, NULL, mkappa, max_iterations, miters, merr, &A);
}

}  // end namespace chrono
