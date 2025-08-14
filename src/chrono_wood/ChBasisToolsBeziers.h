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
// Authors: Erol Lale, Jibril B. Coulibaly
// =============================================================================

#ifndef CHC_BASIS_TOOLS_BEZIERS_H
#define CHC_BASIS_TOOLS_BEZIERS_H


#include <cmath>
#include <vector>

#include "chrono_wood/ChWoodApi.h"
#include "chrono/core/ChMatrix.h"

using namespace chrono;

namespace chrono {
namespace wood {



/// Tools for evaluating basis functions for BEZIER, parametrized with parameter u in [-1, 1] (as lines)
/// These bases are often called "R" in literature.

class ChWoodApi ChBasisToolsBeziers { // TODO JBC: This should just be a namespace, not a class!
public:

    // Bernstein Polynomials and derivatives for x in [0,1]. x = (1+u)/2 for u in [-1, 1]
    // No parameter checks: calling function responsible for passing vector of correct size
    // Unroll recursion since we limit our cases to fairly simple and known cases p in [1,3]

    // Bernstein Polynomials
    static void BernsteinPolynomials(double x, ChVectorDynamic<>& B){
        switch (B.size()-1) {
            case 0:
                B(0) = 0.0;
                break;
            case 1:
                B(0) = 1.0 - x;
                B(1) =  x;
                break;
            case 2:
                B(0) = (1.0 - x) * (1.0 - x);
                B(1) = 2.0 * (1.0 - x) * x;
                B(2) = x * x;
                break;
            case 3:
                B(0) = (1.0 - x) * (1.0 - x) * (1.0 - x);
                B(1) = 3.0 * (1.0 - x) * (1.0 - x) * x;
                B(2) = 3.0 * (1.0 - x) * x * x;
                B(3) = x * x * x;
                break;
        }
    }

    // First derivatives of Bernstein Polynomials
    // Chain rule: dB/du = dB/dx * dx/du, with dx/du = 0.5.
    // Doing dBdx *= dx/du outside this function has shown to be markedly slower (something with Eigen?)
    // Instead, we pass a dx/du as a parameter and multiply each entry of dB/du
    // TODO JBC: Consider re-writing the Bernstein polynomial in function of u directly
    static void BernsteinPolynomialsDerivative1(double x, ChVectorDynamic<>& dBdx, double dxdu = 0.5){
        switch (dBdx.size()-1) {
            case 0:
                dBdx(0) = 0.0;
                break;
            case 1:
                dBdx(0) = -1.0 * dxdu;
                dBdx(1) = 1.0  * dxdu;
                break;
            case 2:
                dBdx(0) = -2.0 * (1.0 - x) * dxdu;
                dBdx(1) = (-2.0 * x + 2.0 *(1.0 - x)) * dxdu;
                dBdx(2) = 2.0 * x * dxdu;
                break;
            case 3:
                dBdx(0) = -3.0 * (1.0 - x) * (1.0 - x) * dxdu;
                dBdx(1) = (3.0 * (1.0 - x) * (1.0 - x) - 6.0 * (1.0 - x) * x) * dxdu;
                dBdx(2) = (-3.0 * x * x  + 6.0 * (1.0 - x) * x) * dxdu;
                dBdx(3) = 3.0 * x * x * dxdu;
                break;
        }
    }

    // Second derivatives of Bernstein Polynomials
    // Chain rule: ddB/ddu = ddB/ddx * (dx/du)^2, with dx/du = 0.5.
    static void BernsteinPolynomialsDerivative2(double x, ChVectorDynamic<>& ddBddx, double dudxsq = 0.25){
        switch (ddBddx.size()-1) {
            case 0:
                ddBddx(0) = 0.0;
                break;
            case 1:
                ddBddx(0) = 0.0;
                ddBddx(1) = 0.0;
                break;
            case 2:
                ddBddx(0) = 2.0 * dudxsq;
                ddBddx(1) = -4.0 * dudxsq;
                ddBddx(2) = 2.0 * dudxsq;
                break;
            case 3:
                ddBddx(0) = 6.0 * (1.0 - x) * dudxsq;
                ddBddx(1) = (18.0 * x - 12.0) * dudxsq;
                ddBddx(2) = (6.0 - 18.0 * x) * dudxsq;
                ddBddx(3) = 6.0 * x * dudxsq;
                break;
        }
    }

    // Third derivatives of Bernstein Polynomials
    // Chain rule: dddB/dddu = dddB/dddx * (dx/du)^3, with dx/du = 0.5.
    static void BernsteinPolynomialsDerivative3(double x, ChVectorDynamic<>& dddBdddx, double dudxcb = 0.125){
        switch (dddBdddx.size()-1) {
            case 0:
                dddBdddx(0) = 0.0;
                break;
            case 1:
                dddBdddx(0) = 0.0;
                dddBdddx(1) = 0.0;
                break;
            case 2:
                dddBdddx(0) = 0.0;
                dddBdddx(1) = 0.0;
                dddBdddx(2) = 0.0;
                break;
            case 3:
                dddBdddx(0) = -6.0 * dudxcb;
                dddBdddx(1) = 18.0 * dudxcb;
                dddBdddx(2) = -18.0 * dudxcb;
                dddBdddx(3) = 6.0 * dudxcb;
                break;
        }
    }

    /// Compute vector of bases R.
    /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) Bezier, at the i-th knot span,
    /// given the parameter u.
    /// Results go into the row vector R = { R1, R2, R3.... R_(p+1) }
    static void BasisEvaluate(const double u,                 ///< parameter in [-1, 1]
                              ChVectorDynamic<>& R            ///< basis functions R evaluated at u, that is: R(u). R must have size p+1
    ) {
        BernsteinPolynomials(0.5 * (1.0 + u), R); // x = (1 + u) / 2
    }


    /// Compute vector of bases R and their first derivatives.
    /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) Bezier, at the i-th knot span,
    /// given the parameter u.
    /// Results go into the row vector     R = {  R1,     R2,     R3,  , ....  R_(p+1)    }
    ///        and into the row vector dR/du =  { dR1/du, dR2/du, dR3/du, .... dR_(p+1)/du }
    /// R and dR/du must have size p+1.
    static void BasisEvaluateDeriv(
            const double u,                 ///< parameter in [-1, 1]
            ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
            ChVectorDynamic<>& dRdu         ///< here return basis functions derivatives dR/du evaluated at u
            ) {
                double x = 0.5 * (1.0 + u);
                BernsteinPolynomials(x, R);
                BernsteinPolynomialsDerivative1(x, dRdu);
    }

    /// Compute vector of bases R and their first, second and third derivatives.
    /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) Bezier, at the i-th knot span,
    /// given the parameter u.
    /// Results go into the row vector         R = {  R1,     R2,     R3,  , ....  R_(p+1)    }
    ///        and into the row vector     dR/du = { dR1/du, dR2/du, dR3/du, .... dR_(p+1)/du }
    ///        and into the row vector   ddR/ddu = { ddR1/ddu, ddR2/ddu, ddR3/ddu, .... ddR_(p+1)/ddu }
    ///        and into the row vector dddR/dddu = { dddR1/dddu, dddR2/dddu, dddR3/dddu, .... dddR_(p+1)/dddu }
    /// R, dRdu, ddRddu, and dddRdddu must have size p+1.
    static void BasisEvaluateDeriv(
        const double u,                 ///< parameter in [-1, 1]
        ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
        ChVectorDynamic<>& dRdu,        ///< here return basis functions derivatives dR/du evaluated at u
        ChVectorDynamic<>& ddRddu,      ///< here return basis functions second derivatives ddR/ddu evaluated at u
        ChVectorDynamic<>& dddRdddu       ///< here return basis functions third derivatives dddR/dddu evaluated at u
        ) {
            double x = 0.5 * (1.0 + u);
            BernsteinPolynomials(x, R);
            BernsteinPolynomialsDerivative1(x, dRdu);
            BernsteinPolynomialsDerivative2(x, ddRddu);
            BernsteinPolynomialsDerivative3(x, dddRdddu);
    }
};



}  // end namespace wood
}  // end namespace chrono

#endif
