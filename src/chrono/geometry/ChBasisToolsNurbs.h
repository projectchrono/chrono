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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHC_BASISTOOLSNURBS_H
#define CHC_BASISTOOLSNURBS_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChBasisToolsBspline.h"

namespace chrono {
namespace geometry {



/// Tools for evaluating basis functions for NURBS, parametrized with parameter u (as lines)
/// These bases are often called "R" in literature.

class ChApi ChBasisToolsNurbs {

public:
 
        /// Compute vector of bases R.
        /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) NURBS, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector Knots, the weights Weights.
        /// Results go into the row vector R = { R1, R2, R3.... R_(p+1) }
    static void BasisEvaluate(
                const int p,                    ///< order
                const double u,                 ///< parameter
                const ChVectorDynamic<>& Weights,///< weights
                const ChVectorDynamic<>& Knots, ///< knots
                ChVectorDynamic<>& R            ///< here return basis functions R evaluated at u, that is: R(u)
                ) {
        int spanU = ChBasisToolsBspline::FindSpan(p, u, Knots);

        ChVectorDynamic<> N(p + 1);
        ChBasisToolsBspline::BasisEvaluate(p, spanU, u, Knots, N);

        int i;

        int uind = spanU - p;
        double w = 0.0;

        for (i = 0; i <= p; i++) {
            w += N(i) * Weights(uind + i);
        }

        for (i = 0; i <= p; i++) {
            R(i) = N(i) * Weights(uind + i) / w;
        }
    }

        /// Compute vector of bases R and their first derivatives.
        /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) NURBS, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector Knots, the weights Weights
        /// Results go into the row vector     R = {  R1,     R2,     R3,  , ....  R_(p+1)    }
        ///        and into the row vector dR/du = { dR1/du, dR2/du, dR3/du, .... dR_(p+1)/du }
    static void BasisEvaluateDeriv(
                const int p,                    ///< order
                const double u,                 ///< parameter
                const ChVectorDynamic<>& Weights,///< weights
                const ChVectorDynamic<>& Knots, ///< knots
                ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
                ChVectorDynamic<>& dRdu         ///< here return basis functions derivatives dR/du evaluated at u
                ) {
        int spanU = ChBasisToolsBspline::FindSpan(p, u, Knots);

        // Compute bases N and first derivatives dNdu:
        ChMatrixDynamic<>  N  (2, p + 1);
        ChBasisToolsBspline::BasisEvaluateDeriv(p, spanU, u, Knots, N);

        int i;
        int uind = spanU - p;
        double W = 0.0;
        double dWdu  = 0.0; 

        for (i = 0; i <= p; i++) {
            double wi = Weights(uind + i);
            W     +=    N(0,i) * wi;
            dWdu  +=    N(1,i) * wi;
        }

        double fac;
        
        for (i = 0; i <= p; i++) {
            double wi = Weights(uind + i);
            fac      = wi/(W*W);
            R(i)     = N(0,i) * wi / W;
            dRdu(i)  = (N(1,i)*W - N(0,i)*dWdu) * fac;  
        }

    }

        /// Compute vector of bases R and their first and second derivatives.
        /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) NURBS, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector Knots, the weights Weights
        /// Results go into the row vector     R = {  R1,     R2,     R3,  , ....  R_(p+1)    }
        ///        and into the row vector dR/du = { dR1/du, dR2/du, dR3/du, .... dR_(p+1)/du }
        ///        and into the row vector ddR/ddu={ ddR1/ddu, ddR2/ddu, ddR3/ddu, .... ddR_(p+1)/ddu }
    static void BasisEvaluateDeriv(
                const int p,                    ///< order
                const double u,                 ///< parameter
                const ChVectorDynamic<>& Weights,///< weights
                const ChVectorDynamic<>& Knots, ///< knots
                ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
                ChVectorDynamic<>& dRdu,        ///< here return basis functions derivatives dR/du evaluated at u
                ChVectorDynamic<>& ddRddu       ///< here return basis functions derivatives ddR/ddu evaluated at u
                ) {
        int spanU = ChBasisToolsBspline::FindSpan(p, u, Knots);

        // Compute bases N and first derivatives dNdu and second derivatives ddNddu:
        ChMatrixDynamic<>  N  (3, p + 1);
        ChBasisToolsBspline::BasisEvaluateDeriv(p, spanU, u, Knots, N);

        int i;
        int uind = spanU - p;
        double W = 0.0;
        double dWdu  = 0.0; 
        double ddWddu = 0.0; 

        for (i = 0; i <= p; i++) {
            double wi = Weights(uind + i);
            W     +=    N(0,i) * wi;
            dWdu  +=    N(1,i) * wi;
            ddWddu+=    N(2,i) * wi;
        }

        double fac;
        
        for (i = 0; i <= p; i++) {
            double wi = Weights(uind + i);
            fac      = wi/(W*W);
            R(i)     = N(0,i) * wi / W;
            dRdu(i)  = (N(1,i)*W - N(0,i)*dWdu) * fac;  
            ddRddu(i)= wi*(N(2,i)/W - 2*N(1,i)*dWdu/W/W - N(0,i)*ddWddu/W/W + 2*N(0,i)*dWdu*dWdu/W/W/W) ; 
        }
    }

};


/// Tools for evaluating basis functions for tensor-product surface NURBS, 
/// parametrized with parameter u,v (as lines)
/// These bases are often called "R" in literature.

class ChApi ChBasisToolsNurbsSurfaces {

public:
 
        /// Compute vector of bases R.
        /// Evaluate ALL the (pu+1) * (pv+1) nonzero basis functions R of a 2D(surface) NURBS, at the needed u-v knot span, 
        /// given the parameters u,v, the orders p_u p_v, the knot vector Knots_u, Knots_v, the weights Weights.
        /// Results go into the matrix R = { R11, R12, R13.... ; R21, R22, R23....; ... } 
        /// where u increases along columns, v along rows.
    static void BasisEvaluate(
                const int p_u,                    ///< order u
                const int p_v,                    ///< order v
                const double u,                   ///< parameter u
                const double v,                   ///< parameter u
                const ChMatrixDynamic<>& Weights, ///< weights (u increases along columns, v along rows)
                const ChVectorDynamic<>& Knots_u, ///< knots u
                const ChVectorDynamic<>& Knots_v, ///< knots vu
                ChMatrixDynamic<>& R              ///< here return bases (u increases along columns, v along rows)
                ) {
        int spanU = ChBasisToolsBspline::FindSpan(p_u, u, Knots_u);
        int spanV = ChBasisToolsBspline::FindSpan(p_v, v, Knots_v);

        ChVectorDynamic<> N_u(p_u + 1);
        ChBasisToolsBspline::BasisEvaluate(p_u, spanU, u, Knots_u, N_u);
        ChVectorDynamic<> N_v(p_v + 1);
        ChBasisToolsBspline::BasisEvaluate(p_v, spanV, v, Knots_v, N_v);


        int uind = spanU - p_u;
        int vind = spanV - p_v;
        double w = 0.0;

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                w += N_u(iu) * N_v(iv) * Weights(uind + iu, vind + iv);
            }
        }

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                R(iu,iv) =  N_u(iu) * N_v(iv) * Weights(uind + iu, vind + iv) / w;
            }
        }
    }

        /// Compute vector of bases R and their first derivatives.
        /// Evaluate ALL the (pu+1) * (pv+1) nonzero basis functions R of a 2D(surface) NURBS, at the needed u-v knot span, 
        /// given the parameters u,v, the orders p_u p_v, the knot vector Knots_u, Knots_v, the weights Weights.
        /// Results go into the matrix R = { R11, R12, R13.... ; R21, R22, R23....; ... } 
        /// where u increases along columns, v along rows.
        /// Same for the derivatives in dRdu and dRdv.
    static void BasisEvaluateDeriv(
                const int p_u,                    ///< order u
                const int p_v,                    ///< order v
                const double u,                   ///< parameter u
                const double v,                   ///< parameter u
                const ChMatrixDynamic<>& Weights, ///< weights (u increases along columns, v along rows)
                const ChVectorDynamic<>& Knots_u, ///< knots u
                const ChVectorDynamic<>& Knots_v, ///< knots vu
                ChMatrixDynamic<>& R,             ///< here return bases (u increases along columns, v along rows)
                ChMatrixDynamic<>& dRdu,          ///< here return du-derivatives (u increases along columns, v along rows)
                ChMatrixDynamic<>& dRdv           ///< here return dv-derivatives (u increases along columns, v along rows)
                ) {
        int spanU = ChBasisToolsBspline::FindSpan(p_u, u, Knots_u);
        int spanV = ChBasisToolsBspline::FindSpan(p_v, v, Knots_v);

        ChVectorDynamic<> N_u(p_u + 1);
        ChBasisToolsBspline::BasisEvaluate(p_u, spanU, u, Knots_u, N_u);
        ChVectorDynamic<> N_v(p_v + 1);
        ChBasisToolsBspline::BasisEvaluate(p_v, spanV, v, Knots_v, N_v);


        int uind = spanU - p_u;
        int vind = spanV - p_v;

        double W = 0.0;
        double dWdu  = 0.0; 
        double dWdv  = 0.0; 

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                double wi = Weights(uind + iu, vind + iv);
                W     +=    N_u(0,iu) * N_v(0,iv) * wi;
                dWdu  +=    N_u(1,iu) * N_v(0,iv) * wi;
                dWdv  +=    N_u(0,iu) * N_v(1,iv) * wi;
            }
        }
 
        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                double wi = Weights(uind + iu, vind + iv);
                R(iu,iv)    = N_u(0,iu) * N_v(0,iv) * wi / W; 
                dRdu(iu,iv) =(N_u(1,iu) * N_v(0,iv)*W - N_u(0,iu) * N_v(0,iv)*dWdu) * wi / (W*W);
                dRdv(iu,iv) =(N_u(0,iu) * N_v(1,iv)*W - N_u(0,iu) * N_v(0,iv)*dWdv) * wi / (W*W);
            }
        }

    }

        /// Compute vector of bases R and their first and second derivatives.
        /// Evaluate ALL the (pu+1) * (pv+1) nonzero basis functions R of a 2D(surface) NURBS, at the needed u-v knot span, 
        /// given the parameters u,v, the orders p_u p_v, the knot vector Knots_u, Knots_v, the weights Weights.
        /// Results go into the matrix R = { R11, R12, R13.... ; R21, R22, R23....; ... } 
        /// where u increases along columns, v along rows.
        /// Same for the derivatives.
    static void BasisEvaluateDeriv(
                const int p_u,                    ///< order u
                const int p_v,                    ///< order v
                const double u,                   ///< parameter u
                const double v,                   ///< parameter u
                const ChMatrixDynamic<>& Weights, ///< weights (u increases along columns, v along rows)
                const ChVectorDynamic<>& Knots_u, ///< knots u
                const ChVectorDynamic<>& Knots_v, ///< knots vu
                ChMatrixDynamic<>& R,             ///< here return bases (u increases along columns, v along rows)
                ChMatrixDynamic<>& dRdu,          ///< here return du-derivatives (u increases along columns, v along rows)
                ChMatrixDynamic<>& dRdv,          ///< here return dv-derivatives (u increases along columns, v along rows)
                ChMatrixDynamic<>& d2Rdudu,       ///< here return dudu-second derivatives (u increases along columns, v along rows)
                ChMatrixDynamic<>& d2Rdvdv,       ///< here return dvdv-second derivatives (u increases along columns, v along rows)
                ChMatrixDynamic<>& d2Rdudv        ///< here return dudv-second derivatives (u increases along columns, v along rows)
                ) {
        int spanU = ChBasisToolsBspline::FindSpan(p_u, u, Knots_u);
        int spanV = ChBasisToolsBspline::FindSpan(p_v, v, Knots_v);

        ChVectorDynamic<> N_u(p_u + 1);
        ChBasisToolsBspline::BasisEvaluate(p_u, spanU, u, Knots_u, N_u);
        ChVectorDynamic<> N_v(p_v + 1);
        ChBasisToolsBspline::BasisEvaluate(p_v, spanV, v, Knots_v, N_v);


        int uind = spanU - p_u;
        int vind = spanV - p_v;

        double W = 0.0;
        double dWdu  = 0.0; 
        double dWdv  = 0.0; 
        double d2Wdudu  = 0.0;
        double d2Wdvdv  = 0.0;
        double d2Wdudv  = 0.0;

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                double wi = Weights(uind + iu, vind + iv);
                W     +=    N_u(0,iu) * N_v(0,iv) * wi;
                dWdu  +=    N_u(1,iu) * N_v(0,iv) * wi;
                dWdv  +=    N_u(0,iu) * N_v(1,iv) * wi;
                d2Wdudu +=  N_u(2,iu) * N_v(0,iv) * wi;
                d2Wdvdv +=  N_u(0,iu) * N_v(2,iv) * wi;
                d2Wdudv +=  N_u(1,iu) * N_v(1,iv) * wi;
            }
        }
     
        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                double wi = Weights(uind + iu, vind + iv);
                R(iu,iv)    = N_u(0,iu) * N_v(0,iv) * wi / W; 
                dRdu(iu,iv) =(N_u(1,iu) * N_v(0,iv)*W - N_u(0,iu) * N_v(0,iv)*dWdu) * wi / (W*W);
                dRdv(iu,iv) =(N_u(0,iu) * N_v(1,iv)*W - N_u(0,iu) * N_v(0,iv)*dWdv) * wi / (W*W);

                d2Rdudu(iu,iv) = wi*(N_u(2,iu)*N_v(0,iv)/W - 2*N_u(1,iu)*N_v(0,iv)*dWdu/W/W - N_u(0,iu)*N_v(0,iv)*d2Wdudu/W/W + 2*N_u(0,iu)*N_v(0,iv)*dWdu*dWdu/W/W/W);
                d2Rdvdv(iu,iv) = wi*(N_u(0,iu)*N_v(2,iv)/W - 2*N_u(0,iu)*N_v(1,iv)*dWdv/W/W - N_u(0,iu)*N_v(0,iv)*d2Wdvdv/W/W + 2*N_u(0,iu)*N_v(0,iv)*dWdv*dWdv/W/W/W);
                d2Rdudv(iu,iv) = wi*(N_u(1,iu)*N_v(1,iv)/W - N_u(1,iu)*N_v(0,iv)*dWdu/W/W - N_u(0,iu)*N_v(1,iv)*dWdv/W/W - N_u(0,iu)*N_v(0,iv)*d2Wdudv/W/W + 2*N_u(0,iu)*N_v(0,iv)*dWdu*dWdv/W/W/W);
            }
        }
    }

};


}  // end namespace geometry
}  // end namespace chrono

#endif
