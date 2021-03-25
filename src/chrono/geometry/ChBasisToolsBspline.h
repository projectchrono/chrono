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

#ifndef CHC_BASISTOOLSBSPLINE_H
#define CHC_BASISTOOLSBSPLINE_H

#include <cmath>
#include <vector>

#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace geometry {

/// Tools for evaluating basis functions for B-splines, parametrized with parameter u (as lines)
/// These bases are often called "N" in literature.

class ChApi ChBasisToolsBspline {
  public:
    /// Find the knot span of a b-spline given the parameter u,
    /// the order p, the knot vector knotU
    static int FindSpan(const int p,                    ///< order
                        const double u,                 ///< parameter
                        const ChVectorDynamic<>& knotU  ///< knot vector
    ) {
        int n = (int)knotU.size() - 2 - p;

        // safeguards, and shortcut (p+1)-multiple end knots
        if (u >= knotU(n + 1))
            return n;
        if (u <= knotU(p))
            return p;

        int lo = p;
        int hi = n + 1;
        int mid = (lo + hi) / 2;
        while (u < knotU(mid) || u >= knotU(mid + 1)) {
            if (u < knotU(mid))
                hi = mid;
            else
                lo = mid;

            mid = (lo + hi) / 2;
        }
        return mid;
    }

    /// Compute uniformly-spaced k knots (in range [kstart, kend]) for open Bsplines, with order p.
    /// Assuming the size of knotU is already k=n+p+1 for n control points. The p+1 initial and end knots are
    /// made multiple.
    static void ComputeKnotUniformMultipleEnds(ChVectorDynamic<>& knotU,  ///< knot vector
                                               const int p,               ///< order
                                               double kstart = 0.0,       ///< range start
                                               double kend = 1.0          ///< range end
    ) {
        if (knotU.size() < 2 * (p + 1))
            throw ChException("ComputeKnotUniformMultipleEnds: knots must have size>=2*(order+1)");

        int k = (int)knotU.size();
        // intermediate knots:
        int nik = k - 2 * p;
        for (int i = 0; i < nik; ++i) {
            knotU(p + i) = kstart + (double(i) / double(nik - 1)) * (kend - kstart);
        }
        // initial and final multiple knots:
        for (int i = 0; i < p; ++i) {
            knotU(i) = kstart;
            knotU(k - i - 1) = kend;
        }
    }

    /// Compute uniformly-spaced k knots (in range [kstart, kend]) for Bsplines, with order p.
    /// If you need that the spline starts and ends exactly at the 1st and last control point,
    /// use ComputeKnotUniformMultipleEnds instead.
    /// This is often used when creating closed splines, where the last and first p control points will overlap.
    static void ComputeKnotUniform(ChVectorDynamic<>& knotU,  ///< knot vector
                                   const int p,               ///< order
                                   double kstart = 0.0,       ///< range start
                                   double kend = 1.0          ///< range end
    ) {
        if (knotU.size() < 2 * (p + 1))
            throw ChException("ComputeKnotUniform: knots must have size>=2*(order+1)");

        int nk = (int)knotU.size();
        // intermediate knots:
        for (int i = 0; i < nk; ++i) {
            knotU(i) = kstart + (double(i) / double(nk - 1)) * (kend - kstart);
        }
    }

    /// Compute vector of bases N.
    /// Evaluate ALL the p+1 nonzero basis functions N of a b-spline, at the i-th knot span,
    /// given the parameter u, the order p, the knot vector knotU.
    /// Results go into the row vector N = { N1, N2, N3.... N_(p+1) }
    static void BasisEvaluate(const int p,                     ///< order
                              const int i,                     ///< knot span, assume aready computed via FindSpan()
                              const double u,                  ///< parameter
                              const ChVectorDynamic<>& knotU,  ///< knot vector
                              ChVectorDynamic<>& N  ///< here return basis functions N evaluated at u, that is: N(u)
    ) {
        N(0) = 1.0;

        int j, r;
        double* left = new double[p + 1];
        double* right = new double[p + 1];
        double saved, temp;

        for (j = 1; j <= p; ++j) {
            left[j] = u - knotU(i + 1 - j);
            right[j] = knotU(i + j) - u;
            saved = 0.0;
            for (r = 0; r < j; ++r) {
                temp = N(r) / (right[r + 1] + left[j - r]);
                N(r) = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N(j) = saved;
        }

        delete[] left;
        delete[] right;
    }

    /// Compute bases and first n-th derivatives of bases dN/du and ddN/ddu etc, arranged in a matrix.
    /// Evaluate derivatives of ALL the p+1 nonzero basis functions N of a b-spline, at the i-th knot span,
    /// given the parameter u, the order p, the knot vector knotU.
    /// Results go into the ChMatrixDynamic<> , where j-th derivative is j-th row:
    ///  DN = |   N1,       N2,       N3,    ....,   N_(p+1)     |
    ///       |  dN1/du,   dN2/du,   dN3/du, ....,  dN_(p+1)/du  |
    ///       | ddN1/ddu, ddN2/ddu, ddN3/ddu ...., ddN_(p+1)/ddu |
    /// The derivative order ranges from 0 (no derivative) to d, where d is the number of
    /// rows of the passed DN matrix. Usually two rows, for N and their shape derivatives.
    static void BasisEvaluateDeriv(const int p,     ///< order of spline
                                   const int i,     ///< knot span, assume aready computed via FindSpan()
                                   const double u,  ///< parameter
                                   const ChVectorDynamic<>& knotU,  ///< knot vector
                                   ChMatrixDynamic<>& DN  ///< here return derivatives evaluated at u, that is: dN/du(u)
    ) {
        int order = (int)DN.rows() - 1;  // max order of derivatives (0th in row 0 of DN, 1st in row 1 of DN, etc.)
        double saved, temp;
        int j, k, j1, j2, r;

        double* left = new double[p + 1];
        double* right = new double[p + 1];

        ChMatrixDynamic<> ndu(p + 1, p + 1);
        ChMatrixDynamic<> a(p + 1, p + 1);

        ndu(0, 0) = 1.;
        for (j = 1; j <= p; j++) {
            left[j] = u - knotU(i + 1 - j);
            right[j] = knotU(i + j) - u;
            saved = 0.0;
            for (r = 0; r < j; r++) {
                ndu(j, r) = right[r + 1] + left[j - r];
                temp = ndu(r, j - 1) / ndu(j, r);

                ndu(r, j) = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu(j, j) = saved;
        }
        for (j = 0; j <= p; j++)
            DN(0, j) = ndu(j, p);

        if (order == 0)
            return;

        for (r = 0; r <= p; r++) {
            int s1 = 0, s2 = 1;
            a(0, 0) = 1.0;

            for (k = 1; k <= order; k++) {
                double d = 0.;
                int rk = r - k, pk = p - k;
                if (r >= k) {
                    a(s2, 0) = a(s1, 0) / ndu(pk + 1, rk);
                    d = a(s2, 0) * ndu(rk, pk);
                }
                j1 = rk >= -1 ? 1 : -rk;
                j2 = (r - 1 <= pk) ? k - 1 : p - r;
                for (j = j1; j <= j2; j++) {
                    a(s2, j) = (a(s1, j) - a(s1, j - 1)) / ndu(pk + 1, rk + j);
                    d += a(s2, j) * ndu(rk + j, pk);
                }
                if (r <= pk) {
                    a(s2, k) = -a(s1, k - 1) / ndu(pk + 1, r);
                    d += a(s2, k) * ndu(r, pk);
                }
                DN(k, r) = d;
                j = s1;
                s1 = s2;
                s2 = j;
            }
        }
        r = p;
        for (k = 1; k <= order; k++) {
            for (j = 0; j <= p; j++)
                DN(k, j) *= r;
            r *= (p - k);
        }

        delete[] left;
        delete[] right;
    }
};

/// Tools for evaluating basis functions for tensor-product surface B-splines,
/// parametrized with parameters u,v (as lines)
/// These bases are often called "R" in literature.

class ChApi ChBasisToolsBsplineSurfaces {
  public:
    /// Compute vector of bases R.
    /// Evaluate ALL the (pu+1) * (pv+1) nonzero basis functions R of a 2D(surface) B-plines, at the needed u-v knot
    /// span, given the parameters u,v, the orders p_u p_v, the knot vector Knots_u, Knots_v. Results go into the matrix
    /// R = { R11, R12, R13.... ; R21, R22, R23....; ... } where u increases along columns, v along rows.
    static void BasisEvaluate(const int p_u,                     ///< order u
                              const int p_v,                     ///< order v
                              const double u,                    ///< parameter u
                              const double v,                    ///< parameter u
                              const ChVectorDynamic<>& Knots_u,  ///< knots u
                              const ChVectorDynamic<>& Knots_v,  ///< knots vu
                              ChMatrixDynamic<>& R  ///< here return bases (u increases along columns, v along rows)
    ) {
        int spanU = ChBasisToolsBspline::FindSpan(p_u, u, Knots_u);
        int spanV = ChBasisToolsBspline::FindSpan(p_v, v, Knots_v);

        ChVectorDynamic<> N_u(p_u + 1);
        ChBasisToolsBspline::BasisEvaluate(p_u, spanU, u, Knots_u, N_u);
        ChVectorDynamic<> N_v(p_v + 1);
        ChBasisToolsBspline::BasisEvaluate(p_v, spanV, v, Knots_v, N_v);

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                R(iu, iv) = N_u(iu) * N_v(iv);
            }
        }
    }

    /// Compute vector of bases R and their first derivatives.
    /// Evaluate ALL the (pu+1) * (pv+1) nonzero basis functions R of a 2D(surface) B-splines, at the needed u-v knot
    /// span, given the parameters u,v, the orders p_u p_v, the knot vector Knots_u, Knots_v. Results go into the matrix
    /// R = { R11, R12, R13.... ; R21, R22, R23....; ... } where u increases along columns, v along rows. Same for the
    /// derivatives in dRdu and dRdv.
    static void BasisEvaluateDeriv(
        const int p_u,                     ///< order u
        const int p_v,                     ///< order v
        const double u,                    ///< parameter u
        const double v,                    ///< parameter u
        const ChVectorDynamic<>& Knots_u,  ///< knots u
        const ChVectorDynamic<>& Knots_v,  ///< knots vu
        ChMatrixDynamic<>& R,              ///< here return bases (u increases along columns, v along rows)
        ChMatrixDynamic<>& dRdu,           ///< here return du-derivatives (u increases along columns, v along rows)
        ChMatrixDynamic<>& dRdv            ///< here return dv-derivatives (u increases along columns, v along rows)
    ) {
        int spanU = ChBasisToolsBspline::FindSpan(p_u, u, Knots_u);
        int spanV = ChBasisToolsBspline::FindSpan(p_v, v, Knots_v);

        ChVectorDynamic<> N_u(p_u + 1);
        ChBasisToolsBspline::BasisEvaluate(p_u, spanU, u, Knots_u, N_u);
        ChVectorDynamic<> N_v(p_v + 1);
        ChBasisToolsBspline::BasisEvaluate(p_v, spanV, v, Knots_v, N_v);

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                R(iu, iv) = N_u(0, iu) * N_v(0, iv);
                dRdu(iu, iv) = N_u(1, iu) * N_v(0, iv);
                dRdv(iu, iv) = N_u(0, iu) * N_v(1, iv);
            }
        }
    }

    /// Compute vector of bases R and their first and second derivatives.
    /// Evaluate ALL the (pu+1) * (pv+1) nonzero basis functions R of a 2D(surface) B-splines, at the needed u-v knot
    /// span, given the parameters u,v, the orders p_u p_v, the knot vector Knots_u, Knots_v, the weights Weights.
    /// Results go into the matrix R = { R11, R12, R13.... ; R21, R22, R23....; ... }
    /// where u increases along columns, v along rows.
    /// Same for the derivatives.
    static void BasisEvaluateDeriv(
        const int p_u,                     ///< order u
        const int p_v,                     ///< order v
        const double u,                    ///< parameter u
        const double v,                    ///< parameter u
        const ChVectorDynamic<>& Knots_u,  ///< knots u
        const ChVectorDynamic<>& Knots_v,  ///< knots vu
        ChMatrixDynamic<>& R,              ///< here return bases (u increases along columns, v along rows)
        ChMatrixDynamic<>& dRdu,           ///< here return du-derivatives (u increases along columns, v along rows)
        ChMatrixDynamic<>& dRdv,           ///< here return dv-derivatives (u increases along columns, v along rows)
        ChMatrixDynamic<>& d2Rdudu,  ///< here return dudu-second derivatives (u increases along columns, v along rows)
        ChMatrixDynamic<>& d2Rdvdv,  ///< here return dvdv-second derivatives (u increases along columns, v along rows)
        ChMatrixDynamic<>& d2Rdudv   ///< here return dudv-second derivatives (u increases along columns, v along rows)
    ) {
        int spanU = ChBasisToolsBspline::FindSpan(p_u, u, Knots_u);
        int spanV = ChBasisToolsBspline::FindSpan(p_v, v, Knots_v);

        ChVectorDynamic<> N_u(p_u + 1);
        ChBasisToolsBspline::BasisEvaluate(p_u, spanU, u, Knots_u, N_u);
        ChVectorDynamic<> N_v(p_v + 1);
        ChBasisToolsBspline::BasisEvaluate(p_v, spanV, v, Knots_v, N_v);

        for (int iv = 0; iv <= p_v; iv++) {
            for (int iu = 0; iu <= p_u; iu++) {
                R(iu, iv) = N_u(0, iu) * N_v(0, iv);
                dRdu(iu, iv) = N_u(1, iu) * N_v(0, iv);
                dRdv(iu, iv) = N_u(0, iu) * N_v(1, iv);

                d2Rdudu(iu, iv) = N_u(2, iu) * N_v(0, iv);
                d2Rdvdv(iu, iv) = N_u(0, iu) * N_v(2, iv);
                d2Rdudv(iu, iv) = N_u(1, iu) * N_v(1, iv);
            }
        }
    }
};

}  // end namespace geometry
}  // end namespace chrono

#endif
