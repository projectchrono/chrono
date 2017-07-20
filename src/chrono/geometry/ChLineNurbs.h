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

#ifndef CHC_LINENURBS_H
#define CHC_LINENURBS_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChLine.h"
#include "chrono/core/ChVectorDynamic.h"

namespace chrono {
namespace geometry {


/// Tools for evaluating basis functions for B-splines, parametrized with parameter u (as lines)
/// These bases are often called "N" in literature.

class ChApi ChBasisToolsBspline {

public:

        /// Find the knot span of a b-spline given the parameter u, 
        /// the order p, the knot vector knotU
    static int FindSpan(
                const int p,                    ///< order
                const double u,                 ///< parameter
                const ChVectorDynamic<>& knotU  ///< knot vector
                )
    {
       int n = knotU.GetRows()-2-p;

       // assuming (p+1)-multiple end knots
       if( u >=knotU(n+1) )
           return n;
       if( u <=knotU(p) )
           return p;

       int lo = p;
       int hi = n+1;
       int mid=(lo+hi)/2;
       while( u<knotU(mid) || u>=knotU(mid+1) ) {

         if( u<knotU(mid) )
           hi=mid;
         else
           lo=mid;

         mid=(lo+hi)/2;
       }
       return mid;
     }

        /// Compute uniformly-spaced k knots (in range [kstart, kend]) for open Bsplines, with order p. 
        /// Assuming the size of knotU is already k=n+p+1 for n control points. The p+1 initial and end knots are 
        /// made multiple.
    static void ComputeKnotUniformMultipleEnds(
                ChVectorDynamic<>& knotU,       ///< knot vector
                const int p,                    ///< order
                double kstart = 0.0,          
                double kend = 1.0) {

        if (knotU.GetLength() < 2*(p+1))
            throw ChException ("ComputeKnotUniformMultipleEnds: knots must have size>=2*(order+1)");

        int k = knotU.GetLength();
        // intermediate knots:
        int nik = k-2*p;
        for (int i=0; i<nik; ++i) {
            knotU(p+i) = kstart+ (double(i)/double(nik-1))*(kend-kstart);
        }
        // initial and final multiple knots:
        for (int i=0; i<p; ++i) {
            knotU(i)    =kstart;
            knotU(k-i-1)=kend;
        }
    }
 
        /// Compute vector of bases N.
        /// Evaluate ALL the p+1 nonzero basis functions N of a b-spline, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector knotU.
        /// Results go into the row vector N = { N1, N2, N3.... N_(p+1) }
    static void BasisEvaluate(
                const int p,                    ///< order
                const int i,                    ///< knot span, assume aready computed via FindSpan()
                const double u,                 ///< parameter
                const ChVectorDynamic<>& knotU, ///< knot vector
                ChVectorDynamic<>& N            ///< here return basis functions N evaluated at u, that is: N(u)
                )
    {
	    N(0) = 1.0;
	
	    int j,r;
        double *left  = new double[p+1];
        double *right = new double[p+1];
	    double saved, temp;
	
	    for( j = 1; j <= p; ++j)
	    {
		    left[j]  = u - knotU(i+1-j);
		    right[j] = knotU(i+j) - u;
		    saved = 0.0;
		    for(r = 0; r < j; ++r)
		    {
			    temp  = N(r) / ( right[r+1] + left[j-r] );
			    N(r)  = saved + right[r+1] * temp;
			    saved = left[j-r] * temp;
		    }
		    N(j)= saved;
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
    static void BasisEvaluateDeriv(
                const int p,                     ///< order of spline
                const int i,                     ///< knot span, assume aready computed via FindSpan()
                const double u,                  ///< parameter
                const ChVectorDynamic<>& knotU,  ///< knot vector
                ChMatrixDynamic<>&   DN          ///< here return derivatives evaluated at u, that is: dN/du(u)
        ) {
        int order = DN.GetRows() -1; // max order of derivatives (0th in row 0 of DN, 1st in row 1 of DN, etc.)
        double saved, temp;
        int j, k, j1, j2, r;

        double *left  = new double[p+1];
        double *right = new double[p+1];

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
            DN(0,j) = ndu(j, p);

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
                DN(k,r) = d;
                j = s1;
                s1 = s2;
                s2 = j;
            }
        }
        r = p;
        for (k = 1; k <= order; k++) {
            for (j = 0; j <= p; j++)
                DN(k,j) *= r;
            r *= (p - k);
        }

        delete[] left;
	    delete[] right;
    }

};



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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


///////////////////////////////////////////////////////////////////////////////////////////////



/// Geometric object representing a NURBS spline.

class ChApi ChLineNurbs : public ChLine {

  public:
    std::vector< ChVector<> > points;
    ChVectorDynamic<>         weights;
    ChVectorDynamic<>         knots;
    int                       p;

  public:
        /// Constructor. By default, a segment (order = 1, two points on X axis, at -1, +1)
    ChLineNurbs();

        /// Constructor from a given array of control points. Input data is copied.
        /// If the knots are not provided, a uniformly spaced knot vector is made.
        /// If the weights are not provided, a constant weight vector is made.
    ChLineNurbs(int morder,                 ///< order p: 1= linear, 2=quadratic, etc.
                std::vector< ChVector<> >& mpoints, ///< control points, size n. Required: at least n >= p+1
                ChVectorDynamic<>* mknots = 0,      ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform. 
                ChVectorDynamic<>* weights = 0     ///< weights, size w. Required w=n. If not provided, all weights as 1. 
                );

    ChLineNurbs(const ChLineNurbs& source);
    ~ChLineNurbs() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineNurbs* Clone() const override { return new ChLineNurbs(*this); }

    //virtual GeometryType GetClassType() const override { return LINE_NURBS; }

    virtual int Get_complexity() const override { return 10; }

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    // NURBS specific functions
    
        /// Access the points
    std::vector< ChVector<> >& Points() { return points; } 

        /// Access the weights
    ChVectorDynamic<>& Weights() { return weights; } 

        /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; } 

        /// Get the order of spline
    int GetOrder() { return p; }

        /// Initial easy setup from a given array of control points. Input data is copied.
        /// If the knots are not provided, a uniformly spaced knot vector is made.
        /// If the weights are not provided, a constant weight vector is made.
    virtual void SetupData( int morder,                 ///< order p: 1= linear, 2=quadratic, etc.
                    std::vector< ChVector<> >& mpoints, ///< control points, size n. Required: at least n >= p+1
                    ChVectorDynamic<>* mknots = 0,      ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform. 
                    ChVectorDynamic<>* weights = 0     ///< weights, size w. Required w=n. If not provided, all weights as 1. 
                    );



    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLineNurbs>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(points);
        marchive << CHNVP(weights);
        marchive << CHNVP(knots);
        marchive << CHNVP(p);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLineNurbs>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(points);
        marchive >> CHNVP(weights);
        marchive >> CHNVP(knots);
        marchive >> CHNVP(p);
    }
};



}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLineNurbs,0)

}  // end namespace chrono

#endif
