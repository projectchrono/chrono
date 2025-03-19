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
// Authors: Erol Lale
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



/// Tools for evaluating basis functions for BEZIER, parametrized with parameter u (as lines)
/// These bases are often called "R" in literature.

class ChWoodApi ChBasisToolsBeziers {
public:
	    // Function to calculate factorial
	static int factorial(int n) {
	    int fact = 1;
	    for (int i = 1; i <= n; ++i) {
		fact *= i;
	    }
	    return fact;
	}

	// Function to calculate binomial coefficient (n choose k)
	static int binomial_coefficient(int n, int k) {
	    return ChBasisToolsBeziers::factorial(n) / (ChBasisToolsBeziers::factorial(k) * ChBasisToolsBeziers::factorial(n - k));
	}

	// Function to evaluate Bernstein polynomial
	static double bernstein_polynomial(int n, int i, double t) {
	    return ChBasisToolsBeziers::binomial_coefficient(n, i) * pow(t, i) * pow(1 - t, n - i);
	}

	static double Bernstein_polynomial(int p, int a, double xi){
	    double B;
	    if (p==0 && a==1){
		B=1;
	    }else if (p==0 && a!=1){
		B=0;
	    }else{
		if (a<1 || a>p+1){
		    B=0;
		}else{
		    double B1=Bernstein_polynomial(p-1,a,xi); 
		    double B2=Bernstein_polynomial(p-1,a-1,xi); 
		    B=0.5*(1-xi)*B1+0.5*(1+xi)*B2;
		}
	    }
	    return B;
       }

    // Unroll recursion since we limit our cases to fairly simple and known cases p in [1,3]
    // More case can be added and unrolled with little effort since we are not going to use large p
    // In the context of Bezier curves here, this is called for x in [0,1]
    static void BernsteinPolynomials(double x, ChVectorDynamic<>& B){
        // No parameter check on order here.
        // It's another functions' responsibility to pass a vector of the correct size
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
    // Bernstein polynomials written in terms of x, but call on x = (1 + u) / 2
    // Chain rule: dB/du = dB/dx * dx/du, with dx/du = 0.5.
    // We could do dBdx *= 0.5 outside this function, but tests have shown
    // it is really expensive to do a *= operation on a ChVectorDynamic!!! Even a small one!
    // Instead, we add a multiplier as a function parameter.
    // Another alternative would be to express the Bernstein polynomial in function of u directly
    static void BernsteinPolynomialsDerivative1(double x, ChVectorDynamic<>& dBdx, double dxdu = 0.5){
        // No parameter check on order here.
        // It's another functions' responsibility to pass a vector of the correct size
        switch (dBdx.size()-1) {
            case 0:
                dBdx(0) = 0.0 * dxdu;
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



    static void unrolled_BasisEvaluate(
                const double u,                 ///< parameter in [-1, 1]      
                ChVectorDynamic<>& R            ///< here return basis functions R evaluated at u, that is: R(u)
                ) {
        // x = (1 + u) / 2
        BernsteinPolynomials(0.5 * (1.0 + u), R);
         
    }


    static void unrolled_BasisEvaluateDeriv(
            const double u,                 ///< parameter in [-1, 1]                
            ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
            ChVectorDynamic<>& dRdu         ///< here return basis functions derivatives dR/du evaluated at u
            ) {
                double x = 0.5 * (1.0 + u);
                double dxdu = 0.5;
                BernsteinPolynomials(x, R);
                BernsteinPolynomialsDerivative1(x, dRdu, dxdu);
    }

	
        /// Compute vector of bases R.
        /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) Bezier, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector Knots, the weights Weights.
        /// Results go into the row vector R = { R1, R2, R3.... R_(p+1) }
    static void BasisEvaluate(
                const int p,                    ///< order
                const double u,                 ///< parameter                
                ChVectorDynamic<>& R            ///< here return basis functions R evaluated at u, that is: R(u)
                ) {
        
        //double u=xi; //(1.+xi)/2.;        
        for (int i = 1; i <= p+1; i++) {
            R(i-1) = Bernstein_polynomial(p, i, u);
        }
    }

        /// Compute vector of bases R and their first derivatives.
        /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) Bezier, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector Knots, the weights Weights
        /// Results go into the row vector     R = {  R1,     R2,     R3,  , ....  R_(p+1)    }
        ///        and into the row vector dR/du = { dR1/du, dR2/du, dR3/du, .... dR_(p+1)/du }
    static void BasisEvaluateDeriv(
                const int p,                    ///< order
                const double u,                 ///< parameter                
                ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
                ChVectorDynamic<>& dRdu         ///< here return basis functions derivatives dR/du evaluated at u
                ) {
       
        int ii;        
        //double u=xi; //(1.+xi)/2.;
        
        for (int i = 1; i <= p+1; i++) {  
            ii = i-1;         
            R(ii)     = Bernstein_polynomial(p, i, u);
            dRdu(ii)  = 0.5*p*(Bernstein_polynomial(p-1,i-1,u)-Bernstein_polynomial(p-1,i,u));  
        }

    }

        /// Compute vector of bases R and their first and second derivatives.
        /// Evaluate ALL the p+1 nonzero basis functions R of a 1D(line) Bezier, at the i-th knot span, 
        /// given the parameter u, the order p, the knot vector Knots, the weights Weights
        /// Results go into the row vector     R = {  R1,     R2,     R3,  , ....  R_(p+1)    }
        ///        and into the row vector dR/du = { dR1/du, dR2/du, dR3/du, .... dR_(p+1)/du }
        ///        and into the row vector ddR/ddu={ ddR1/ddu, ddR2/ddu, ddR3/ddu, .... ddR_(p+1)/ddu }
    static void BasisEvaluateDeriv(
                const int p,                    ///< order
                const double u,                 ///< parameter                
                ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
                ChVectorDynamic<>& dRdu,        ///< here return basis functions derivatives dR/du evaluated at u
                ChVectorDynamic<>& ddRddu,      ///< here return basis functions derivatives ddR/ddu evaluated at u
                ChVectorDynamic<>& dddRdddu       ///< here return basis functions derivatives ddR/ddu evaluated at u
                ) {
        
        int ii;
        //double u=xi; //(1.+xi)/2.;
        //std::cout<<"UUUUUUUUUUUUUUUUUUUU: "<<u<<std::endl;
        for (int i = 1; i <= p+1; i++) {  
            ii=i-1;
            //std::cout<<i<<".th ----------------------------------\n" ;        
            R(ii)     = Bernstein_polynomial(p, i, u);
            dRdu(ii)  = 0.5*p*(Bernstein_polynomial(p-1,i-1,u)-Bernstein_polynomial(p-1,i,u)); 
            //std::cout<<"bernstein_polynomial(p-2,i-2,u): "<<Bernstein_polynomial(p-2,i-2,u)<<std::endl;
            //std::cout<<"bernstein_polynomial(p-2,i-1,u): "<<Bernstein_polynomial(p-2,i-1,u)<<std::endl;
            //std::cout<<"bernstein_polynomial(p-2,i,u): "<<Bernstein_polynomial(p-2,i,u)<<std::endl;
            ddRddu(ii)  = 0.5*p*(0.5*(p-1)*(Bernstein_polynomial(p-2,i-2,u)-Bernstein_polynomial(p-2,i-1,u)-Bernstein_polynomial(p-2,i-1,u)+
            			Bernstein_polynomial(p-2,i,u))); 
            			
            dddRdddu(ii)  =  0.5*p*(0.5*(p-1)*(0.5*(p-2)*(Bernstein_polynomial(p-3,i-3,u)-Bernstein_polynomial(p-3,i-2,u)-Bernstein_polynomial(p-3,i-2,u)+
            			Bernstein_polynomial(p-3,i-1,u)-Bernstein_polynomial(p-3,i-2,u)+Bernstein_polynomial(p-3,i-1,u)+
            			Bernstein_polynomial(p-3,i-1,u)-Bernstein_polynomial(p-3,i,u))));
        }
    }

};



}  // end namespace wood
}  // end namespace chrono

#endif
