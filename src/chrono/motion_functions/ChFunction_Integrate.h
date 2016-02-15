//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHFUNCT_INTEGRATE_H
#define CHFUNCT_INTEGRATE_H

//////////////////////////////////////////////////
//
//   ChFunction_Integrate.h
//
//   Function objects,
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Base.h"

namespace chrono {

#define FUNCT_INTEGRATE 17

/// INTEGRAL OF A FUNCTION:
/// y = int{ f(x) dx
///
/// Uses a numerical quadrature method to compute the definite integral.

class ChApi ChFunction_Integrate : public ChFunction {
    CH_RTTI(ChFunction_Integrate, ChFunction);

  private:
    std::shared_ptr<ChFunction> fa;
    int order;  // 1= Integrate one time, 2= two times, etc.
    double C_start;
    double x_start;
    double x_end;
    int num_samples;
    ChMatrix<>* array_x;

  public:
    ChFunction_Integrate();
    ~ChFunction_Integrate() {
        if (array_x)
            delete array_x;
    };
    void Copy(ChFunction_Integrate* source);
    ChFunction* new_Duplicate();

    void ComputeIntegral();

    void Set_order(int m_order) { order = m_order; }
    int Get_order() { return order; }
    void Set_num_samples(int m_samples) {
        num_samples = m_samples;
        array_x->Reset(num_samples, 1);
        ComputeIntegral();
    }
    int Get_num_samples() { return num_samples; }
    void Set_C_start(double m_val) {
        C_start = m_val;
        ComputeIntegral();
    }
    double Get_C_start() { return C_start; }
    void Set_x_start(double m_val) {
        x_start = m_val;
        ComputeIntegral();
    }
    double Get_x_start() { return x_start; }
    void Set_x_end(double m_val) {
        x_end = m_val;
        ComputeIntegral();
    }
    double Get_x_end() { return x_end; }

    /// Set the function to be integrated
    void Set_fa(std::shared_ptr<ChFunction> m_fa) {
        fa = m_fa;
        ComputeIntegral();
    }

    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    double Get_y(double x);

    void Estimate_x_range(double& xmin, double& xmax);

    int Get_Type() { return (FUNCT_INTEGRATE); }

    int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
    OPT_VARIABLES_START
    "C_start", OPT_VARIABLES_END

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(fa);
        marchive << CHNVP(order);
        marchive << CHNVP(C_start);
        marchive << CHNVP(x_start);
        marchive << CHNVP(x_end);
        marchive << CHNVP(num_samples);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(fa);
        marchive >> CHNVP(order);
        marchive >> CHNVP(C_start);
        marchive >> CHNVP(x_start);
        marchive >> CHNVP(x_end);
        marchive >> CHNVP(num_samples);
        array_x->Reset(num_samples, 1);
        ComputeIntegral();
    }

};

}  // END_OF_NAMESPACE____

#endif
