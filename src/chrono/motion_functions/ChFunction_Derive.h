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

#ifndef CHFUNCT_DERIVE_H
#define CHFUNCT_DERIVE_H

//////////////////////////////////////////////////
//
//   ChFunction_Derive.h
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
#include "ChFunction_Const.h"

namespace chrono {

#define FUNCT_DERIVE 16

/// DERIVATIVE OF A FUNCTION:
///  y = df/dx
///
/// Uses a numerical differentiation method to compute the derivative
/// of a generic function.

class ChApi ChFunction_Derive : public ChFunction {
    CH_RTTI(ChFunction_Derive, ChFunction);

  private:
    std::shared_ptr<ChFunction> fa;
    int order;  // 1= derive one time, 2= two times, etc.

  public:
    ChFunction_Derive() { order = 1; }
    ~ChFunction_Derive(){};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
    void Copy(ChFunction_Derive* source);
#pragma GCC diagnostic pop
    ChFunction* new_Duplicate() override;

    void Set_order(int m_order) { order = m_order; }
    int Get_order() { return order; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    double Get_y(double x) override;

    void Estimate_x_range(double& xmin, double& xmax) override;

    int Get_Type() override { return (FUNCT_DERIVE); }

    int MakeOptVariableTree(ChList<chjs_propdata>* mtree) override;


    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) const override
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP_OUT(fa);
        marchive << CHNVP_OUT(order);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override
    {
        // version number
        // int version =
        marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP_IN(fa);
        marchive >> CHNVP_IN(order);
    }

};

}  // END_OF_NAMESPACE____

#endif
