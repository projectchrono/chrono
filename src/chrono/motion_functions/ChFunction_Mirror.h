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

#ifndef CHFUNCT_MIRROR_H
#define CHFUNCT_MIRROR_H

//////////////////////////////////////////////////
//
//   ChFunction_Mirror.h
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

#define FUNCT_MIRROR 18

/// MIRROR FUNCTION:
/// y = __/\__
///
/// Mirrors a function about a vertical axis.

class ChApi ChFunction_Mirror : public ChFunction {
    CH_RTTI(ChFunction_Mirror, ChFunction);

  private:
    std::shared_ptr<ChFunction> fa;
    double mirror_axis;  // simmetry axis position on x

  public:
    ChFunction_Mirror() {
        mirror_axis = 0;
        fa = std::make_shared<ChFunction_Const>(); // default
    }
    ~ChFunction_Mirror(){};
    void Copy(ChFunction_Mirror* source);
    ChFunction* new_Duplicate();

    void Set_mirror_axis(double m_axis) { mirror_axis = m_axis; }
    double Get_mirror_axis() { return mirror_axis; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    double Get_y(double x);

    void Estimate_x_range(double& xmin, double& xmax);
    int Get_Type() { return (FUNCT_MIRROR); }

    int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
    OPT_VARIABLES_START
    "mirror_axis", OPT_VARIABLES_END

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
        marchive << CHNVP(mirror_axis);
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
        marchive >> CHNVP(mirror_axis);
    }

};

}  // END_OF_NAMESPACE____

#endif
