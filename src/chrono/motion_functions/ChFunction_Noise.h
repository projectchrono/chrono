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

#ifndef CHFUNCT_NOISE_H
#define CHFUNCT_NOISE_H

//////////////////////////////////////////////////
//
//   ChFunction_Noise.h
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

#define FUNCT_NOISE 15

/// NOISE FUNCTION:
/// y = multi-octave noise with cubic interpolation
///

class ChApi ChFunction_Noise : public ChFunction {
    CH_RTTI(ChFunction_Noise, ChFunction);

  private:
    double amp;
    double freq;
    double amp_ratio;
    int octaves;

  public:
    ChFunction_Noise();
    ~ChFunction_Noise(){};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
    void Copy(ChFunction_Noise* source);
#pragma GCC diagnostic pop
    ChFunction* new_Duplicate() override;

    void Set_Amp(double mamp) { amp = mamp; }
    double Get_Amp() { return amp; };
    void Set_Freq(double mf) { freq = mf; }
    double Get_Freq() { return freq; };
    void Set_AmpRatio(double ma) { amp_ratio = ma; }
    double Get_AmpRatio() { return amp_ratio; };
    void Set_Octaves(int mo) { octaves = mo; }
    int Get_Octaves() { return octaves; };

    double Get_y(double x) override;

    int Get_Type() override { return (FUNCT_NOISE); }

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
        marchive << CHNVP_OUT(amp);
        marchive << CHNVP_OUT(freq);
        marchive << CHNVP_OUT(amp_ratio);
        marchive << CHNVP_OUT(octaves);
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
        marchive >> CHNVP_IN(amp);
        marchive >> CHNVP_IN(freq);
        marchive >> CHNVP_IN(amp_ratio);
        marchive >> CHNVP_IN(octaves);
    }

};

}  // END_OF_NAMESPACE____

#endif
