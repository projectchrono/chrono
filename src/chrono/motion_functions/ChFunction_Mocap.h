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

#ifndef CHFUNCT_MOCAP_H
#define CHFUNCT_MOCAP_H

//////////////////////////////////////////////////
//
//   ChFunction_Mocap.h
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

#define FUNCT_MOCAP 3

/// MOTION CAPTURE (SAMPLE) FUNCTION
/// y = (linear interpolated array of samples)

class ChApi ChFunction_Mocap : public ChFunction {
    CH_RTTI(ChFunction_Mocap, ChFunction);

  private:
    ChMatrix<>* array_y;
    ChMatrix<>* array_y_dt;
    ChMatrix<>* array_y_dtdt;

    double samp_freq;
    int samples;
    double timetot;

  public:
    ChFunction_Mocap();                            //  see .cpp
    ChFunction_Mocap(int m_samples, double freq);  //  see .cpp
    ~ChFunction_Mocap();                           //  see .cpp
    void Copy(ChFunction_Mocap* source);
    ChFunction* new_Duplicate();

    void Set_samp_freq(double m_fr);  // see .cpp
    void Set_samples(int m_samples);  // see .cpp

    double Get_samp_freq() { return samp_freq; };
    int Get_samples() { return samples; };
    double Get_timetot() { return ((double)samples / samp_freq); };
    double Get_timeslice() { return (1 / samp_freq); };

    ChMatrix<>* Get_array_y() { return array_y; };
    ChMatrix<>* Get_array_y_dt() { return array_y_dt; };
    ChMatrix<>* Get_array_y_dtdt() { return array_y_dtdt; };

    void Set_array_y(ChMatrix<>* m_array_y);            // see cpp
    void Set_array_y_dt(ChMatrix<>* m_array_y_dt);      // *** TO DO
    void Set_array_y_dtdt(ChMatrix<>* m_array_y_dtdt);  // *** TO DO

    int Parse_array_AOA();    // *** TO DO
    int Parse_array_Elite();  // *** TO DO

    void Compute_array_dt(ChMatrix<>* array_A, ChMatrix<>* array_A_dt);
    double LinInterp(ChMatrix<>* m_array, double x, double x_max);

    double Get_y(double x);       // see.cpp
    double Get_y_dx(double x);    // see.cpp
    double Get_y_dxdx(double x);  // see.cpp

    void Estimate_x_range(double& xmin, double& xmax);
    int Get_Type() { return (FUNCT_MOCAP); }

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
        marchive << CHNVP(array_y);
        marchive << CHNVP(array_y_dt);
        marchive << CHNVP(array_y_dtdt);
        marchive << CHNVP(samp_freq);
        marchive << CHNVP(samples);
        marchive << CHNVP(timetot);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(array_y);
        marchive >> CHNVP(array_y_dt);
        marchive >> CHNVP(array_y_dtdt);
        marchive >> CHNVP(samp_freq);
        marchive >> CHNVP(samples);
        marchive >> CHNVP(timetot);
    }

    //***OBSOLETE***
    void StreamOUT(ChStreamOutAscii& mstream);
    void StreamIN(ChStreamInBinary& mstream);
    void StreamOUT(ChStreamOutBinary& mstream);
};

}  // END_OF_NAMESPACE____

#endif
