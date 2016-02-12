//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_LINECAM_H
#define CHC_LINECAM_H


#include <math.h>

#include "ChCLine.h"
#include "physics/ChFunction.h"

namespace chrono {
namespace geometry {

/// For ChLineCam: types of cams
enum eChCamType {
    CAM_TYPE_SLIDEFOLLOWER = 0,
    CAM_TYPE_ROTATEFOLLOWER,
    CAM_TYPE_ECCENTRICFOLLOWER,
    CAM_TYPE_FLAT,
    CAM_TYPE_FLATOSCILLATE,
};
CH_ENUM_MAPPER_BEGIN(eChCamType);
  CH_ENUM_VAL(CAM_TYPE_SLIDEFOLLOWER);
  CH_ENUM_VAL(CAM_TYPE_ROTATEFOLLOWER);
  CH_ENUM_VAL(CAM_TYPE_ECCENTRICFOLLOWER);
  CH_ENUM_VAL(CAM_TYPE_FLAT);
  CH_ENUM_VAL(CAM_TYPE_FLATOSCILLATE);
CH_ENUM_MAPPER_END(eChCamType);

#define CH_GEOCLASS_LINECAM 6

///
/// CAM-PROFILE LINE
///
/// The object which describes analitycally the shape of a cam,
/// given the ChFunction which defines the motion law of the follower.

class ChApi ChLineCam : public ChLine {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLineCam, ChLine);

  private:
    //
    // DATA
    //

    eChCamType type;  // see codes
    std::shared_ptr<ChFunction> law;
    double phase;     // 0..2PI  phase (rotation). Def=0, neutral position

    double Rb;  // base radius
    double Rr;  // radius of contact wheel

    double p;   // length of rotating mover
    double d;   // distance center of cam - center of rotation of mover
    double b0;  // initial rotation for mover

    double e;  // eccentricity of sliding follower
    double s;  // distance of sliding follower

    int negative;  // negative cam: for desmodromic stuff, (cam is also Y or X mirrored, depend.on type )
    int internal;  // follower roller is inside the cam

    Vector center;  // center of cam in space (def.alignment on xy plane)

  public:
    //
    // CONSTRUCTORS
    //

    ChLineCam();

    virtual ~ChLineCam();

    ChLineCam(const ChLineCam& source) {
        Copy(&source);
    }

    void Copy(const ChLineCam* source);

    ChGeometry* Duplicate() {
        ChGeometry* mgeo = new ChLineCam();
        mgeo->Copy(this);
        return mgeo;
    };

    /// Get the class type as unique numerical ID (faster
    /// than using ChronoRTTI mechanism).
    /// Each inherited class must return an unique ID.
    virtual int GetClassType() { return CH_GEOCLASS_LINECAM; };

    bool Get_closed() { return true; };
    void Set_closed(bool mc){};

    void Set_Phase(double mf) { phase = mf; };
    double Get_Phase() { return phase; }

    /// Base radius of cam
    void Set_Rb(double mrb) {
        Rb = mrb;
        if (e > 0.9 * Rb)
            e = 0.9 * Rb;
        if (e < -0.9 * Rb)
            e = -0.9 * Rb;
    };
    double Get_Rb() { return Rb; }

    /// Radius of contact wheel
    void Set_Rr(double mrr) { Rr = mrr; }
    double Get_Rr() { return Rr; }

    /// The motion law, as a ChFunction.
    void Set_motion_law(std::shared_ptr<ChFunction> mlaw) {
        law = mlaw;
    }

    std::shared_ptr<ChFunction> Get_motion_law() { return law; }

    /// Type of cam (see the eChCamType enum values below).
    void Set_type(eChCamType mt) { type = mt; }
    eChCamType Get_type() { return type; }

    /// position of center of cam in 3d space.
    void Set_center(Vector mc) { center = mc; }
    Vector Get_center() { return center; }

    /// If true, creates a negative cam.
    void Set_Negative(int mne) { negative = mne; };
    int Get_Negative() { return negative; }

    /// If true, creates an internal cam.
    void Set_Internal(int min) { internal = min; };
    int Get_Internal() { return internal; }

    /// Sets the data for the rotating follower (length, distance from cam center, initial phase mb0)
    void Set_rotating_follower(double mp, double md, double mb0) {
        p = mp;
        d = md;
        b0 = mb0;
        Rb = sqrt(p * p + d * d - 2 * p * d * cos(b0));
    };
    double Get_p() { return p; };
    double Get_d() { return d; };
    double Get_b0() { return b0; };

    /// Sets the data for the sliding follower (if eccentrical, with me eccentricity)
    void Set_sliding_eccentrical(double me) { e = me; };
    double Get_e() { return e; };
    double Get_s() {
        s = sqrt(Rb * Rb - e * e);
        return s;
    };

    /// Sets the data for the flat rotating follower (length, distance from cam center, initial phase mb0)
    void Set_flat_oscillate(double me, double md, double mb0) {
        e = me;
        d = md;
        b0 = mb0;
        Rb = e + d * sin(b0);
    };

    /// Evaluate at once all important properties of cam,
    /// function of rotation 'par'
    /// (par in range 0..1, since 1 means 360°!):
    /// Gets point res, pressure angle g, curvature radius q.
    virtual void EvaluateCamPoint(double par, Vector& res, double& g, double& q);

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    /// Curve evaluation.
    /// Given a parameter "u", finds position on line of the
    /// kind p=p(u); note that u is in the range 0...1, to make a complete cycle along the cam
    virtual void Evaluate(Vector& pos, const double parU, const double parV = 0., const double parW = 0.);

    /// Weight evaluation.
    /// Given that the shape is defined by a Ch_function, the
    /// returned weight is the weight of the function (Ch_function_sequence can
    /// have different 'weight' values depending on the function segment)
    virtual double Get_weight(double par) { return law->Get_weight(par * 2 * CH_C_PI); };

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:

        eChCamType_mapper mmapper;
        marchive << CHNVP(mmapper(type),"type");
        marchive << CHNVP(law);
        marchive << CHNVP(phase);
        marchive << CHNVP(Rb);
        marchive << CHNVP(Rr);
        marchive << CHNVP(p);
        marchive << CHNVP(d);
        marchive << CHNVP(b0);
        marchive << CHNVP(e);
        marchive << CHNVP(s);
        marchive << CHNVP(negative);
        marchive << CHNVP(internal);
        marchive << CHNVP(center);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        eChCamType_mapper mmapper;
        marchive >> CHNVP(mmapper(type),"type");
        marchive >> CHNVP(law);
        marchive >> CHNVP(phase);
        marchive >> CHNVP(Rb);
        marchive >> CHNVP(Rr);
        marchive >> CHNVP(p);
        marchive >> CHNVP(d);
        marchive >> CHNVP(b0);
        marchive >> CHNVP(e);
        marchive >> CHNVP(s);
        marchive >> CHNVP(negative);
        marchive >> CHNVP(internal);
        marchive >> CHNVP(center);
    }

};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif  // END of header
