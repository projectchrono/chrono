// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_LINECAM_H
#define CHC_LINECAM_H

#include <cmath>

#include "chrono/geometry/ChLine.h"
#include "chrono/motion_functions/ChFunction.h"

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

/// Geometric object describing the profile of a cam.
/// The shape of a cam is specified through a ChFunction which defines the motion law of the follower.

class ChApi ChLineCam : public ChLine {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLineCam)

  private:
    eChCamType type;                  ///< type of cam
    std::shared_ptr<ChFunction> law;  ///< follower motion law
    double phase;                     ///< 0..2PI  phase (rotation). Def=0, neutral position

    double Rb;  ///< base radius
    double Rr;  ///< radius of contact wheel

    double p;   ///< length of rotating mover
    double d;   ///< distance center of cam - center of rotation of mover
    double b0;  ///< initial rotation for mover

    double e;  ///< eccentricity of sliding follower
    double s;  ///< distance of sliding follower

    bool negative;  ///< negative cam: for desmodromic stuff, (cam is also Y or X mirrored, depend.on type )
    bool internal;  ///< follower roller is inside the cam

    ChVector<> center;  ///< center of cam in space (def.alignment on xy plane)

  public:
    ChLineCam();
    ChLineCam(const ChLineCam& source);
    ~ChLineCam() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCam* Clone() const override { return new ChLineCam(*this); }

    /// Get the class type as unique numerical ID.
    /// Each inherited class must return an unique ID.
    virtual GeometryType GetClassType() const override { return LINE_CAM; }

    virtual bool Get_closed() const override { return true; }
    virtual void Set_closed(bool mc) override {}

    void Set_Phase(double mf) { phase = mf; }
    double Get_Phase() const { return phase; }

    /// Base radius of cam
    void Set_Rb(double mrb);
    double Get_Rb() const { return Rb; }

    /// Radius of contact wheel
    void Set_Rr(double mrr) { Rr = mrr; }
    double Get_Rr() const { return Rr; }

    /// The motion law, as a ChFunction.
    void Set_motion_law(std::shared_ptr<ChFunction> mlaw) { law = mlaw; }

    std::shared_ptr<ChFunction> Get_motion_law() const { return law; }

    /// Type of cam (see the eChCamType enum values below).
    void Set_type(eChCamType mt) { type = mt; }
    eChCamType Get_type() const { return type; }

    /// position of center of cam in 3d space.
    void Set_center(ChVector<> mc) { center = mc; }
    ChVector<> Get_center() const { return center; }

    /// If true, creates a negative cam.
    void Set_Negative(bool val) { negative = val; }
    bool Get_Negative() const { return negative; }

    /// If true, creates an internal cam.
    void Set_Internal(bool val) { internal = val; }
    bool Get_Internal() const { return internal; }

    /// Sets the data for the rotating follower (length, distance from cam center, initial phase mb0)
    void Set_rotating_follower(double mp, double md, double mb0);
    double Get_p() const { return p; }
    double Get_d() const { return d; }
    double Get_b0() const { return b0; }

    /// Sets the data for the sliding follower (if eccentrical, with me eccentricity)
    void Set_sliding_eccentrical(double me) { e = me; };
    double Get_e() const { return e; }
    double Get_s() const { return sqrt(Rb * Rb - e * e); }

    /// Sets the data for the flat rotating follower (length, distance from cam center, initial phase mb0)
    void Set_flat_oscillate(double me, double md, double mb0);

    /// Evaluate at once all important properties of cam,
    /// function of rotation 'par'
    /// (par in range 0..1, since 1 means 360°!):
    /// Gets point res, pressure angle g, curvature radius q.
    void EvaluateCamPoint(double par, ChVector<>& res, double& g, double& q) const;

    /// Curve evaluation.
    /// Given a parameter "u", finds position on line of the
    /// kind p=p(u); note that u is in the range 0...1, to make a complete cycle along the cam
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    /// Weight evaluation.
    /// Given that the shape is defined by a Ch_function, the
    /// returned weight is the weight of the function (Ch_function_sequence can
    /// have different 'weight' values depending on the function segment)
    double Get_weight(double par) const { return law->Get_weight(par * 2 * CH_C_PI); }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLineCam>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:

        eChCamType_mapper mmapper;
        marchive << CHNVP(mmapper(type), "type");
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
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLineCam>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        eChCamType_mapper mmapper;
        marchive >> CHNVP(mmapper(type), "type");
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

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLineCam,0)

}  // end namespace chrono

#endif