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

#ifndef CHLINKSPRING_H
#define CHLINKSPRING_H

#include "chrono/physics/ChLinkMarkers.h"

namespace chrono {

/// Class for spring-damper systems, acting along the polar
/// distance of two markers

class ChApi ChLinkSpring : public ChLinkMarkers {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkSpring)

  protected:
    double spr_restlength;                    ///< spring rest (undeformed) length
    double spr_k;                             ///< spring coefficient
    double spr_r;                             ///< damping coefficient
    double spr_f;                             ///< actuator force
    std::shared_ptr<ChFunction> mod_f_time;   ///< f(t)
    std::shared_ptr<ChFunction> mod_k_d;      ///< k(d)
    std::shared_ptr<ChFunction> mod_r_d;      ///< r(d)
    std::shared_ptr<ChFunction> mod_r_speed;  ///< k(speed)
    std::shared_ptr<ChFunction> mod_k_speed;  ///< r(speed)
    double spr_react;                         ///< resulting force in dist. coord / readonly

  public:
    ChLinkSpring();
    ChLinkSpring(const ChLinkSpring& other);
    virtual ~ChLinkSpring() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkSpring* Clone() const override { return new ChLinkSpring(*this); }

    // data fetch/store
    double Get_SpringRestLength() const { return spr_restlength; }
    double Get_SpringDeform() const { return (dist - spr_restlength); }
    double Get_SpringLength() const { return dist; }
    double Get_SpringVelocity() const { return dist_dt; }
    double Get_SpringK() const { return spr_k; }
    double Get_SpringR() const { return spr_r; }
    double Get_SpringF() const { return spr_f; }
    double Get_SpringReact() const { return spr_react; }

    void Set_SpringRestLength(double m_r) { spr_restlength = m_r; }
    void Set_SpringK(double m_r) { spr_k = m_r; }
    void Set_SpringR(double m_r) { spr_r = m_r; }
    void Set_SpringF(double m_r) { spr_f = m_r; }

    std::shared_ptr<ChFunction> Get_mod_f_time() const { return mod_f_time; }
    std::shared_ptr<ChFunction> Get_mod_k_d() const { return mod_k_d; }
    std::shared_ptr<ChFunction> Get_mod_r_d() const { return mod_r_d; }
    std::shared_ptr<ChFunction> Get_mod_k_speed() const { return mod_k_speed; }
    std::shared_ptr<ChFunction> Get_mod_r_speed() const { return mod_r_speed; }

    void Set_mod_f_time(std::shared_ptr<ChFunction> mf) { mod_f_time = mf; }
    void Set_mod_k_d(std::shared_ptr<ChFunction> mf) { mod_k_d = mf; }
    void Set_mod_r_d(std::shared_ptr<ChFunction> mf) { mod_r_d = mf; }
    void Set_mod_k_speed(std::shared_ptr<ChFunction> mf) { mod_k_speed = mf; }
    void Set_mod_r_speed(std::shared_ptr<ChFunction> mf) { mod_r_speed = mf; }

    /// Specialized initialization for springs, given the two bodies to be connected,
    /// the positions of the two anchor endpoints of the spring (each expressed
    /// in body or abs. coordinates) and the imposed rest length of the spring.
    /// NOTE! As in ChLinkMarkers::Initialize(), the two markers are automatically
    /// created and placed inside the two connected bodies.
    void Initialize(
        std::shared_ptr<ChBody> mbody1,  ///< first body to link
        std::shared_ptr<ChBody> mbody2,  ///< second body to link
        bool pos_are_relative,           ///< true: following pos. are relative to bodies
        ChVector<> mpos1,                ///< position of spring endpoint, for 1st body (rel. or abs., see flag above)
        ChVector<> mpos2,                ///< position of spring endpoint, for 2nd body (rel. or abs., see flag above)
        bool auto_rest_length = true,    ///< if true, initializes the rest-length as the distance between mpos1 and mpos2
        double mrest_length = 0          ///< imposed rest_length (no need to define, if auto_rest_length=true.)
        );

    /// Get the 1st spring endpoint (expressed in Body1 coordinate system)
    ChVector<> GetEndPoint1Rel() { return marker1->GetPos(); }
    /// Set the 1st spring endpoint (expressed in Body1 coordinate system)
    void SetEndPoint1Rel(const ChVector<>& mset) { marker1->Impose_Rel_Coord(ChCoordsys<>(mset, QUNIT)); }
    /// Get the 1st spring endpoint (expressed in absolute coordinate system)
    ChVector<> GetEndPoint1Abs() { return marker1->GetAbsCoord().pos; }
    /// Set the 1st spring endpoint (expressed in absolute coordinate system)
    void SetEndPoint1Abs(ChVector<>& mset) { marker1->Impose_Abs_Coord(ChCoordsys<>(mset, QUNIT)); }

    /// Get the 2nd spring endpoint (expressed in Body2 coordinate system)
    ChVector<> GetEndPoint2Rel() { return marker2->GetPos(); };
    /// Set the 2nd spring endpoint (expressed in Body2 coordinate system)
    void SetEndPoint2Rel(const ChVector<>& mset) { marker2->Impose_Rel_Coord(ChCoordsys<>(mset, QUNIT)); }
    /// Get the 1st spring endpoint (expressed in absolute coordinate system)
    ChVector<> GetEndPoint2Abs() { return marker2->GetAbsCoord().pos; }
    /// Set the 1st spring endpoint (expressed in absolute coordinate system)
    void SetEndPoint2Abs(ChVector<>& mset) { marker2->Impose_Abs_Coord(ChCoordsys<>(mset, QUNIT)); }

    /// Inherits, then also adds the spring custom forces to the C_force and C_torque.
    virtual void UpdateForces(double mytime) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkSpring,0)

}  // end namespace chrono

#endif
