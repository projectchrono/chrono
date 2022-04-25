// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// Template for a PAC02 tire model
//
// ChPac02Tire is based on the Pacejka 2002 formulae as written in
// Hans B. Pacejka's "Tire and Vehicle Dynamics" Third Edition, Elsevier 2012
// ISBN: 978-0-08-097016-5
//
// Actually implemented:
// - steady state longitudinal, lateral force, alignment torque, overturning torque
// - can run in combined (Pacejka or Friction Ellipsis Method) or uncombined mode
//
// Aim of this implementation is the replacement of ChPacejkaTire, which is more
// complete but unreliable in practical usage.
// =============================================================================

#ifndef CH_PAC02TIRE_H
#define CH_PAC02TIRE_H

#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Pacjeka 02 tire model.
class CH_VEHICLE_API ChPac02Tire : public ChForceElementTire {
  public:
    ChPac02Tire(const std::string& name);

    virtual ~ChPac02Tire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Pac02Tire"; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_tireforce; }

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return m_PacCoeff.width; }

    /// Get the tire deflection
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return m_PacCoeff.width; }

    /// Get the slip angle used in Pac89 (expressed in radians).
    /// The reported value will have opposite sign to that reported by ChTire::GetSlipAngle
    /// because ChPac89 uses internally a different frame convention.
    double GetSlipAngle_internal() const { return m_states.cp_side_slip; }

    /// Get the longitudinal slip used in Pac89.
    /// The reported value will be similar to that reported by ChTire::GetLongitudinalSlip.
    double GetLongitudinalSlip_internal() const { return m_states.cp_long_slip; }

    /// Get the camber angle used in Pac89 (expressed in radians).
    /// The reported value will be similar to that reported by ChTire::GetCamberAngle.
    double GetCamberAngle_internal() { return m_gamma * CH_C_DEG_TO_RAD; }

  protected:
    /// Set the parameters in the Pac89 model.
    virtual void SetPac02Params() = 0;

    double m_kappa;  ///< longitudinal slip ratio
    double m_alpha;  ///< slip angle
    double m_gamma;  ///< camber angle

    double m_gamma_limit;  ///< limit camber angle

    bool m_use_friction_ellipsis;

    /// Road friction
    double m_mu;

    double m_Shf;
    double m_Cy;
    double m_By;

    VehicleSide m_measured_side;
    bool m_allow_mirroring;

    unsigned int m_use_mode;
    // combined forces calculation
    double m_kappa_c;
    double m_alpha_c;
    double m_mu_x_act;
    double m_mu_x_max;
    double m_mu_y_act;
    double m_mu_y_max;

    struct Pac02ScalingFactors {
        double lfz0;
        double lcx;
        double lex;
        double lkx;
        double lhx;
        double lmux;
        double lvx;
        double lxal;
        double lmx;
        double lvmx;
        double lmy;

        double lcy;
        double ley;
        double lhy;
        double lky;
        double lmuy;
        double lvy;
        double lyka;
        double lvyka;
        double ltr;
        double lgax;
        double lgay;
        double lgaz;
        double lres;
        double lsgkp;
        double lsgal;
        double lgyr;
        double ls;
    };

    struct Pac02Coeff {
        double mu0;           // road friction coefficient at test conditions for the handling parameters
        double R0;            // unloaded radius
        double width;         // tire width
        double aspect_ratio;  // actually unused
        double rim_width;     // actually unused
        double rim_radius;    // actually unused
        double FzNomin;       // nominla wheel load
        double Cz;            // vertical tire stiffness
        double Kz;            // vertical tire damping

        // Longitudinal Coefficients
        double pcx1;
        double pdx1;
        double pdx2;
        double pdx3;
        double pex1;
        double pex2;
        double pex3;
        double pex4;
        double phx1;
        double phx2;
        double pkx1;
        double pkx2;
        double pkx3;
        double pvx1;
        double pvx2;
        double rbx1;
        double rbx2;
        double rbx3;
        double rcx1;
        double rex1;
        double rex2;
        double rhx1;
        double ptx1;
        double ptx2;
        double ptx3;
        double ptx4;

        // overturning coefficients
        double qsx1;
        double qsx2;
        double qsx3;
        double qsx4;
        double qsx5;
        double qsx6;
        double qsx7;
        double qsx8;
        double qsx9;
        double qsx10;
        double qsx11;

        // rolling resistance coefficients
        double qsy1;
        double qsy2;
        double qsy3;
        double qsy4;
        double qsy5;
        double qsy6;
        double qsy7;
        double qsy8;

        // Lateral Coefficients
        double pcy1;
        double pdy1;
        double pdy2;
        double pdy3;
        double pey1;
        double pey2;
        double pey3;
        double pey4;
        double pey5;
        double phy1;
        double phy2;
        double phy3;
        double pky1;
        double pky2;
        double pky3;
        double pvy1;
        double pvy2;
        double pvy3;
        double pvy4;
        double rby1;
        double rby2;
        double rby3;
        double rby4;
        double rcy1;
        double rey1;
        double rey2;
        double rhy1;
        double rhy2;
        double rvy1;
        double rvy2;
        double rvy3;
        double rvy4;
        double rvy5;
        double rvy6;
        double pty1;
        double pty2;

        // alignment coefficients
        double qbz1;
        double qbz2;
        double qbz3;
        double qbz4;
        double qbz5;
        double qbz6;
        double qbz9;
        double qbz10;
        double qcz1;
        double qdz1;
        double qdz2;
        double qdz3;
        double qdz4;
        double qdz5;
        double qdz6;
        double qdz7;
        double qdz8;
        double qdz9;
        double qez1;
        double qez2;
        double qez3;
        double qez4;
        double qez5;
        double qhz1;
        double qhz2;
        double qhz3;
        double qhz4;
        double ssz1;
        double ssz2;
        double ssz3;
        double ssz4;
        double qtz1;
        double mbelt;
    };

    Pac02ScalingFactors m_PacScal;
    Pac02Coeff m_PacCoeff;

  //private:
    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TerrainForce GetTireForce() const override { return m_tireforce; }

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    struct ContactData {
        bool in_contact;      // true if disc in contact with terrain
        ChCoordsys<> frame;   // contact frame (x: long, y: lat, z: normal)
        ChVector<> vel;       // relative velocity expressed in contact frame
        double normal_force;  // magnitude of normal contact force
        double depth;         // penetration depth
    };

    struct TireStates {
        double cp_long_slip;     // Contact Path - Longitudinal Slip State (Kappa)
        double cp_side_slip;     // Contact Path - Side Slip State (Alpha)
        double vx;               // Longitudinal speed
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis
        double R_eff;            // Effective Radius
        ChVector<> disc_normal;  //(temporary for debug)
    };

    ChFunction_Recorder m_areaDep;  // lookup table for estimation of penetration depth from intersection area

    ContactData m_data;
    TireStates m_states;

    TerrainForce m_tireforce;

    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset

    double CalcFx(double kappa, double Fz, double gamma);
    double CalcFy(double alpha, double Fz, double gamma);
    double CalcMx(double Fy, double Fz, double gamma);
    double CalcMy(double Fx, double Fy, double gamma);
    double CalcMz(double alpha, double Fz, double gamma, double Fy);
    double CalcTrail(double alpha, double Fz, double gamma);
    double CalcMres(double alpha, double Fz, double gamma);
    double CalcFxComb(double kappa, double alpha, double Fz, double gamma);
    double CalcFyComb(double kappa, double alpha, double Fz, double gamma);
    double CalcMzComb(double kappa, double alpha, double Fz, double gamma, double Fx, double Fy);
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
