// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Marvin Struijk, Rainer Gericke
// =============================================================================
//
// Template for a tire model based on the Pacejka MF5.2-6.2 tire model
// Implementation is based on: https://mfeval.wordpress.com/usingmfeval/
//
// =============================================================================
// UNDER DEVELOPMENT
// =============================================================================

#ifndef CH_MFTIRE_H
#define CH_MFTIRE_H

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
class CH_VEHICLE_API ChMFTire : public ChForceElementTire {
  public:
    ChMFTire(const std::string& name);

    virtual ~ChMFTire() {}

    virtual void SetMFParams() {};

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ChMFTire"; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return par.WIDTH; }

    /// Get the tire deflection
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return par.WIDTH; }

    /// Get the slip angle used in Pac89 (expressed in radians).
    /// The reported value will have opposite sign to that reported by ChTire::GetSlipAngle because ChPac89 uses
    /// internally a different frame convention.
    // double GetSlipAngle_internal() const { return m_states.cp_side_slip; }
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

    // bool m_use_friction_ellipsis;

    /// Road friction
    double m_mu = 1.0;

    // default road friction
    double m_mu0 = 1.0;

    // double m_Shf;
    // double m_Cy;
    // double m_By;

    // VehicleSide m_measured_side;
    // bool m_allow_mirroring;

    bool useStarInputs = true;
    bool useTurnSlip = false;

    bool useIsoRef = false;

    bool flipSide = false;

    unsigned int m_use_mode;

    // Internal variable for tire forces to be used in Fz calculation
    double m_fx_fb = 0.0;
    double m_fy_fb = 0.0;

    // Internal variables for norm. delta pressure and load
    double m_dpi;
    double m_dfz;
    double m_fz_scl;
    double m_fz;
    double m_fz_unlim;

    // Grip scaling factors
    double LMUX_star = 1.0;
    double LMUY_star = 1.0;
    double LMUX_prime = 1.0;
    double LMUY_prime = 1.0;

    // Auxilary variables for internal calculation
    // double m_Fy0_sub0 = 0.0;
    // double m_Gyk_sub0 = 0.0;
    double m_Fy_prime = 0.0;
    double m_By = 0.0;
    double m_Cy = 0.0;
    double m_Gyk = 0.0;
    double m_SHy = 0.0;
    double m_SVy = 0.0;
    double m_SVyk = 0.0;
    double m_Kxk = 0.0;
    double m_Kya = 0.0;
    double m_muy = 0.0;

    // Transient slip quantities
    double epsilonx = 0.0;
    double epsilony = 0.0;

    double m_sigma_x = 0.0;
    double m_sigma_y = 0.0;

    typedef struct TireData {
        /*
        std::string FILE_NAME = "";
        std::string FILE_TYPE = "default";
        std::string FUNCTION_NAME = "default";
        std::string FILE_FORMAT = "default";
        std::string LENGTH = "default";
        std::string FORCE = "default";
        std::string ANGLE = "default";
        std::string MASS = "default";
        std::string TIME = "default";
        // std::string TYRESIDE = "LEFT";
        std::string PROPERTY_FILE_FORMAT = "default";
        */

        VehicleSide TYRESIDE = LEFT;

        int FITTYP = 0;
        int SWITCH_INTEG = 0;

        double FILE_VERSION = 3.0;
        double LONGVL = 16.7;
        double VXLOW = 1;
        double ROAD_INCREMENT = 0.01;
        double ROAD_DIRECTION = 1;
        double USER_SUB_ID = 815;
        double N_TIRE_STATES = 4;
        double USE_MODE = 124;
        double HMAX_LOCAL = 2.5E-4;
        double TIME_SWITCH_INTEG = 0.1;
        double UNLOADED_RADIUS = 0.3135;
        double WIDTH = 0.205;
        double ASPECT_RATIO = 0.6;
        double RIM_RADIUS = 0.1905;
        double RIM_WIDTH = 0.152;
        double INFLPRES = 220000;
        double NOMPRES = 220000;
        double MASS1 = 9.3;
        double IXX = 0.391;
        double IYY = 0.736;
        double BELT_MASS = 7;
        double BELT_IXX = 0.34;
        double BELT_IYY = 0.6;
        double GRAVITY = -9.81;
        double FNOMIN = 4000;
        double VERTICAL_STIFFNESS = 200000;
        double VERTICAL_DAMPING = 50;
        double MC_CONTOUR_A = 0.5;
        double MC_CONTOUR_B = 0.5;
        double BREFF = 8.4;
        double DREFF = 0.27;
        double FREFF = 0.07;
        double Q_RE0 = 1;
        double Q_V1 = 0;
        double Q_V2 = 0;
        double Q_FZ2 = 1.0E-4;
        double Q_FCX = 0;
        double Q_FCY = 0;
        double Q_CAM = 0;
        double PFZ1 = 0;
        double BOTTOM_OFFST = 0.01;
        double BOTTOM_STIFF = 2000000;
        double LONGITUDINAL_STIFFNESS = 300000;
        double LATERAL_STIFFNESS = 100000;
        double YAW_STIFFNESS = 5000;
        double FREQ_LONG = 80;
        double FREQ_LAT = 40;
        double FREQ_YAW = 50;
        double FREQ_WINDUP = 70;
        double DAMP_LONG = 0.04;
        double DAMP_LAT = 0.04;
        double DAMP_YAW = 0.04;
        double DAMP_WINDUP = 0.04;
        double DAMP_RESIDUAL = 0.0020;
        double DAMP_VLOW = 0.0010;
        double Q_BVX = 0;
        double Q_BVT = 0;
        double PCFX1 = 0;
        double PCFX2 = 0;
        double PCFX3 = 0;
        double PCFY1 = 0;
        double PCFY2 = 0;
        double PCFY3 = 0;
        double PCMZ1 = 0;
        double Q_RA1 = 0.5;
        double Q_RA2 = 1;
        double Q_RB1 = 1;
        double Q_RB2 = -1;
        double ELLIPS_SHIFT = 0.8;
        double ELLIPS_LENGTH = 1;
        double ELLIPS_HEIGHT = 1;
        double ELLIPS_ORDER = 1.8;
        double ELLIPS_MAX_STEP = 0.025;
        double ELLIPS_NWIDTH = 10;
        double ELLIPS_NLENGTH = 10;
        double PRESMIN = 10000;
        double PRESMAX = 1000000;
        double FZMIN = 100;
        double FZMAX = 10000;
        double KPUMIN = -1.5;
        double KPUMAX = 1.5;
        double ALPMIN = -1.5;
        double ALPMAX = 1.5;
        double CAMMIN = -0.175;
        double CAMMAX = 0.175;
        double LFZO = 1;
        double LCX = 1;
        double LMUX = 1;
        double LEX = 1;
        double LKX = 1;
        double LHX = 1;
        double LVX = 1;
        double LCY = 1;
        double LMUY = 1;
        double LEY = 1;
        double LKY = 1;
        double LHY = 1;
        double LVY = 1;
        double LTR = 1;
        double LRES = 1;
        double LXAL = 1;
        double LYKA = 1;
        double LVYKA = 1;
        double LS = 1;
        double LKYC = 1;
        double LKZC = 1;
        double LVMX = 1;
        double LMX = 1;
        double LMY = 1;
        double LMP = 1;
        double PCX1 = 1.65;
        double PDX1 = 1.3;
        double PDX2 = -0.15;
        double PDX3 = 0;
        double PEX1 = 0;
        double PEX2 = 0;
        double PEX3 = 0;
        double PEX4 = 0;
        double PKX1 = 20;
        double PKX2 = 0;
        double PKX3 = 0;
        double PHX1 = 0;
        double PHX2 = 0;
        double PVX1 = 0;
        double PVX2 = 0;
        double PPX1 = 0;
        double PPX2 = 0;
        double PPX3 = 0;
        double PPX4 = 0;
        double RBX1 = 20;
        double RBX2 = 15;
        double RBX3 = 0;
        double RCX1 = 1;
        double REX1 = 0;
        double REX2 = 0;
        double RHX1 = 0;
        double QSX1 = 0;
        double QSX2 = 0;
        double QSX3 = 0;
        double QSX4 = 5;
        double QSX5 = 1;
        double QSX6 = 10;
        double QSX7 = 0;
        double QSX8 = 0;
        double QSX9 = 0.4;
        double QSX10 = 0;
        double QSX11 = 5;
        double QSX12 = 0;
        double QSX13 = 0;
        double QSX14 = 0;
        double PPMX1 = 0;
        double PCY1 = 1.3;
        double PDY1 = 1.1;
        double PDY2 = -0.15;
        double PDY3 = 0;
        double PEY1 = 0;
        double PEY2 = 0;
        double PEY3 = 0;
        double PEY4 = 0;
        double PEY5 = 0;
        double PKY1 = -20;
        double PKY2 = 1;
        double PKY3 = 0;
        double PKY4 = 2;
        double PKY5 = 0;
        double PKY6 = -1;
        double PKY7 = 0;
        double PHY1 = 0;
        double PHY2 = 0;
        double PVY1 = 0;
        double PVY2 = 0;
        double PVY3 = 0;
        double PVY4 = 0;
        double PPY1 = 0;
        double PPY2 = 0;
        double PPY3 = 0;
        double PPY4 = 0;
        double PPY5 = 0;
        double RBY1 = 10;
        double RBY2 = 10;
        double RBY3 = 0;
        double RBY4 = 0;
        double RCY1 = 1;
        double REY1 = 0;
        double REY2 = 0;
        double RHY1 = 0;
        double RHY2 = 0;
        double RVY1 = 0;
        double RVY2 = 0;
        double RVY3 = 0;
        double RVY4 = 20;
        double RVY5 = 2;
        double RVY6 = 10;
        double QSY1 = 0.01;
        double QSY2 = 0;
        double QSY3 = 4.0E-4;
        double QSY4 = 4.0E-5;
        double QSY5 = 0;
        double QSY6 = 0;
        double QSY7 = 0.85;
        double QSY8 = -0.4;
        double QBZ1 = 10;
        double QBZ2 = 0;
        double QBZ3 = 0;
        double QBZ4 = 0;
        double QBZ5 = 0;
        double QBZ9 = 10;
        double QBZ10 = 0;
        double QCZ1 = 1.1;
        double QDZ1 = 0.12;
        double QDZ2 = 0;
        double QDZ3 = 0;
        double QDZ4 = 0;
        double QDZ6 = 0;
        double QDZ7 = 0;
        double QDZ8 = -0.05;
        double QDZ9 = 0;
        double QDZ10 = 0;
        double QDZ11 = 0;
        double QEZ1 = 0;
        double QEZ2 = 0;
        double QEZ3 = 0;
        double QEZ4 = 0;
        double QEZ5 = 0;
        double QHZ1 = 0;
        double QHZ2 = 0;
        double QHZ3 = 0;
        double QHZ4 = 0;
        double PPZ1 = 0;
        double PPZ2 = 0;
        double SSZ1 = 0;
        double SSZ2 = 0;
        double SSZ3 = 0;
        double SSZ4 = 0;
        double PDXP1 = 0.4;
        double PDXP2 = 0;
        double PDXP3 = 0;
        double PKYP1 = 1;
        double PDYP1 = 0.4;
        double PDYP2 = 0;
        double PDYP3 = 0;
        double PDYP4 = 0;
        double PHYP1 = 1;
        double PHYP2 = 0.15;
        double PHYP3 = 0;
        double PHYP4 = -4;
        double PECP1 = 0.5;
        double PECP2 = 0;
        double QDTP1 = 10;
        double QCRP1 = 0.2;
        double QCRP2 = 0.1;
        double QBRP1 = 0.1;
        double QDRP1 = 1;
        double Q_FCY2 = 0;
        double Q_CAM1 = 0;
        double Q_CAM2 = 0;
        double Q_CAM3 = 0;
        double Q_FYS1 = 0;
        double Q_FYS2 = 0;
        double Q_FYS3 = 0;
        double ENV_C1 = 0;
        double ENV_C2 = 0;
        double Q_A1 = 0;
        double Q_A2 = 0;
        double PHY3 = 0;
        double PTX1 = 0;
        double PTX2 = 0;
        double PTX3 = 0;
        double PTY1 = 0;
        double PTY2 = 0;
        double LSGKP = 1;
        double LSGAL = 1;
    } TireData;

    TireData par;

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
        double cp_long_slip;  // Contact Path - Longitudinal Slip State (Kappa)
        double cp_side_slip;  // Contact Path - Side Slip State (Alpha)
        double vx;            // Longitudinal speed
        double vsx;           // Longitudinal slip velocity
        double vsy;           // Lateral slip velocity = Lateral velocity
        double omega;         // Wheel angular velocity about its spin axis
        double R_eff;         // Effective Radius
        double p;
        ChVector<> disc_normal;  //(temporary for debug)
    };

    void CopyContactData();
    void CopyTireStates();
    void CalculateForcesMoments(double step);

    TireStates m_states;
    TireStates m_states_in;
    ContactData m_data_in;
    TerrainForce m_tireforce_out;

    std::shared_ptr<ChVisualShape> m_cyl_shape;  ///< visualization cylinder asset

    double CalculateGamma(bool flip);
    double CalculatePressureDifference(double p);
    double CalculateEffectiveRollingRadius(double omega, double dpi, double Fz);

    void CalculateRelaxationLength(double Fz, double gamma, double dfz, double dpi);

    void TransientSlipNonlinear(double step,
                                double vx,
                                double vsx,
                                double vsy,
                                double Fx,
                                double Fy,
                                double Fz,
                                double dpi,
                                double dfz);

    void TransientSlip(double step, double vx, double vsx, double vsy, double sigma_x, double sigma_y);

    void CalculateFrictionScaling(double mu, double Vs);
    double ContactPatch(double dpi, double Fz);

    double CalculateMx(double Fz, double gamma, double dpi, double Fy);
    double CalculateMy(double Fz, double gamma, double dpi, double Fx, double Vcx);
    double CalculateMz(double Fz,
                       double kappa,
                       double alpha,
                       double alpha_prime,
                       double gamma,
                       double dfz,
                       double dpi,
                       double phi,
                       double Fx,
                       double Fy,
                       double Vcx);
    double CalculateFx(double Fz, double kappa, double alpha, double gamma, double dfz, double dpi, double phi);
    double
    CalculateFy(double Fz, double kappa, double alpha, double gamma, double dfz, double dpi, double phi, double Vcx);
    double CalculateFz(double depth, double omega, double gamma, double p, double Fx, double Fy);
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
