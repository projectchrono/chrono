// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// Base class for a Pacejka type Magic formula 2002 tire model
//
// =============================================================================

#ifndef CH_PACEJKATIRE_H
#define CH_PACEJKATIRE_H

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac2002_data.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

// Forward declarations for private structures
struct slips;
struct Pac2002_data;
struct pureLongCoefs;
struct pureLatCoefs;
struct pureTorqueCoefs;
struct combinedLongCoefs;
struct combinedLatCoefs;
struct combinedTorqueCoefs;
struct zetaCoefs;
struct relaxationL;
struct bessel;

/// Concrete tire class that implements the Pacejka tire model.
class CH_VEHICLE_API ChPacejkaTire : public ChTire {
  public:
    /// Default constructor for a Pacejka tire.
    /// Construct a Pacejka tire for which the vertical load is calculated
    /// internally.  The model includes transient slip calculations.
    /// chrono can suggest a time step for use with the ODE slips
    ChPacejkaTire(const std::string& name,              ///< [in] name of this tire
                  const std::string& pacTire_paramFile  ///< [in] name of the parameter file
                  );

    /// Construct a Pacejka tire with specified vertical load, for testing purposes
    ChPacejkaTire(const std::string& name,               ///< [in] name of this tire
                  const std::string& pacTire_paramFile,  ///< [in] name of the parameter file
                  double Fz_override,                    ///< [in] prescribed vertical load
                  bool use_transient_slip = true         ///< [in] indicate if using transient slip model
                  );

    ~ChPacejkaTire();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "PacejkaTire"; }

    /// Specify whether or not the associated wheel is driven.
    /// By default, the wheel is assumed not driven.
    void SetDrivenWheel(bool val) { m_driven = val; }

    /// specify the file name to read the Pactire input from
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< handle to the associated wheel body
                            VehicleSide side                ///< [in] left/right vehicle side
                            ) override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_R_eff; }

    /// Get visualization tire width.
    virtual double GetVisualizationWidth() const { return 0.25; }

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TerrainForce GetTireForce() const override;

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    ///  Return the reactions for the pure slip EQs, in local or global coords
    TerrainForce GetTireForce_pureSlip(const bool local = true) const;

    /// Return the reactions for the combined slip EQs, in local or global coords
    TerrainForce GetTireForce_combinedSlip(const bool local = true) const;

    /// Update the state of this tire system at the current time.
    /// Set the PacTire spindle state data from the global wheel body state.
    virtual void Synchronize(double time,                    ///< [in] current time
                             const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                             const ChTerrain& terrain,       ///< [in] reference to the terrain system
                             CollisionType collision_type = CollisionType::SINGLE_POINT  ///< [in] collision type
                             ) override;

    /// Get the tire slip angle computed internally by the Pacejka model (in radians).
    /// The reported value will be the same as that reported by ChTire::GetSlipAngle.
    double GetSlipAngle_internal() const { return m_slip->alpha; }

    /// Get the tire longitudinal slip computed internally by the Pacejka model.
    /// The reported value will be the same as that reported by ChTire::GetLongitudinalSlip.
    double GetLongitudinalSlip_internal() const { return m_slip->kappa; }

    /// Get the camber angle for the Pacejka tire model (in radians).
    /// The reported value will be the same as that reported by ChTire::GetCamberAngle.
    double GetCamberAngle_internal() const { return m_slip->gamma; }

    /// Advance the state of this tire by the specified time step.
    /// Use the new body state, calculate all the relevant quantities over the
    /// time increment.
    virtual void Advance(double step) override;

    /// Write output data to a file.
    void WriteOutData(double time, const std::string& outFilename);

    /// Manually set the vertical wheel load as an input.
    void set_Fz_override(double Fz) { m_Fz_override = Fz; }

    /// Return orientation, Vx (global) and omega/omega_y (global).
    /// Assumes the tire is going straight forward (global x-dir), and the
    /// returned state's orientation yields gamma and alpha, as x and z NASA angles
    WheelState getState_from_KAG(double kappa,  ///< [in] ...
                                 double alpha,  ///< [in] ...
                                 double gamma,  ///< [in] ...
                                 double Vx      ///< [in] tire forward velocity x-dir
                                 );

    /// Get the average simulation time per step spent in advance()
    double get_average_Advance_time() { return m_sum_Advance_time / (double)m_num_Advance_calls; }

    /// Get the average simulation time per step spent in calculating ODEs
    double get_average_ODE_time() { return m_sum_ODE_time / (double)m_num_ODE_calls; }

    /// Get current wheel longitudinal slip.
    double get_kappa() const;

    /// Get current wheel slip angle.
    double get_alpha() const;

    /// Get current wheel camber angle.
    double get_gamma() const;

    /// Get current long slip rate used in Magic Formula EQs.
    double get_kappaPrime() const;

    /// Get current slip angle using in Magic Formula.
    double get_alphaPrime() const;

    /// Get current camber angle used in Magic Formula.
    double get_gammaPrime() const;

    /// Get minimum longitudinal slip rate.
    double get_min_long_slip() const;

    /// Get maximum longitudinal slip rate.
    double get_max_long_slip() const;

    /// Get the minimum allowable lateral slip angle, alpha.
    double get_min_lat_slip() const;

    /// Get the maximum allowable lateral slip angle, alpha.
    double get_max_lat_slip() const;

    /// Get the longitudinal velocity.
    double get_longvl() const;

    /// Get the tire rolling radius, ideally updated each step.
    double get_tire_rolling_rad() const { return m_R_eff; }

  protected:
    const std::string& getPacTireParamFile() const { return m_paramFile; }
    std::string m_paramFile;  // input parameter file

  private:
    virtual void Create(const rapidjson::Document& d) override {}

    // look for this data file
    void loadPacTireParamFile();

    // once Pac tire input text file has been successfully opened, read the input
    // data, and populate the data struct
    void readPacTireInput(std::ifstream& inFile);

    // functions for reading each section in the parameter file
    void readSection_UNITS(std::ifstream& inFile);
    void readSection_MODEL(std::ifstream& inFile);
    void readSection_DIMENSION(std::ifstream& inFile);
    void readSection_SHAPE(std::ifstream& inFile);
    void readSection_VERTICAL(std::ifstream& inFile);
    void readSection_RANGES(std::ifstream& inFile);
    void readSection_scaling(std::ifstream& inFile);
    void readSection_longitudinal(std::ifstream& inFile);
    void readSection_overturning(std::ifstream& inFile);
    void readSection_lateral(std::ifstream& inFile);
    void readSection_rolling(std::ifstream& inFile);
    void readSection_aligning(std::ifstream& inFile);

    /// update the tire contact coordinate system, TYDEX W-Axis
    /// checks for contact, sets m_in_contact and m_depth
    void update_W_frame(const ChTerrain& terrain, CollisionType collisionType);

    // update the vertical load, tire deflection, and tire rolling radius
    void update_verticalLoad(double step);

    // find the vertical load, using a spring-damper model
    double calc_Fz();

    // calculate the various stiffness/relaxation lengths
    void relaxationLengths();

    // calculate the slips assume the steer/throttle/brake input to wheel has an
    // instantaneous effect on contact patch slips
    void slip_kinematic();

    // set the tire m_slip vector to all zeros
    void zero_slips();

    // check if slips fall within a specified valid range
    void evaluate_slips();

    // after calculating all the reactions, evaluate output for any fishy business
    // write_violations: any output threshold exceeded gets written to console
    // enforce_threshold: enforce the limits when forces/moments exceed thresholds (except on Fz)
    void evaluate_reactions(bool write_violations, bool enforce_threshold);

    void advance_tire(double step);

    // calculate transient slip properties, using first order ODEs to find slip
    // displacements from velocities
    // appends m_slips for the slip displacements, and integrated slip velocity terms
    void advance_slip_transient(double step);

    // calculate the increment delta_x using RK 45 integration for linear u, v_alpha
    // Eq. 7.9 and 7.7, respectively
    double ODE_RK_uv(double V_s,        // slip velocity
                     double sigma,      // either sigma_kappa or sigma_alpha
                     double V_cx,       // wheel body center velocity
                     double step_size,  // the simulation timestep size
                     double x_curr);    // f(x_curr)

    // calculate the linearized slope of Fy vs. alphaP at the current value
    double get_dFy_dtan_alphaP(double x_curr);

    // calculate v_alpha differently at low speeds
    // relaxation length is non-linear
    double ODE_RK_kappaAlpha(double V_sy, double V_cx, double C_Fy, double step_size, double x_curr);

    // calculate the increment delta_gamma of the gamma ODE
    // Note: gamma should be the kinematic camber, gamma
    double ODE_RK_gamma(double C_Fgamma,
                        double C_Falpha,
                        double sigma_alpha,
                        double V_cx,
                        double step_size,
                        double gamma,
                        double v_gamma);

    // calculate the ODE dphi/dt = f(phi), return the increment delta_phi
    double ODE_RK_phi(double C_Fphi,
                      double C_Falpha,
                      double V_cx,
                      double psi_dot,
                      double omega,
                      double gamma,
                      double sigma_alpha,
                      double v_phi,
                      double eps_gamma,
                      double step_size);

    // calculate and set the transient slip values (kappaP, alphaP, gammaP) from
    // u, v deflections. optionally turn on/off Besselink low velocity damping
    void slip_from_uv(bool use_besselink = true,
                      double bessel_Cx = 350.0,
                      double bessel_Cy = 200.0,
                      double V_low = 5.0);

    /// calculate the reaction forces and moments, pure slip cases
    /// assign longitudinal, lateral force, aligning moment:
    /// Fx, Fy and Mz
    void pureSlipReactions();

    /// calculate combined slip reactions
    /// assign Fx, Fy, Mz
    void combinedSlipReactions();

    /// longitudinal force, alpha ~= 0
    /// assign to m_FM.force.x
    /// assign m_pureLong, trigonometric function calculated constants
    double Fx_pureLong(double gamma, double kappa);

    /// lateral force,  kappa ~= 0
    /// assign to m_FM.force.y
    /// assign m_pureLong, trigonometric function calculated constants
    double Fy_pureLat(double alpha, double gamma);

    /// aligning moment,  kappa ~= 0
    /// assign to m_FM.moment.z
    /// assign m_pureLong, trigonometric function calculated constants
    double Mz_pureLat(double alpha, double gamma, double Fy_pureSlip);

    /// longitudinal force, combined slip (general case)
    /// assign m_FM_combined.force.x
    /// assign m_combinedLong
    double Fx_combined(double alpha, double gamma, double kappa, double Fx_pureSlip);

    /// calculate lateral force, combined slip (general case)
    /// assign m_FM_combined.force.y
    /// assign m_combinedLat
    double Fy_combined(double alpha, double gamma, double kappa, double Fy_pureSlip);

    // calculate aligning torque, combined slip (general case)
    /// assign m_FM_combined.moment.z
    /// assign m_combinedTorque
    double Mz_combined(double alpha_r,
                       double alpha_t,
                       double gamma,
                       double kappa,
                       double Fx_combined,
                       double Fy_combined);

    /// calculate the overturning couple moment
    /// assign m_FM.moment.x and m_FM_combined.moment.x
    double calc_Mx(double gamma, double Fy_combined);

    /// calculate the rolling resistance moment,
    /// assign m_FM.moment.y and m_FM_combined.moment.y
    double calc_My(double Fx_combined);

    // ----- Data members
    bool m_use_transient_slip;
    bool m_driven;   // is this a driven tire?
    int m_sameSide;  // does parameter file side equal m_side? 1 = true, -1 opposite

    WheelState m_tireState;  // tire state, global coordinates
    ChCoordsys<> m_W_frame;  // tire contact coordinate system, TYDEX W-Axis
    double m_simTime;        // Chrono simulation time

    bool m_in_contact;  // indicates if there is tire-terrain contact
    double m_depth;     // if in contact, this is the contact depth

    double m_R0;     // unloaded radius
    double m_R_eff;  // current effect rolling radius
    double m_R_l;    // statically loaded radius

    double m_Fz;             // vertical load to use in Magic Formula
    double m_dF_z;           // (Fz - Fz,nom) / Fz,nom
    bool m_use_Fz_override;  // calculate Fz using collision, or user input
    double m_Fz_override;    // if manually inputting the vertical wheel load

    double m_time_since_last_step;  // init. to -1 in Initialize()
    bool m_initial_step;            // so Advance() gets called at time = 0
    int m_num_ODE_calls;
    double m_sum_ODE_time;
    int m_num_Advance_calls;
    double m_sum_Advance_time;

    TerrainForce m_FM_pure;      // output tire forces, based on pure slip
    TerrainForce m_FM_combined;  // output tire forces, based on combined slip
    // previous steps calculated reaction
    TerrainForce m_FM_pure_last;
    TerrainForce m_FM_combined_last;

    // TODO: could calculate these using sigma_kappa_adams and sigma_alpha_adams, in getRelaxationLengths()
    // HARDCODED IN Initialize() for now
    double m_C_Fx;
    double m_C_Fy;

    std::string m_outFilename;  // output filename

    int m_Num_WriteOutData;  // number of times WriteOut was called

    bool m_params_defined;  // indicates if model params. have been defined/loaded

    // MODEL PARAMETERS

    // important slip quantities
    slips* m_slip;

    // model parameter factors stored here
    Pac2002_data* m_params;

    // for keeping track of intermediate factors in the PacTire model
    pureLongCoefs* m_pureLong;
    pureLatCoefs* m_pureLat;
    pureTorqueCoefs* m_pureTorque;

    combinedLongCoefs* m_combinedLong;
    combinedLatCoefs* m_combinedLat;
    combinedTorqueCoefs* m_combinedTorque;

    zetaCoefs* m_zeta;

    double m_mu;   // current road friction coefficient
    double m_mu0;  // tire reference friction coeffient

    ChFunction_Recorder m_areaDep;  // lookup table for estimation of penetration depth from intersection area

    // for transient contact point tire model
    relaxationL* m_relaxation;
    bessel* m_bessel;

    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
    std::shared_ptr<ChTexture> m_texture;          ///< visualization texture asset
};

// -----------------------------------------------------------------------------
// String manipulation utility functions.
// -----------------------------------------------------------------------------

inline std::vector<std::string>& splitStr(const std::string& s, char delim, std::vector<std::string>& elems) {
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline std::vector<std::string> splitStr(const std::string& s, char delim) {
    std::vector<std::string> elems;
    return splitStr(s, delim, elems);
}

// convert a string to another type
template <class T>
T fromString(const std::string& s) {
    std::istringstream stream(s);
    T t;
    stream >> t;
    return t;
}

// read an input line from tm.config, return the number that matters
template <class T>
T fromTline(const std::string& tline) {
    T t = fromString<T>(splitStr(splitStr(tline, '/')[0], '=')[1]);
    return t;
}

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
