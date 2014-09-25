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
// Authors: Justin Madsen
// =============================================================================
//
// Base class for a Pacjeka type Magic formula 2002 tire model
//
// =============================================================================

#ifndef CH_PACEJKATIRE_H
#define CH_PACEJKATIRE_H

#include <string>
#include <fstream>

#include "physics/ChBody.h"

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"

namespace chrono {

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

///
/// Concrete tire class that implements the Pacejka tire model.
/// Detailed description goes here...
///
class CH_SUBSYS_API ChPacejkaTire : public ChTire
{
public:


  ChPacejkaTire(
    const std::string& pacTire_paramFile,
    const ChTerrain&   terrain
    );


  ChPacejkaTire(
    const std::string& pacTire_paramFile,
    const ChTerrain&   terrain,
    bool               use_transient_slip = true,
    double             Fz_override = -1
    );

  /// Copy constructor, only tire side will be different
  ChPacejkaTire(
    const ChPacejkaTire& tire,   ///< [in] source object
    ChWheelId            which   ///< [in] 
    );

  ~ChPacejkaTire();

  /// Return the most recently computed forces as a function of slip rates.
  virtual ChTireForce GetTireForce() const { return m_FM_combined; }

  ///  Return the most recently computed forces as a function of slip rates.
  ChTireForce GetTireForce_pureSlip() const { return m_FM; }

  /// Update the state of this tire system at the current time.
  /// Set the PacTire spindle state data from the global wheel body state.
  virtual void Update(
    double              time,          ///< [in] current time
    const ChBodyState&  wheel_state    ///< [in] current state of associated wheel body
    );

  /// Advance the state of this tire by the specified time step.
  /// Use the new body state, calculate all the relevant quantities over the
  /// time increment.
  virtual void Advance(double step);

  /// Write output data to a file.
  void WriteOutData(
    double             time,
    const std::string& outFilename
    );

  /// Manually set the vertical wheel load as an input.
  void set_Fz_override(const double Fz) { m_Fz_override = Fz; }

  /// Return orientation, Vx, Vy (global) and omega_y (local).
  ChBodyState getState_from_KAG(
    double kappa,   ///< [in] ...
    double alpha,   ///< [in] ...
    double gamma,   ///< [in] ...
    double Vxy      ///< [in] tire velocity magnitude in the global (x,y) plane
    );

  /// Return kappa, alpha, gamma.
  ChVector<> getKAG_from_State(const ChBodyState& state);

  /// Get current long slip rate.
  double get_kappa() const;

  /// Get current slip angle.
  double get_alpha() const;

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

  /// Set the value of the integration step size.
  void SetStepsize(double val) { m_step_size = val; }

  /// Get the current value of the integration step size.
  double GetStepsize() const { return m_step_size; }

private:

  // where to find the input parameter file
  const std::string& getPacTireParamFile() const { return m_paramFile; }

  // specify the file name to read the Pactire input from
  void Initialize();

  // look for this data file
  virtual void loadPacTireParamFile(void);

  // once Pac tire input text file has been succesfully opened, read the input
  // data, and populate the data struct
  virtual void readPacTireInput(std::ifstream& inFile);

  // functions for reading each section in the paramter file
  void readSection_UNITS(std::ifstream& inFile);
  void readSection_MODEL(std::ifstream& inFile);
  void readSection_DIMENSION(std::ifstream& inFile);
  void readSection_SHAPE(std::ifstream& inFile);
  void readSection_VERTICAL(std::ifstream& inFile);
  void readSection_RANGES(std::ifstream& inFile);
  void readSection_SCALING_COEFFICIENTS(std::ifstream& inFile);
  void readSection_LONGITUDINAL_COEFFICIENTS(std::ifstream& inFile);
  void readSection_OVERTURNING_COEFFICIENTS(std::ifstream& inFile);
  void readSection_LATERAL_COEFFICIENTS(std::ifstream& inFile);
  void readSection_ROLLING_COEFFICIENTS(std::ifstream& inFile);
  void readSection_ALIGNING_COEFFICIENTS(std::ifstream& inFile);

  // calculate the various stiffness/relaxation lengths
  void calc_relaxationLengths();

  // calculate the slips assume the steer/throttle/brake input to wheel has an
  // instantaneous effect on contact patch slips
  void calc_slip_kinematic();

  // calculate transient slip properties, using first order ODEs to find slip
  // displacements from velocities
  void advance_slip_transient(double step_size);

  // calculate the increment delta_x using RK 45 integration
  double calc_ODE_RK_uv(
    double V_s,          // slip velocity
    double sigma,        // either sigma_kappa or sigma_alpha
    double V_cx,         // wheel body center velocity
    double step_size,    // the simulation timestep size
    double x_curr);      // f(x_curr)

  // calculate the increment delta_gamma of the gamma ODE
  double calc_ODE_RK_gamma(
    double C_Fgamma,
    double C_Falpha,
    double sigma_alpha,
    double V_cx,
    double step_size,
    double gamma,
    double v_gamma);

  // calculate the ODE dphi/dt = f(phi), return the increment delta_phi
  double calc_ODE_RK_phi(const double C_Fphi,
    const double C_Falpha,
    const double V_cx,
    const double psi_dot,
    const double w_y,
    const double gamma,
    const double sigma_alpha,
    const double v_phi,
    const double eps_gamma,
    const double step_size);

  // calculate and set the transient slip values (kappaP, alphaP, gammaP) from
  // u, v deflections
  void calc_slip_from_uv(void);

  // calculate the forces, moments when Update() is called
  // calculate pure longitudinal, pure lateral slip reactions
  void calc_pureSlipReactions(void);

  // calculate combined slip reactions
  void calc_combinedSlipReactions(void);

  // calculate the current effective rolling radius, w.r.t. wy, Fz as inputs
  void calc_rho(const double F_z);

  // calculate the force for pure longitudinal slip
  void calcFx_pureLong(void);

  // calculate the force for pure lateral slip
  void calcFy_pureLat(void);

  // find the vertical load
  void calc_Fz();

  // calc aligning torque, pure slip case
  void calcMz_pure(void);

  // calculate Fx for combined slip
  void calcFx_combined(void);

  // calculate Fy for combined slip
  void calcFy_combined(void);

  // calculate Mz for combined slip
  void calcMz_combined(void);

  // update M_x, apply to both m_FM and m_FM_combined
  void calc_overturningCouple(void);

  // update M_y, apply to both m_FM and m_FM_combined
  void calc_rollingResistance(void);

  // ----- Data members

  bool m_use_transient_slip;

  double m_R0;              // unloaded radius
  double m_R_eff;           // current effect rolling radius
  double m_R_l;             // relaxation length
  double m_rho;             // vertical deflection w.r.t. R0

  double m_dF_z;            // (Fz - Fz,nom) / Fz,nom
  bool m_use_Fz_override;   // calculate Fz using collision, or user input
  double m_Fz_override;     // if manually inputting the vertical wheel load

  double m_step_size;       // integration step size

  // OUTPUTS

  ChTireForce m_FM;            // based on pure slip
  ChTireForce m_FM_combined;   // based on combined slip

  ChBodyState m_tireState;     // current tire state
	ChCoordsys<> m_tire_frame;	// 

  std::string m_paramFile;     // input parameter file
  std::string m_outFilename;   // output filename

  int m_Num_WriteOutData;      // number of times WriteOut was called

  bool m_params_defined;       // indicates if model params. have been defined/loaded

  // MODEL PARAMETERS

  // important slip quantities
  slips*               m_slip;

  // model parameter factors stored here
  Pac2002_data*        m_params;

  // for keeping track of intermediate factors in the PacTire model
  pureLongCoefs*       m_pureLong;
  pureLatCoefs*        m_pureLat;
  pureTorqueCoefs*     m_pureTorque;

  combinedLongCoefs*   m_combinedLong;
  combinedLatCoefs*    m_combinedLat;
  combinedTorqueCoefs* m_combinedTorque;

  zetaCoefs*           m_zeta;

  // for transient contact point tire model
  relaxationL*         m_relaxation;

};


} // end namespace chrono


#endif
