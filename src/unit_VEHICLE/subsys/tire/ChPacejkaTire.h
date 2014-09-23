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
// has the data structs for the pacTire parameters
#include "ChPac2002_data.h"
#include "../terrain/FlatTerrain.h"

namespace chrono {

// @brief class that reads a ver3.0 pac2002 *.tire file upon init.
//      calling update will calculate the current F_x, F_y and M_z
class CH_SUBSYS_API ChPacejkaTire : public ChTire {

public:

	// @brief when using terrain that is not necessarily flat
  ChPacejkaTire(const std::string& pacTire_paramFile, const ChTerrain& terrain,
		const ChBodyState& ICs, 
		const double step_size = 0.01,
		const bool use_transient_slip = true,
		const double Fz_override = -1);

	// @brief copy constructor, only tyreside will be different
	ChPacejkaTire(const ChPacejkaTire& tire, chrono::ChWheelId which);

  virtual ~ChPacejkaTire() {}


	// ---------------------
	// Public functions

	// @brief set the PacTire spindle state data from the global wheel body state
  virtual void Update(double time, const ChBodyState&  state);

	// @brief use the new body state, calculate all the relevant quantities over the time increment
	virtual void Advance(double step);

  // @brief have the tire do a timestep, prepare 

   // @brief write output data to a file
  void WriteOutData(const double       time,
										const std::string& outFilename);

	// @brief manually set the vertical wheel load as an input
	void set_Fz_override(const double Fz) { m_Fz_override = Fz; }

	// ------------
	// Utility functions
	// @brief return orientation, Vx, Vy (global) and omega_y (local)
	// @param Vxy the tire velocity vector magnitude in the global x,y plane
	// 
	ChBodyState getState_from_KAG(const double kappa, const double alpha, const double gamma, const double Vxy);

	// @brief return kappa, lapha, gamma
	ChVector<> getKAG_from_State(const ChBodyState& state);

	// ---------------------
	//	ACCESSORS

		// @brief get current long slip rate
	double get_kappa() const { return m_slip.kappa;}
	// @brief get current slip angle
	double get_alpha() const {return m_slip.alpha;}

	// @brief get min/max longitudinal slip rates
	double get_min_long_slip() const { return m_params.long_slip_range.kpumin;}
	double get_max_long_slip() const { return m_params.long_slip_range.kpumax;}
	// @brief get the min/max allowable lateral slip angle, alpha
	double get_min_lat_slip() const { return m_params.slip_angle_range.alpmin;}
	double get_max_lat_slip() const { return m_params.slip_angle_range.alpmax;}
	// @brief what's the longtiduinal velocity?
	double get_longvl() const { return m_params.model.longvl; }
	// @brief get the tire rolling radius, ideally updated each step
	double get_tire_rolling_rad() const { return m_R_eff; }

  // @brief return the most recently computed forces as a function of slip rates
	ChTireForce GetTireForce() const { return m_FM;}
	// @brief combined slip case
	ChTireForce GetTireForce_combined() const {return m_FM_combined;}


protected:

  // @brief where to find the input parameter file
  virtual std::string getPacTireParamFile();

  // @brief specify the file name to read the Pactire input from
  void Initialize();

  // @brief look for this data file
  virtual void loadPacTireParamFile(void);

  // @brief once Pac tire input text file has been succesfully opened, read 
  //    the input data, and populate the data struct
  virtual void readPacTireInput(std::ifstream& inFile);

	// @brief functions for reading each section in the paramter file
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


	// @brief calculate the various stiffness/relaxation lengths
	void calc_relaxationLengths();

	// @brief calculate the slips assume the steer/throttle/brake input
	//				to wheel has an instantaneous effect on contact patch slips
	void calc_slip_kinematic();

	// @brief calculate transient slip properties, using first order ODEs
	//				to find slip displacements from velocities
	void advance_slip_transient(const double step_size);

	// @brief calculate the increment delta_x using RK 45 integration
	// @param V_s	slip velocity
	// @param sigma	either sigma_kappa or sigma_alpha
	// @param V_c	wheel body center velocity
	// @param step_size the simulation timestep size
	// @param x_curr	f(x_curr)
	double calc_ODE_RK_uv(const double V_s, const double sigma,  
		const double V_cx, const double step_size, const double x_curr);

	// @brief calculate the increment delta_gamma of the gamma ODE
	double calc_ODE_RK_gamma(const double C_Fgamma,
													const double C_Falpha,
													const double sigma_alpha,
													const double V_cx,
													const double step_size,
													const double gamma,
													const double v_gamma);

	// @brief calculate the ODE dphi/dt = f(phi), return the increment delta_phi
	double ChPacejkaTire::calc_ODE_RK_phi(const double C_Fphi,
																			const double C_Falpha,
																			const double V_cx,
																			const double psi_dot,
																			const double w_y,
																			const double gamma,
																			const double sigma_alpha,
																			const double v_phi,
																			const double eps_gamma,
																			const double step_size);

	// @brief calculate and set the transient slip values (kappaP, alphaP, gammaP) from u, v deflections
	void calc_slip_from_uv(void);

	// calculate the forces, moments when Update() is called
	// @brief calculate pure longitudinal, pure lateral slip reactions
	void calc_pureSlipReactions(void);

	// @brief calculate combined slip reactions
	void calc_combinedSlipReactions(void);

	// @brief calculate the current effective rolling radius, w.r.t. wy, Fz as inputs
	// Fz is used as an 
	void calc_rho(const double F_z);

	// @brief calculate the force for pure longitudinal slip
	void calcFx_pureLong(void);

	// @brief calculate the force for pure lateral slip
	void calcFy_pureLat(void);

	// @brief find the vertical load
	void calc_Fz();	// const double step_size);

	// @brief calc aligning torque, pure slip case
	void calcMz_pure(void);

	// @brief calculate Fx for combined slip
	void calcFx_combined(void);

	// @brief calculate Fy for combined slip
	void calcFy_combined(void);

	// @brief calc. Mz for combined slip
	void calcMz_combined(void);

	// @brief update M_x, apply to both m_FM and m_FM_combined
	void calc_overturningCouple(void);

	// @brief update M_y, apply to both m_FM and m_FM_combined
	void calc_rollingResistance(void);


	// ----- Data members
	// important slip quantities
	struct slips m_slip;
	bool m_use_transient_slip;

	// unloaded radius
	double m_R0;
	// effective rolling radius at test load
	// double m_R_eff_0;
	// current effect rolling radius
	double m_R_eff;
	// relaxation length
	double m_R_l;
	// vertical deflection w.r.t. R0
	double m_rho;

	double m_dF_z;		// (Fz - Fz,nom) / Fz,nom
	bool m_use_Fz_override;	// calculate Fz using collision, or user input
	double m_Fz_override;	// if manually inputting the vertical wheel load
	// integration step size
	double m_step_size;

	// OUTPUTS
	// based on pure slip
  ChTireForce m_FM;
  // based on combined slip
  ChTireForce m_FM_combined;

	// current tire state
	ChBodyState m_tireState;

  // all pactire models require an input parameter file
  std::string m_paramFile;

  // write output data to this file
  std::string m_outFilename;

	// calkled WriteOutData this many times
	int m_Num_WriteOutData;

  // have the tire model parameters been defined/read from file yet?
  // must call load_pacFile_tire
  bool m_params_defined;

  // model parameter factors stored here
  struct Pac2002_data m_params;

	// for keeping track of intermediate factors in the PacTire model
	struct pureLongCoefs m_pureLong;
	struct pureLatCoefs m_pureLat;
	struct PureTorqueCoefs m_pureTorque;

	struct combinedLongCoefs m_combinedLong;
	struct combinedLatCoefs m_combinedLat;
	struct combinedTorqueCoefs m_combinedTorque;

	struct zetaCoefs m_zeta;

	// for transient contact point tire model
	struct relaxationL m_relaxation;


};


} // end namespace chrono


#endif
