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
// Authors: Justin Madsen, Markus Schmid
// =============================================================================
//
// Base class for a Pacjeka type Magic formula 2002 tire model
//
// =============================================================================

#include "subsys/tire/ChPacejkaTire.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsStringFunctions.h"

#include "subsys/tire/ChPac2002_data.h"

namespace chrono{

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
ChPacejkaTire::ChPacejkaTire(const std::string& pacTire_paramFile,
                             const ChTerrain&   terrain)
: ChTire(terrain),
  m_paramFile(pacTire_paramFile),
  m_params_defined(false),
  m_use_transient_slip(true),
  m_use_Fz_override(false),
  m_step_size(0.01)
{
  Initialize();
}


ChPacejkaTire::ChPacejkaTire(const std::string& pacTire_paramFile,
                             const ChTerrain&   terrain,
                             double             Fz_override,
                             bool               use_transient_slip)
: ChTire(terrain),
  m_paramFile(pacTire_paramFile),
  m_params_defined(false),
  m_use_transient_slip(use_transient_slip),
  m_use_Fz_override(Fz_override > 0),
  m_Fz_override(Fz_override),
  m_step_size(0.01)
{
  Initialize();
}

ChPacejkaTire::ChPacejkaTire(const ChPacejkaTire& tire,
                             ChWheelId            which)
: ChTire(tire.m_terrain),
  m_paramFile(tire.m_paramFile),
  m_params_defined(false),
  m_use_transient_slip(tire.m_use_transient_slip),
  m_use_Fz_override(tire.m_use_Fz_override),
  m_Fz_override(tire.m_Fz_override)
{
  Initialize();

  if (which == FRONT_RIGHT || which == REAR_RIGHT)
    m_params->model.tyreside = "RIGHT";
  else
    m_params->model.tyreside = "LEFT";
}

// -----------------------------------------------------------------------------
// Destructor
//
// Delete private structures
// -----------------------------------------------------------------------------
ChPacejkaTire::~ChPacejkaTire()
{
  delete m_slip;
  delete m_params;
  delete m_pureLong;
  delete m_pureLat;
  delete m_pureTorque;
  delete m_combinedLong;
  delete m_combinedLat;
  delete m_combinedTorque;
  delete m_zeta;
  delete m_relaxation;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// NOTE: no initial conditions passed in at this point, e.g. m_tireState is empty
void ChPacejkaTire::Initialize()
{
  // Create private structures
  m_slip = new slips;
  m_params = new Pac2002_data;

  m_pureLong = new pureLongCoefs;
  m_pureLat = new pureLatCoefs;
  m_pureTorque = new pureTorqueCoefs;

  m_combinedLong = new combinedLongCoefs;
  m_combinedLat = new combinedLatCoefs;
  m_combinedTorque = new combinedTorqueCoefs;

  m_zeta = new zetaCoefs;
  m_relaxation = new relaxationL;

  // load all the empirical tire parameters from *.tir file
  loadPacTireParamFile();

  if (m_params_defined) {
    // any variables that are calculated once
    m_R0 = m_params->dimension.unloaded_radius;
    // find a better way to calculate rolling radius
    // m_R_eff_0 = m_R0*(1.0 - 0.05);

    // m_R_l = m_R0 - m_params->vertical.fnomin / m_params->vertical.vertical_stiffness;
    m_R_l = m_R0 - 8000.0 / m_params->vertical.vertical_stiffness;

    // assume rho decreases with increasing w_y
    double qV1 = 1.5;
    // omega_y ~= V0/ R_eff ~= V0/(.05*R0)
    m_rho = (m_R0 - m_R_l) * exp(-qV1 * m_R0 * pow(1.05 * m_params->model.longvl / m_params->model.longvl, 2));

    // Note: rho is always > 0 with the modified eq.
    m_R_eff = m_R0 - m_rho;

    // not sure what these are used for
    {
      zetaCoefs tmp = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };
      *m_zeta = tmp;
    }

    // any of the structs associated with multipliers in reaction calcs
    m_combinedTorque->alpha_r_eq = 0.0;
    m_pureLat->D_y = m_params->vertical.fnomin;	// initial approximation

    // init all other variables
    m_Num_WriteOutData = 0;
    {
      slips tmp = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      *m_slip = tmp;
    }
  }
  else {
    GetLog() << " couldn't load pacTire parameters from file, not updating initial quantities \n\n";

  }
}


ChTireForce ChPacejkaTire::GetTireForce() const
{
  // reactions are on wheel CM
  ChTireForce m_FM_global;
  m_FM_global.point = m_tireState.pos;
  // only transform the directions of the forces, moments, from local to global
  m_FM_global.force = m_tire_frame.TransformDirectionLocalToParent(m_FM_combined.force);
  m_FM_global.moment = m_tire_frame.TransformDirectionLocalToParent(m_FM_combined.moment);

  return m_FM_global;
}


ChTireForce ChPacejkaTire::GetTireForce_pureSlip(const bool local) const
{
  if (local)
    return m_FM;
  else
  {
    // reactions are on wheel CM
    ChTireForce m_FM_global;
    m_FM_global.point = m_tireState.pos;
    // only transform the directions of the forces, moments, from local to global
    m_FM_global.force = m_tire_frame.TransformDirectionLocalToParent(m_FM.force);
    m_FM_global.moment = m_tire_frame.TransformDirectionLocalToParent(m_FM.moment);

    return m_FM_global;
  }
}

/// Return the reactions for the combined slip EQs, in local or global coords
ChTireForce ChPacejkaTire::GetTireForce_combinedSlip(const bool local) const
{
  if (local)
    return m_FM_combined;
  else
    return GetTireForce();
}


// -----------------------------------------------------------------------------
// Functions providing access to private structures
// -----------------------------------------------------------------------------
double ChPacejkaTire::get_kappa() const
{
  return m_slip->kappa;
}

double ChPacejkaTire::get_alpha() const
{
  return m_slip->alpha;
}

double ChPacejkaTire::get_gamma() const
{
  return m_slip->gamma;
}

double ChPacejkaTire::get_min_long_slip() const
{
  return m_params->long_slip_range.kpumin;
}

double ChPacejkaTire::get_max_long_slip() const
{
  return m_params->long_slip_range.kpumax;
}

double ChPacejkaTire::get_min_lat_slip() const
{
  return m_params->slip_angle_range.alpmin;
}

double ChPacejkaTire::get_max_lat_slip() const
{
  return m_params->slip_angle_range.alpmax;
}

double ChPacejkaTire::get_longvl() const
{
  return m_params->model.longvl;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPacejkaTire::Update(double               time,
                           const ChWheelState&  state)
{
  // check that input tire model parameters are defined
  if (!m_params_defined)
  {
    GetLog() << " ERROR: cannot update tire w/o setting the model parameters first! \n\n\n";
    return;
  }

  // check x velocity, vertical load (only when not using transient slip model)
  double v_mag = ChVector<>(state.lin_vel.x, state.lin_vel.y, 0).Length();
  if (!m_use_transient_slip && (v_mag < 0.1))
  {
    GetLog() << " ERROR: forward velocity below threshold.... \n\n";
    return;
  }

  // update tire input state, and associated tire local coordinate system
  m_tireState = state;
  update_tireFrame();

  // Is the vertical load specified as an input?
  if (m_use_Fz_override)
  {
    // update tire deflection, rho, based on input vertical load, wheel state
    calc_rho(m_Fz_override);
  }
  else {
    // not specified or in applicable range, calculate vertical force
    calc_Fz();
  }

  m_dF_z = (m_FM.force.z - m_params->vertical.fnomin) / m_params->vertical.fnomin;

  // calculate slips based on wheel body info first
  // assume input to wheel spindle instantly affect tire contact
  calc_slip_kinematic();
  // note: kappaP, alphaP, gammaP can be overridden in Advance
}

void ChPacejkaTire::Advance(double step)
{
  // check that input tire model parameters are defined
  if (!m_params_defined)
  {
    GetLog() << " ERROR: cannot update tire w/o setting the model parameters first! \n\n\n";
    return;
  }

  // if using single point contact model, slips are calculated from
  // compliance between tire and contact patch
  if (m_use_transient_slip)
  {
    // 1 of 2 ways to deal with user input time step increment
    // a) step <= m_step_size, so integrate using input step
    // b) step > m_step_size, use m_step_size until step <= m_step_size
    double remaining_time = step;
    while (remaining_time > m_step_size)
    {
      advance_slip_transient(m_step_size);
      remaining_time -= m_step_size;
    }
    // now, remaining_time <= m_step_size, use that to get final slip displacements
    advance_slip_transient(remaining_time);
  }

  // calculate the force and moment reaction, pure slip case
  calc_pureSlipReactions();

  // updates m_FM_combined.forces, m_FM_combined.moment.z
  calc_combinedSlipReactions();

  // update M_x, apply to both m_FM and m_FM_combined
  calc_Mx();

  // update M_y, apply to both m_FM and m_FM_combined
  calc_My();
}


void ChPacejkaTire::calc_rho(double F_z)
{
  double qV1 = 1.5;	// guess
  // first, set the vertical force
  m_FM.force.z = F_z;
  m_FM_combined.force.z = F_z;

  // loaded radius: if the tire were loaded statically
  m_R_l = m_R0 - F_z / m_params->vertical.vertical_stiffness;
  // assume rho decreases with increasing w_y
  double rho = (m_R0 - m_R_l) * exp(-qV1 * m_R0 * pow((m_tireState.ang_vel.y * m_R0 / m_params->model.longvl), 2));

  // Note: rho is always > 0 with the modified eq.
  // if( rho > 0.0 ) {
  m_rho = rho;
  m_R_eff = m_R0 - m_rho;
  // } else {
  //	GetLog () << " this step rho < 0 !  \n";
  //	m_rho = 0;
  //	m_R_eff = m_R0;
  // }
}


void ChPacejkaTire::calc_Fz()
{
  double qV1 = 0.8;  // guess
  double h_ground = m_terrain.GetHeight(m_tireState.pos.x, m_tireState.pos.y);
  // dz should be negative
  double dz = (m_tireState.pos.z - m_R0) - h_ground;

  // is the bottom center of a circle below the ground?
  if (dz > 0)
  {
    m_FM.force.z = 0;
    m_FM_combined.force.z = 0;
    m_R_l = m_R0;
    m_rho = 0;
    m_R_eff = m_R0;
    ////GetLog() << " no contact detected on this tire \n";
  }
  else {
    // spring force from deflection
    double f_k = -dz * m_params->vertical.vertical_stiffness;
    // dz_dt is a finite difference for rho
    double dz_dt = -dz - (m_R0 - m_R_l);
    // damper force from deflection_dt
    double f_c = dz_dt * m_params->vertical.vertical_damping;

    // use a simple spring damper to find the vertical force
    double Fz = f_k + f_c;

    m_FM.force.z = Fz;
    m_FM_combined.force.z = Fz;
    // set the tire deflections, vertical force
    m_R_l = m_R0 + dz;

    double rho = (m_R0 - m_R_l) * (1.0 - qV1 * m_R0  * pow((m_tireState.ang_vel.y * m_R0 / m_params->model.longvl), 2));
    if (rho > 0.0) {
      m_rho = rho;
      m_R_eff = m_R0 - m_rho;
    }
    else {
      m_rho = 0;
      m_R_eff = m_R0;
    }

  }
}

void ChPacejkaTire::calc_slip_kinematic()
{
  // absolaute angle of the wheel velocity vector
  double Vx_ang = atan(m_tireState.lin_vel.y / m_tireState.lin_vel.x);
  // find alpha from orientation matrix
  double ang_z = chrono::Q_to_NasaAngles(m_tireState.rot).z;
  double gamma = chrono::Q_to_NasaAngles(m_tireState.rot).x;
  // note: should find alpha relative to wheel center velocity
  double alpha = ang_z - Vx_ang;

  double v_mag = ChVector<>(m_tireState.lin_vel.x, m_tireState.lin_vel.y, 0).Length();
  // local c-sys velocities
  double V_cx = v_mag * cos(alpha);
  double V_cy = v_mag * sin(alpha);

  double kappa = 0;
  // can get in here when use_transient_slip = true when v_mag is low or zero.
  if (v_mag < m_params->model.vxlow)
  {
    kappa = (m_R_eff * m_tireState.ang_vel.y - V_cx) / abs(m_params->model.vxlow);
  }
  else {
    // s_x = (w_y * r_rolling - |v|) / |v|
    kappa = (m_R_eff * m_tireState.ang_vel.y - V_cx) / abs(V_cx);
  }


  // alpha_star = tan(alpha) = v_y / v_x  (no negative since I use z-up)
  double alpha_star = 0;
  // don't use rightmost eq. when v_x is too small
  if (v_mag < m_params->model.vxlow)
  {
    alpha_star = tan(alpha);
  }
  else {
    alpha_star = V_cy / abs(V_cx);
  }

  // set the struct data members, input slips to wheel
  m_slip->kappa = kappa;
  m_slip->alpha = alpha;
  m_slip->alpha_star = alpha_star;
  m_slip->gamma = gamma;

  m_slip->V_cx = V_cx;	// tire center x-vel, tire c-sys
  m_slip->V_cy = V_cy;	// tire center y-vel, tire c-sys
  m_slip->V_sx = V_cx - m_tireState.ang_vel.y * m_R_eff;	// x-slip vel, tire c-sys
  m_slip->V_sy = V_cy;	// approx.
  // psi_dot is turn slip velocity, in global coords (always normal to road)
  ChCoordsys<> loc_csys(ChVector<>(), m_tireState.rot);
  // global omega vector
  ChVector<> omega_global = loc_csys.TransformLocalToParent(m_tireState.ang_vel);
  m_slip->psi_dot = omega_global.z;

  // for aligning torque, to handle large slips, and backwards operation
  m_slip->cosPrime_alpha = V_cx / (v_mag + 0.1);

  // finally, if non-transient, use wheel slips as input to Magic Formula
  // these get over-written if enable transient slips to be calculated
  m_slip->kappaP = kappa;
  m_slip->alphaP = alpha_star;
  m_slip->gammaP = sin(gamma);

}

void ChPacejkaTire::advance_slip_transient(double step_size)
{
  // hard-coded, for now
  double EPS_GAMMA = 0.6;
  // update relaxation lengths
  calc_relaxationLengths();

  double v_mag = ChVector<>(m_tireState.lin_vel.x, m_tireState.lin_vel.y, 0).Length();
  // local c-sys velocities
  double V_cx = v_mag * cos(m_slip->alpha);
  double V_cx_low = 2.5;	// cut-off for low velocity zone

  // see if low velocity considerations should be made
  double alpha_sl = 3.0 * m_pureLat->D_y / m_relaxation->C_Falpha;
  // Eq. 7.25 from Pacejka (2006)
  if ((abs(m_combinedTorque->alpha_r_eq) > alpha_sl) && (abs(V_cx) < V_cx_low))
  {
    // Eq. 7.9, else du/dt = 0 and u remains unchanged
    if ((m_slip->V_sx + abs(V_cx) * m_slip->u / m_relaxation->sigma_kappa) * m_slip->u >= 0)
    {
      // solve the ODE using RK - 45 integration
      double delta_u = calc_ODE_RK_uv(m_slip->V_sx, m_relaxation->sigma_kappa, V_cx, step_size, m_slip->u);
      m_slip->u += delta_u;
    }

    // Eq. 7.7, else dv/dt = 0 and v remains unchanged
    if ((m_slip->V_sy + abs(V_cx) * m_slip->v_alpha / m_relaxation->sigma_alpha) * m_slip->v_alpha >= 0)
    {
      double delta_v = calc_ODE_RK_uv(m_slip->V_sy, m_relaxation->sigma_alpha, V_cx, step_size, m_slip->v_alpha);
      m_slip->v_alpha += delta_v;
    }

  }
  else {
    // don't check for du/dt =0 or dv/dt = 0

    // Eq 7.9 
    double delta_u = calc_ODE_RK_uv(m_slip->V_sx, m_relaxation->sigma_kappa, V_cx, step_size, m_slip->u);
    m_slip->u += delta_u;
    // Eq. 7.7
    double delta_v = calc_ODE_RK_uv(m_slip->V_sy, m_relaxation->sigma_alpha, V_cx, step_size, m_slip->v_alpha);
    m_slip->v_alpha += delta_v;

  }

  // Eq. 7.11, lateral force from wheel camber
  double delta_gamma = calc_ODE_RK_gamma(m_relaxation->C_Fgamma, m_relaxation->C_Falpha, m_relaxation->sigma_alpha,
    V_cx, step_size, m_slip->gamma, m_slip->v_gamma);
  m_slip->v_gamma += delta_gamma;

  // Eq. 7.12, total spin, phi, including slip and camber
  double delta_phi = calc_ODE_RK_phi(m_relaxation->C_Fphi, m_relaxation->C_Falpha,
    V_cx, m_slip->psi_dot, m_tireState.ang_vel.y, m_slip->gamma, m_relaxation->sigma_alpha,
    m_slip->v_phi, EPS_GAMMA, step_size);
  m_slip->v_phi += delta_phi;

  // calculate slips from contact point deflections u and v
  calc_slip_from_uv();

}


double ChPacejkaTire::calc_ODE_RK_uv(double V_s,
                                     double sigma,
                                     double V_cx,
                                     double step_size,
                                     double x_curr)
{
  double k1 = -V_s - (1.0 / sigma) * abs(V_cx) * x_curr;
  double k2 = -V_s - (1.0 / sigma) * abs(V_cx) * (x_curr + 0.5 * step_size * k1);
  double k3 = -V_s - (1.0 / sigma) * abs(V_cx) * (x_curr + 0.5 * step_size * k2);
  double k4 = -V_s - (1.0 / sigma) * abs(V_cx) * (x_curr + step_size * k3);

  double delta_x = (step_size / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

  return delta_x;
}

// pure slip reactions. If Lateral slip isn't nearly 0, calculate
// pure longitudinal slip case, else pure lateral slip case.
void ChPacejkaTire::calc_pureSlipReactions()
{
  // calculate Fx, pure long. slip condition
  calcFx_pureLong();

  // calc. Fy, pure lateral slip
  calcFy_pureLat();

  // calc Mz, pure lateral slip
  calcMz_pureLat();

}


double ChPacejkaTire::calc_ODE_RK_gamma(double C_Fgamma,
                                        double C_Falpha,
                                        double sigma_alpha,
                                        double V_cx,
                                        double step_size,
                                        double gamma,
                                        double v_gamma)
{
  double g0 = C_Fgamma / C_Falpha * abs(V_cx) * gamma;
  double g1 = abs(V_cx) / sigma_alpha;
  double k1 = g0 - g1 * v_gamma;
  double k2 = g0 - g1 * (v_gamma + 0.5 * step_size * k1);
  double k3 = g0 - g1 * (v_gamma + 0.5 * step_size * k2);
  double k4 = g0 - g1 * (v_gamma + step_size * k3);

  double delta_g = (step_size / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  return delta_g;
}

double ChPacejkaTire::calc_ODE_RK_phi(double C_Fphi,
                                      double C_Falpha,
                                      double V_cx,
                                      double psi_dot,
                                      double w_y,
                                      double gamma,
                                      double sigma_alpha,
                                      double v_phi,
                                      double eps_gamma,
                                      double step_size)
{
  int sign_Vcx = 0;
  if (V_cx < 0)
    sign_Vcx = -1;
  else
    sign_Vcx = 1;

  double p0 = (C_Fphi / C_Falpha) * sign_Vcx * (psi_dot - (1.0 - eps_gamma) * w_y * sin(gamma));
  double p1 = (1.0 / sigma_alpha) * abs(V_cx);

  double k1 = -p0 - p1 * v_phi;
  double k2 = -p0 - p1 * (v_phi + 0.5 * step_size * k1);
  double k3 = -p0 - p1 * (v_phi + 0.5 * step_size * k2);
  double k4 = -p0 - p1 * (v_phi + step_size * k3);

  double delta_phi = (step_size / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  return delta_phi;
}


void ChPacejkaTire::calc_slip_from_uv()
{
  // Markus adds artificial damping at low velocity (Besselink)
  // damp gradually to zero velocity at low velocity
  double V_low = 1.7;
  double d_vlow = 0;
  if (abs(m_slip->V_cx) <= V_low)
  {
    d_vlow = 0.5 * 770. * (1.0 + cos(CH_C_PI * abs(m_slip->V_cx) / V_low));
  }

  // Besselink is RH term in kappa_p, alpha_p
  double kappa_p = m_slip->u / m_relaxation->sigma_kappa - d_vlow * m_slip->V_sx / m_relaxation->C_Fkappa;
  double alpha_p = m_slip->v_alpha / m_relaxation->sigma_alpha - d_vlow * m_slip->V_sy / m_relaxation->C_Falpha;
  double gamma_p = m_relaxation->C_Falpha * m_slip->v_gamma / (m_relaxation->C_Fgamma * m_relaxation->sigma_alpha);
  double phi_p = (m_relaxation->C_Falpha * m_slip->v_phi) / (m_relaxation->C_Fphi * m_relaxation->sigma_alpha);	// turn slip
  double phi_t = -m_slip->psi_dot / (m_slip->V_cx + 0.1);	// for turn slip

  // set transient slips
  m_slip->alphaP = alpha_p;
  m_slip->kappaP = kappa_p;
  m_slip->gammaP = gamma_p;
  m_slip->phiP = phi_p;
  m_slip->phiT = phi_t;
}


//  combined slip reactions
void ChPacejkaTire::calc_combinedSlipReactions()
{
  // calculate Fx for combined slip
  calcFx_combined();

  // calc Fy for combined slip
  calcFy_combined();

  // calc Mz for combined slip
  calcMz_combined();

}



void ChPacejkaTire::update_tireFrame()
{
  // need a local frame to transform output forces/moments to global frame
  //  Pacejka (2006), Fig 2.3, all forces are calculated at the contact point "C"
  // Moment calculations take this into account, so all reactions are global

  // z-axis is parallel to global, x-y plane orientation is needed
  m_tire_frame.pos = m_tireState.pos;
  ChMatrix33<> A_state(m_tireState.rot);
  // z_hat = (0,0,1)
  // the wheel body spins about the y-axis, assume not vertical.
  ChVector<> y_hat(A_state(0, 1), A_state(1, 1), 0);
  y_hat.Normalize();
  // y cross z = x
  ChVector<> x_hat(0, 0, 0);
  x_hat.Cross(y_hat, ChVector<>(0, 0, 1));
  x_hat.Normalize();

  ChVector<> z_hat(0, 0, 1);
  ChMatrix33<> A_hat(0);
  // A_hat = [ x_i  y_i  0  
  //					 x_j  y_j  0
  //            0    0   1 ]
  A_hat(0, 0) = x_hat.x;
  A_hat(1, 0) = x_hat.y;
  A_hat(0, 1) = y_hat.x;
  A_hat(1, 1) = y_hat.y;
  A_hat(2, 2) = 1;
  m_tire_frame.rot = A_hat.Get_A_quaternion();
}

void ChPacejkaTire::calc_relaxationLengths()
{
  double p_Ky4 = 2;
  double C_Fx = 177000;  // calibrated using Fx - pure long slip case.      77000
  double C_Fy = 144000; // calibrated using Fy - pure lateral slip case.  144000
  double p_Ky5 = 0;
  double p_Ky6 = 0.92;
  double p_Ky7 = 0.24;

  // all these C values should probably be positive, since negative stiffness doesn't make sense
  // parameter pky1 from A/Car parameter file is negative? not correct, force to positive
  double C_Falpha = abs(m_params->lateral.pky1 * m_params->vertical.fnomin * sin(p_Ky4 * atan(m_FM.force.z / (m_params->lateral.pky2 * m_params->vertical.fnomin))) * m_zeta->z3 * m_params->scaling.lyka);
  double sigma_alpha = abs(C_Falpha / C_Fy);
  double C_Fkappa = m_FM.force.z * (m_params->longitudinal.pkx1 + m_params->longitudinal.pkx2 * m_dF_z) * exp(m_params->longitudinal.pkx3 * m_dF_z) * m_params->scaling.lky;
  double sigma_kappa = C_Fkappa / C_Fx;
  double C_Fgamma = m_FM.force.z * (p_Ky6 + p_Ky7 * m_dF_z) * m_params->scaling.lgay;
  double C_Fphi = (C_Fgamma * m_R0) / (1 - 0.5);

	double sigma_kappa_adams = m_FM.force.z * (m_params->longitudinal.ptx1 + m_params->longitudinal.ptx2 * m_dF_z)*(m_R0*m_params->scaling.lsgkp / m_params->vertical.fnomin) * exp( m_params->longitudinal.ptx3 * m_dF_z);
	double sigma_alpha_adams = m_params->lateral.pty1 * (1.0 - m_params->lateral.pky3 * abs( m_slip->gammaP ) ) * m_R0 * m_params->scaling.lsgal * sin(p_Ky4 * atan(m_FM.force.z / (m_params->lateral.pty2 * m_params->vertical.fnomin) ) );

  {
    relaxationL tmp = { C_Falpha, sigma_alpha, C_Fkappa, sigma_kappa, C_Fgamma, C_Fphi };
    *m_relaxation = tmp;
  }
}


void ChPacejkaTire::calcFx_pureLong()
{
  // double eps_Vx = 0.6;
  double eps_x = 0;
  // Fx, pure long slip
  double S_Hx = (m_params->longitudinal.phx1 + m_params->longitudinal.phx2*m_dF_z)*m_params->scaling.lhx;
  double kappa_x = -m_slip->kappaP + S_Hx;  // * 0.1;

  double mu_x = (m_params->longitudinal.pdx1 + m_params->longitudinal.pdx2*m_dF_z) * (1.0 - m_params->longitudinal.pdx3 * pow(m_slip->gammaP,2) ) * m_params->scaling.lmux;	// >0
  double K_xKappa = m_FM.force.z * (m_params->longitudinal.pkx1 + m_params->longitudinal.pkx2 * m_dF_z) * exp(m_params->longitudinal.pkx3 * m_dF_z) * m_params->scaling.lkx;
  double C_x = m_params->longitudinal.pcx1 * m_params->scaling.lcx;	// >0
  double D_x = mu_x * m_FM.force.z * m_zeta->z1;  // >0
  double B_x = K_xKappa / (C_x * D_x + eps_x);

  double sign_kap = 0;
  if (kappa_x >= 0)
    sign_kap = 1;
  else
    sign_kap = -1;
  double E_x = (m_params->longitudinal.pex1 + m_params->longitudinal.pex2 * m_dF_z + m_params->longitudinal.pex3 *  pow(m_dF_z,2) ) * (1.0 - m_params->longitudinal.pex4*sign_kap)*m_params->scaling.lex;
  double S_Vx = m_FM.force.z * (m_params->longitudinal.pvx1 + m_params->longitudinal.pvx2 * m_dF_z) * m_params->scaling.lvx * m_params->scaling.lmux * m_zeta->z1; //  * abs(m_tireState.lin_vel.x) / (eps_Vx + abs(m_tireState.lin_vel.x))
  double F_x = -D_x * sin(C_x * atan(B_x * kappa_x - E_x * (B_x * kappa_x - atan(B_x * kappa_x)))) - S_Vx;

  // set the longitudinal force
  m_FM.force.x = F_x;

  // hold onto these coefs
  {
    pureLongCoefs tmp = { S_Hx, kappa_x, mu_x, K_xKappa, B_x, C_x, D_x, E_x, F_x, S_Vx };
    *m_pureLong = tmp;
  }
}

void ChPacejkaTire::calcFy_pureLat()
{
  double p_Ky4 = 2.0;
  double p_Ky5 = 0;
  // double p_Ky6 = 0.92;	// not in the default pac2002 file
  // double p_Ky7 = 0.24;	// "
  double p_Ey5 = 0;

  // double K_y0 = m_FM.force.z * (p_Ky6 + p_Ky7 * m_dF_z) * m_params->scaling.lgay;
  double C_y = m_params->lateral.pcy1 * m_params->scaling.lcy;	// > 0
  double mu_y = (m_params->lateral.pdy1 + m_params->lateral.pdy2 * m_dF_z) * (1.0 - m_params->lateral.pdy3 * pow(m_slip->gammaP,2) ) * m_params->scaling.lmuy;	// > 0
  double D_y = mu_y * m_FM.force.z * m_zeta->z2;

  // TODO: does this become negative??? 
  double K_y = m_params->lateral.pky1 * m_params->vertical.fnomin * sin(p_Ky4 * atan(m_FM.force.z / ( (m_params->lateral.pky2 + p_Ky5 * pow(m_slip->gammaP,2) )* m_params->vertical.fnomin))) * (1.0 - m_params->lateral.pky3 * abs(m_slip->gammaP) ) * m_zeta->z3 * m_params->scaling.lyka;

  // doesn't make sense to ever have K_yAlpha be negative (it can be interpreted as lateral stiffnesss)
  double B_y = K_y/ (C_y * D_y + 0.1);

  // double S_Hy = (m_params->lateral.phy1 + m_params->lateral.phy2 * m_dF_z) * m_params->scaling.lhy + (K_yGamma_0 * m_slip->gammaP - S_VyGamma) * m_zeta->z0 / (K_yAlpha + 0.1) + m_zeta->z4 - 1.0;
  // Adasms S_Hy is a bit different
  double S_Hy =  (m_params->lateral.phy1 + m_params->lateral.phy2 * m_dF_z) * m_params->scaling.lhy + (m_params->lateral.phy3 * m_slip->gammaP * m_zeta->z0) + m_zeta->z4 - 1;

  double alpha_y = m_slip->alphaP + S_Hy;

  int sign_alpha = 0;
  if (alpha_y >= 0)
    sign_alpha = 1;
  else
    sign_alpha = -1;

  double E_y = (m_params->lateral.pey1 + m_params->lateral.pey2 * m_dF_z) * (1.0 + p_Ey5 * pow(m_slip->gammaP,2) - (m_params->lateral.pey3 + m_params->lateral.pey4 * m_slip->gammaP) * sign_alpha) * m_params->scaling.ley;
  double S_Vy = m_FM.force.z * ((m_params->lateral.pvy1 + m_params->lateral.pvy2 * m_dF_z) * m_params->scaling.lvy + (m_params->lateral.pvy3 + m_params->lateral.pvy4 * m_dF_z) * m_slip->gammaP) * m_params->scaling.lmuy * m_zeta->z2;

  double F_y = D_y * sin(C_y * atan(B_y * alpha_y - E_y * (B_y * alpha_y - atan(B_y * alpha_y)))) + S_Vy;

  m_FM.force.y = F_y;

  // hold onto coefs
  {
    pureLatCoefs tmp = { S_Hy, alpha_y, mu_y, K_y, S_Vy, B_y, C_y, D_y, E_y, F_y };
    *m_pureLat = tmp;
  }
}

void ChPacejkaTire::calcMz_pureLat()
{
  // some constants
  int sign_Vx = 0;
  if (m_tireState.lin_vel.x >= 0)
    sign_Vx = 1;
  else
    sign_Vx = -1;

  double B_r = (m_params->aligning.qbz9 * (m_params->scaling.lky / m_params->scaling.lmuy) + m_params->aligning.qbz10 * m_pureLat->B_y * m_pureLat->C_y) * m_zeta->z6;
  double C_r = m_zeta->z7;
  double D_r = m_FM.force.z * m_R0 * ( (m_params->aligning.qdz6 + m_params->aligning.qdz7 * m_dF_z) * m_params->scaling.lgyr * m_zeta->z2 + (m_params->aligning.qdz8 + m_params->aligning.qdz9 * m_dF_z) * m_slip->gammaP * m_params->scaling.lgaz * m_zeta->z0 ) * m_slip->cosPrime_alpha * m_params->scaling.lmuy * sign_Vx + m_zeta->z8 - 1.0;
  double B_t = (m_params->aligning.qbz1 + m_params->aligning.qbz2 * m_dF_z + m_params->aligning.qbz3 * pow(m_dF_z,2) ) * (1.0 + m_params->aligning.qbz4 * abs(m_slip->gammaP) + m_params->aligning.qbz5 * m_slip->gammaP) * m_params->scaling.lvyka / m_params->scaling.lmuy;
  double C_t = m_params->aligning.qcz1;
  double D_t0 = m_FM.force.z * (m_R0 / m_params->vertical.fnomin) * (m_params->aligning.qdz1 + m_params->aligning.qdz2 * m_dF_z) * m_params->scaling.ltr * sign_Vx;
  double D_t = D_t0 * (1.0 + m_params->aligning.qdz3 * abs(m_slip->gammaP) + m_params->aligning.qdz4 * pow(m_slip->gammaP,2) ) * m_zeta->z5;
  double KP_yAlpha = m_pureLat->K_y + 0.1;
  double S_Hf = m_pureLat->S_Hy + m_pureLat->S_Vy / KP_yAlpha;

  double alpha_r = -m_slip->alphaP + S_Hf;
  double S_Ht = m_params->aligning.qhz1 + m_params->aligning.qhz2 * m_dF_z + (m_params->aligning.qhz3 + m_params->aligning.qhz4 * m_dF_z) * m_slip->gammaP;
  double alpha_t = -m_slip->alphaP + S_Ht;
  double E_t = (m_params->aligning.qez1 + m_params->aligning.qez2 * m_dF_z + m_params->aligning.qez3 * pow(m_dF_z,2) ) * (1.0 + (m_params->aligning.qez4 + m_params->aligning.qez5 * m_slip->gammaP) * (2.0 / chrono::CH_C_PI) * atan(B_t * C_t * alpha_t) );
  double t0 = D_t * cos(C_t * atan(B_t * alpha_t - E_t * (B_t * alpha_t - atan(B_t * alpha_t)))) * m_slip->cosPrime_alpha;

  double MP_z0 = -t0 * m_FM.force.y;
  double M_zr0 = -D_r * cos(C_r * atan(B_r * alpha_r));

  double M_z = MP_z0 + M_zr0;
  m_FM.moment.z = M_z;

  // hold onto coefs
  {
    pureTorqueCoefs tmp = {
      S_Hf, alpha_r, S_Ht, alpha_t, m_slip->cosPrime_alpha, KP_yAlpha,
      B_r, C_r, D_r,
      B_t, C_t, D_t0, D_t, E_t, t0,
      MP_z0, M_zr0 };
    *m_pureTorque = tmp;
  }
}


// calculate Fx for combined slip
void ChPacejkaTire::calcFx_combined()
{
  double rbx3 = 1.0;

  double S_HxAlpha = m_params->longitudinal.rhx1;
  double alpha_S = -m_slip->alphaP - S_HxAlpha;
  double B_xAlpha = (m_params->longitudinal.rbx1 + rbx3 * pow(m_slip->gammaP, 2)) * cos(atan(m_params->longitudinal.rbx2 * m_slip->kappa)) * m_params->scaling.lxal;
  double C_xAlpha = m_params->longitudinal.rcx1;
  double E_xAlpha = m_params->longitudinal.rex1 + m_params->longitudinal.rex2 * m_dF_z;

  // double G_xAlpha0 = cos(C_xAlpha * atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha - atan(B_xAlpha * S_HxAlpha)) ) );
  double G_xAlpha0 = cos(C_xAlpha * atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha - atan(B_xAlpha * S_HxAlpha))));

  // double G_xAlpha = cos(C_xAlpha * atan(B_xAlpha * alpha_S - E_xAlpha * (B_xAlpha * alpha_S - atan(B_xAlpha * alpha_S)) ) ) / G_xAlpha0;
  double G_xAlpha = cos(C_xAlpha * atan(B_xAlpha * alpha_S - E_xAlpha * (B_xAlpha * alpha_S - atan(B_xAlpha * alpha_S)))) / G_xAlpha0;

  double F_x = G_xAlpha * m_FM.force.x;
  m_FM_combined.force.x = F_x;

  {
    combinedLongCoefs tmp = { S_HxAlpha, alpha_S, B_xAlpha, C_xAlpha, E_xAlpha, G_xAlpha0, G_xAlpha };
    *m_combinedLong = tmp;
  }
}

// calc Fy for combined slip
void ChPacejkaTire::calcFy_combined()
{
  double rby4 = 0;

  double S_HyKappa = m_params->lateral.rhy1 + m_params->lateral.rhy2 * m_dF_z;
  double kappa_S = -m_slip->kappaP + S_HyKappa;
  double B_yKappa = (m_params->lateral.rby1 + rby4 * pow(m_slip->gammaP,2) ) * cos(atan(m_params->lateral.rby2 * (-m_slip->alphaP - m_params->lateral.rby3))) * m_params->scaling.lyka;
  double C_yKappa = m_params->lateral.rcy1;
  double E_yKappa = m_params->lateral.rey1 + m_params->lateral.rey2 * m_dF_z;
  double D_VyKappa = m_pureLat->mu_y * m_FM.force.z * (m_params->lateral.rvy1 + m_params->lateral.rvy2 * m_dF_z + m_params->lateral.rvy3 * m_slip->gammaP) * cos(atan(m_params->lateral.rvy4 * -m_slip->alphaP)) * m_zeta->z2;
  double S_VyKappa = D_VyKappa * sin(m_params->lateral.rvy5 * atan(m_params->lateral.rvy6 * m_slip->kappaP)) * m_params->scaling.lvyka;
  double G_yKappa0 = cos(C_yKappa * atan(B_yKappa * S_HyKappa - E_yKappa * (B_yKappa * S_HyKappa - atan(B_yKappa * S_HyKappa))));
  double G_yKappa = cos(C_yKappa * atan(B_yKappa * kappa_S - E_yKappa * (B_yKappa * kappa_S - atan(B_yKappa * kappa_S)))) / G_yKappa0;

  double F_y = G_yKappa * m_FM.force.y - S_VyKappa;
  m_FM_combined.force.y = F_y;

  {
    combinedLatCoefs tmp = { S_HyKappa, kappa_S, B_yKappa, C_yKappa, E_yKappa, D_VyKappa, S_VyKappa, G_yKappa0, G_yKappa };
    *m_combinedLat = tmp;
  }
}

// calc Mz for combined slip
void ChPacejkaTire::calcMz_combined()
{
  double FP_y = m_FM_combined.force.y - m_combinedLat->S_VyKappa;
  double s = m_R0 * (m_params->aligning.ssz1 + m_params->aligning.ssz2 * (m_FM_combined.force.y / m_params->vertical.fnomin) + (m_params->aligning.ssz3 + m_params->aligning.ssz4 * m_dF_z) * m_slip->gammaP) * m_params->scaling.ls;
  int sign_alpha_t = 0;
  int sign_alpha_r = 0;
  if (m_pureTorque->alpha_t >= 0){
    sign_alpha_t = -1;
  }
  else {
    sign_alpha_t = 1;
  }
  if (m_pureTorque->alpha_r >= 0){
    sign_alpha_r = -1;
  }
  else {
    sign_alpha_r = 1;
  }

  double alpha_t_eq = sign_alpha_t * sqrt(pow(m_pureTorque->alpha_t, 2) + pow(m_pureLong->K_xKappa / m_pureTorque->KP_yAlpha, 2)
    * pow(m_slip->kappa, 2));
  double alpha_r_eq = sign_alpha_r * sqrt(pow(m_pureTorque->alpha_r, 2) + pow(m_pureLong->K_xKappa / m_pureTorque->KP_yAlpha, 2)
    * pow(m_slip->kappa, 2));

  double M_zr = m_pureTorque->D_r * cos(m_pureTorque->C_r * atan(m_pureTorque->B_r * alpha_r_eq));
  double t = m_pureTorque->D_t * cos(m_pureTorque->C_t * atan(m_pureTorque->B_t*alpha_t_eq - m_pureTorque->E_t * (m_pureTorque->B_t * alpha_t_eq - atan(m_pureTorque->B_t * alpha_t_eq)))) * m_slip->cosPrime_alpha;
  double MP_z = -t * FP_y;

  double M_z = MP_z + M_zr  + (s * m_FM_combined.force.x);
  m_FM_combined.moment.z = M_z;

  {
    combinedTorqueCoefs tmp = { m_slip->cosPrime_alpha, FP_y, s, alpha_t_eq, alpha_r_eq, M_zr, t, MP_z };
    *m_combinedTorque = tmp;
  }
}



void ChPacejkaTire::calc_Mx()
{
  double M_x = m_FM.force.z * m_R0 * (m_params->overturning.qsx1 - m_params->overturning.qsx2 * m_slip->gammaP
    - m_params->overturning.qsx3 * (m_FM_combined.force.y / m_params->vertical.fnomin)) * m_params->scaling.lmx;
  m_FM.moment.x = M_x;
  m_FM_combined.moment.x = M_x;
}


void ChPacejkaTire::calc_My()
{
  double V_r = m_tireState.ang_vel.y * m_R_eff;
  double M_y = -m_FM.force.z * m_R0 * (m_params->rolling.qsy1 * atan(V_r / m_params->model.longvl) - m_params->rolling.qsy2 * (m_FM_combined.force.x / m_params->vertical.fnomin)) * m_params->scaling.lmy;
  m_FM.moment.y = M_y;
  m_FM_combined.moment.y = M_y;
}


// what does the PacTire in file look like?
// see models/data/hmmwv/pactest.tir
void ChPacejkaTire::loadPacTireParamFile()
{
  // try to load the file
  std::ifstream inFile(this->getPacTireParamFile().c_str(), std::ios::in);

  // if not loaded, say something and exit
  if (!inFile.is_open())
  {
    GetLog() << "\n\n !!!!!!! couldn't load the pac tire file: " << getPacTireParamFile().c_str() << "\n\n";
    GetLog() << " I bet you have the pacTire param file opened in a text editor somewhere.... \n\n\n";
    return;
  }

  // success in opening file, load the data, broken down into sections
  // according to what is found in the PacTire input file
  readPacTireInput(inFile);

  // this bool will allow you to query the pac tire for output
  // Forces, moments based on wheel state info.
  m_params_defined = true;
}

void ChPacejkaTire::readPacTireInput(std::ifstream& inFile)
{
  // advance to the first part of the file with data we need to read
  std::string tline;

  while (std::getline(inFile, tline))
  {
    // first main break
    if (tline[0] == '$')
      break;
  }

  // to keep things sane (and somewhat orderly), create a function to read 
  // each subsection. there may be overlap between different PacTire versions,
  // where these section read functions can be reused

  // 0:  [UNITS], all token values are strings
  readSection_UNITS(inFile);

  // 1: [MODEL]
  readSection_MODEL(inFile);

  // 2: [DIMENSION]
  readSection_DIMENSION(inFile);

  // 3: [SHAPE]
  readSection_SHAPE(inFile);

  // 4: [VERTICAL]
  readSection_VERTICAL(inFile);

  // 5-8, ranges for: LONG_SLIP, SLIP_ANGLE, INCLINATION_ANGLE, VETRICAL_FORCE,
  // in that order
  readSection_RANGES(inFile);

  // 9: [scaling]
  readSection_scaling(inFile);

  // 10: [longitudinal]
  readSection_longitudinal(inFile);

  // 11: [overturning]
  readSection_overturning(inFile);

  // 12: [lateral]
  readSection_lateral(inFile);

  // 13: [rolling]
  readSection_rolling(inFile);

  // 14: [aligning]
  readSection_aligning(inFile);

}


void ChPacejkaTire::readSection_UNITS(std::ifstream& inFile)
{
  // skip the first line
  std::string tline;
  std::getline(inFile, tline);

  // string util stuff
  std::string tok;    // name of token
  std::string val_str; // temp for string token values
  std::vector<std::string> split;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
  }
}

void ChPacejkaTire::readSection_MODEL(std::ifstream& inFile)
{
  // skip the first line
  std::string tline;
  std::getline(inFile, tline);

  // string util stuff
  std::vector<std::string> split;
  std::string val_str; // string token values

  // token value changes type in this section, do it manually
  std::getline(inFile, tline);

  // get the token / value
  split = utils::splitStr(tline, '=');
  m_params->model.property_file_format = utils::splitStr(split[1], '\'')[1];

  std::getline(inFile, tline);
  m_params->model.use_mode = utils::fromTline<int>(tline);

  std::getline(inFile, tline);
  m_params->model.vxlow = utils::fromTline<double>(tline);

  std::getline(inFile, tline);
  m_params->model.longvl = utils::fromTline<double>(tline);

  std::getline(inFile, tline);
  split = utils::splitStr(tline, '=');
  m_params->model.tyreside = utils::splitStr(split[1], '\'')[1];

}

void ChPacejkaTire::readSection_DIMENSION(std::ifstream& inFile)
{
  // skip the first two lines
  std::string tline;
  std::getline(inFile, tline);
  std::getline(inFile, tline);
  // if all the data types are the same in a subsection, life is a little easier
  // push each token value to this vector, check the # of items added, then 
  // create the struct by hand only
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 5) {
    GetLog() << " error reading DIMENSION section of pactire input file!!! \n\n";
    return;
  }
  // right size, create the struct
  struct dimension dim = { dat[0], dat[1], dat[2], dat[3], dat[4] };
  m_params->dimension = dim;

}


void ChPacejkaTire::readSection_SHAPE(std::ifstream& inFile)
{
  // skip the first two lines
  std::string tline;
  std::getline(inFile, tline);
  std::getline(inFile, tline);
  // if all the data types are the same in a subsection, life is a little easier
  // push each token value to this vector, check the # of items added, then 
  // create the struct by hand only
  std::vector<double> rad;
  std::vector<double> wid;
  std::vector<std::string> split;
  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    split = utils::splitStr(tline, ' ');
    rad.push_back(std::atof(split[1].c_str()));
    wid.push_back(std::atof(split[5].c_str()));
  }
  m_params->shape.radial = rad;
  m_params->shape.width = wid;
}


void ChPacejkaTire::readSection_VERTICAL(std::ifstream& inFile){
  // skip the first line
  std::string tline;
  std::getline(inFile, tline);

  // push each token value to this vector, check the # of items added
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 6) {
    GetLog() << " error reading VERTICAL section of pactire input file!!! \n\n";
    return;
  }
  // right size, create the struct
  struct vertical vert = { dat[0], dat[1], dat[2], dat[3], dat[4], dat[5] };
  m_params->vertical = vert;

}


void ChPacejkaTire::readSection_RANGES(std::ifstream& inFile){
  // skip the first line
  std::string tline;
  std::getline(inFile, tline);
  // if all the data types are the same in a subsection, life is a little easier
  // push each token value to this vector, check the # of items added, then 
  // create the struct by hand only
  std::vector<double> dat;

  // LONG_SLIP_RANGE
  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 2) {
    GetLog() << " error reading LONG_SLIP_RANGE section of pactire input file!!! \n\n";
    return;
  }
  // right size, create the struct
  struct long_slip_range long_slip = { dat[0], dat[1] };
  m_params->long_slip_range = long_slip;
  dat.clear();
  std::getline(inFile, tline);

  // SLIP_ANGLE_RANGE
  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 2) {
    GetLog() << " error reading LONG_SLIP_RANGE section of pactire input file!!! \n\n";
    return;
  }
  // right size, create the struct
  struct slip_angle_range slip_ang = { dat[0], dat[1] };
  m_params->slip_angle_range = slip_ang;
  dat.clear();
  std::getline(inFile, tline);

  // INCLINATION_ANGLE_RANGE
  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 2) {
    GetLog() << " error reading INCLINATION_ANGLE_RANGE section of pactire input file!!! \n\n";
    return;
  }
  struct inclination_angle_range incl_ang = { dat[0], dat[1] };
  m_params->inclination_angle_range = incl_ang;
  dat.clear();
  std::getline(inFile, tline);

  // VERTICAL_FORCE_RANGE
  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 2) {
    GetLog() << " error reading VERTICAL_FORCE_RANGE section of pactire input file!!! \n\n";
    return;
  }
  struct vertical_force_range vert_range = { dat[0], dat[1] };
  m_params->vertical_force_range = vert_range;

}


void ChPacejkaTire::readSection_scaling(std::ifstream& inFile)
{
  std::string tline;
  std::getline(inFile, tline);
  // if all the data types are the same in a subsection, life is a little easier
  // push each token value to this vector, check the # of items added, then 
  // create the struct by hand only
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 28) {
    GetLog() << " error reading scaling section of pactire input file!!! \n\n";
    return;
  }
  struct scaling_coefficients coefs = { dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7],
    dat[8], dat[9], dat[10], dat[11], dat[12], dat[13], dat[14], dat[15], dat[16], dat[17],
    dat[18], dat[19], dat[20], dat[21], dat[22], dat[23], dat[24], dat[25], dat[26], dat[27] };
  m_params->scaling = coefs;
}


void ChPacejkaTire::readSection_longitudinal(std::ifstream& inFile)
{
  std::string tline;
  std::getline(inFile, tline);
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 24) {
    GetLog() << " error reading longitudinal section of pactire input file!!! \n\n";
    return;
  }
  struct longitudinal_coefficients coefs = { dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7],
    dat[8], dat[9], dat[10], dat[11], dat[12], dat[13], dat[14], dat[15], dat[16], dat[17],
    dat[18], dat[19], dat[20], dat[21], dat[22], dat[23] };
  m_params->longitudinal = coefs;

}


void ChPacejkaTire::readSection_overturning(std::ifstream& inFile)
{
  std::string tline;
  std::getline(inFile, tline);
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 3) {
    GetLog() << " error reading  overturning section of pactire input file!!! \n\n";
    return;
  }
  struct overturning_coefficients coefs = { dat[0], dat[1], dat[2] };
  m_params->overturning = coefs;

}


void ChPacejkaTire::readSection_lateral(std::ifstream& inFile)
{
  std::string tline;
  std::getline(inFile, tline);
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 34) {
    GetLog() << " error reading lateral section of pactire input file!!! \n\n";
    return;
  }
  struct lateral_coefficients coefs = { dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7],
    dat[8], dat[9], dat[10], dat[11], dat[12], dat[13], dat[14], dat[15], dat[16], dat[17],
    dat[18], dat[19], dat[20], dat[21], dat[22], dat[23], dat[24], dat[25], dat[26], dat[27],
    dat[28], dat[29], dat[30], dat[31], dat[32], dat[33] };
  m_params->lateral = coefs;

}


void ChPacejkaTire::readSection_rolling(std::ifstream& inFile)
{
  std::string tline;
  std::getline(inFile, tline);
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 4) {
    GetLog() << " error reading rolling section of pactire input file!!! \n\n";
    return;
  }
  struct rolling_coefficients coefs = { dat[0], dat[1], dat[2], dat[3] };
  m_params->rolling = coefs;

}


void ChPacejkaTire::readSection_aligning(std::ifstream& inFile)
{
  std::string tline;
  std::getline(inFile, tline);
  std::vector<double> dat;

  while (std::getline(inFile, tline)) {
    // made it to the next section
    if (tline[0] == '$')
      break;
    dat.push_back(utils::fromTline<double>(tline));
  }

  if (dat.size() != 31) {
    GetLog() << " error reading LONG_SLIP_RANGE section of pactire input file!!! \n\n";
    return;
  }
  struct aligning_coefficients coefs = { dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7],
    dat[8], dat[9], dat[10], dat[11], dat[12], dat[13], dat[14], dat[15], dat[16], dat[17],
    dat[18], dat[19], dat[20], dat[21], dat[22], dat[23], dat[24], dat[25], dat[26], dat[27],
    dat[28], dat[29], dat[30] };
  m_params->aligning = coefs;

}


// write for output to python pandas module
// 
void ChPacejkaTire::WriteOutData(double             time,
                                 const std::string& outFilename)
{
  // first time thru, write headers
  if (m_Num_WriteOutData == 0) {
    m_outFilename = outFilename;
    std::ofstream oFile(outFilename.c_str(), std::ios_base::out);
    if (!oFile.is_open()) {
      std::cout << " couldn't open file for writing: " << outFilename << " \n\n";
      return;
    }
    else {
      // write the headers
      oFile << "time,kappa,alpha,gamma,kappaP,alphaP,gammaP,Vx,Vy,Fx,Fy,Fz,Mx,My,Mz,Fxc,Fyc,Mzc" << std::endl;
      m_Num_WriteOutData++;
      oFile.close();
    }
  }
  else {
    // already written the headers, just increment the function counter
    m_Num_WriteOutData++;
  }
  // ensure file was able to be opened, headers are written
  if (m_Num_WriteOutData > 0) {
    // open file, append
    std::ofstream appFile(outFilename.c_str(), std::ios_base::app);
    // write the slip info, reaction forces for pure & combined slip cases
    appFile << time << "," << m_slip->kappa << "," << m_slip->alpha*180. / 3.14159 << "," << m_slip->gamma << ","
      << m_slip->kappaP << "," << atan(m_slip->alphaP)*180. / 3.14159 << "," << m_slip->gammaP << ","
      << m_tireState.lin_vel.x << "," << m_tireState.lin_vel.y << ","
      << m_FM.force.x << "," << m_FM.force.y << "," << m_FM.force.z << ","
      << m_FM.moment.x << "," << m_FM.moment.y << "," << m_FM.moment.z << ","
      << m_FM_combined.force.x << "," << m_FM_combined.force.y << "," << m_FM_combined.moment.z
      << std::endl;
    // close the file
    appFile.close();
  }
}


// -----------------------------------------------------------------------------
// Calculate the wheel state from the current kappa, alpha, and gamma values.
// -----------------------------------------------------------------------------
ChWheelState ChPacejkaTire::getState_from_KAG(double kappa,
                                              double alpha,
                                              double gamma,
                                              double Vx)
{
  ChWheelState state;

  // orientation is purely a function of gamma, alpha
  ChQuaternion<> m_quat = chrono::Q_from_NasaAngles(ChVector<>(gamma, 0, alpha));

  // wheel center velocity, tire coords
  double Vcx = Vx * cos(alpha);
  double Vcy = Vcx * tan(alpha);
  // double Vcy_check = Vx * sin(alpha);
  // kappa = r_e*w_y - |v| / |v|

  // wheel spin rate, tire coords
  double w_y = (kappa*Vcx + Vcx) / m_R_eff;

  // set the orientation, 1 lin vel and 1 ang vel (in 2 places)
  state.rot = m_quat;
  state.lin_vel.x = Vx;
	state.ang_vel.y = w_y;
	state.omega = w_y;

  return state;
}

// -----------------------------------------------------------------------------
// Calculate kappa, alpha, and gamma from the specified wheel state.
// -----------------------------------------------------------------------------
ChVector<> ChPacejkaTire::getKAG_from_State(const ChWheelState& state)
{
  // ATTENTION: with the current reference frames, when the vehicle moves
  // forward, typically V_cx is negative and the wheel omega is negative!!!
  // Here, we want to use a frame with Z up, X forward, and Y to the left.
  // Until the chrono-T reference frames change, we explicitly convert
  // everything here, by applying a rotation of 180 degrees around Z.
  // The only two quantities we must adjust are the wheel normal and the wheel
  // angular speed.
  ChQuaternion<> R = Q_from_AngZ(CH_C_PI);

  // Wheel normal (expressed in global frame)
  ChVector<> wheel_normal = state.rot.GetYaxis();
  wheel_normal = R.Rotate(wheel_normal);           // reference frame conversion

  double omega = -state.omega;                     // reference frame conversion

  // Terrain normal at wheel center location (expressed in global frame)
  ChVector<> Z_dir = m_terrain.GetNormal(state.pos.x, state.pos.y);

  // Longitudinal (heading) and lateral directions, in the terrain plane.
  ChVector<> X_dir = Vcross(wheel_normal, Z_dir);
  ChVector<> Y_dir = Vcross(Z_dir, X_dir);

  // Decompose the wheel velocity in the X and Y directions and calculate the
  // slip angle, alpha.
  double V_x = Vdot(state.lin_vel, X_dir);
  double V_y = Vdot(state.lin_vel, Y_dir);
  double alpha = atan2(V_y, V_x);

  // Decompose the wheel normal in the Z and Y directions and calculate the
  // wheel camber angle, gamma.
  double n_z = Vdot(wheel_normal, Z_dir);
  double n_y = Vdot(wheel_normal, Y_dir);
  double gamma = atan2(n_z, n_y);

  // Longitudinal slip rate.
  double kappa = (m_R_eff * omega - V_x) / abs(V_x);

  /*
  // alpha, gamma from wheel orientation
  double alpha = chrono::Q_to_NasaAngles(state.rot).z;
  double gamma = chrono::Q_to_NasaAngles(state.rot).x;

  // kappa = r_e*w_y - v_x / v_x
  double Vcx = state.lin_vel.x * cos(alpha) + state.lin_vel.y * sin(alpha);
  double Vcy = -state.lin_vel.x * sin(alpha) + state.lin_vel.y * cos(alpha);
  double kappa = (m_R_eff * state.ang_vel.y - Vcx) / (Vcx);

  double check_a1 = atan(-Vcy / Vcx);
  */

  ChVector<> kag(kappa, alpha, gamma);

  return kag;
}


}  // end namespace chrono
