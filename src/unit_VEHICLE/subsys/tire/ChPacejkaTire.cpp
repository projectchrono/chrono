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

#include <cmath>

#include "subsys/tire/ChPacejkaTire.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsStringFunctions.h"

#include "subsys/tire/ChPac2002_data.h"

namespace chrono{

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

// Threshold value for small forward tangential velocity.
static const double v_x_threshold = 0.1;

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

  //// TODO:  Need to figure out a better way of indicating errors
  ////        Right now, we cannot have this function throw an exception since
  ////        it's called from the constructor!   Must fix this...
  if (!m_params_defined) {
    GetLog() << " couldn't load pacTire parameters from file, not updating initial quantities \n\n";
    return;
  }

  // any variables that are calculated once
  m_R0 = m_params->dimension.unloaded_radius;

  //// TODO:  why do we have to initialize m_R_l and m_R_eff here?
  ////        This is done in Update(), when we have a proper wheel state.

  m_R_l = m_R0 - 8000.0 / m_params->vertical.vertical_stiffness;
  double qV1 = 1.5;
  double rho = (m_R0 - m_R_l) * exp(-qV1 * m_R0 * pow(1.05 * m_params->model.longvl / m_params->model.longvl, 2));
  m_R_eff = m_R0 - rho;

  // not sure what these are used for
  {
    zetaCoefs tmp = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };
    *m_zeta = tmp;
  }

  // any of the structs associated with multipliers in reaction calcs
  m_combinedTorque->alpha_r_eq = 0.0;
  m_pureLat->D_y = m_params->vertical.fnomin;  // initial approximation

  // init all other variables
  m_Num_WriteOutData = 0;
  {
    slips tmp = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    *m_slip = tmp;
  }
}

// -----------------------------------------------------------------------------
// Return computed tire forces and moment (pure slip or combined slip and in
// local or global frame). The main GetTireForce() function returns the combined
// slip tire forces, expressed in the global frame.
// -----------------------------------------------------------------------------
ChTireForce ChPacejkaTire::GetTireForce() const
{
  return GetTireForce_combinedSlip(false);
}

ChTireForce ChPacejkaTire::GetTireForce_pureSlip(const bool local) const
{
  if (local)
    return m_FM;

  // reactions are on wheel CM
  ChTireForce m_FM_global;
  m_FM_global.point = m_tireState.pos;
  // only transform the directions of the forces, moments, from local to global
  m_FM_global.force = m_tire_frame.TransformDirectionLocalToParent(m_FM.force);
  m_FM_global.moment = m_tire_frame.TransformDirectionLocalToParent(m_FM.moment);

  return m_FM_global;
}

/// Return the reactions for the combined slip EQs, in local or global coords
ChTireForce ChPacejkaTire::GetTireForce_combinedSlip(const bool local) const
{
  if (local)
    return m_FM_combined;

  // reactions are on wheel CM
  ChTireForce m_FM_global;
  m_FM_global.point = m_tireState.pos;
  // only transform the directions of the forces, moments, from local to global
  m_FM_global.force = m_tire_frame.TransformDirectionLocalToParent(m_FM_combined.force);
  m_FM_global.moment = m_tire_frame.TransformDirectionLocalToParent(m_FM_combined.moment);

  return m_FM_global;
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

double ChPacejkaTire::get_kappaPrime() const
{
  return m_slip->kappaP;
}

double ChPacejkaTire::get_alphaPrime() const
{
  return m_slip->alphaP;
}

double ChPacejkaTire::get_gammaPrime() const
{
  return m_slip->gammaP;
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
// Update the internal state of this tire using the specified wheel state. The
// quantities calculated here will be kept constant until the next call to the
// Update() function.
// -----------------------------------------------------------------------------
void ChPacejkaTire::Update(double               time,
                           const ChWheelState&  state)
{
  //// TODO: This must be removed from here.  A tire with unspecified or
  ////       incorrect parameters should have been invalidate at initialization.
  // Check that input tire model parameters are defined
  if (!m_params_defined)
  {
    GetLog() << " ERROR: cannot update tire w/o setting the model parameters first! \n\n\n";
    return;
  }

  // Cache the wheel state and update the tire coordinate system.
  m_tireState = state;
  update_tireFrame();

  // If not using the transient slip model, check that the tangential forward
  // velocity is not too small.
  ChVector<> V = m_tire_frame.TransformDirectionParentToLocal(m_tireState.lin_vel);
  if (!m_use_transient_slip && std::abs(V.x) < 0.1)
  {
    GetLog() << " ERROR: tangential forward velocity below threshold.... \n\n";
    return;
  }

  // Initialize the output tire forces to ensure that we do not report any tire
  // forces if the wheel does not contact the terrain.
  m_FM.point = ChVector<>(0, 0, 0);
  m_FM.force = ChVector<>(0, 0, 0);
  m_FM.moment = ChVector<>(0, 0, 0);

  m_FM_combined.point = ChVector<>(0, 0, 0);
  m_FM_combined.force = ChVector<>(0, 0, 0);
  m_FM_combined.moment = ChVector<>(0, 0, 0);

  // Calculate the vertical load and update tire deflection and tire rolling
  // radius.
  update_verticalLoad();

  m_dF_z = (m_FM.force.z - m_params->vertical.fnomin) / m_params->vertical.fnomin;

  // Calculate kinematic slip quantities, based on specified wheel state. This
  // assumes that the inputs to wheel spindle instantly affect tire contact.
  // Note: kappaP, alphaP, gammaP may be overridden in Advance
  calc_slip_kinematic();
}


// -----------------------------------------------------------------------------
// Advance the state of this tire by the specified time step. If using the
// transient slip model, perform integration taking as many integration steps
// as needed.
// -----------------------------------------------------------------------------
void ChPacejkaTire::Advance(double step)
{
  // Do nothing if the wheel does not contact the terrain.  In this case, all
  // reported tire forces will be zero.
  if (!m_in_contact)
    return;

  // If using single point contact model, slips are calculated from compliance
  // between tire and contact patch.
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
    // take one final step to reach the specified time.
    advance_slip_transient(remaining_time);
  }

  // Calculate the force and moment reaction, pure slip case
  calc_pureSlipReactions();

  // Update m_FM_combined.forces, m_FM_combined.moment.z
  calc_combinedSlipReactions();

  // Update M_x, apply to both m_FM and m_FM_combined
  calc_Mx();

  // Update M_y, apply to both m_FM and m_FM_combined
  calc_My();
}


// -----------------------------------------------------------------------------
// Calculate the current tire coordinate system, centered at the wheel origin,
// with the Z-axis normal to the terrain and X-axis in the heading direction.
//
//// TODO: check comment below for correctness.
// need a local frame to transform output forces/moments to global frame
// Pacejka (2006), Fig 2.3, all forces are calculated at the contact point "C"
// Moment calculations take this into account, so all reactions are global
// -----------------------------------------------------------------------------
void ChPacejkaTire::update_tireFrame()
{
  // Wheel normal (expressed in global frame)
  ChVector<> wheel_normal = m_tireState.rot.GetYaxis();

  // Terrain normal at wheel center location (expressed in global frame)
  ChVector<> Z_dir = m_terrain.GetNormal(m_tireState.pos.x, m_tireState.pos.y);

  // Longitudinal (heading) and lateral directions, in the terrain plane.
  ChVector<> X_dir = Vcross(wheel_normal, Z_dir);
  X_dir.Normalize();
  ChVector<> Y_dir = Vcross(Z_dir, X_dir);

  // Create a rotation matrix from the three unit vectors
  ChMatrix33<> rot;
  rot.Set_A_axis(X_dir, Y_dir, Z_dir);

  // Construct the tire coordinate system.
  m_tire_frame.pos = m_tireState.pos;
  m_tire_frame.rot = rot.Get_A_quaternion();
}


// -----------------------------------------------------------------------------
// Calculate the tire vertical load at the current configuration and update the
// tire deflection and tire rolling radius.
// -----------------------------------------------------------------------------
void ChPacejkaTire::update_verticalLoad()
{
  double Fz;

  if (m_use_Fz_override) {
    // Vertical load specified as input.
    Fz = m_Fz_override;

    // Estimate the tire deformation and set the statically loaded radius
    // (assuming static loading).
    m_R_l = m_R0 - Fz / m_params->vertical.vertical_stiffness;

    // In this case, the tire is always assumed to be in contact with the
    // terrain.
    m_in_contact = true;
  }
  else {
    // Calculate the tire vertical load using a spring-damper model. Note that
    // this also sets the statically loaded radius m_R_l, as well as the boolean
    // flag m_in_contact.
    Fz = calc_Fz();
  }

  // Calculate tire vertical deflection, rho.
  double qV1 = 0.000071;      // from example
  double K1 = pow(m_tireState.omega * m_R0 / m_params->model.longvl, 2);
  double rho = m_R0 - m_R_l + qV1 * m_R0 * K1;
  double rho_Fz0 = m_params->vertical.fnomin / (m_params->vertical.vertical_stiffness);
  double rho_d = rho / rho_Fz0;

  // Calculate tire rolling radius, R_eff.  Clamp this value to R0.
  m_R_eff = m_R0 + qV1 * m_R0 * K1 - rho_Fz0 * (m_params->vertical.dreff * std::atan(m_params->vertical.breff * rho_d) + m_params->vertical.freff * rho_d);
  if (m_R_eff > m_R0)
    m_R_eff = m_R0;

  // Load vertical component of tire force.
  m_FM.force.z = Fz;
  m_FM_combined.force.z = Fz;
}


double ChPacejkaTire::calc_Fz()
{
  // Initialize the statically loaded radius to R0.  This is the returned value
  // if no vertical force is generated.
  m_R_l = m_R0;

  // Check contact with terrain, using a disc of radius R0.
  ChCoordsys<> contact_frame;
  double       depth;

  m_in_contact = disc_terrain_contact(m_tireState.pos, m_tireState.rot.GetYaxis(), m_R0,
                                      contact_frame, depth);

  if (!m_in_contact)
    return 0;

  // Calculate relative velocity (wheel - terrain) at contact point, in the
  // global frame and then express it in the contact frame.
  ChVector<> relvel_abs = m_tireState.lin_vel + Vcross(m_tireState.ang_vel, contact_frame.pos - m_tireState.pos);
  ChVector<> relvel_loc = contact_frame.TransformDirectionParentToLocal(relvel_abs);

  // Calculate normal contact force, using a spring-damper model. If the
  // resulting force is negative, the wheel is moving away from the terrain so
  // fast that no contact force is generated.
  double Fz = m_params->vertical.vertical_stiffness * depth - m_params->vertical.vertical_damping * relvel_loc.z;

  if (Fz < 0)
    return 0;

  m_R_l = m_R0 - depth;

  return Fz;
}


// -----------------------------------------------------------------------------
// Calculate kinematic slip quantities from the current wheel state.
//
// The wheel state strcture contains the following member variables:
//   ChVector<>     pos;        global position
//   ChQuaternion<> rot;        orientation with respect to global frame
//   ChVector<>     lin_vel;    linear velocity, expressed in the global frame
//   ChVector<>     ang_vel;    angular velocity, expressed in the global frame
//   double         omega;      wheel angular speed about its rotation axis
// -----------------------------------------------------------------------------
void ChPacejkaTire::calc_slip_kinematic()
{
  // Express the wheel velocity in the tire coordinate system and calculate the
  // slip angle alpha. (Override V.x if too small)
  ChVector<> V = m_tire_frame.TransformDirectionParentToLocal(m_tireState.lin_vel);

  if (std::abs(V.x) < v_x_threshold) {
    V.x = (V.x < 0) ? -v_x_threshold : v_x_threshold;
  }

  double alpha = std::atan2(V.y, V.x);

  // Express the wheel normal in the tire coordinate system and calculate the
  // wheel camber angle gamma.
  ChVector<> n = m_tire_frame.TransformDirectionParentToLocal(m_tireState.rot.GetYaxis());
  double gamma = std::atan2(n.z, n.y);

  // Longitudinal slip rate.
  double V_x_abs = std::abs(V.x);
  double kappa = (m_R_eff * m_tireState.omega - V.x) / V_x_abs;

  // alpha_star = tan(alpha) = v_y / v_x
  double alpha_star = V.y / V_x_abs;

  // Set the struct data members, input slips to wheel
  m_slip->kappa = kappa;
  m_slip->alpha = alpha;
  m_slip->alpha_star = alpha_star;
  m_slip->gamma = gamma;

  m_slip->V_cx = V.x;    // tire center x-vel, tire c-sys
  m_slip->V_cy = V.y;    // tire center y-vel, tire c-sys
  m_slip->V_sx = V.x - m_tireState.omega * m_R_eff;  // x-slip vel, tire c-sys
  m_slip->V_sy = V.y;                                // approx.

  // Express the wheel angular velocity in the tire coordinate system and 
  // extract the turn slip velocity, psi_dot
  ChVector<> w = m_tire_frame.TransformDirectionParentToLocal(m_tireState.ang_vel);
  m_slip->psi_dot = w.z;

  // For aligning torque, to handle large slips, and backwards operation
  double V_mag = std::sqrt(V.x * V.x + V.y * V.y);
  m_slip->cosPrime_alpha = V.x / V_mag;

  // Finally, if non-transient, use wheel slips as input to Magic Formula.
  // These get over-written if enable transient slips to be calculated
  m_slip->kappaP = kappa;
  m_slip->alphaP = alpha_star;
  m_slip->gammaP = std::sin(gamma);
}

// -----------------------------------------------------------------------------
// Calculate transient slip properties, using first order ODEs to find slip
// displacements from velocities.
// -----------------------------------------------------------------------------
void ChPacejkaTire::advance_slip_transient(double step_size)
{
  // hard-coded, for now
  double EPS_GAMMA = 0.6;
  // update relaxation lengths
  calc_relaxationLengths();

  // local c-sys velocities
  double V_cx = m_slip->V_cx;
  double V_cx_abs = std::abs(V_cx);
  double V_cx_low = 2.5;   // cut-off for low velocity zone

  // see if low velocity considerations should be made
  double alpha_sl = 3.0 * m_pureLat->D_y / m_relaxation->C_Falpha;
  // Eq. 7.25 from Pacejka (2006)
  if ((std::abs(m_combinedTorque->alpha_r_eq) > alpha_sl) && (V_cx_abs < V_cx_low))
  {
    // Eq. 7.9, else du/dt = 0 and u remains unchanged
    if ((m_slip->V_sx + V_cx_abs * m_slip->u / m_relaxation->sigma_kappa) * m_slip->u >= 0)
    {
      // solve the ODE using RK - 45 integration
      double delta_u = calc_ODE_RK_uv(m_slip->V_sx, m_relaxation->sigma_kappa, V_cx, step_size, m_slip->u);
      m_slip->u += delta_u;
    }

    // Eq. 7.7, else dv/dt = 0 and v remains unchanged
    if ((m_slip->V_sy + std::abs(V_cx) * m_slip->v_alpha / m_relaxation->sigma_alpha) * m_slip->v_alpha >= 0)
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
    V_cx, m_slip->psi_dot, m_tireState.omega, m_slip->gamma, m_relaxation->sigma_alpha,
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
  double V_cx_abs = std::abs(V_cx);
  double k1 = -V_s - (1.0 / sigma) * V_cx_abs * x_curr;
  double k2 = -V_s - (1.0 / sigma) * V_cx_abs * (x_curr + 0.5 * step_size * k1);
  double k3 = -V_s - (1.0 / sigma) * V_cx_abs * (x_curr + 0.5 * step_size * k2);
  double k4 = -V_s - (1.0 / sigma) * V_cx_abs * (x_curr + step_size * k3);

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
  double V_cx_abs = std::abs(V_cx);
  double g0 = C_Fgamma / C_Falpha * V_cx_abs * gamma;
  double g1 = V_cx_abs / sigma_alpha;
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
                                      double omega,
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

  double p0 = (C_Fphi / C_Falpha) * sign_Vcx * (psi_dot - (1.0 - eps_gamma) * omega * std::sin(gamma));
  double p1 = (1.0 / sigma_alpha) * std::abs(V_cx);

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
  double d_vlow = 1;
  double V_cx_abs = std::abs(m_slip->V_cx);
  if (V_cx_abs <= V_low)
  {
    d_vlow = 0.5 * 770. * (1.0 + std::cos(CH_C_PI * V_cx_abs / V_low));
  }

  // Besselink is RH term in kappa_p, alpha_p
  double kappa_p = m_slip->u / m_relaxation->sigma_kappa - d_vlow * m_slip->V_sx / m_relaxation->C_Fkappa;
  // double alpha_p = -std::atan(m_slip->v_alpha / m_relaxation->sigma_alpha) - d_vlow * m_slip->V_sy / m_relaxation->C_Falpha;
  double alpha_p = -m_slip->v_alpha / m_relaxation->sigma_alpha - d_vlow * m_slip->V_sy / m_relaxation->C_Falpha;
  double gamma_p = m_relaxation->C_Falpha * m_slip->v_gamma / (m_relaxation->C_Fgamma * m_relaxation->sigma_alpha);
  double phi_p = (m_relaxation->C_Falpha * m_slip->v_phi) / (m_relaxation->C_Fphi * m_relaxation->sigma_alpha);	// turn slip
  double phi_t = -m_slip->psi_dot / m_slip->V_cx;   // for turn slip

  // set transient slips
  m_slip->alphaP = alpha_p;
  m_slip->kappaP = kappa_p;
  m_slip->gammaP = gamma_p;
  m_slip->phiP = phi_p;
  m_slip->phiT = phi_t;
}


// -----------------------------------------------------------------------------
// Calculate tire reactions.
// -----------------------------------------------------------------------------
void ChPacejkaTire::calc_combinedSlipReactions()
{
  // calculate Fx for combined slip
  calcFx_combined();

  // calc Fy for combined slip
  calcFy_combined();

  // calc Mz for combined slip
  calcMz_combined();
}

void ChPacejkaTire::calc_relaxationLengths()
{
  double p_Ky4 = 2;
  double C_Fx = 161000;  // calibrated, sigma_kappa = sigma_kappa_ref = 1.29
  double C_Fy = 144000; // calibrated, sigma_alpha = sigma_alpha_ref = 0.725
  double p_Ky5 = 0;
  double p_Ky6 = 0.92;
  double p_Ky7 = 0.24;

  // all these C values should probably be positive, since negative stiffness doesn't make sense
  // parameter pky1 from A/Car parameter file is negative? not correct, force to positive
  double C_Falpha = std::abs(m_params->lateral.pky1 * m_params->vertical.fnomin * std::sin(p_Ky4 * std::atan(m_FM.force.z / (m_params->lateral.pky2 * m_params->vertical.fnomin))) * m_zeta->z3 * m_params->scaling.lyka);
  double sigma_alpha = std::abs(C_Falpha / C_Fy);
  double C_Fkappa = m_FM.force.z * (m_params->longitudinal.pkx1 + m_params->longitudinal.pkx2 * m_dF_z) * exp(m_params->longitudinal.pkx3 * m_dF_z) * m_params->scaling.lky;
  double sigma_kappa = C_Fkappa / C_Fx;
  double C_Fgamma = m_FM.force.z * (p_Ky6 + p_Ky7 * m_dF_z) * m_params->scaling.lgay;
  double C_Fphi = (C_Fgamma * m_R0) / (1 - 0.5);
  
  // NOTE: reference does not include the negative in the exponent for sigma_kappa_ref
  // double sigma_kappa_ref = m_FM.force.z * (m_params->longitudinal.ptx1 + m_params->longitudinal.ptx2 * m_dF_z)*(m_R0*m_params->scaling.lsgkp / m_params->vertical.fnomin) * exp( -m_params->longitudinal.ptx3 * m_dF_z);
  // double sigma_alpha_ref = m_params->lateral.pty1 * (1.0 - m_params->lateral.pky3 * abs( m_slip->gammaP ) ) * m_R0 * m_params->scaling.lsgal * sin(p_Ky4 * atan(m_FM.force.z / (m_params->lateral.pty2 * m_params->vertical.fnomin) ) );
  {
    relaxationL tmp = { C_Falpha, sigma_alpha, C_Fkappa, sigma_kappa, C_Fgamma, C_Fphi };
    // relaxationL tmp = { C_Falpha, sigma_alpha_ref, C_Fkappa, sigma_kappa_ref, C_Fgamma, C_Fphi };
    *m_relaxation = tmp;
  }
}

void ChPacejkaTire::calcFx_pureLong()
{
  // double eps_Vx = 0.6;
  double eps_x = 0;
  // Fx, pure long slip
  double S_Hx = (m_params->longitudinal.phx1 + m_params->longitudinal.phx2*m_dF_z)*m_params->scaling.lhx;
  double kappa_x = m_slip->kappaP + S_Hx;  // * 0.1;

  double mu_x = (m_params->longitudinal.pdx1 + m_params->longitudinal.pdx2*m_dF_z) * (1.0 - m_params->longitudinal.pdx3 * pow(m_slip->gammaP,2) ) * m_params->scaling.lmux;	// >0
  double K_x = m_FM.force.z * (m_params->longitudinal.pkx1 + m_params->longitudinal.pkx2 * m_dF_z) * exp(m_params->longitudinal.pkx3 * m_dF_z) * m_params->scaling.lkx;
  double C_x = m_params->longitudinal.pcx1 * m_params->scaling.lcx;	// >0
  double D_x = mu_x * m_FM.force.z * m_zeta->z1;  // >0
  double B_x = K_x / (C_x * D_x + eps_x);

  double sign_kap = 0;
  if (kappa_x >= 0)
    sign_kap = 1;
  else
    sign_kap = -1;
  double E_x = (m_params->longitudinal.pex1 + m_params->longitudinal.pex2 * m_dF_z + m_params->longitudinal.pex3 *  pow(m_dF_z,2) ) * (1.0 - m_params->longitudinal.pex4*sign_kap)*m_params->scaling.lex;
  double S_Vx = m_FM.force.z * (m_params->longitudinal.pvx1 + m_params->longitudinal.pvx2 * m_dF_z) * m_params->scaling.lvx * m_params->scaling.lmux * m_zeta->z1;
  double F_x = D_x * std::sin(C_x * std::atan(B_x * kappa_x - E_x * (B_x * kappa_x - std::atan(B_x * kappa_x)))) - S_Vx;

  // set the longitudinal force
  m_FM.force.x = F_x;

  // hold onto these coefs
  {
    pureLongCoefs tmp = { S_Hx, kappa_x, mu_x, K_x, B_x, C_x, D_x, E_x, F_x, S_Vx };
    *m_pureLong = tmp;
  }
}

void ChPacejkaTire::calcFy_pureLat()
{
  double p_Ky4 = 2.0;
  double p_Ky5 = 0;
  // double p_Ky6 = 0.92;   // not in the default pac2002 file
  // double p_Ky7 = 0.24;   // "
  double p_Ey5 = 0;

  // double K_y0 = m_FM.force.z * (p_Ky6 + p_Ky7 * m_dF_z) * m_params->scaling.lgay;
  double C_y = m_params->lateral.pcy1 * m_params->scaling.lcy;  // > 0
  double mu_y = (m_params->lateral.pdy1 + m_params->lateral.pdy2 * m_dF_z) * (1.0 - m_params->lateral.pdy3 * pow(m_slip->gammaP,2) ) * m_params->scaling.lmuy;	// > 0
  double D_y = mu_y * m_FM.force.z * m_zeta->z2;

  //// TODO: does this become negative??? 
  double K_y = std::abs(m_params->lateral.pky1 * m_params->vertical.fnomin * std::sin(p_Ky4 * std::atan(m_FM.force.z / ( (m_params->lateral.pky2 + p_Ky5 * pow(m_slip->gammaP,2) )* m_params->vertical.fnomin))) * (1.0 - m_params->lateral.pky3 * std::abs(m_slip->gammaP) ) * m_zeta->z3 * m_params->scaling.lyka);

  // doesn't make sense to ever have K_yAlpha be negative (it can be interpreted as lateral stiffnesss)
  double B_y = -K_y / (C_y * D_y);

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

  double F_y = D_y * std::sin(C_y * std::atan(B_y * alpha_y - E_y * (B_y * alpha_y - std::atan(B_y * alpha_y)))) + S_Vy;

  m_FM.force.y = F_y;

  // hold onto coefs
  {
    pureLatCoefs tmp = { S_Hy, alpha_y, mu_y, K_y, S_Vy, B_y, C_y, D_y, E_y };
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

  double S_Hf = m_pureLat->S_Hy + m_pureLat->S_Vy / m_pureLat->K_y;
  double alpha_r = m_slip->alphaP + S_Hf;
  double S_Ht = m_params->aligning.qhz1 + m_params->aligning.qhz2 * m_dF_z + (m_params->aligning.qhz3 + m_params->aligning.qhz4 * m_dF_z) * m_slip->gammaP;
  double alpha_t = m_slip->alphaP + S_Ht;

  double B_r = (m_params->aligning.qbz9 * (m_params->scaling.lky / m_params->scaling.lmuy) + m_params->aligning.qbz10 * m_pureLat->B_y * m_pureLat->C_y) * m_zeta->z6;
  double C_r = m_zeta->z7;
  // double D_r = m_FM.force.z * m_R0 * ( (m_params->aligning.qdz6 + m_params->aligning.qdz7 * m_dF_z) * m_params->scaling.lgyr * m_zeta->z2 + (m_params->aligning.qdz8 + m_params->aligning.qdz9 * m_dF_z) * m_slip->gammaP * m_params->scaling.lgaz * m_zeta->z0 ) * m_slip->cosPrime_alpha * m_params->scaling.lmuy * sign_Vx + m_zeta->z8 - 1.0;
  double D_r = m_FM.force.z * m_R0 * ( (m_params->aligning.qdz6 + m_params->aligning.qdz7 * m_dF_z) * m_params->scaling.lgyr + (m_params->aligning.qdz8 + m_params->aligning.qdz9 * m_dF_z) * m_slip->gammaP ) * m_params->scaling.lmuy + m_zeta->z8 - 1.0;
  
  double B_t = (m_params->aligning.qbz1 + m_params->aligning.qbz2 * m_dF_z + m_params->aligning.qbz3 * pow(m_dF_z,2) ) * (1.0 + m_params->aligning.qbz4 * m_slip->gammaP + m_params->aligning.qbz5 * std::abs(m_slip->gammaP) ) * m_params->scaling.lvyka / m_params->scaling.lmuy;
  double C_t = m_params->aligning.qcz1;
  double D_t0 = m_FM.force.z * (m_R0 / m_params->vertical.fnomin) * (m_params->aligning.qdz1 + m_params->aligning.qdz2 * m_dF_z);
  double D_t = D_t0 * (1.0 + m_params->aligning.qdz3 * m_slip->gammaP + m_params->aligning.qdz4 * pow(m_slip->gammaP,2) ) * m_zeta->z5 * m_params->scaling.ltr;
  
  double E_t = (m_params->aligning.qez1 + m_params->aligning.qez2 * m_dF_z + m_params->aligning.qez3 * pow(m_dF_z,2) ) * (1.0 + (m_params->aligning.qez4 + m_params->aligning.qez5 * m_slip->gammaP) * (2.0 / chrono::CH_C_PI) * std::atan(B_t * C_t * alpha_t) );
  double t0 = D_t * std::cos(C_t * std::atan(B_t * alpha_t - E_t * (B_t * alpha_t - std::atan(B_t * alpha_t)))) * m_slip->cosPrime_alpha;

  double MP_z0 = -t0 * m_FM.force.y;
  double M_zr0 = D_r * std::cos(C_r * std::atan(B_r * alpha_r));

  double M_z = MP_z0 + M_zr0;
  m_FM.moment.z = M_z;

  // hold onto coefs
  {
    pureTorqueCoefs tmp = {
      S_Hf, alpha_r, S_Ht, alpha_t, m_slip->cosPrime_alpha, m_pureLat->K_y,
      B_r, C_r, D_r,
      B_t, C_t, D_t0, D_t, E_t, t0,
      MP_z0, M_zr0 };
    *m_pureTorque = tmp;
  }
}

void ChPacejkaTire::calcFx_combined()
{
  double rbx3 = 1.0;

  double S_HxAlpha = m_params->longitudinal.rhx1;
  double alpha_S = m_slip->alphaP + S_HxAlpha;
  double B_xAlpha = (m_params->longitudinal.rbx1 + rbx3 * pow(m_slip->gammaP, 2)) * std::cos(std::atan(m_params->longitudinal.rbx2 * m_slip->kappaP)) * m_params->scaling.lxal;
  double C_xAlpha = m_params->longitudinal.rcx1;
  double E_xAlpha = m_params->longitudinal.rex1 + m_params->longitudinal.rex2 * m_dF_z;

  // double G_xAlpha0 = std::cos(C_xAlpha * std::atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha - std::atan(B_xAlpha * S_HxAlpha)) ) );
  double G_xAlpha0 = std::cos(C_xAlpha * std::atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha - std::atan(B_xAlpha * S_HxAlpha))));

  // double G_xAlpha = std::cos(C_xAlpha * std::atan(B_xAlpha * alpha_S - E_xAlpha * (B_xAlpha * alpha_S - std::atan(B_xAlpha * alpha_S)) ) ) / G_xAlpha0;
  double G_xAlpha = std::cos(C_xAlpha * std::atan(B_xAlpha * alpha_S - E_xAlpha * (B_xAlpha * alpha_S - std::atan(B_xAlpha * alpha_S)))) / G_xAlpha0;

  double F_x = G_xAlpha * m_FM.force.x;
  m_FM_combined.force.x = F_x;

  {
    combinedLongCoefs tmp = { S_HxAlpha, alpha_S, B_xAlpha, C_xAlpha, E_xAlpha, G_xAlpha0, G_xAlpha };
    *m_combinedLong = tmp;
  }
}

void ChPacejkaTire::calcFy_combined()
{
  double rby4 = 0;

  double S_HyKappa = m_params->lateral.rhy1 + m_params->lateral.rhy2 * m_dF_z;
  double kappa_S = m_slip->kappaP + S_HyKappa;
  double B_yKappa = (m_params->lateral.rby1 + rby4 * pow(m_slip->gammaP,2) ) * std::cos( std::atan(m_params->lateral.rby2 * (m_slip->alphaP - m_params->lateral.rby3) ) )*m_params->scaling.lyka;
  double C_yKappa = m_params->lateral.rcy1;
  double E_yKappa = m_params->lateral.rey1 + m_params->lateral.rey2 * m_dF_z;
  double D_VyKappa = m_pureLat->mu_y * m_FM.force.z * (m_params->lateral.rvy1 + m_params->lateral.rvy2 * m_dF_z + m_params->lateral.rvy3 * m_slip->gammaP) * std::cos(std::atan(m_params->lateral.rvy4 * m_slip->alphaP)) * m_zeta->z2;
  double S_VyKappa = D_VyKappa * std::sin(m_params->lateral.rvy5 * std::atan(m_params->lateral.rvy6 * m_slip->kappaP)) * m_params->scaling.lvyka;
  double G_yKappa0 = std::cos(C_yKappa * std::atan(B_yKappa * S_HyKappa - E_yKappa * (B_yKappa * S_HyKappa - std::atan(B_yKappa * S_HyKappa))));
  double G_yKappa = std::cos(C_yKappa * std::atan(B_yKappa * kappa_S - E_yKappa * (B_yKappa * kappa_S - std::atan(B_yKappa * kappa_S)))) / G_yKappa0;

  double F_y = G_yKappa * m_FM.force.y + S_VyKappa;
  m_FM_combined.force.y = F_y;

  {
    combinedLatCoefs tmp = { S_HyKappa, kappa_S, B_yKappa, C_yKappa, E_yKappa, D_VyKappa, S_VyKappa, G_yKappa0, G_yKappa };
    *m_combinedLat = tmp;
  }
}

void ChPacejkaTire::calcMz_combined()
{
  double FP_y = m_FM_combined.force.y - m_combinedLat->S_VyKappa;
  double s = m_R0 * (m_params->aligning.ssz1 + m_params->aligning.ssz2 * (m_FM_combined.force.y / m_params->vertical.fnomin) + (m_params->aligning.ssz3 + m_params->aligning.ssz4 * m_dF_z) * m_slip->gammaP) * m_params->scaling.ls;
  int sign_alpha_t = 0;
  int sign_alpha_r = 0;
  if (m_pureTorque->alpha_t >= 0)
    sign_alpha_t = 1;
  else
    sign_alpha_t = -1;
  if (m_pureTorque->alpha_r >= 0)
    sign_alpha_r = 1;
  else 
    sign_alpha_r = -1;
 

  double alpha_t_eq = sign_alpha_t * sqrt( pow(m_pureTorque->alpha_t, 2) + pow(m_pureLong->K_x / m_pureTorque->K_y, 2)
    * pow(m_slip->kappaP, 2));
  double alpha_r_eq = sign_alpha_r * sqrt( pow(m_pureTorque->alpha_r, 2) + pow(m_pureLong->K_x / m_pureTorque->K_y, 2)
    * pow(m_slip->kappaP, 2) );

  // TODO: cos(alpha) is in Adams/Car, not in Pacejka. why???
  double M_zr = m_pureTorque->D_r * std::cos(m_pureTorque->C_r * std::atan(m_pureTorque->B_r * alpha_r_eq)) * m_slip->cosPrime_alpha;
  double t = m_pureTorque->D_t * std::cos(m_pureTorque->C_t * std::atan(m_pureTorque->B_t*alpha_t_eq - m_pureTorque->E_t * (m_pureTorque->B_t * alpha_t_eq - std::atan(m_pureTorque->B_t * alpha_t_eq)))) * m_slip->cosPrime_alpha;

  double M_z = (-t * FP_y) + M_zr  + (s * m_FM_combined.force.x);
  double M_z_y = -t * FP_y;
  double M_z_x = s * m_FM_combined.force.x;
  m_FM_combined.moment.z = M_z;

  {
    combinedTorqueCoefs tmp = { m_slip->cosPrime_alpha, FP_y, s, alpha_t_eq, alpha_r_eq, M_zr, t, M_z_x, M_z_y };
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
  double V_r = m_tireState.omega * m_R_eff;
  double M_y = -m_FM.force.z * m_R0 * (m_params->rolling.qsy1 * std::atan(V_r / m_params->model.longvl) - m_params->rolling.qsy2 * (m_FM_combined.force.x / m_params->vertical.fnomin)) * m_params->scaling.lmy;
  m_FM.moment.y = M_y;
  m_FM_combined.moment.y = M_y;
}

// -----------------------------------------------------------------------------
// Load a PacTire specification file.
//
// For an example, see the file models/data/hmmwv/pactest.tir
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Write output file for post-processing with the Python pandas module.
// -----------------------------------------------------------------------------
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
      oFile << "time,kappa,alpha,gamma,kappaP,alphaP,gammaP,Vx,Vy,Fx,Fy,Fz,Mx,My,Mz,Fxc,Fyc,Mzc,Mzx,Mzy" << std::endl;
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
      << m_slip->kappaP << "," << std::atan(m_slip->alphaP)*180. / 3.14159 << "," << m_slip->gammaP << ","
      << m_tireState.lin_vel.x << "," << m_tireState.lin_vel.y << ","
      << m_FM.force.x << "," << m_FM.force.y << "," << m_FM.force.z << ","
      << m_FM.moment.x << "," << m_FM.moment.y << "," << m_FM.moment.z << ","
      << m_FM_combined.force.x << "," << m_FM_combined.force.y << "," << m_FM_combined.moment.z <<","
      << m_combinedTorque->M_z_x <<","<< m_combinedTorque->M_z_y
      << std::endl;
    // close the file
    appFile.close();
  }
}


// -----------------------------------------------------------------------------
// Utility function for validation studies.
//
// Calculate a wheel state consistent with the specified kinematic slip
// quantities (kappa, alpha, and gamma) and magnitude of tangential velocity.
// Out of the infinitely many consistent wheel states, we
// - set position at origin
// - set linear velocity along global X direction with given magnitude
// - set orientation based on given alpha and gamma
// - set omega from given kappa and using current R_eff
// - set angular velocity along the local wheel Y axis
// -----------------------------------------------------------------------------
ChWheelState ChPacejkaTire::getState_from_KAG(double kappa,
                                              double alpha,
                                              double gamma,
                                              double Vx)
{
  ChWheelState state;

  // Set wheel position at origin.
  state.pos = ChVector<>(0, 0, 0);

  // Set the wheel velocity aligned with the global X axis.
  state.lin_vel = ChVector<>(Vx, 0, 0);

  // Rotate wheel to satisfy specified camber angle and slip angle.  For this,
  // we first rotate the wheel about the global Z-axis by an angle (-alpha),
  // followed by a rotation about the new X-axis by an angle gamma.
  state.rot = Q_from_AngZ(-alpha) * Q_from_AngX(gamma);

  // Calculate forward tangential velocity.
  double Vcx = Vx * std::cos(alpha);

  // Set the wheel angular velocity about its axis, calculated from the given
  // value kappa and using the current value m_R_eff.
  // Note that here we assume that the specified velocity is such that Vcx is
  // larger than the threshold model.vxlow.
  state.omega = (kappa * std::abs(Vcx) + Vcx) / m_R_eff;

  // Set the wheel angular velocity (expressed in global frame).
  state.ang_vel = state.rot.RotateBack(ChVector<>(0, state.omega, 0));

  return state;
}


}  // end namespace chrono
