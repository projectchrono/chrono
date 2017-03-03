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
#include <cstdlib>
#include <algorithm>

#include "chrono/physics/ChGlobal.h"
#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
static const double default_step_size = 0.01;
// Threshold value for small forward tangential velocity.
// static const double v_x_threshold = 0.2;

// large Fz leads to large m_dF_z, leads to large Fx, Fy, etc.
static double Fz_thresh = 30000;
// max reaction threshold
static double Fx_thresh = 20000.0;
static double Fy_thresh = 20000.0;
static double Mx_thresh = Fz_thresh / 20.0;
static double My_thresh = Fx_thresh / 20.0;
static double Mz_thresh = Fz_thresh / 20.0;

static double kappaP_thresh = 0.01;
static double alphaP_thresh = 0.1;
static double gammaP_thresh = 0.05;
static double phiP_thresh = 99;
static double phiT_thresh = 99;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
ChPacejkaTire::ChPacejkaTire(const std::string& name, const std::string& pacTire_paramFile)
    : ChTire(name),
      m_paramFile(pacTire_paramFile),
      m_params_defined(false),
      m_use_transient_slip(true),
      m_use_Fz_override(false),
      m_driven(false),
      m_step_size(default_step_size) {
}

ChPacejkaTire::ChPacejkaTire(const std::string& name,
                             const std::string& pacTire_paramFile,
                             double Fz_override,
                             bool use_transient_slip)
    : ChTire(name),
      m_paramFile(pacTire_paramFile),
      m_params_defined(false),
      m_use_transient_slip(use_transient_slip),
      m_use_Fz_override(Fz_override > 0),
      m_Fz_override(Fz_override),
      m_driven(false),
      m_step_size(default_step_size) {
}

// -----------------------------------------------------------------------------
// Destructor
//
// Delete private structures
// -----------------------------------------------------------------------------
ChPacejkaTire::~ChPacejkaTire() {
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
    delete m_bessel;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// NOTE: no initial conditions passed in at this point, e.g. m_tireState is empty
void ChPacejkaTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    // Create private structures
    m_slip = new slips;
    m_params = new Pac2002_data;

    m_pureLong = new pureLongCoefs;
    m_pureLat = new pureLatCoefs;
    m_pureTorque = new pureTorqueCoefs;

    m_combinedLong = new combinedLongCoefs;
    m_combinedLat = new combinedLatCoefs;
    m_combinedTorque = new combinedTorqueCoefs;
    // m_combinedTorque->M_zr = 0;
    // m_combinedTorque->M_z_x = 0;
    // m_combinedTorque->M_z_y = 0;
    m_zeta = new zetaCoefs;
    m_relaxation = new relaxationL;
    m_bessel = new bessel;

    // negative number indicates no steps have been taken yet
    m_time_since_last_step = 0;
    m_initial_step = false;  // have not taken a step at time = 0 yet
    m_num_ODE_calls = 0;
    m_sum_ODE_time = 0.0;
    m_num_Advance_calls = 0;
    m_sum_Advance_time = 0.0;

    // load all the empirical tire parameters from *.tir file
    loadPacTireParamFile();

    //// TODO:  Need to figure out a better way of indicating errors
    ////        Right now, we cannot have this function throw an exception since
    ////        it's called from the constructor!   Must fix this...
    if (!m_params_defined) {
        GetLog() << " couldn't load pacTire parameters from file, not updating initial quantities \n\n";
        return;
    }

    // LEFT or RIGHT side of the vehicle?
    if (m_side == LEFT) {
        m_sameSide = (!m_params->model.tyreside.compare("LEFT")) ? 1 : -1;
    } else {
        // on right
        m_sameSide = (!m_params->model.tyreside.compare("RIGHT")) ? 1 : -1;
    }

    // any variables that are calculated once
    m_R0 = m_params->dimension.unloaded_radius;

    //// TODO:  why do we have to initialize m_R_l and m_R_eff here?
    ////        This is done in Synchronize(), when we have a proper wheel state.

    m_R_l = m_R0 - 8000.0 / m_params->vertical.vertical_stiffness;
    double qV1 = 1.5;
    double rho = (m_R0 - m_R_l) * exp(-qV1 * m_R0 * pow(1.05 * m_params->model.longvl / m_params->model.longvl, 2));
    m_R_eff = m_R0 - rho;

    m_Fz = 0;
    m_dF_z = 0;

    // spin slip coefficients,  unused for now
    {
        zetaCoefs tmp = {1, 1, 1, 1, 1, 1, 1, 1, 1};
        *m_zeta = tmp;
    }

    m_combinedTorque->alpha_r_eq = 0.0;
    m_pureLat->D_y = m_params->vertical.fnomin;  // initial approximation
    m_C_Fx = 161000;                             // calibrated, sigma_kappa = sigma_kappa_ref = 1.29
    m_C_Fy = 144000;                             // calibrated, sigma_alpha = sigma_alpha_ref = 0.725

    // init all other variables
    m_Num_WriteOutData = 0;
    zero_slips();  // zeros slips, and some other vars
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPacejkaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = std::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = GetRadius();
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -GetVisualizationWidth() / 2, 0);
    m_wheel->AddAsset(m_cyl_shape);

    m_texture = std::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(m_texture);
}

void ChPacejkaTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChPacejkaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_cyl_shape);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_texture);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
}

// -----------------------------------------------------------------------------
// Return computed tire forces and moment (pure slip or combined slip and in
// local or global frame). The main GetTireForce() function returns the combined
// slip tire forces, expressed in the global frame.
// -----------------------------------------------------------------------------
TireForce ChPacejkaTire::GetTireForce(bool cosim) const {
    return GetTireForce_combinedSlip(false);
}

TireForce ChPacejkaTire::GetTireForce_pureSlip(const bool local) const {
    if (local)
        return m_FM_pure;

    // reactions are on wheel CM
    TireForce m_FM_global;
    m_FM_global.point = m_tireState.pos;
    // only transform the directions of the forces, moments, from local to global
    m_FM_global.force = m_W_frame.TransformDirectionLocalToParent(m_FM_pure.force);
    m_FM_global.moment = m_W_frame.TransformDirectionLocalToParent(m_FM_pure.moment);

    return m_FM_global;
}

/// Return the reactions for the combined slip EQs, in local or global coords
TireForce ChPacejkaTire::GetTireForce_combinedSlip(const bool local) const {
    if (local)
        return m_FM_combined;

    // reactions are on wheel CM
    TireForce m_FM_global;
    m_FM_global.point = m_W_frame.pos;
    // only transform the directions of the forces, moments, from local to global
    m_FM_global.force = m_W_frame.TransformDirectionLocalToParent(m_FM_combined.force);
    m_FM_global.moment = m_W_frame.TransformDirectionLocalToParent(m_FM_combined.moment);

    return m_FM_global;
}

// -----------------------------------------------------------------------------
// Update the internal state of this tire using the specified wheel state. The
// quantities calculated here will be kept constant until the next call to the
// Synchronize() function.
// -----------------------------------------------------------------------------
void ChPacejkaTire::Synchronize(double time, const WheelState& state, const ChTerrain& terrain) {
    //// TODO: This must be removed from here.  A tire with unspecified or
    ////       incorrect parameters should have been invalidate at initialization.
    // Check that input tire model parameters are defined
    if (!m_params_defined) {
        GetLog() << " ERROR: cannot update tire w/o setting the model parameters first! \n\n\n";
        return;
    }

    // Invoke the base class function.
    ChTire::Synchronize(time, state, terrain);

    // Cache the wheel state and update the tire coordinate system.
    m_tireState = state;
    m_simTime = time;
    update_W_frame(terrain);

    // If not using the transient slip model, check that the tangential forward
    // velocity is not too small.
    ChVector<> V = m_W_frame.TransformDirectionParentToLocal(m_tireState.lin_vel);
    if (!m_use_transient_slip && std::abs(V.x()) < 0.1) {
        GetLog() << " ERROR: tangential forward velocity below threshold.... \n\n";
        return;
    }

    // keep the last calculated reaction force or moment, to use later
    m_FM_pure_last = m_FM_pure;
    m_FM_combined_last = m_FM_combined;

    // Initialize the output tire forces to ensure that we do not report any tire
    // forces if the wheel does not contact the terrain.
    m_FM_pure.point = ChVector<>();
    m_FM_pure.force = ChVector<>();
    m_FM_pure.moment = ChVector<>();

    m_FM_combined.point = ChVector<>();
    m_FM_combined.force = ChVector<>();
    m_FM_combined.moment = ChVector<>();
}

// -----------------------------------------------------------------------------
// Advance the state of this tire by the specified time step. If using the
// transient slip model, perform integration taking as many integration steps
// as needed.
// -----------------------------------------------------------------------------
void ChPacejkaTire::Advance(double step) {
    // increment the counter
    ChTimer<double> advance_time;
    m_num_Advance_calls++;

    // Do nothing if the wheel does not contact the terrain.  In this case, all
    // reported tire forces will be zero. Still have to update slip quantities, since
    // displacements won't go to zero imeediately
    /*
    if (!m_in_contact)
    {
      zero_slips();
      return;
    }
    */

    // If using single point contact model, slips are calculated from compliance
    // between tire and contact patch.
    if (m_use_transient_slip) {
        // 1 of 2 ways to deal with user input time step increment

        // 1) ...
        // a) step <= m_step_size, so integrate using input step
        // b) step > m_step_size, use m_step_size until step <= m_step_size
        double remaining_time = step;
        // only count the time take to do actual calculations in Adanvce time
        advance_time.start();
        // keep track of the ODE calculation time
        ChTimer<double> ODE_timer;
        ODE_timer.start();
        while (remaining_time > m_step_size) {
            advance_tire(m_step_size);
            remaining_time -= m_step_size;
        }
        // take one final step to reach the specified time.
        advance_tire(remaining_time);

        // stop the timers
        ODE_timer.stop();
        m_num_ODE_calls++;
        m_sum_ODE_time += ODE_timer();

        /*

        // ... or, 2)
        // assume input step is smaller than it has to be. Then, take x steps
        // until actually re-calculating reactions.
        m_time_since_last_step =+ step;
        // enough time has accumulated to do a macro step, OR, it's the first step
        if( m_time_since_last_step >= m_step_size || !m_initial_step)
        {
          // only count the time take to do actual calculations in Adanvce time
          advance_time.start();
          // keep track of the ODE calculation time
          ChTimer<double> ODE_timer;
          ODE_timer.start();
          advance_slip_transient(m_time_since_last_step);
          ODE_timer.stop();
          // increment the timer, counter
          m_num_ODE_calls++;
          m_sum_ODE_time += ODE_timer();

          m_time_since_last_step = 0;
          m_initial_step = true;  // 2nd term in if() above will always be false
        } else {
          return;
        }
        */

    } else {
        // Calculate the vertical load and update tire deflection and rolling radius
        update_verticalLoad(step);

        // Calculate kinematic slip quantities
        slip_kinematic();
    }

    // Calculate the force and moment reaction, pure slip case
    pureSlipReactions();

    // Update m_FM_combined.forces, m_FM_combined.moment.z
    combinedSlipReactions();

    // Update M_x, apply to both m_FM and m_FM_combined
    // gamma should already be corrected for L/R side, so need to swap Fy if on opposite side
    double Mx = m_sameSide * calc_Mx(m_sameSide * m_FM_combined.force.y(), m_slip->gammaP);
    m_FM_pure.moment.x() = Mx;
    m_FM_combined.moment.x() = Mx;

    // Update M_y, apply to both m_FM and m_FM_combined
    // be sure signs ARE THE SAME, else it becomes self-energizing on one side
    double My = calc_My(m_FM_combined.force.x());
    m_FM_pure.moment.y() = My;
    m_FM_combined.moment.y() = My;

    // all the reactions have been calculated, stop the advance timer
    advance_time.stop();
    m_sum_Advance_time += advance_time();

    // DEBUGGING
    // m_FM_combined.moment.y() = 0;
    // m_FM_combined.moment.z() = 0;

    // evaluate the reaction forces calculated
    evaluate_reactions(false, false);
}

void ChPacejkaTire::advance_tire(double step) {
    // Calculate the vertical load and update tire deflection and tire rolling
    // radius.
    update_verticalLoad(step);

    // Calculate kinematic slip quantities, based on specified wheel state. This
    // assumes that the inputs to wheel spindle instantly affect tire contact.
    // Note: kappaP, alphaP, gammaP may be overridden in Advance
    slip_kinematic();

    advance_slip_transient(step);
}

// Calculate the tire contact coordinate system.
// TYDEX W-axis system is at the contact point "C", Z-axis normal to the terrain
//  and X-axis along the wheel centerline.
// Local frame transforms output forces/moments to global frame, to be applied
//  to the wheel body CM.
// Pacejka (2006), Fig 2.3, all forces are calculated at the contact point "C"
void ChPacejkaTire::update_W_frame(const ChTerrain& terrain) {
    // Check contact with terrain, using a disc of radius R0.
    ChCoordsys<> contact_frame;
    double depth;
    m_in_contact =
        disc_terrain_contact(terrain, m_tireState.pos, m_tireState.rot.GetYaxis(), m_R0, contact_frame, depth);

    // set the depth if there is contact with terrain
    m_depth = (m_in_contact) ? depth : 0;

    // Wheel normal (expressed in global frame)
    ChVector<> wheel_normal = m_tireState.rot.GetYaxis();

    // Terrain normal at wheel center location (expressed in global frame)
    ChVector<> Z_dir = terrain.GetNormal(m_tireState.pos.x(), m_tireState.pos.y());

    // Longitudinal (heading) and lateral directions, in the terrain plane.
    ChVector<> X_dir = Vcross(wheel_normal, Z_dir);
    X_dir.Normalize();
    ChVector<> Y_dir = Vcross(Z_dir, X_dir);

    // Create a rotation matrix from the three unit vectors
    ChMatrix33<> rot;
    rot.Set_A_axis(X_dir, Y_dir, Z_dir);

    // W-Axis system position at the contact point
    m_W_frame.pos = contact_frame.pos;
    m_W_frame.rot = rot.Get_A_quaternion();
}

// -----------------------------------------------------------------------------
// Calculate the tire vertical load at the current configuration and update the
// tire deflection and tire rolling radius.
// -----------------------------------------------------------------------------
void ChPacejkaTire::update_verticalLoad(double step) {
    double Fz;

    if (m_use_Fz_override) {
        // Vertical load specified as input.
        Fz = m_Fz_override;
        m_Fz = m_Fz_override;

        // Estimate the tire deformation and set the statically loaded radius
        // (assuming static loading).
        m_R_l = m_R0 - Fz / m_params->vertical.vertical_stiffness;

        // In this case, the tire is always assumed to be in contact with the
        // terrain.
        m_in_contact = true;
    } else {
        // Calculate the tire vertical load using a spring-damper model. Note that
        // this also sets the statically loaded radius m_R_l, as well as the boolean
        // flag m_in_contact.
        Fz = calc_Fz();
        // assert( Fz >= m_params->vertical_force_range.fzmin);

        double capped_Fz = 0;
        // use the vertical load in the reaction to the wheel, but not in the Magic Formula Eqs
        if (Fz > Fz_thresh) {
            // large Fz leads to large m_dF_z, leads to large Fx, Fy, etc.
            capped_Fz = Fz_thresh;
        } else {
            capped_Fz = Fz;
        }

        // damp the change in Fz
        m_Fz = capped_Fz;  //  - delta_Fz_damped;
                           // if( Fz_damp * delta_Fz_damped > capped_Fz)
                           // {
                           //  m_Fz = m_params->vertical_force_range.fzmin;
        //  GetLog() << " \n **** \n the damper on m_Fz should not cause it to go negative !!!! \n\n\n";
        // }
    }

    assert(m_Fz > 0);
    // ratio of actual load to nominal
    m_dF_z = (m_Fz - m_params->vertical.fnomin) / m_params->vertical.fnomin;

    // Calculate tire vertical deflection, rho.
    double qV1 = 0.000071;  // from example
    double K1 = pow(m_tireState.omega * m_R0 / m_params->model.longvl, 2);
    double rho = m_R0 - m_R_l + qV1 * m_R0 * K1;
    double rho_Fz0 = m_params->vertical.fnomin / (m_params->vertical.vertical_stiffness);
    double rho_d = rho / rho_Fz0;

    // Calculate tire rolling radius, R_eff.  Clamp this value to R0.
    m_R_eff = m_R0 + qV1 * m_R0 * K1 -
              rho_Fz0 * (m_params->vertical.dreff * std::atan(m_params->vertical.breff * rho_d) +
                         m_params->vertical.freff * rho_d);
    if (m_R_eff > m_R0)
        m_R_eff = m_R0;

    if (m_in_contact) {
        // Load vertical component of tire force
        m_FM_pure.force.z() = Fz;
        m_FM_combined.force.z() = Fz;
    }
}

// NOTE: must
double ChPacejkaTire::calc_Fz() {
    // Initialize the statically loaded radius to R0.  This is the returned value
    // if no vertical force is generated.
    m_R_l = m_R0;

    if (!m_in_contact)
        return m_params->vertical_force_range.fzmin;

    // Calculate relative velocity (wheel - terrain) at contact point, in the
    // global frame and then express it in the contact frame.
    ChVector<> relvel_abs = m_tireState.lin_vel + Vcross(m_tireState.ang_vel, m_W_frame.pos - m_tireState.pos);
    ChVector<> relvel_loc = m_W_frame.TransformDirectionParentToLocal(relvel_abs);

    // Calculate normal contact force, using a spring-damper model.
    // Note: depth is always positive, so the damping should always subtract
    double Fz = m_params->vertical.vertical_stiffness * m_depth - m_params->vertical.vertical_damping * relvel_loc.z();

    // Adams reference, Fz = Fz(omega, Fx, Fy, gamma, m_depth, relvel_loc.z)
    double q_v2 = 2.0;                                     // linear stiffness increase with spin
    double q_Fcx = 0.2;                                    // Fx stiffness reduction
    double q_Fcy = 0.35;                                   // Fy stiffness reduction
    double q_FcG = 0.001;                                  // camber stiffness increase
    double C_Fz = m_params->vertical.vertical_damping;     // damping
    double q_Fz1 = m_params->vertical.vertical_stiffness;  // linear stiffness
    double q_Fz2 = 500.0;                                  // 2nd order stiffness
    // scale the stiffness by considering forces and spin rate
    double force_term = 1.0 + q_v2 * std::abs(m_tireState.omega) * m_R0 / m_params->model.longvl -
                        pow(q_Fcx * m_FM_combined_last.force.x() / m_params->vertical.fnomin, 2) -
                        pow(q_Fcy * m_FM_combined_last.force.y() / m_params->vertical.fnomin, 2) +
                        q_FcG * pow(m_slip->gammaP, 2);
    double rho_term = q_Fz1 * m_depth + q_Fz2 * pow(m_depth, 2);
    //  Fz = force_term*rho_term*Fz0 + C_Fz * v_z
    double Fz_adams = force_term * rho_term - C_Fz * relvel_loc.z();

    // for a 37x12.5 R16.5 Wrangler MT @ 30 psi
    // double k1 = 550000.0;
    // double k2 = 1e5;
    // vertical_damping can be some small percentage of vertical_stiffness (e.g., 2%)
    // double c = 0.001 * k1;
    // double Fz = k1 * m_depth - c * relvel_loc.z(); // + k2 * pow(m_depth,2);

    // if (Fz < m_params->vertical_force_range.fzmin)
    if (Fz_adams < m_params->vertical_force_range.fzmin)
        return m_params->vertical_force_range.fzmin;

    m_R_l = m_R0 - m_depth;

    return Fz;
    // return Fz_adams;
}

// -----------------------------------------------------------------------------
// Calculate kinematic slip quantities from the current wheel state.
// Note: when not in contact, all slips are assumed zero, but velocities still set
// The wheel state strcture contains the following member variables:
//   ChVector<>     pos;        global position
//   ChQuaternion<> rot;        orientation with respect to global frame
//   ChVector<>     lin_vel;    linear velocity, expressed in the global frame
//   ChVector<>     ang_vel;    angular velocity, expressed in the global frame
//   double         omega;      wheel angular speed about its rotation axis
// -----------------------------------------------------------------------------
void ChPacejkaTire::slip_kinematic() {
    // Express the wheel velocity in the tire coordinate system and calculate the
    // slip angle alpha. (Override V.x if too small)
    ChVector<> V = m_W_frame.TransformDirectionParentToLocal(m_tireState.lin_vel);
    // regardless of contact
    m_slip->V_cx = V.x();  // tire center x-vel, tire c-sys
    m_slip->V_cy = V.y();  // tire center y-vel, tire c-sys

    if (m_in_contact) {
        m_slip->V_sx = V.x() - m_tireState.omega * m_R_eff;  // x-slip vel, tire c-sys
        m_slip->V_sy = V.y();                                // approx.

        // ensure V_x is not too small, else scale V_x to the threshold
        if (std::abs(V.x()) < m_params->model.vxlow) {
            V.x() = (V.x() < 0) ? -m_params->model.vxlow : m_params->model.vxlow;
        }

        double V_x_abs = std::abs(V.x());
        // lateral slip angle, in the range: (-pi/2 ,pi/2)
        double alpha = std::atan(V.y() / V_x_abs);

        // Express the wheel normal in the tire coordinate system and calculate the
        // wheel camber angle gamma.
        ChVector<> n = m_W_frame.TransformDirectionParentToLocal(m_tireState.rot.GetYaxis());
        double gamma = std::atan2(n.z(), n.y());

        double kappa = -m_slip->V_sx / V_x_abs;

        // alpha_star = tan(alpha) = v_y / v_x
        double alpha_star = m_slip->V_sy / V_x_abs;

        // Set the struct data members, input slips to wheel
        m_slip->kappa = kappa;
        m_slip->alpha = alpha;
        m_slip->alpha_star = alpha_star;
        m_slip->gamma = gamma;

        // Express the wheel angular velocity in the tire coordinate system and
        // extract the turn slip velocity, psi_dot
        ChVector<> w = m_W_frame.TransformDirectionParentToLocal(m_tireState.ang_vel);
        m_slip->psi_dot = w.z();

        // For aligning torque, to handle large slips, and backwards operation
        double V_mag = std::sqrt(V.x() * V.x() + V.y() * V.y());
        m_slip->cosPrime_alpha = V.x() / V_mag;

        // Finally, if non-transient, use wheel slips as input to Magic Formula.
        // These get over-written if enable transient slips to be calculated
        m_slip->kappaP = kappa;
        m_slip->alphaP = alpha_star;
        m_slip->gammaP = std::sin(gamma);
    } else {
        // not in contact, set input slips to 0
        m_slip->V_sx = 0;
        m_slip->V_sy = 0;

        m_slip->kappa = 0;
        m_slip->alpha = 0;
        m_slip->alpha_star = 0;
        m_slip->gamma = 0;
        // further, set the slips used in the MF eqs. to zero
        m_slip->kappaP = 0;
        m_slip->alphaP = 0;
        m_slip->gammaP = 0;
        // slip velocities should likely be zero also
        m_slip->cosPrime_alpha = 1;

        // Express the wheel angular velocity in the tire coordinate system and
        // extract the turn slip velocity, psi_dot
        ChVector<> w = m_W_frame.TransformDirectionParentToLocal(m_tireState.ang_vel);
        m_slip->psi_dot = w.z();
    }
}

void ChPacejkaTire::zero_slips() {
    // reset relevant slip variables here
    slips zero_slip = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    *m_slip = zero_slip;
}

// -----------------------------------------------------------------------------
// Calculate transient slip properties, using first order ODEs to find slip
// displacements from velocities.
// -----------------------------------------------------------------------------
void ChPacejkaTire::advance_slip_transient(double step_size) {
    // hard-coded, for now
    double EPS_GAMMA = 0.6;
    // update relaxation lengths
    relaxationLengths();

    // local c-sys velocities
    double V_cx = m_slip->V_cx;
    double V_cx_abs = std::abs(V_cx);
    double V_cx_low = 2.5;  // cut-off for low velocity zone
    double V_sx = m_slip->V_sx;
    double V_sy = m_slip->V_sy * m_sameSide;    // due to asymmetry about centerline
    double gamma = m_slip->gamma * m_sameSide;  // due to asymmetry
    // see if low velocity considerations should be made
    double alpha_sl = std::abs(3.0 * m_pureLat->D_y / m_relaxation->C_Falpha);
    // Eq. 7.25 from Pacejka (2006), solve du_dt and dvalpha_dt
    if ((std::abs(m_combinedTorque->alpha_r_eq) > alpha_sl) && (V_cx_abs < V_cx_low)) {
        // Eq. 7.9, else du/dt = 0 and u remains unchanged
        if ((V_sx + V_cx_abs * m_slip->u / m_relaxation->sigma_kappa) * m_slip->u >= 0) {
            // solve the ODE using RK - 45 integration
            m_slip->Idu_dt = ODE_RK_uv(V_sx, m_relaxation->sigma_kappa, V_cx, step_size, m_slip->u);
            m_slip->u += m_slip->Idu_dt;
        } else {
            m_slip->Idu_dt = 0;
        }

        // Eq. 7.7, else dv/dt = 0 and v remains unchanged
        if ((V_sy + std::abs(V_cx) * m_slip->v_alpha / m_relaxation->sigma_alpha) * m_slip->v_alpha >= 0) {
            m_slip->Idv_alpha_dt = ODE_RK_uv(V_sy, m_relaxation->sigma_alpha, V_cx, step_size, m_slip->v_alpha);
            m_slip->v_alpha += m_slip->Idv_alpha_dt;
        } else {
            m_slip->Idv_alpha_dt = 0;
        }
    } else {
        // don't check for du/dt =0 or dv/dt = 0

        // Eq 7.9
        m_slip->Idu_dt = ODE_RK_uv(V_sx, m_relaxation->sigma_kappa, V_cx, step_size, m_slip->u);
        m_slip->u += m_slip->Idu_dt;

        // Eq. 7.7
        m_slip->Idv_alpha_dt = ODE_RK_uv(V_sy, m_relaxation->sigma_alpha, V_cx, step_size, m_slip->v_alpha);
        m_slip->v_alpha += m_slip->Idv_alpha_dt;
    }

    // Eq. 7.11, lateral force from wheel camber
    m_slip->Idv_gamma_dt = ODE_RK_gamma(m_relaxation->C_Fgamma, m_relaxation->C_Falpha, m_relaxation->sigma_alpha, V_cx,
                                        step_size, gamma, m_slip->v_gamma);
    m_slip->v_gamma += m_slip->Idv_gamma_dt;

    // Eq. 7.12, total spin, phi, including slip and camber
    m_slip->Idv_phi_dt =
        ODE_RK_phi(m_relaxation->C_Fphi, m_relaxation->C_Falpha, V_cx, m_slip->psi_dot, m_tireState.omega, gamma,
                   m_relaxation->sigma_alpha, m_slip->v_phi, EPS_GAMMA, step_size);
    m_slip->v_phi += m_slip->Idv_phi_dt;

    // calculate slips from contact point deflections u and v
    slip_from_uv(m_in_contact, 600.0, 100.0, 2.0);
}

// don't have to call this each advance_tire(), but once per macro-step (at least)
void ChPacejkaTire::evaluate_slips() {
    if (std::abs(m_slip->kappaP) > kappaP_thresh) {
        GetLog() << "\n ~~~~~~~~~  kappaP exceeded threshold:, tire " << m_name << ", = " << m_slip->kappaP << "\n";
    }
    if (std::abs(m_slip->alphaP) > alphaP_thresh) {
        GetLog() << "\n ~~~~~~~~~  alphaP exceeded threshold:, tire " << m_name << ", = " << m_slip->alphaP << "\n";
    }
    if (std::abs(m_slip->gammaP) > gammaP_thresh) {
        GetLog() << "\n ~~~~~~~~~  gammaP exceeded threshold:, tire " << m_name << ", = " << m_slip->gammaP << "\n";
    }
    if (std::abs(m_slip->phiP) > phiP_thresh) {
        GetLog() << "\n ~~~~~~~~~  phiP exceeded threshold:, tire " << m_name << ", = " << m_slip->phiP << "\n";
    }
    if (std::abs(m_slip->phiT) > phiT_thresh) {
        GetLog() << "\n ~~~~~~~~~  phiT exceeded threshold:, tire " << m_name << ", = " << m_slip->phiT << "\n";
    }
}

// after calculating all the reactions, evaluate output for any fishy business
void ChPacejkaTire::evaluate_reactions(bool write_violations, bool enforce_threshold) {
    // any thresholds exceeded? then print some details about slip state
    bool output_slip_to_console = false;

    if (std::abs(m_FM_combined.force.x()) > Fx_thresh) {
        // GetLog() << "\n ***  !!!  ***  Fx exceeded threshold, tire " << m_name << ", = " << m_FM_combined.force.x() <<
        // "\n";
        output_slip_to_console = true;
        if (enforce_threshold)
            m_FM_combined.force.x() = m_FM_combined.force.x() * (Fx_thresh / std::abs(m_FM_combined.force.x()));
    }
    if (std::abs(m_FM_combined.force.y()) > Fy_thresh) {
        // GetLog() << "\n ***  !!!  ***  Fy exceeded threshold, tire " << m_name << ", = " << m_FM_combined.force.y() <<
        // "\n";
        output_slip_to_console = true;
        if (enforce_threshold)
            m_FM_combined.force.y() = m_FM_combined.force.y() * (Fy_thresh / std::abs(m_FM_combined.force.y()));
    }

    //  m_Fz, the Fz input to the tire model, must be limited based on the Fz_threshold
    // e.g., should never need t;his
    if (std::abs(m_Fz) > Fz_thresh) {
        GetLog() << "\n ***  !!!  ***  Fz exceeded threshold:, tire " << m_name << ", = " << m_Fz << "\n";
        output_slip_to_console = true;
    }

    if (std::abs(m_FM_combined.moment.x()) > Mx_thresh) {
        // GetLog() << " ***  !!!  ***  Mx exceeded threshold, tire " << m_name << ", = " << m_FM_combined.moment.x() <<
        // "\n";
        if (enforce_threshold)
            m_FM_combined.moment.x() = m_FM_combined.moment.x() * (Mx_thresh / std::abs(m_FM_combined.moment.x()));
        output_slip_to_console = true;
    }

    if (std::abs(m_FM_combined.moment.y()) > My_thresh) {
        // GetLog() << " ***  !!!  ***  My exceeded threshold, tire " << m_name << ", = " << m_FM_combined.moment.y() <<
        // "\n";
        if (enforce_threshold)
            m_FM_combined.moment.y() = m_FM_combined.moment.y() * (My_thresh / std::abs(m_FM_combined.moment.y()));
        output_slip_to_console = true;
    }

    if (std::abs(m_FM_combined.moment.z()) > Mz_thresh) {
        if (enforce_threshold)
            m_FM_combined.moment.z() = m_FM_combined.moment.z() * (Mz_thresh / std::abs(m_FM_combined.moment.z()));

        GetLog() << " ***  !!!  ***  Mz exceeded threshold, tire " << m_name << ", = " << m_FM_combined.moment.z()
                 << "\n";
        output_slip_to_console = true;
    }

    if (write_violations) {
        GetLog() << " ***********  time = " << m_simTime << ", slip data:  \n(u,v_alpha,v_gamma) = " << m_slip->u
                 << ", " << m_slip->v_alpha << ", " << m_slip->v_gamma << "\n velocity, center (x,y) = " << m_slip->V_cx
                 << ", " << m_slip->V_cy << "\n velocity, slip (x,y) = " << m_slip->V_sx << ", " << m_slip->V_sy
                 << "\n\n";
    }
}

// these are both for the linear case, small alpha
double ChPacejkaTire::ODE_RK_uv(double V_s, double sigma, double V_cx, double step_size, double x_curr) {
    double V_cx_abs = std::abs(V_cx);
    double k1 = -V_s - (V_cx_abs / sigma) * x_curr;
    double k2 = -V_s - (V_cx_abs / sigma) * (x_curr + 0.5 * step_size * k1);
    double k3 = -V_s - (V_cx_abs / sigma) * (x_curr + 0.5 * step_size * k2);
    double k4 = -V_s - (V_cx_abs / sigma) * (x_curr + step_size * k3);

    double delta_x = (step_size / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    return delta_x;
}

double ChPacejkaTire::get_dFy_dtan_alphaP(double x_curr) {
    // I suppose I could do a diff(Fx_c, alphaP) w/ a symbolic toolkit.
    // for now, just use forward differencing
    double dx = 0.01;  // x_curr is tan(alpha'), range (-1,1) over x < (-pi/2,pi/2)
    // changing tan(alpha') has no effect on kappa', gamma'
    double Fy_x_dx = Fy_combined(m_slip->alphaP + dx, m_slip->gammaP, m_slip->kappaP, m_FM_combined.force.y());
    // dFy_dx = (f(x+dx) - f(x)) / dx
    double d_Fy = Fy_x_dx - m_FM_combined.force.y();

    return d_Fy / dx;
}

// non-linear model, just use alphaP = alpha'
// small alpha, use Eq. 7.37
// here, we integrate d[tan(alphaP)]/dt for the step size
// returns delta_v_alpha = delta_tan_alphaP * sigma_a
double ChPacejkaTire::ODE_RK_kappaAlpha(double V_s, double sigma, double V_cx, double step_size, double x_curr) {
    double V_cx_abs = std::abs(V_cx);
    // double sigma_alpha = get_dFy_dtan_alphaP(x_curr) / C_Fy;
    double k1 = (-V_s - V_cx_abs * x_curr) / sigma;
    double k2 = (-V_s - V_cx_abs * (x_curr + 0.5 * step_size * k1)) / sigma;
    double k3 = (-V_s - V_cx_abs * (x_curr + 0.5 * step_size * k2)) / sigma;
    double k4 = (-V_s - V_cx_abs * (x_curr + step_size * k3)) / sigma;

    double delta_x = (step_size / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    return delta_x;
}

double ChPacejkaTire::ODE_RK_gamma(double C_Fgamma,
                                   double C_Falpha,
                                   double sigma_alpha,
                                   double V_cx,
                                   double step_size,
                                   double gamma,
                                   double v_gamma) {
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

double ChPacejkaTire::ODE_RK_phi(double C_Fphi,
                                 double C_Falpha,
                                 double V_cx,
                                 double psi_dot,
                                 double omega,
                                 double gamma,
                                 double sigma_alpha,
                                 double v_phi,
                                 double eps_gamma,
                                 double step_size) {
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

void ChPacejkaTire::slip_from_uv(bool use_besselink, double bessel_Cx, double bessel_Cy, double V_low) {
    // separate long and lateral damping components
    double d_Vxlow = 0;
    double d_Vylow = 0;
    double V_cx_abs = std::abs(m_slip->V_cx);
    // damp gradually to zero velocity at low velocity
    if (V_cx_abs <= V_low && use_besselink) {
        // d_vlow = bessel_c * (1.0 + std::cos(CH_C_PI * V_cx_abs / V_low));
        // damp between C * (2, 1) for V_cx (0,V_low)
        d_Vxlow = bessel_Cx * (1.0 + std::cos(CH_C_PI * V_cx_abs / 2.0 * V_low));
        d_Vylow = bessel_Cy * (1.0 + std::cos(CH_C_PI * V_cx_abs / 2.0 * V_low));
    } else {
        // also damp if not a driven wheel (can linger around slip = 0)
        // if(!m_driven && use_besselink)
        {
            d_Vxlow = bessel_Cx * (std::exp(-(V_cx_abs - V_low) / m_params->model.longvl));
            d_Vylow = bessel_Cy * (std::exp(-(V_cx_abs - V_low) / m_params->model.longvl));
        }
    }

    // Besselink is RH term in kappa_p, alpha_p
    double u_sigma = m_slip->u / m_relaxation->sigma_kappa;
    double u_Bessel = d_Vxlow * m_slip->V_sx / m_relaxation->C_Fkappa;

    // either u_Bessel or u_tow should be zero (or both)
    double kappa_p = u_sigma - u_Bessel;
    // don't allow damping to switch sign of kappa, clamp to zero
    if (u_sigma * kappa_p < 0)
        kappa_p = 0;

    // tan(alpha') ~= alpha' for small slip
    double v_sigma = -m_slip->v_alpha / m_relaxation->sigma_alpha;
    // double v_sigma = std::atan(m_slip->v_alpha / m_relaxation->sigma_alpha);
    double v_Bessel = -d_Vylow * m_slip->V_sy * m_sameSide / m_relaxation->C_Falpha;

    double alpha_p = v_sigma - v_Bessel;  //  - v_tow;
    // don't allow damping to switch sign of alpha
    if (v_sigma * alpha_p < 0)
        alpha_p = 0;

    // gammaP, phiP, phiT not effected by Besselink
    double gamma_p = m_relaxation->C_Falpha * m_slip->v_gamma / (m_relaxation->C_Fgamma * m_relaxation->sigma_alpha);
    double phi_p =
        (m_relaxation->C_Falpha * m_slip->v_phi) / (m_relaxation->C_Fphi * m_relaxation->sigma_alpha);  // turn slip
    double phi_t = -m_slip->psi_dot / m_slip->V_cx;                                                     // for turn slip

    // set transient slips
    m_slip->alphaP = alpha_p;
    m_slip->kappaP = kappa_p;
    m_slip->gammaP = gamma_p;
    m_slip->phiP = phi_p;
    m_slip->phiT = phi_t;

    bessel tmp = {u_Bessel, u_sigma, v_Bessel, v_sigma};
    *m_bessel = tmp;
}

// -----------------------------------------------------------------------------
// Calculate tire reactions.
// -----------------------------------------------------------------------------
// pure slip reactions, if the tire is in contact with the ground, in the TYDEX W-Axis system.
// calcFx               alphaP ~= 0
// calcFy and calcMz    kappaP ~= 0
// NOTE: alphaP, gammaP and kappaP defined with respect to tire side specified from *.tir input file.
// e.g., positive alpha = turn wheel toward vehicle centerline
// e.g., positive gamma = wheel vertical axis top pointing toward vehicle center
void ChPacejkaTire::pureSlipReactions() {
    if (m_in_contact) {
        // calculate Fx, pure long. slip condition
        m_FM_pure.force.x() = Fx_pureLong(m_slip->gammaP, m_slip->kappaP);

        // calc. Fy, pure lateral slip.
        m_FM_pure.force.y() = m_sameSide * Fy_pureLat(m_slip->alphaP, m_slip->gammaP);

        // calc Mz, pure lateral slip. Negative y-input force, and also the output Mz.
        m_FM_pure.moment.z() = m_sameSide * Mz_pureLat(m_slip->alphaP, m_slip->gammaP, m_sameSide * m_FM_pure.force.y());
    }
}

// combined slip reactions, if the tire is in contact with the ground
// MUST call pureSlipReactions() first, as these functions rely on intermediate values
// NOTE: alphaP and gammaP have already been modified for being on the L/R side
// e.g., positive alpha = turn wheel toward vehicle centerline
// e.g., positive gamma = wheel vertical axis top pointing toward vehicle center
void ChPacejkaTire::combinedSlipReactions() {
    if (m_in_contact) {
        // calculate Fx for combined slip
        m_FM_combined.force.x() = Fx_combined(m_slip->alphaP, m_slip->gammaP, m_slip->kappaP, m_FM_pure.force.x());

        // calc Fy for combined slip.
        m_FM_combined.force.y() =
            m_sameSide * Fy_combined(m_slip->alphaP, m_slip->gammaP, m_slip->kappaP, m_sameSide * m_FM_pure.force.y());

        // calc Mz for combined slip
        m_FM_combined.moment.z() =
            m_sameSide * Mz_combined(m_pureTorque->alpha_r, m_pureTorque->alpha_t, m_slip->gammaP, m_slip->kappaP,
                                     m_FM_combined.force.x(), m_sameSide * m_FM_combined.force.y());
    }
}

void ChPacejkaTire::relaxationLengths() {
    double p_Ky4 = 2;  // according to Pac2002 model
    double p_Ky5 = 0;
    double p_Ky6 = 2.5;  // 0.92;
    double p_Ky7 = 0.24;

    // NOTE: C_Falpha is positive according to Pacejka, negative in Pac2002.
    // Positive stiffness makes sense, so ensure all these are positive
    double C_Falpha =
        std::abs(m_params->lateral.pky1 * m_params->vertical.fnomin *
                 std::sin(p_Ky4 * std::atan(m_Fz / (m_params->lateral.pky2 * m_params->vertical.fnomin))) * m_zeta->z3 *
                 m_params->scaling.lyka);
    double sigma_alpha = C_Falpha / m_C_Fy;
    double C_Fkappa = m_Fz * (m_params->longitudinal.pkx1 + m_params->longitudinal.pkx2 * m_dF_z) *
                      exp(m_params->longitudinal.pkx3 * m_dF_z) * m_params->scaling.lky;
    double sigma_kappa = C_Fkappa / m_C_Fx;
    double C_Fgamma = m_Fz * (p_Ky6 + p_Ky7 * m_dF_z) * m_params->scaling.lgay;
    double C_Fphi = 2.0 * (C_Fgamma * m_R0);

    assert(C_Falpha > 0);
    assert(sigma_alpha > 0);
    assert(C_Fkappa > 0);
    assert(sigma_kappa > 0);
    assert(C_Fgamma > 0);
    assert(C_Fphi > 0);

    // NOTE: reference does not include the negative in the exponent for sigma_kappa_ref
    // double sigma_kappa_ref = m_Fz * (m_params->longitudinal.ptx1 + m_params->longitudinal.ptx2 *
    // m_dF_z)*(m_R0*m_params->scaling.lsgkp / m_params->vertical.fnomin) * exp( -m_params->longitudinal.ptx3 * m_dF_z);
    // double sigma_alpha_ref = m_params->lateral.pty1 * (1.0 - m_params->lateral.pky3 * std::abs( m_slip->gammaP ) ) *
    // m_R0 * m_params->scaling.lsgal * sin(p_Ky4 * atan(m_Fz / (m_params->lateral.pty2 * m_params->vertical.fnomin) )
    // );
    {
        relaxationL tmp = {C_Falpha, sigma_alpha, C_Fkappa, sigma_kappa, C_Fgamma, C_Fphi};
        *m_relaxation = tmp;
    }
}

double ChPacejkaTire::Fx_pureLong(double gamma, double kappa) {
    // double eps_Vx = 0.6;
    double eps_x = 0;
    // Fx, pure long slip
    double S_Hx = (m_params->longitudinal.phx1 + m_params->longitudinal.phx2 * m_dF_z) * m_params->scaling.lhx;
    double kappa_x = kappa + S_Hx;  // * 0.1;

    double mu_x = (m_params->longitudinal.pdx1 + m_params->longitudinal.pdx2 * m_dF_z) *
                  (1.0 - m_params->longitudinal.pdx3 * pow(gamma, 2)) * m_params->scaling.lmux;  // >0
    double K_x = m_Fz * (m_params->longitudinal.pkx1 + m_params->longitudinal.pkx2 * m_dF_z) *
                 exp(m_params->longitudinal.pkx3 * m_dF_z) * m_params->scaling.lkx;
    double C_x = m_params->longitudinal.pcx1 * m_params->scaling.lcx;  // >0
    double D_x = mu_x * m_Fz * m_zeta->z1;                             // >0
    double B_x = K_x / (C_x * D_x + eps_x);

    double sign_kap = (kappa_x >= 0) ? 1 : -1;

    double E_x = (m_params->longitudinal.pex1 + m_params->longitudinal.pex2 * m_dF_z +
                  m_params->longitudinal.pex3 * pow(m_dF_z, 2)) *
                 (1.0 - m_params->longitudinal.pex4 * sign_kap) * m_params->scaling.lex;
    double S_Vx = m_Fz * (m_params->longitudinal.pvx1 + m_params->longitudinal.pvx2 * m_dF_z) * m_params->scaling.lvx *
                  m_params->scaling.lmux * m_zeta->z1;
    double F_x =
        D_x * std::sin(C_x * std::atan(B_x * kappa_x - E_x * (B_x * kappa_x - std::atan(B_x * kappa_x)))) - S_Vx;

    // hold onto these coefs
    {
        pureLongCoefs tmp = {S_Hx, kappa_x, mu_x, K_x, B_x, C_x, D_x, E_x, F_x, S_Vx};
        *m_pureLong = tmp;
    }

    return F_x;
}

double ChPacejkaTire::Fy_pureLat(double alpha, double gamma) {
    double C_y = m_params->lateral.pcy1 * m_params->scaling.lcy;  // > 0
    double mu_y = (m_params->lateral.pdy1 + m_params->lateral.pdy2 * m_dF_z) *
                  (1.0 - m_params->lateral.pdy3 * pow(gamma, 2)) * m_params->scaling.lmuy;  // > 0
    double D_y = mu_y * m_Fz * m_zeta->z2;

    // doesn't make sense to ever have K_y be negative (it can be interpreted as lateral stiffnesss)
    double K_y = m_params->lateral.pky1 * m_params->vertical.fnomin *
                 std::sin(2.0 * std::atan(m_Fz / (m_params->lateral.pky2 * m_params->vertical.fnomin))) *
                 (1.0 - m_params->lateral.pky3 * std::abs(gamma)) * m_zeta->z3 * m_params->scaling.lyka;
    double B_y = K_y / (C_y * D_y);

    // double S_Hy = (m_params->lateral.phy1 + m_params->lateral.phy2 * m_dF_z) * m_params->scaling.lhy + (K_yGamma_0 *
    // m_slip->gammaP - S_VyGamma) * m_zeta->z0 / (K_yAlpha + 0.1) + m_zeta->z4 - 1.0;
    // Adasms S_Hy is a bit different
    double S_Hy = (m_params->lateral.phy1 + m_params->lateral.phy2 * m_dF_z) * m_params->scaling.lhy +
                  (m_params->lateral.phy3 * gamma * m_zeta->z0) + m_zeta->z4 - 1;

    double alpha_y = alpha + S_Hy;

    int sign_alpha = (alpha_y >= 0) ? 1 : -1;

    double E_y = (m_params->lateral.pey1 + m_params->lateral.pey2 * m_dF_z) *
                 (1.0 - (m_params->lateral.pey3 + m_params->lateral.pey4 * gamma) * sign_alpha) *
                 m_params->scaling.ley;  // + p_Ey5 * pow(gamma,2)
    double S_Vy = m_Fz * ((m_params->lateral.pvy1 + m_params->lateral.pvy2 * m_dF_z) * m_params->scaling.lvy +
                          (m_params->lateral.pvy3 + m_params->lateral.pvy4 * m_dF_z) * gamma) *
                  m_params->scaling.lmuy * m_zeta->z2;

    double F_y =
        D_y * std::sin(C_y * std::atan(B_y * alpha_y - E_y * (B_y * alpha_y - std::atan(B_y * alpha_y)))) + S_Vy;

    // hold onto coefs
    {
        pureLatCoefs tmp = {S_Hy, alpha_y, mu_y, K_y, S_Vy, B_y, C_y, D_y, E_y};
        *m_pureLat = tmp;
    }

    return F_y;
}

double ChPacejkaTire::Mz_pureLat(double alpha, double gamma, double Fy_pureSlip) {
    // some constants
    int sign_Vx = (m_slip->V_cx >= 0) ? 1 : -1;

    double S_Hf = m_pureLat->S_Hy + m_pureLat->S_Vy / m_pureLat->K_y;
    double alpha_r = alpha + S_Hf;
    double S_Ht = m_params->aligning.qhz1 + m_params->aligning.qhz2 * m_dF_z +
                  (m_params->aligning.qhz3 + m_params->aligning.qhz4 * m_dF_z) * gamma;
    double alpha_t = alpha + S_Ht;

    double B_r = (m_params->aligning.qbz9 * (m_params->scaling.lky / m_params->scaling.lmuy) +
                  m_params->aligning.qbz10 * m_pureLat->B_y * m_pureLat->C_y) *
                 m_zeta->z6;
    double C_r = m_zeta->z7;
    // no terms (Dz10, Dz11) for gamma^2 term seen in Pacejka
    // double D_r = m_Fz*m_R0 * ((m_params->aligning.qdz6 + m_params->aligning.qdz7 *
    // m_dF_z)*m_params->scaling.lres*m_zeta->z2 + (m_params->aligning.qdz8 + m_params->aligning.qdz9 *
    // m_dF_z)*gamma*m_params->scaling.lgaz*m_zeta->z0) * m_slip->cosPrime_alpha*m_params->scaling.lmuy*sign_Vx +
    // m_zeta->z8 - 1.0;
    // reference
    double D_r = m_Fz * m_R0 * ((m_params->aligning.qdz6 + m_params->aligning.qdz7 * m_dF_z) * m_params->scaling.lres +
                                (m_params->aligning.qdz8 + m_params->aligning.qdz9 * m_dF_z) * gamma) *
                     m_params->scaling.lmuy * m_slip->cosPrime_alpha * sign_Vx +
                 m_zeta->z8 - 1.0;
    // qbz4 is not in Pacejka
    double B_t =
        (m_params->aligning.qbz1 + m_params->aligning.qbz2 * m_dF_z + m_params->aligning.qbz3 * pow(m_dF_z, 2)) *
        (1.0 + m_params->aligning.qbz4 * gamma + m_params->aligning.qbz5 * std::abs(gamma)) * m_params->scaling.lvyka /
        m_params->scaling.lmuy;
    double C_t = m_params->aligning.qcz1;
    double D_t0 = m_Fz * (m_R0 / m_params->vertical.fnomin) *
                  (m_params->aligning.qdz1 + m_params->aligning.qdz2 * m_dF_z) * sign_Vx;
    // no abs on qdz3 gamma in reference
    double D_t = D_t0 * (1.0 + m_params->aligning.qdz3 * std::abs(gamma) + m_params->aligning.qdz4 * pow(gamma, 2)) *
                 m_zeta->z5 * m_params->scaling.ltr;
    double E_t =
        (m_params->aligning.qez1 + m_params->aligning.qez2 * m_dF_z + m_params->aligning.qez3 * pow(m_dF_z, 2)) *
        (1.0 +
         (m_params->aligning.qez4 + m_params->aligning.qez5 * gamma) * (2.0 / chrono::CH_C_PI) *
             std::atan(B_t * C_t * alpha_t));
    double t = D_t * std::cos(C_t * std::atan(B_t * alpha_t - E_t * (B_t * alpha_t - std::atan(B_t * alpha_t)))) *
               m_slip->cosPrime_alpha;

    double MP_z = -t * Fy_pureSlip;
    double M_zr = D_r * std::cos(C_r * std::atan(B_r * alpha_r));  // this is in the D_r term: * m_slip->cosPrime_alpha;

    double M_z = MP_z + M_zr;

    // hold onto coefs
    {
        pureTorqueCoefs tmp = {S_Hf,
                               alpha_r,
                               S_Ht,
                               alpha_t,
                               m_slip->cosPrime_alpha,
                               m_pureLat->K_y,
                               B_r,
                               C_r,
                               D_r,
                               B_t,
                               C_t,
                               D_t0,
                               D_t,
                               E_t,
                               t,
                               MP_z,
                               M_zr};
        *m_pureTorque = tmp;
    }

    return M_z;
}

double ChPacejkaTire::Fx_combined(double alpha, double gamma, double kappa, double Fx_pureSlip) {
    double rbx3 = 1.0;

    double S_HxAlpha = m_params->longitudinal.rhx1;
    double alpha_S = alpha + S_HxAlpha;
    double B_xAlpha = (m_params->longitudinal.rbx1 + rbx3 * pow(gamma, 2)) *
                      std::cos(std::atan(m_params->longitudinal.rbx2 * kappa)) * m_params->scaling.lxal;
    double C_xAlpha = m_params->longitudinal.rcx1;
    double E_xAlpha = m_params->longitudinal.rex1 + m_params->longitudinal.rex2 * m_dF_z;

    // double G_xAlpha0 = std::cos(C_xAlpha * std::atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha -
    // std::atan(B_xAlpha * S_HxAlpha)) ) );
    double G_xAlpha0 =
        std::cos(C_xAlpha *
                 std::atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha - std::atan(B_xAlpha * S_HxAlpha))));

    // double G_xAlpha = std::cos(C_xAlpha * std::atan(B_xAlpha * alpha_S - E_xAlpha * (B_xAlpha * alpha_S -
    // std::atan(B_xAlpha * alpha_S)) ) ) / G_xAlpha0;
    double G_xAlpha = std::cos(C_xAlpha * std::atan(B_xAlpha * alpha_S -
                                                    E_xAlpha * (B_xAlpha * alpha_S - std::atan(B_xAlpha * alpha_S)))) /
                      G_xAlpha0;

    double F_x = G_xAlpha * Fx_pureSlip;

    {
        combinedLongCoefs tmp = {S_HxAlpha, alpha_S, B_xAlpha, C_xAlpha, E_xAlpha, G_xAlpha0, G_xAlpha};
        *m_combinedLong = tmp;
    }

    return F_x;
}

double ChPacejkaTire::Fy_combined(double alpha, double gamma, double kappa, double Fy_pureSlip) {
    double rby4 = 0;

    double S_HyKappa = m_params->lateral.rhy1 + m_params->lateral.rhy2 * m_dF_z;
    double kappa_S = kappa + S_HyKappa;
    double B_yKappa = (m_params->lateral.rby1 + rby4 * pow(gamma, 2)) *
                      std::cos(std::atan(m_params->lateral.rby2 * (alpha - m_params->lateral.rby3))) *
                      m_params->scaling.lyka;
    double C_yKappa = m_params->lateral.rcy1;
    double E_yKappa = m_params->lateral.rey1 + m_params->lateral.rey2 * m_dF_z;
    double D_VyKappa = m_pureLat->mu_y * m_Fz *
                       (m_params->lateral.rvy1 + m_params->lateral.rvy2 * m_dF_z + m_params->lateral.rvy3 * gamma) *
                       std::cos(std::atan(m_params->lateral.rvy4 * alpha)) * m_zeta->z2;
    double S_VyKappa = D_VyKappa * std::sin(m_params->lateral.rvy5 * std::atan(m_params->lateral.rvy6 * kappa)) *
                       m_params->scaling.lvyka;
    double G_yKappa0 =
        std::cos(C_yKappa *
                 std::atan(B_yKappa * S_HyKappa - E_yKappa * (B_yKappa * S_HyKappa - std::atan(B_yKappa * S_HyKappa))));
    double G_yKappa = std::cos(C_yKappa * std::atan(B_yKappa * kappa_S -
                                                    E_yKappa * (B_yKappa * kappa_S - std::atan(B_yKappa * kappa_S)))) /
                      G_yKappa0;

    double F_y = G_yKappa * Fy_pureSlip + S_VyKappa;

    {
        combinedLatCoefs tmp = {
            S_HyKappa, kappa_S, B_yKappa, C_yKappa, E_yKappa, D_VyKappa, S_VyKappa, G_yKappa0, G_yKappa};
        *m_combinedLat = tmp;
    }

    return F_y;
}

double ChPacejkaTire::Mz_combined(double alpha_r,
                                  double alpha_t,
                                  double gamma,
                                  double kappa,
                                  double Fx_combined,
                                  double Fy_combined) {
    double FP_y = Fy_combined - m_combinedLat->S_VyKappa;
    double s = m_R0 * (m_params->aligning.ssz1 + m_params->aligning.ssz2 * (Fy_combined / m_params->vertical.fnomin) +
                       (m_params->aligning.ssz3 + m_params->aligning.ssz4 * m_dF_z) * gamma) *
               m_params->scaling.ls;
    int sign_alpha_t = (alpha_t >= 0) ? 1 : -1;
    int sign_alpha_r = (alpha_r >= 0) ? 1 : -1;

    double alpha_t_eq =
        sign_alpha_t * sqrt(pow(alpha_t, 2) + pow(m_pureLong->K_x / m_pureTorque->K_y, 2) * pow(kappa, 2));
    double alpha_r_eq =
        sign_alpha_r * sqrt(pow(alpha_r, 2) + pow(m_pureLong->K_x / m_pureTorque->K_y, 2) * pow(kappa, 2));

    double M_zr = m_pureTorque->D_r * std::cos(m_pureTorque->C_r * std::atan(m_pureTorque->B_r * alpha_r_eq)) *
                  m_slip->cosPrime_alpha;
    double t =
        m_pureTorque->D_t *
        std::cos(m_pureTorque->C_t * std::atan(m_pureTorque->B_t * alpha_t_eq -
                                               m_pureTorque->E_t * (m_pureTorque->B_t * alpha_t_eq -
                                                                    std::atan(m_pureTorque->B_t * alpha_t_eq)))) *
        m_slip->cosPrime_alpha;

    double M_z_y = -t * FP_y;
    double M_z_x = s * Fx_combined;
    double M_z = M_z_y + M_zr + M_z_x;

    {
        combinedTorqueCoefs tmp = {m_slip->cosPrime_alpha, FP_y, s, alpha_t_eq, alpha_r_eq, M_zr, t, M_z_x, M_z_y};
        *m_combinedTorque = tmp;
    }

    return M_z;
}

double ChPacejkaTire::calc_Mx(double gamma, double Fy_combined) {
    double M_x = 0;
    if (m_in_contact) {
        // according to Pacejka
        M_x = m_Fz * m_R0 * (m_params->overturning.qsx1 - m_params->overturning.qsx2 * gamma +
                             m_params->overturning.qsx3 * (Fy_combined / m_params->vertical.fnomin)) *
              m_params->scaling.lmx;
    }

    return M_x;
}

double ChPacejkaTire::calc_My(double Fx_combined) {
    double M_y = 0;
    if (m_in_contact) {
        double V_r = m_tireState.omega * m_R_eff;
        M_y = -m_Fz * m_R0 * (m_params->rolling.qsy1 * std::atan(V_r / m_params->model.longvl) +
                              m_params->rolling.qsy2 * (Fx_combined / m_params->vertical.fnomin)) *
              m_params->scaling.lmy;
        // reference calc
        // M_y = m_R0*m_Fz * (m_params->rolling.qsy1 + m_params->rolling.qsy2*Fx_combined/m_params->vertical.fnomin +
        // m_params->rolling.qsy3*std::abs(m_slip->V_cx/m_params->model.longvl) +
        // m_params->rolling.qsy4*pow(m_slip->V_cx/m_params->model.longvl,4));
    }
    return M_y;
}

// -----------------------------------------------------------------------------
// Load a PacTire specification file.
//
// For an example, see the file data/vehicle/hmmwv/tire/HMMWV_pacejka.tir
// -----------------------------------------------------------------------------
void ChPacejkaTire::loadPacTireParamFile() {
    // try to load the file
    std::ifstream inFile(this->getPacTireParamFile().c_str(), std::ios::in);

    // if not loaded, say something and exit
    if (!inFile.is_open()) {
        GetLog() << "\n\n !!!!!!! couldn't load the pac tire file: " << getPacTireParamFile().c_str() << "\n\n";
        GetLog() << " pacTire param file opened in a text editor somewhere ??? \n\n\n";
        return;
    }

    // success in opening file, load the data, broken down into sections
    // according to what is found in the PacTire input file
    readPacTireInput(inFile);

    // this bool will allow you to query the pac tire for output
    // Forces, moments based on wheel state info.
    m_params_defined = true;
}

void ChPacejkaTire::readPacTireInput(std::ifstream& inFile) {
    // advance to the first part of the file with data we need to read
    std::string tline;

    while (std::getline(inFile, tline)) {
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

void ChPacejkaTire::readSection_UNITS(std::ifstream& inFile) {
    // skip the first line
    std::string tline;
    std::getline(inFile, tline);

    // string util stuff
    std::string tok;      // name of token
    std::string val_str;  // temp for string token values
    std::vector<std::string> split;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
    }
}

void ChPacejkaTire::readSection_MODEL(std::ifstream& inFile) {
    // skip the first line
    std::string tline;
    std::getline(inFile, tline);

    // string util stuff
    std::vector<std::string> split;
    std::string val_str;  // string token values

    // token value changes type in this section, do it manually
    std::getline(inFile, tline);

    // get the token / value
    split = splitStr(tline, '=');
    m_params->model.property_file_format = splitStr(split[1], '\'')[1];

    std::getline(inFile, tline);
    m_params->model.use_mode = fromTline<int>(tline);

    std::getline(inFile, tline);
    m_params->model.vxlow = fromTline<double>(tline);

    std::getline(inFile, tline);
    m_params->model.longvl = fromTline<double>(tline);

    std::getline(inFile, tline);
    split = splitStr(tline, '=');
    m_params->model.tyreside = splitStr(split[1], '\'')[1];
}

void ChPacejkaTire::readSection_DIMENSION(std::ifstream& inFile) {
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
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 5) {
        GetLog() << " error reading DIMENSION section of pactire input file!!! \n\n";
        return;
    }
    // right size, create the struct
    struct dimension dim = {dat[0], dat[1], dat[2], dat[3], dat[4]};
    m_params->dimension = dim;
}

void ChPacejkaTire::readSection_SHAPE(std::ifstream& inFile) {
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
        split = splitStr(tline, ' ');
        rad.push_back(std::atof(split[1].c_str()));
        wid.push_back(std::atof(split[5].c_str()));
    }
    m_params->shape.radial = rad;
    m_params->shape.width = wid;
}

void ChPacejkaTire::readSection_VERTICAL(std::ifstream& inFile) {
    // skip the first line
    std::string tline;
    std::getline(inFile, tline);

    // push each token value to this vector, check the # of items added
    std::vector<double> dat;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 6) {
        GetLog() << " error reading VERTICAL section of pactire input file!!! \n\n";
        return;
    }
    // right size, create the struct
    struct vertical vert = {dat[0], dat[1], dat[2], dat[3], dat[4], dat[5]};
    m_params->vertical = vert;
}

void ChPacejkaTire::readSection_RANGES(std::ifstream& inFile) {
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
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 2) {
        GetLog() << " error reading LONG_SLIP_RANGE section of pactire input file!!! \n\n";
        return;
    }
    // right size, create the struct
    struct long_slip_range long_slip = {dat[0], dat[1]};
    m_params->long_slip_range = long_slip;
    dat.clear();
    std::getline(inFile, tline);

    // SLIP_ANGLE_RANGE
    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 2) {
        GetLog() << " error reading LONG_SLIP_RANGE section of pactire input file!!! \n\n";
        return;
    }
    // right size, create the struct
    struct slip_angle_range slip_ang = {dat[0], dat[1]};
    m_params->slip_angle_range = slip_ang;
    dat.clear();
    std::getline(inFile, tline);

    // INCLINATION_ANGLE_RANGE
    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 2) {
        GetLog() << " error reading INCLINATION_ANGLE_RANGE section of pactire input file!!! \n\n";
        return;
    }
    struct inclination_angle_range incl_ang = {dat[0], dat[1]};
    m_params->inclination_angle_range = incl_ang;
    dat.clear();
    std::getline(inFile, tline);

    // VERTICAL_FORCE_RANGE
    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 2) {
        GetLog() << " error reading VERTICAL_FORCE_RANGE section of pactire input file!!! \n\n";
        return;
    }
    struct vertical_force_range vert_range = {dat[0], dat[1]};
    m_params->vertical_force_range = vert_range;
}

void ChPacejkaTire::readSection_scaling(std::ifstream& inFile) {
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
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 28) {
        GetLog() << " error reading scaling section of pactire input file!!! \n\n";
        return;
    }
    struct scaling_coefficients coefs = {dat[0],  dat[1],  dat[2],  dat[3],  dat[4],  dat[5],  dat[6],
                                         dat[7],  dat[8],  dat[9],  dat[10], dat[11], dat[12], dat[13],
                                         dat[14], dat[15], dat[16], dat[17], dat[18], dat[19], dat[20],
                                         dat[21], dat[22], dat[23], dat[24], dat[25], dat[26], dat[27]};
    m_params->scaling = coefs;
}

void ChPacejkaTire::readSection_longitudinal(std::ifstream& inFile) {
    std::string tline;
    std::getline(inFile, tline);
    std::vector<double> dat;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 24) {
        GetLog() << " error reading longitudinal section of pactire input file!!! \n\n";
        return;
    }
    struct longitudinal_coefficients coefs = {dat[0],  dat[1],  dat[2],  dat[3],  dat[4],  dat[5],  dat[6],  dat[7],
                                              dat[8],  dat[9],  dat[10], dat[11], dat[12], dat[13], dat[14], dat[15],
                                              dat[16], dat[17], dat[18], dat[19], dat[20], dat[21], dat[22], dat[23]};
    m_params->longitudinal = coefs;
}

void ChPacejkaTire::readSection_overturning(std::ifstream& inFile) {
    std::string tline;
    std::getline(inFile, tline);
    std::vector<double> dat;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 3) {
        GetLog() << " error reading  overturning section of pactire input file!!! \n\n";
        return;
    }
    struct overturning_coefficients coefs = {dat[0], dat[1], dat[2]};
    m_params->overturning = coefs;
}

void ChPacejkaTire::readSection_lateral(std::ifstream& inFile) {
    std::string tline;
    std::getline(inFile, tline);
    std::vector<double> dat;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 34) {
        GetLog() << " error reading lateral section of pactire input file!!! \n\n";
        return;
    }
    struct lateral_coefficients coefs = {
        dat[0],  dat[1],  dat[2],  dat[3],  dat[4],  dat[5],  dat[6],  dat[7],  dat[8],  dat[9],  dat[10], dat[11],
        dat[12], dat[13], dat[14], dat[15], dat[16], dat[17], dat[18], dat[19], dat[20], dat[21], dat[22], dat[23],
        dat[24], dat[25], dat[26], dat[27], dat[28], dat[29], dat[30], dat[31], dat[32], dat[33]};
    m_params->lateral = coefs;
}

void ChPacejkaTire::readSection_rolling(std::ifstream& inFile) {
    std::string tline;
    std::getline(inFile, tline);
    std::vector<double> dat;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 4) {
        GetLog() << " error reading rolling section of pactire input file!!! \n\n";
        return;
    }
    struct rolling_coefficients coefs = {dat[0], dat[1], dat[2], dat[3]};
    m_params->rolling = coefs;
}

void ChPacejkaTire::readSection_aligning(std::ifstream& inFile) {
    std::string tline;
    std::getline(inFile, tline);
    std::vector<double> dat;

    while (std::getline(inFile, tline)) {
        // made it to the next section
        if (tline[0] == '$')
            break;
        dat.push_back(fromTline<double>(tline));
    }

    if (dat.size() != 31) {
        GetLog() << " error reading LONG_SLIP_RANGE section of pactire input file!!! \n\n";
        return;
    }
    struct aligning_coefficients coefs = {dat[0],  dat[1],  dat[2],  dat[3],  dat[4],  dat[5],  dat[6],  dat[7],
                                          dat[8],  dat[9],  dat[10], dat[11], dat[12], dat[13], dat[14], dat[15],
                                          dat[16], dat[17], dat[18], dat[19], dat[20], dat[21], dat[22], dat[23],
                                          dat[24], dat[25], dat[26], dat[27], dat[28], dat[29], dat[30]};
    m_params->aligning = coefs;
}

// -----------------------------------------------------------------------------
// Functions providing access to private structures
// -----------------------------------------------------------------------------
double ChPacejkaTire::get_kappa() const {
    return m_slip->kappa;
}

double ChPacejkaTire::get_alpha() const {
    return m_slip->alpha;
}

double ChPacejkaTire::get_gamma() const {
    return m_slip->gamma;
}

double ChPacejkaTire::get_kappaPrime() const {
    return m_slip->kappaP;
}

double ChPacejkaTire::get_alphaPrime() const {
    return m_slip->alphaP;
}

double ChPacejkaTire::get_gammaPrime() const {
    return m_slip->gammaP;
}

double ChPacejkaTire::get_min_long_slip() const {
    return m_params->long_slip_range.kpumin;
}

double ChPacejkaTire::get_max_long_slip() const {
    return m_params->long_slip_range.kpumax;
}

double ChPacejkaTire::get_min_lat_slip() const {
    return m_params->slip_angle_range.alpmin;
}

double ChPacejkaTire::get_max_lat_slip() const {
    return m_params->slip_angle_range.alpmax;
}

double ChPacejkaTire::get_longvl() const {
    return m_params->model.longvl;
}

// -----------------------------------------------------------------------------
// Write output file for post-processing with the Python pandas module.
// -----------------------------------------------------------------------------
void ChPacejkaTire::WriteOutData(double time, const std::string& outFilename) {
    // first time thru, write headers
    if (m_Num_WriteOutData == 0) {
        m_outFilename = outFilename;
        std::ofstream oFile(outFilename.c_str(), std::ios_base::out);
        if (!oFile.is_open()) {
            std::cout << " couldn't open file for writing: " << outFilename << " \n\n";
            return;
        } else {
            // write the headers, Fx, Fy are pure forces, Fxc and Fyc are the combined forces
            oFile << "time,kappa,alpha,gamma,kappaP,alphaP,gammaP,Vx,Vy,omega,Fx,Fy,Fz,Mx,My,Mz,Fxc,Fyc,Mzc,Mzx,Mzy,M_"
                     "zrc,contact,m_Fz,m_dF_z,u,valpha,vgamma,vphi,du,dvalpha,dvgamma,dvphi,R0,R_l,Reff,MP_z,M_zr,t,s,"
                     "FX,FY,FZ,MX,MY,MZ,u_Bessel,u_sigma,v_Bessel,v_sigma" << std::endl;
            m_Num_WriteOutData++;
            oFile.close();
        }
    } else {
        // already written the headers, just increment the function counter
        m_Num_WriteOutData++;
    }
    // ensure file was able to be opened, headers are written
    if (m_Num_WriteOutData > 0) {
        // open file, append
        std::ofstream appFile(outFilename.c_str(), std::ios_base::app);
        // global force/moments applied to wheel rigid body
        TireForce global_FM = GetTireForce_combinedSlip(false);
        // write the slip info, reaction forces for pure & combined slip cases
        appFile << time << "," << m_slip->kappa << "," << m_slip->alpha * 180. / 3.14159 << "," << m_slip->gamma << ","
                << m_slip->kappaP << "," << m_slip->alphaP << "," << m_slip->gammaP << "," << m_slip->V_cx << ","
                << m_slip->V_cy << "," << m_tireState.omega
                << ","  // m_tireState.lin_vel.x() <<","<< m_tireState.lin_vel.y() <<","
                << m_FM_pure.force.x() << "," << m_FM_pure.force.y() << "," << m_FM_pure.force.z() << ","
                << m_FM_pure.moment.x() << "," << m_FM_pure.moment.y() << "," << m_FM_pure.moment.z() << ","
                << m_FM_combined.force.x() << "," << m_FM_combined.force.y() << "," << m_FM_combined.moment.z() << ","
                << m_combinedTorque->M_z_x << "," << m_combinedTorque->M_z_y << "," << m_combinedTorque->M_zr << ","
                << (int)m_in_contact << "," << m_Fz << "," << m_dF_z << "," << m_slip->u << "," << m_slip->v_alpha
                << "," << m_slip->v_gamma << "," << m_slip->v_phi << "," << m_slip->Idu_dt << ","
                << m_slip->Idv_alpha_dt << "," << m_slip->Idv_gamma_dt << "," << m_slip->Idv_phi_dt << "," << m_R0
                << "," << m_R_l << "," << m_R_eff << "," << m_pureTorque->MP_z << "," << m_pureTorque->M_zr << ","
                << m_combinedTorque->t << "," << m_combinedTorque->s << "," << global_FM.force.x() << ","
                << global_FM.force.y() << "," << global_FM.force.z() << "," << global_FM.moment.x() << ","
                << global_FM.moment.y() << "," << global_FM.moment.z() << "," << m_bessel->u_Bessel << ","
                << m_bessel->u_sigma << "," << m_bessel->v_Bessel << "," << m_bessel->v_sigma << std::endl;
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
WheelState ChPacejkaTire::getState_from_KAG(double kappa, double alpha, double gamma, double Vx) {
    WheelState state;

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

}  // end namespace vehicle
}  // end namespace chrono
