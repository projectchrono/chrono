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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for a tire model based on LuGre friction
//
// Ref: C.Canudas de Wit, P.Tsiotras, E.Velenis, M.Basset and
// G.Gissinger.Dynamics friction models for longitudinal road / tire
// interaction.Vehicle System Dynamics.Oct 14, 2002.
//
// =============================================================================


#include "ChLugreTire.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChLugreTire::ChLugreTire(const ChTerrain& terrain)
: ChTire(terrain),
  m_stepsize(1e-3)
{
  m_tireForce.force = ChVector<>(0, 0, 0);
  m_tireForce.point = ChVector<>(0, 0, 0);
  m_tireForce.moment = ChVector<>(0, 0, 0);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Initialize()
{
  m_data.resize(getNumDiscs());
  m_state.resize(getNumDiscs());

  SetLugreParams();

  // Initialize disc states
  for (int id = 0; id < getNumDiscs(); id++) {
    m_state[id].z0 = 0;
    m_state[id].z1 = 0;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Update(double              time,
                         const ChBodyState&  wheel_state)
{
  double disc_radius = getRadius();
  const double* disc_locs = getDiscLocations();

  // Clear the force accumulators and set the application point to the wheel
  // center.
  m_tireForce.force = ChVector<>(0, 0, 0);
  m_tireForce.moment = ChVector<>(0, 0, 0);
  m_tireForce.point = wheel_state.pos;

  // Extract the wheel normal (expressed in global frame)
  ChMatrix33<> A(wheel_state.rot);
  ChVector<> disc_normal = A.Get_A_Yaxis();

  // Loop over all discs, check contact with terrain and accumulate normal tire
  // forces.
  double depth;

  for (int id = 0; id < getNumDiscs(); id++) {
    // Calculate center of disk (expressed in global frame)
    ChVector<> disc_center = wheel_state.pos + disc_locs[id] * disc_normal;

    // Check contact with terrain and calculate contact points.
    m_data[id].in_contact = disc_terrain_contact(disc_center, disc_normal, disc_radius,
                                                 m_data[id].frame, depth);
    if (!m_data[id].in_contact)
      continue;

    // Relative velocity at contact point (expressed in global frame and in the contact frame)
    ChVector<> vel = wheel_state.lin_vel + Vcross(wheel_state.ang_vel, m_data[id].frame.pos - wheel_state.pos);
    m_data[id].vel = m_data[id].frame.TransformDirectionParentToLocal(vel);

    // Generate normal contact force and add to accumulators (recall, all forces
    // are reduced to the wheel center)
    double     Fn_mag = getNormalStiffness() * depth - getNormalDamping() * m_data[id].vel.z;
    ChVector<> Fn = Fn_mag * m_data[id].frame.rot.GetZaxis();

    m_data[id].normal_force = Fn_mag;

    m_tireForce.force += Fn;
    m_tireForce.moment += Vcross(m_data[id].frame.pos - m_tireForce.point, Fn);

    // ODE coefficients for longitudinal direction: z' = a + b * z
    {
      double v = abs(m_data[id].vel.x);
      double g = m_Fc[0] + (m_Fs[0] - m_Fc[0]) * exp(-sqrt(v / m_vs[0]));
      m_data[id].ode_coef_a[0] = v;
      m_data[id].ode_coef_b[0] = -m_sigma0[0] * v / g;
    }
    
    // ODE coefficients for lateral direction: z' = a + b * z
    {
      double v = abs(m_data[id].vel.y);
      double g = m_Fc[1] + (m_Fs[1] - m_Fc[1]) * exp(-sqrt(v / m_vs[1]));
      m_data[id].ode_coef_a[1] = v;
      m_data[id].ode_coef_b[1] = -m_sigma0[1] * v / g;
    }

  }

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Advance(double step)
{
  for (int id = 0; id < getNumDiscs(); id++) {
    if (!m_data[id].in_contact)
      continue;

    // Magnitude of normal contact force for this disc
    double Fn_mag = m_data[id].normal_force;

    // Advance disc state (using trapezoidal integration scheme):
    //         z_{n+1} = alpha * z_{n} + beta
    // Evaluate friction force
    // Add to accumulators for tire force

    // Longitudinal direction
    {
      double denom = (2 - m_data[id].ode_coef_b[0] * m_stepsize);
      double alpha = (2 + m_data[id].ode_coef_b[0] * m_stepsize) / denom;
      double beta = 2 * m_data[id].ode_coef_a[0] * m_stepsize / denom;
      double z = alpha * m_state[id].z0 + beta;
      double zd = m_data[id].ode_coef_a[0] + m_data[id].ode_coef_b[0] * z;
      m_state[id].z0 = z;

      double v = m_data[id].vel.x;
      double Ft_mag = Fn_mag * (m_sigma0[0] * z + m_sigma1[0] * zd + m_sigma2[0] * abs(v));
      ChVector<> dir = (v < 0) ? m_data[id].frame.rot.GetXaxis() : -m_data[id].frame.rot.GetXaxis();
      ChVector<> Ft = -Ft_mag * dir;

      // Include tangential forces in accumulators
      //m_tireForce.force += Ft;
      //m_tireForce.moment += Vcross(m_data[id].frame.pos - m_tireForce.point, Ft);
    }

    // Lateral direction
    {
      double denom = (2 - m_data[id].ode_coef_b[1] * m_stepsize);
      double alpha = (2 + m_data[id].ode_coef_b[1] * m_stepsize) / denom;
      double beta = 2 * m_data[id].ode_coef_a[1] * m_stepsize / denom;
      double z = alpha * m_state[id].z1 + beta;
      double zd = m_data[id].ode_coef_a[1] + m_data[id].ode_coef_b[1] * z;
      m_state[id].z1 = z;

      double v = m_data[id].vel.y;
      double Ft_mag = Fn_mag * (m_sigma0[1] * z + m_sigma1[1] * zd + m_sigma2[1] * abs(v));
      ChVector<> dir = (v < 0) ? m_data[id].frame.rot.GetYaxis() : -m_data[id].frame.rot.GetYaxis();
      ChVector<> Ft = -Ft_mag * dir;

      // Include tangential forces in accumulators
      //m_tireForce.force += Ft;
      //m_tireForce.moment += Vcross(m_data[id].frame.pos - m_tireForce.point, Ft);
    }

  }
}



} // end namespace chrono
