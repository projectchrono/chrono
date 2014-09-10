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
  for (int id = 0; id < getNumDiscs(); id++)
	  m_state[id] = 0;
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
  ChVector<> normal;
  ChVector<> ptD;
  ChVector<> ptT;
  double depth;

  for (int id = 0; id < getNumDiscs(); id++) {
    // Calculate center of disk (expressed in global frame)
    ChVector<> disc_center = wheel_state.pos + disc_locs[id] * disc_normal;

    // Check contact with terrain and calculate contact points.
    m_data[id].contact = disc_terrain_contact(disc_center, disc_normal, disc_radius,
                                              ptD, ptT, normal, depth);
    if (!m_data[id].contact)
      continue;

	m_data[id].contact_point = ptD;

    // Relative velocity at contact point (expressed in global frame)
    ChVector<> vel = wheel_state.lin_vel + Vcross(wheel_state.ang_vel, ptD - wheel_state.pos);

    // Generate normal contact force and add to accumulators (recall, all forces
    // are reduced to the wheel center)
    double     normalVel_mag = Vdot(vel, normal);
    double     Fn_mag = getNormalStiffness() * depth - getNormalDamping() * normalVel_mag;
    ChVector<> Fn = Fn_mag * normal;

    m_tireForce.force += Fn;
    m_tireForce.moment += Vcross(ptD - m_tireForce.point, Fn);

    // Calculate and store relative sliding velocity
	ChVector<> slidingVel = vel - normalVel_mag * normal;
	double slidingVel_mag = slidingVel.Length();
	m_data[id].slidingVel = slidingVel;
	m_data[id].slidingVel_mag = slidingVel_mag;
	m_data[id].normal_force = Fn_mag;

	// Calculate coefficients in the ODE for this disc
	double tmp = exp(-sqrt(slidingVel_mag / m_vs));
	double g = m_Fc + (m_Fs - m_Fc) * tmp; 
	double a = slidingVel_mag;
	double b = -m_sigma0 * slidingVel_mag / g;

	m_data[id].coef_a = a;
	m_data[id].coef_b = b;
	m_data[id].alpha = (2 + b * m_stepsize) / (2 - b * m_stepsize);
	m_data[id].beta = 2 * a * m_stepsize / (2 - b * m_stepsize);
  }

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Advance(double step)
{
  for (int id = 0; id < getNumDiscs(); id++) {
    if (!m_data[id].contact)
      continue;

	// Advance disc state (using trapezoidal integration scheme)
	double z = m_data[id].alpha * m_state[id] + m_data[id].beta; 
	double zd = m_data[id].coef_a + m_data[id].coef_b * z;
	m_state[id] = z;

	double Fn_mag = m_data[id].normal_force;    
	ChVector<> slidingVel = m_data[id].slidingVel;
	double slidingVel_mag = m_data[id].slidingVel_mag;

	double Ft_mag = Fn_mag * (m_sigma0 * z + m_sigma1 * zd + m_sigma2 * slidingVel_mag); //eg. 25

	ChVector<> Ft = -Ft_mag * slidingVel / slidingVel_mag;

	// Include tangential forces in accumulators
	m_tireForce.force += Ft;
	m_tireForce.moment += Vcross(m_data[id].contact_point - m_tireForce.point, Ft);

  }
}



} // end namespace chrono
