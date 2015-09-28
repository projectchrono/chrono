// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Template for a tire model based on the MSC ADAMS Transient Fiala Tire Model
//
// Ref: Adams/Tire help - Adams 2014.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC10645&cat=2014_ADAMS_DOCS&actp=LIST
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT - DO NOT USE
// =============================================================================
// =============================================================================

#include <algorithm>
#include <cmath>

#include "physics/ChGlobal.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"

#include "chrono_vehicle/tire/ChFialaTire.h"


namespace chrono {


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChFialaTire::ChFialaTire(const std::string& name,
                         const ChTerrain&   terrain)
: ChTire(name, terrain),
  m_stepsize(1e-6)
{
  m_tireforce.force = ChVector<>(0, 0, 0);
  m_tireforce.point = ChVector<>(0, 0, 0);
  m_tireforce.moment = ChVector<>(0, 0, 0);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Initialize()
{
  SetFialaParams();

  //Initialize contact patach state variables to 0;
  m_states.cp_long_slip = 0;
  m_states.cp_side_slip = 0;

}

void ChFialaTire::Initialize(ChSharedPtr<ChBody> wheel)
{
  // Perform the actual initialization
  Initialize();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Update(double               time,
                         const ChWheelState&  wheel_state)
{
  ChCoordsys<> contact_frame;
  // Clear the force accumulators and set the application point to the wheel
  // center.
  m_tireforce.force = ChVector<>(0, 0, 0);
  m_tireforce.moment = ChVector<>(0, 0, 0);
  m_tireforce.point = wheel_state.pos;

  // Extract the wheel normal (expressed in global frame)
  ChMatrix33<> A(wheel_state.rot);
  ChVector<> disc_normal = A.Get_A_Yaxis();

  //Assuuming the tire is a disc, check contact with terrain
  m_data.in_contact = disc_terrain_contact(wheel_state.pos, disc_normal, m_unloaded_radius,
                                           m_data.frame, m_data.depth);
  if (m_data.in_contact) {
    // Wheel velocity in the ISO-C Frame 
    ChVector<> vel = wheel_state.lin_vel;
    m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

    // Generate normal contact force (recall, all forces are reduced to the wheel
    // center). If the resulting force is negative, the disc is moving away from 
    // the terrain so fast that no contact force is generated.
    double Fn_mag = getNormalStiffness(m_data.depth) * m_data.depth - getNormalDamping(m_data.depth) * m_data.vel.z;

    if (Fn_mag < 0) {  
      Fn_mag = 0;
    }

    m_data.normal_force = Fn_mag;
    m_states.abs_vx     = std::abs(m_data.vel.x);
    m_states.vsx        = m_data.vel.x-wheel_state.omega*(m_unloaded_radius- m_data.depth);
    m_states.vsy        = m_data.vel.y;
    m_states.omega      = wheel_state.omega;
    m_states.disc_normal = disc_normal;
  }
  else {
    //Reset all states if the tire comes off the ground.
    m_data.normal_force   = 0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
    m_states.abs_vx       = 0;
    m_states.vsx          = 0;
    m_states.vsy          = 0;
    m_states.omega        = 0;
    m_states.disc_normal  = ChVector<>(0, 0, 0);
  }


  //For Debugging:
  //if (m_states.abs_vx != 0) {
  //  std::cout << "Time:" << time << std::endl;
  //  std::cout << "Simple Kappa:" << -m_states.vsx/m_data.vel.x << " Simple Alpha:" << m_states.vsy/ m_data.vel.x << std::endl;
  //  std::cout << "Tire States: Normal Force:" << m_data.normal_force << std::endl;
  //  std::cout << "Tire States: abs_vx:" << m_states.abs_vx << std::endl;
  //  std::cout << "Tire States: vsx:" << m_states.vsx << std::endl;
  //  std::cout << "Tire States: vsy:" << m_states.vsy << std::endl;
  //  std::cout << "Tire States: omega:" << m_states.omega << std::endl;
  //  std::cout << "Wheel States" << std::endl
  //    << wheel_state.pos.x << ", "
  //    << wheel_state.pos.y << ", "
  //    << wheel_state.pos.z << std::endl
  //    << wheel_state.lin_vel.x << ", "
  //    << wheel_state.lin_vel.y << ", "
  //    << wheel_state.lin_vel.z << std::endl
  //    << wheel_state.ang_vel.x << ", "
  //    << wheel_state.ang_vel.y << ", "
  //    << wheel_state.ang_vel.z << ", "
  //    << wheel_state.omega << std::endl;
  //}
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Advance(double step)
{
  if (m_data.in_contact) {
    //integrate contact patch states using trapezoidal rule intergration
    //ref: https://en.wikipedia.org/wiki/Trapezoidal_rule_(differential_equations)

    // Take as many integration steps as needed to reach the value 'step'
    double t = 0;
    while (t < step) {
      // Ensure we integrate exactly to 'step'
      double h = std::min<>(m_stepsize, step - t);

      // Advance state for longitudinal direction
      m_states.cp_long_slip = ((2*m_relax_length_x - h*m_states.abs_vx)*m_states.cp_long_slip - 2*h*m_states.vsx) /
                              (2*m_relax_length_x + h*m_states.abs_vx);

      // Advance state for lateral direction
      m_states.cp_side_slip = ((2*m_relax_length_y - h*m_states.abs_vx)*m_states.cp_side_slip + 2*h*m_states.vsy) /
                              (2*m_relax_length_y + h*m_states.abs_vx);

      //Ensure that cp_lon_slip stays between -1 & 1
      m_states.cp_long_slip = std::max<>(-1.,std::min<>(1., m_states.cp_long_slip));

      //Ensure that cp_side_slip stays between -pi()/2 & pi()/2
      m_states.cp_side_slip = std::max<>(-CH_C_PI_2,std::min<>(CH_C_PI_2, m_states.cp_side_slip));

      t += h;
    }

    ////Overwrite with steady-state alpha & kappa for debugging
    //if (m_states.abs_vx != 0) {
    //  m_states.cp_long_slip = -m_states.vsx / m_states.abs_vx;
    //  m_states.cp_side_slip = std::atan2(m_states.vsy , m_states.abs_vx);
    //}
    //else {
    //  m_states.cp_long_slip = 0;
    //  m_states.cp_side_slip = 0;
    //}

    //For debugging:
    //std::cout<< "Tire States - K: "<< m_states.cp_long_slip << " A: " << m_states.cp_side_slip << std::endl << std::endl;

    //Now calculate the new force and moment values (normal force and moment has already been accounted for in Update())
    //See reference for more detail on the calculations
    double SsA = std::sqrt(std::pow(m_states.cp_long_slip,2)+std::pow(std::tan(m_states.cp_side_slip),2));
    double U =   m_u_max-(m_u_max-m_u_min)*SsA;
    double S_critical = std::abs(U*m_data.normal_force/(2*m_c_slip));
    double Alpha_critical = std::atan(3*U*std::abs(m_data.normal_force)/m_c_alpha);
    double Fx;
    double Fy;
    double My;
    double Mz;

    //Vertical Force:
    ChVector<> Fn = m_data.normal_force * m_data.frame.rot.GetZaxis();
    m_tireforce.force += Fn;

    //Longitudinal Force:
    if(std::abs(m_states.cp_long_slip)<S_critical){
      Fx = m_c_slip*m_states.cp_long_slip;
    }
    else{
      double Fx1 = -U*m_data.normal_force;
      double Fx2 = std::abs(std::pow((U*m_data.normal_force),2)/(4*std::abs(m_states.cp_long_slip)*m_c_slip));
      Fx = -sgn(m_states.cp_long_slip)*(Fx1-Fx2);
    }
    
    //Lateral Force & Aligning Moment (Mz):
    if(std::abs(m_states.cp_side_slip)<=Alpha_critical){
      double H = 1-m_c_alpha*std::abs(std::tan(m_states.cp_side_slip))/(3*U*m_data.normal_force);

      Fy = -U*m_data.normal_force*(1-std::pow(H,3))*sgn(m_states.cp_side_slip);
      Mz = U*m_data.normal_force*m_width*(1-H)*std::pow(H,3)*sgn(m_states.cp_side_slip);
    }
    else {
      Fy = -U*m_data.normal_force*sgn(m_states.cp_side_slip);
      Mz = 0;
    }
    
    //Rolling Resistance
    My = -m_rolling_resistance*m_data.normal_force*sgn(m_states.omega);
    
    //Debugging - trying the calculation the same way as the Simulink version...needs to ISO section as well
    m_tireforce.force = ChVector<>(Fx, Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(0, My, Mz);

    //Rotate into global coordinates
    m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
    m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

    //Move the tire forces from the contact patch to the wheel center
    m_tireforce.moment += Vcross((m_data.frame.pos + ChVector<>(0, 0, m_data.depth)) - m_tireforce.point, m_tireforce.force);

  }
  //Else do nothing since the "m_tireForce" force and moment values are already 0 (set in Update())

}



} // end namespace chrono
