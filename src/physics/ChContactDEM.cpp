//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include <cmath>
#include <algorithm>

#include "physics/ChContactDEM.h"
#include "physics/ChSystemDEM.h"
#include "physics/ChBodyDEM.h"

// Memory leak debugger (must be included last).
#include "core/ChMemory.h"


namespace chrono {

using namespace collision;


// Initialize static members
double ChContactDEM::m_minSlipVelocity = 1e-4;
double ChContactDEM::m_characteristicVelocity = 1;


// Construct a new DEM contact between two models using the specified contact pair information.
ChContactDEM::ChContactDEM(collision::ChModelBulletBody*     mod1,
                           collision::ChModelBulletBody*     mod2,
                           const collision::ChCollisionInfo& cinfo)
{
  Reset(mod1, mod2, cinfo);
}


// Worker function for populating a new or reusing an exsting contact.
void ChContactDEM::Reset(collision::ChModelBulletBody*     mod1,
                         collision::ChModelBulletBody*     mod2,
                         const collision::ChCollisionInfo& cinfo)
{
  assert(cinfo.distance < 0);

  m_mod1 = mod1;
  m_mod2 = mod2;

  ChBodyDEM* body1 = (ChBodyDEM*)m_mod1->GetBody();
  ChBodyDEM* body2 = (ChBodyDEM*)m_mod2->GetBody();

  // Contact points, penetration, normal
  m_p1 = cinfo.vpA;
  m_p2 = cinfo.vpB;
  m_delta = -cinfo.distance;
  m_normal = cinfo.vN;

  // Contact plane
  ChVector<> Vx, Vy, Vz;
  cinfo.vN.DirToDxDyDz(Vx, Vy, Vz);
  m_contact_plane.Set_A_axis(Vx, Vy, Vz);

  // Contact points in local frames
  m_p1_loc = body1->Point_World2Body(m_p1);
  m_p2_loc = body2->Point_World2Body(m_p2);

  // Calculate contact force
  CalculateForce();
}


// Calculate the contact force to be applied to body2, based on the
// globally specified contact force model.
void ChContactDEM::CalculateForce()
{
  ChBodyDEM* body1 = (ChBodyDEM*)m_mod1->GetBody();
  ChBodyDEM* body2 = (ChBodyDEM*)m_mod2->GetBody();

  ChSystemDEM* sys = static_cast<ChSystemDEM*>(body1->GetSystem());

  double dT = sys->GetStep();
  bool use_mat_props = sys->UseMaterialProperties();
  ContactForceModel force_model = sys->GetContactForceModel();

  // Relative velocity at contact
  ChVector<> vel2 = body2->PointSpeedLocalToParent(m_p2_loc);
  ChVector<> vel1 = body1->PointSpeedLocalToParent(m_p1_loc);
  ChVector<> relvel = vel2 - vel1;
  double     relvel_n_mag = relvel.Dot(m_normal);
  ChVector<> relvel_n = relvel_n_mag * m_normal;
  ChVector<> relvel_t = relvel - relvel_n;
  double     relvel_t_mag = relvel_t.Length();

  // Calculate effective mass
  double m_eff = body1->GetMass() * body2->GetMass() / (body1->GetMass() + body2->GetMass());

  // Calculate effective contact radius
  //// TODO:  how can I get this with current collision system!?!?!?
  double R_eff = 1;

  // Calculate composite material properties
  ChCompositeMaterialDEM mat = ChMaterialSurfaceDEM::CompositeMaterial(body1->GetMaterialSurfaceDEM(), body2->GetMaterialSurfaceDEM());

  // Contact forces.
  // All models use the following formulas for normal and tangential forces:
  //     Fn = kn * delta_n - gn * v_n
  //     Ft = kt * delta_t - gt * v_t
  double kn;
  double kt;
  double gn;
  double gt;

  double delta_t = 0;

  switch (force_model) {
  case Hooke:
    if (use_mat_props) {
      double tmp_k = (16.0 / 15) * std::sqrt(R_eff) * mat.E_eff;
      double v2 = m_characteristicVelocity * m_characteristicVelocity;
      double tmp_g = 1 + pow(CH_C_PI / std::log(mat.cr_eff) , 2);
      kn = tmp_k * std::pow(m_eff * v2 / tmp_k, 1.0 / 5);
      kt = 0;
      gn = std::sqrt(4 * m_eff * kn / tmp_g);
      gt = gn / 2;
    } else {

    }

    break;

  case Hooke_history:
    if (use_mat_props) {
      double tmp_k = (16.0 / 15) * std::sqrt(R_eff) * mat.E_eff;
      double v2 = m_characteristicVelocity * m_characteristicVelocity;
      double tmp_g = 1 + pow(CH_C_PI / std::log(mat.cr_eff), 2);
      kn = tmp_k * std::pow(m_eff * v2 / tmp_k, 1.0 / 5);
      kt = 2 * (1 - mat.poisson_eff) / (2 - mat.poisson_eff) * kn;
      gn = std::sqrt(4 * m_eff * kn / tmp_g);
      gt = gn / 2;

      delta_t = relvel_t_mag * dT;
    } else {

    }

    break;

  case Hertz:
    if (use_mat_props) {
      double sqrt_Rd = std::sqrt(R_eff * m_delta);
      double Sn = 2 * mat.E_eff * sqrt_Rd;
      double St = 8 * mat.G_eff * sqrt_Rd;
      double loge = std::log(mat.cr_eff);
      double beta = loge / std::sqrt(loge * loge + CH_C_PI * CH_C_PI);

      kn = (2.0 / 3) * Sn;
      kt = 0;
      gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * m_eff);
      gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * m_eff);
    } else {

    }

    break;

  case Hertz_history:
    if (use_mat_props) {
      double sqrt_Rd = std::sqrt(R_eff * m_delta);
      double Sn = 2 * mat.E_eff * sqrt_Rd;
      double St = 8 * mat.G_eff * sqrt_Rd;
      double loge = std::log(mat.cr_eff);
      double beta = loge / std::sqrt(loge * loge + CH_C_PI * CH_C_PI);

      kn = (2.0 / 3) * Sn;
      kt = St;
      gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * m_eff);
      gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * m_eff);

      delta_t = relvel_t_mag * dT;
    } else {

    }

    break;
  }

  // Calculate the magnitudes of the normal and tangential contact forces
  double forceN = kn * m_delta - gn * relvel_n_mag;
  double forceT = kt * delta_t - gt * relvel_t_mag;

  // Coulomb law
  forceT = std::min<double>(forceT, mat.mu_eff * std::abs(forceN));

  // Accumulate normal and tangential forces
  m_force = forceN * m_normal;
  if (relvel_t_mag >= m_minSlipVelocity)
    m_force += (forceT / relvel_t_mag) * relvel_t;
}


// Include the contact force from this contact into the body forces and
// torques of the two bodies involved in this contact.  Recall that the
// contact force as calculated must be applied to body2 and inverted for
// body1.
// NOTE: SEE NEW VERSION BELOW...
void ChContactDEM::ConstraintsFbLoadForces(double factor)
{
  ChBodyDEM* body1 = (ChBodyDEM*)m_mod1->GetBody();
  ChBodyDEM* body2 = (ChBodyDEM*)m_mod2->GetBody();

  ChVector<> force1_loc = body1->Dir_World2Body(m_force);
  ChVector<> force2_loc = body2->Dir_World2Body(m_force);
  ChVector<> torque1_loc = Vcross(m_p1_loc, -force1_loc);
  ChVector<> torque2_loc = Vcross(m_p2_loc, force2_loc);

  body1->Variables().Get_fb().PasteSumVector(-m_force*factor, 0, 0);
  body1->Variables().Get_fb().PasteSumVector(torque1_loc*factor, 3, 0);

  body2->Variables().Get_fb().PasteSumVector(m_force*factor, 0, 0);
  body2->Variables().Get_fb().PasteSumVector(torque2_loc*factor, 3, 0);
}

// Apply contact forces to bodies (new version, for interfacing to ChTimestepper and ChIntegrable)
// Replaces ConstraintsFbLoadForces.
// Include the contact force from this contact into the body forces and
// torques of the two bodies involved in this contact.  Recall that the
// contact force as calculated must be applied to body2 and inverted for
// body1.
void  ChContactDEM::DemIntLoadResidual_F(ChVectorDynamic<>& R, const double c )
{
  ChBodyDEM* body1 = (ChBodyDEM*)m_mod1->GetBody();
  ChBodyDEM* body2 = (ChBodyDEM*)m_mod2->GetBody();

  ChVector<> force1_loc = body1->Dir_World2Body(m_force);
  ChVector<> force2_loc = body2->Dir_World2Body(m_force);
  ChVector<> torque1_loc = Vcross(m_p1_loc, -force1_loc);
  ChVector<> torque2_loc = Vcross(m_p2_loc, force2_loc);

  R.PasteSumVector(-m_force*c, body1->Variables().GetOffset() + 0, 0);
  R.PasteSumVector(torque1_loc*c, body1->Variables().GetOffset() + 3, 0);

  R.PasteSumVector(m_force*c, body2->Variables().GetOffset() + 0, 0);
  R.PasteSumVector(torque2_loc*c, body2->Variables().GetOffset() + 3, 0);
}


// Return the coordinate system for this contact (centered at point P2
// and oriented based on the contact plane and normal)
ChCoordsys<> ChContactDEM::GetContactCoords() const
{
  ChCoordsys<> mcsys;
  ChQuaternion<float> mrot = m_contact_plane.Get_A_quaternion();

  mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
  mcsys.pos = m_p2;

  return mcsys;
}


} // end namespace chrono
