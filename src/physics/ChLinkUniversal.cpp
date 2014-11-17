//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "physics/ChLinkUniversal.h"

#include "core/ChMemory.h"


namespace chrono {


// Register into the object factory.
ChClassRegister<ChLinkUniversal> a_registration_ChLinkUniversal;


// -----------------------------------------------------------------------------
// Constructor and destructor
// -----------------------------------------------------------------------------
ChLinkUniversal::ChLinkUniversal()
{
  m_C = new ChMatrixDynamic<>(4, 1);

  m_cache_speed[0] = 0;
  m_cache_speed[1] = 0;
  m_cache_speed[2] = 0;
  m_cache_speed[3] = 0;

  m_cache_pos[0] = 0;
  m_cache_pos[1] = 0;
  m_cache_pos[2] = 0;
  m_cache_pos[3] = 0;
}

ChLinkUniversal::~ChLinkUniversal()
{
  delete m_C;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinkUniversal::Copy(ChLinkUniversal* source)
{
  ChLink::Copy(source);

  Body1 = source->Body1;
  Body2 = source->Body2;
  system = source->system;

  m_frame1 = source->m_frame1;
  m_frame2 = source->m_frame2;

  m_cnstr_x.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_y.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_z.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

  m_cache_speed[0] = source->m_cache_speed[0];
  m_cache_speed[1] = source->m_cache_speed[1];
  m_cache_speed[2] = source->m_cache_speed[2];
  m_cache_speed[3] = source->m_cache_speed[3];

  m_cache_pos[0] = source->m_cache_pos[0];
  m_cache_pos[1] = source->m_cache_pos[1];
  m_cache_pos[2] = source->m_cache_pos[2];
  m_cache_pos[3] = source->m_cache_pos[3];
}

ChLink* ChLinkUniversal::new_Duplicate()
{
  ChLinkUniversal* link = new ChLinkUniversal;
  link->Copy(this);
  return(link);
}


// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkUniversal::Initialize(ChSharedPtr<ChBodyFrame> body1,    // first body frame
                                 ChSharedPtr<ChBodyFrame> body2,    // second body frame
                                 const ChFrame<>&         frame)    // joint frame (in absolute frame)
{
  Body1 = body1.get_ptr();
  Body2 = body2.get_ptr();

  m_cnstr_x.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_y.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_z.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

  ((ChFrame<>*)Body1)->TransformParentToLocal(frame, m_frame1);
  ((ChFrame<>*)Body2)->TransformParentToLocal(frame, m_frame2);

  m_u1_tilde.Set_X_matrix(m_frame1.GetA()->Get_A_Xaxis());
  m_v2_tilde.Set_X_matrix(m_frame2.GetA()->Get_A_Yaxis());

  m_C->SetElement(0, 0, 0.0);
  m_C->SetElement(1, 0, 0.0);
  m_C->SetElement(2, 0, 0.0);
  m_C->SetElement(3, 0, 0.0);
}

void ChLinkUniversal::Initialize(ChSharedPtr<ChBodyFrame> body1,    // first body frame
                                 ChSharedPtr<ChBodyFrame> body2,    // second body frame
                                 bool                     local,    // true if data given in body local frames
                                 const ChFrame<>&         frame1,   // joint frame on body 1
                                 const ChFrame<>&         frame2)   // joint frame on body 2
{
  Body1 = body1.get_ptr();
  Body2 = body2.get_ptr();

  m_cnstr_x.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_y.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_z.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

  ChFrame<> frame1_abs;
  ChFrame<> frame2_abs;

  if (local) {
    m_frame1 = frame1;
    m_frame2 = frame2;
    frame1_abs = frame1 >> *Body1;
    frame2_abs = frame2 >> *Body2;
  }
  else {
    ((ChFrame<>*)Body1)->TransformParentToLocal(frame1, m_frame1);
    ((ChFrame<>*)Body2)->TransformParentToLocal(frame2, m_frame2);
    frame1_abs = frame1;
    frame2_abs = frame2;
  }

  m_u1_tilde.Set_X_matrix(m_frame1.GetA()->Get_A_Xaxis());
  m_v2_tilde.Set_X_matrix(m_frame2.GetA()->Get_A_Yaxis());

  m_C->SetElement(0, 0, frame2_abs.coord.pos.x - frame1_abs.coord.pos.x);
  m_C->SetElement(1, 0, frame2_abs.coord.pos.y - frame1_abs.coord.pos.y);
  m_C->SetElement(2, 0, frame2_abs.coord.pos.z - frame1_abs.coord.pos.z);
  m_C->SetElement(3, 0, Vdot(frame1_abs.GetA()->Get_A_Xaxis(), frame2_abs.GetA()->Get_A_Yaxis()));
}


// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkUniversal::Update(double time)
{
  // Inherit time changes of parent class
  ChLink::UpdateTime(time);

  // Express the joint frames in absolute frame
  ChFrame<> frame1_abs = m_frame1 >> *Body1;
  ChFrame<> frame2_abs = m_frame2 >> *Body2;

  // Calculate violations of the spherical constraints
  m_C->SetElement(0, 0, frame2_abs.coord.pos.x - frame1_abs.coord.pos.x);
  m_C->SetElement(1, 0, frame2_abs.coord.pos.y - frame1_abs.coord.pos.y);
  m_C->SetElement(2, 0, frame2_abs.coord.pos.z - frame1_abs.coord.pos.z);

  // Compute Jacobian of the spherical constraints
  //    pos2_abs - pos1_abs = 0
  {
    ChMatrix33<> tilde1;
    ChMatrix33<> tilde2;
    tilde1.Set_X_matrix(m_frame1.coord.pos);
    tilde2.Set_X_matrix(m_frame2.coord.pos);
    ChMatrix33<> Phi_pi1 = *(Body1->GetA()) * tilde1;
    ChMatrix33<> Phi_pi2 = *(Body2->GetA()) * tilde2;

    m_cnstr_x.Get_Cq_a()->ElementN(0) = -1;              m_cnstr_x.Get_Cq_b()->ElementN(0) = +1;
    m_cnstr_x.Get_Cq_a()->ElementN(1) = 0;               m_cnstr_x.Get_Cq_b()->ElementN(1) = 0;
    m_cnstr_x.Get_Cq_a()->ElementN(2) = 0;               m_cnstr_x.Get_Cq_b()->ElementN(2) = 0;
    m_cnstr_x.Get_Cq_a()->ElementN(3) = Phi_pi1(0, 0);    m_cnstr_x.Get_Cq_b()->ElementN(3) = -Phi_pi2(0, 0);
    m_cnstr_x.Get_Cq_a()->ElementN(4) = Phi_pi1(0, 1);    m_cnstr_x.Get_Cq_b()->ElementN(4) = -Phi_pi2(0, 1);
    m_cnstr_x.Get_Cq_a()->ElementN(5) = Phi_pi1(0, 2);    m_cnstr_x.Get_Cq_b()->ElementN(5) = -Phi_pi2(0, 2);

    m_cnstr_y.Get_Cq_a()->ElementN(0) = 0;               m_cnstr_y.Get_Cq_b()->ElementN(0) = 0;
    m_cnstr_y.Get_Cq_a()->ElementN(1) = -1;              m_cnstr_y.Get_Cq_b()->ElementN(1) = +1;
    m_cnstr_y.Get_Cq_a()->ElementN(2) = 0;               m_cnstr_y.Get_Cq_b()->ElementN(2) = 0;
    m_cnstr_y.Get_Cq_a()->ElementN(3) = Phi_pi1(1, 0);   m_cnstr_y.Get_Cq_b()->ElementN(3) = -Phi_pi2(1, 0);
    m_cnstr_y.Get_Cq_a()->ElementN(4) = Phi_pi1(1, 1);   m_cnstr_y.Get_Cq_b()->ElementN(4) = -Phi_pi2(1, 1);
    m_cnstr_y.Get_Cq_a()->ElementN(5) = Phi_pi1(1, 2);   m_cnstr_y.Get_Cq_b()->ElementN(5) = -Phi_pi2(1, 2);

    m_cnstr_z.Get_Cq_a()->ElementN(0) = 0;               m_cnstr_z.Get_Cq_b()->ElementN(0) = 0;
    m_cnstr_z.Get_Cq_a()->ElementN(1) = 0;               m_cnstr_z.Get_Cq_b()->ElementN(1) = 0;
    m_cnstr_z.Get_Cq_a()->ElementN(2) = -1;              m_cnstr_z.Get_Cq_b()->ElementN(2) = +1;
    m_cnstr_z.Get_Cq_a()->ElementN(3) = Phi_pi1(2, 0);   m_cnstr_z.Get_Cq_b()->ElementN(3) = -Phi_pi2(2, 0);
    m_cnstr_z.Get_Cq_a()->ElementN(4) = Phi_pi1(2, 1);   m_cnstr_z.Get_Cq_b()->ElementN(4) = -Phi_pi2(2, 1);
    m_cnstr_z.Get_Cq_a()->ElementN(5) = Phi_pi1(2, 2);   m_cnstr_z.Get_Cq_b()->ElementN(5) = -Phi_pi2(2, 2);
  }

  // Calculate violation of the dot constraint
  ChVector<> u1 = frame1_abs.GetA()->Get_A_Xaxis();
  ChVector<> v2 = frame2_abs.GetA()->Get_A_Yaxis();

  m_C->SetElement(3, 0, Vdot(u1, v2));

  // Compute Jacobian of the dot constraint
  //    dot(u1_abs, v2_abs) = 0
  {
    ChMatrix33<> mat1 = *Body1->GetA() * m_u1_tilde;
    ChMatrix33<> mat2 = *Body2->GetA() * m_v2_tilde;
    ChVector<> Phi_pi1 = mat1.MatrT_x_Vect(v2);
    ChVector<> Phi_pi2 = mat2.MatrT_x_Vect(u1);

    m_cnstr_dot.Get_Cq_a()->ElementN(0) = 0;
    m_cnstr_dot.Get_Cq_a()->ElementN(1) = 0;
    m_cnstr_dot.Get_Cq_a()->ElementN(2) = 0;
    m_cnstr_dot.Get_Cq_a()->ElementN(3) = -Phi_pi1.x;
    m_cnstr_dot.Get_Cq_a()->ElementN(4) = -Phi_pi1.y;
    m_cnstr_dot.Get_Cq_a()->ElementN(5) = -Phi_pi1.z;

    m_cnstr_dot.Get_Cq_b()->ElementN(0) = 0;
    m_cnstr_dot.Get_Cq_b()->ElementN(1) = 0;
    m_cnstr_dot.Get_Cq_b()->ElementN(2) = 0;
    m_cnstr_dot.Get_Cq_b()->ElementN(3) = -Phi_pi2.x;
    m_cnstr_dot.Get_Cq_b()->ElementN(4) = -Phi_pi2.y;
    m_cnstr_dot.Get_Cq_b()->ElementN(5) = -Phi_pi2.z;
  }
}


// -----------------------------------------------------------------------------
// Implementation of solver interface functions
// -----------------------------------------------------------------------------
void ChLinkUniversal::InjectConstraints(ChLcpSystemDescriptor& descriptor)
{
  if (!IsActive())
    return;

  descriptor.InsertConstraint(&m_cnstr_x);
  descriptor.InsertConstraint(&m_cnstr_y);
  descriptor.InsertConstraint(&m_cnstr_z);
  descriptor.InsertConstraint(&m_cnstr_dot);
}

void ChLinkUniversal::ConstraintsBiReset()
{
  m_cnstr_x.Set_b_i(0.0);
  m_cnstr_y.Set_b_i(0.0);
  m_cnstr_z.Set_b_i(0.0);
  m_cnstr_dot.Set_b_i(0.0);
}

void ChLinkUniversal::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
  if (!IsActive())
    return;

  double cnstr_x_violation = factor * m_C->GetElement(0, 0);
  double cnstr_y_violation = factor * m_C->GetElement(1, 0);
  double cnstr_z_violation = factor * m_C->GetElement(2, 0);
  double cnstr_dot_violation = factor * m_C->GetElement(3, 0);

  if (do_clamp) {
    cnstr_x_violation = ChMin(ChMax(cnstr_x_violation, -recovery_clamp), recovery_clamp);
    cnstr_y_violation = ChMin(ChMax(cnstr_y_violation, -recovery_clamp), recovery_clamp);
    cnstr_z_violation = ChMin(ChMax(cnstr_z_violation, -recovery_clamp), recovery_clamp);
    cnstr_dot_violation = ChMin(ChMax(cnstr_dot_violation, -recovery_clamp), recovery_clamp);
  }

  m_cnstr_x.Set_b_i(m_cnstr_x.Get_b_i() + cnstr_x_violation);
  m_cnstr_y.Set_b_i(m_cnstr_y.Get_b_i() + cnstr_y_violation);
  m_cnstr_z.Set_b_i(m_cnstr_z.Get_b_i() + cnstr_z_violation);
  m_cnstr_dot.Set_b_i(m_cnstr_dot.Get_b_i() + cnstr_dot_violation);
}

void ChLinkUniversal::ConstraintsLoadJacobians()
{
  // Nothing to do here. Jacobians were loaded in Update().
}


void ChLinkUniversal::ConstraintsFetch_react(double factor)
{
  //// TODO
  double l1 = m_cnstr_x.Get_l_i();
  double l2 = m_cnstr_y.Get_l_i();
  double l3 = m_cnstr_z.Get_l_i();
  double l4 = m_cnstr_dot.Get_l_i();
}


// -----------------------------------------------------------------------------
// Load and store multipliers (caching to allow warm starting)
// -----------------------------------------------------------------------------
void  ChLinkUniversal::ConstraintsLiLoadSuggestedSpeedSolution()
{
  // Set multipliers to those cached at previous step.
  m_cnstr_x.Set_l_i(m_cache_speed[0]);
  m_cnstr_y.Set_l_i(m_cache_speed[1]);
  m_cnstr_z.Set_l_i(m_cache_speed[2]);
  m_cnstr_dot.Set_l_i(m_cache_speed[3]);
}

void  ChLinkUniversal::ConstraintsLiLoadSuggestedPositionSolution()
{
  // Set multipliers to those cached at previous step.
  m_cnstr_x.Set_l_i(m_cache_pos[0]);
  m_cnstr_y.Set_l_i(m_cache_pos[1]);
  m_cnstr_z.Set_l_i(m_cache_pos[2]);
  m_cnstr_dot.Set_l_i(m_cache_pos[3]);
}

void  ChLinkUniversal::ConstraintsLiFetchSuggestedSpeedSolution()
{
  // Cache current multipliers.
  m_cache_speed[0] = m_cnstr_x.Get_l_i();
  m_cache_speed[1] = m_cnstr_y.Get_l_i();
  m_cache_speed[2] = m_cnstr_z.Get_l_i();
  m_cache_speed[3] = m_cnstr_dot.Get_l_i();
}

void  ChLinkUniversal::ConstraintsLiFetchSuggestedPositionSolution()
{
  // Cache current multipliers.
  m_cache_pos[0] = m_cnstr_x.Get_l_i();
  m_cache_pos[1] = m_cnstr_y.Get_l_i();
  m_cache_pos[2] = m_cnstr_z.Get_l_i();
  m_cache_pos[3] = m_cnstr_dot.Get_l_i();
}


} // END_OF_NAMESPACE____


