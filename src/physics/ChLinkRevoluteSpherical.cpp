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


#include "physics/ChLinkRevoluteSpherical.h"

#include "core/ChMemory.h"


namespace chrono {


// Register into the object factory.
ChClassRegister<ChLinkRevoluteSpherical> a_registration_ChLinkRevoluteSpherical;


// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
ChLinkRevoluteSpherical::ChLinkRevoluteSpherical()
  : m_pos1(ChVector<>(0, 0, 0)),
  m_pos2(ChVector<>(0, 0, 0)),
  m_dir1(ChVector<>(0, 0, 1)),
  m_dist(0),
  m_cur_dist(0),
  m_cur_dot(0)
{
  m_cache_speed[0] = 0;
  m_cache_speed[1] = 0;

  m_cache_pos[0] = 0;
  m_cache_pos[1] = 0;
}

void ChLinkRevoluteSpherical::Copy(ChLinkRevoluteSpherical* source)
{
  // first copy the parent class data...
  ChLink::Copy(source);

  Body1 = source->Body1;
  Body2 = source->Body2;
  system = source->system;

  m_pos1 = source->m_pos1;
  m_pos2 = source->m_pos2;
  m_dir1 = source->m_dir1;
  m_dist = source->m_dist;
  m_cur_dist = source->m_cur_dist;
  m_cur_dot = source->m_cur_dot;

  m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

  m_cache_speed[0] = source->m_cache_speed[0];
  m_cache_speed[1] = source->m_cache_speed[1];

  m_cache_pos[0] = source->m_cache_pos[0];
  m_cache_pos[1] = source->m_cache_pos[1];
}

ChLink* ChLinkRevoluteSpherical::new_Duplicate()
{
  ChLinkRevoluteSpherical* link = new ChLinkRevoluteSpherical;
  link->Copy(this);
  return(link);
}


// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Initialize(
  ChSharedPtr<ChBodyFrame> body1,           // first frame (revolute side)
  ChSharedPtr<ChBodyFrame> body2,           // second frame (spherical side)
  const ChCoordsys<>&      csys,            // joint coordinate system (in absolute frame)
  double                   distance)        // imposed distance
{
  Body1 = body1.get_ptr();
  Body2 = body2.get_ptr();

  m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

  ChVector<> x_Axis = csys.rot.GetXaxis();
  ChVector<> z_axis = csys.rot.GetZaxis();

  m_pos1 = Body1->TransformPointParentToLocal(csys.pos);
  m_dir1 = Body1->TransformDirectionParentToLocal(z_axis);
  m_pos2 = Body2->TransformPointParentToLocal(csys.pos + distance * x_Axis);

  m_dist = distance;
  m_cur_dist = distance;
  m_cur_dot = 0;
}

void ChLinkRevoluteSpherical::Initialize(
  ChSharedPtr<ChBodyFrame> body1,           // first frame (revolute side)
  ChSharedPtr<ChBodyFrame> body2,           // second frame (spherical side)
  bool                     local,           // true if data given in body local frames
  const ChVector<>&        pos1,            // point on first frame
  const ChVector<>&        dir1,            // direction of revolute on first frame
  const ChVector<>&        pos2,            // point on second frame
  bool                     auto_distance,   // true if imposed distance equal to |pos1 - po2|
  double                   distance)        // imposed distance (used only if auto_distance = false)
{
  Body1 = body1.get_ptr();
  Body2 = body2.get_ptr();

  m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());
  m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

  ChVector<> pos1_abs;
  ChVector<> pos2_abs;
  ChVector<> dir1_abs;

  if (local) {
    m_pos1 = pos1;
    m_pos2 = pos2;
    m_dir1 = dir1;
    pos1_abs = Body1->TransformPointLocalToParent(pos1);
    pos2_abs = Body2->TransformPointLocalToParent(pos2);
    dir1_abs = Body1->TransformDirectionLocalToParent(dir1);
  }
  else {
    m_pos1 = Body1->TransformPointParentToLocal(pos1);
    m_pos2 = Body2->TransformPointParentToLocal(pos2);
    m_dir1 = Body1->TransformDirectionParentToLocal(dir1);
    pos1_abs = pos1;
    pos2_abs = pos2;
    dir1_abs = dir1;
  }

  ChVector<> d12_abs = pos2_abs - pos1_abs;

  m_cur_dist = d12_abs.Length();
  m_dist = auto_distance ? m_cur_dist : distance;

  m_cur_dot = Vdot(d12_abs, dir1_abs);
}


// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Update(double time)
{
  // Inherit time changes of parent class (ChLink)
  ChLink::UpdateTime(time);

  // Express the body locations and direction in absolute frame
  ChVector<> pos1_abs = Body1->TransformPointLocalToParent(m_pos1);
  ChVector<> pos2_abs = Body2->TransformPointLocalToParent(m_pos2);
  ChVector<> dir1_abs = Body1->TransformDirectionLocalToParent(m_dir1);
  ChVector<> d12_abs = pos2_abs - pos1_abs;

  // Update current distance and dot product
  m_cur_dist = d12_abs.Length();
  m_cur_dot = Vdot(d12_abs, dir1_abs);

  // Calculate a unit vector in the direction d12, expressed in absolute frame
  // Then express it in the two body frames
  ChVector<> u12_abs = d12_abs / m_cur_dist;
  ChVector<> u12_loc1 = Body1->TransformDirectionParentToLocal(u12_abs);
  ChVector<> u12_loc2 = Body2->TransformDirectionParentToLocal(u12_abs);

  // Express the direction vector in the frame of body 2
  ChVector<> dir1_loc2 = Body2->TransformDirectionParentToLocal(dir1_abs);

  // Compute Jacobian of the distance constraint
  //    ||pos2_abs - pos1_abs|| - dist = 0
  {
    ChVector<> Phi_r1 = -u12_abs;
    ChVector<> Phi_pi1 = Vcross(u12_loc1, m_pos1);

    m_cnstr_dist.Get_Cq_a()->ElementN(0) = (float)Phi_r1.x;
    m_cnstr_dist.Get_Cq_a()->ElementN(1) = (float)Phi_r1.y;
    m_cnstr_dist.Get_Cq_a()->ElementN(2) = (float)Phi_r1.z;

    m_cnstr_dist.Get_Cq_a()->ElementN(3) = (float)Phi_pi1.x;
    m_cnstr_dist.Get_Cq_a()->ElementN(4) = (float)Phi_pi1.y;
    m_cnstr_dist.Get_Cq_a()->ElementN(5) = (float)Phi_pi1.z;

    ChVector<> Phi_r2 = u12_abs;
    ChVector<> Phi_pi2 = -Vcross(u12_loc2, m_pos2);

    m_cnstr_dist.Get_Cq_b()->ElementN(0) = (float)Phi_r2.x;
    m_cnstr_dist.Get_Cq_b()->ElementN(1) = (float)Phi_r2.y;
    m_cnstr_dist.Get_Cq_b()->ElementN(2) = (float)Phi_r2.z;

    m_cnstr_dist.Get_Cq_b()->ElementN(3) = (float)Phi_pi2.x;
    m_cnstr_dist.Get_Cq_b()->ElementN(4) = (float)Phi_pi2.y;
    m_cnstr_dist.Get_Cq_b()->ElementN(5) = (float)Phi_pi2.z;
  }

  // Compute Jacobian of the dot constraint
  //    dot(dir1_abs, pos2_abs - pos1_abs) = 0
    {
      ChVector<> Phi_r1 = -dir1_abs;
      ChVector<> Phi_pi1 = Vcross(m_dir1, m_pos1) - Vcross(u12_loc1, m_pos1);

      m_cnstr_dot.Get_Cq_a()->ElementN(0) = (float)Phi_r1.x;
      m_cnstr_dot.Get_Cq_a()->ElementN(1) = (float)Phi_r1.y;
      m_cnstr_dot.Get_Cq_a()->ElementN(2) = (float)Phi_r1.z;

      m_cnstr_dot.Get_Cq_a()->ElementN(3) = (float)Phi_pi1.x;
      m_cnstr_dot.Get_Cq_a()->ElementN(4) = (float)Phi_pi1.y;
      m_cnstr_dot.Get_Cq_a()->ElementN(5) = (float)Phi_pi1.z;

      ChVector<> Phi_r2 = dir1_abs;
      ChVector<> Phi_pi2 = -Vcross(dir1_loc2, m_pos2);

      m_cnstr_dot.Get_Cq_b()->ElementN(0) = (float)Phi_r2.x;
      m_cnstr_dot.Get_Cq_b()->ElementN(1) = (float)Phi_r2.y;
      m_cnstr_dot.Get_Cq_b()->ElementN(2) = (float)Phi_r2.z;

      m_cnstr_dot.Get_Cq_b()->ElementN(3) = (float)Phi_pi2.x;
      m_cnstr_dot.Get_Cq_b()->ElementN(4) = (float)Phi_pi2.y;
      m_cnstr_dot.Get_Cq_b()->ElementN(5) = (float)Phi_pi2.z;
    }
}


// -----------------------------------------------------------------------------
// Implementation of solver interface functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::InjectConstraints(ChLcpSystemDescriptor& descriptor)
{
  if (!IsActive())
    return;

  descriptor.InsertConstraint(&m_cnstr_dist);
  descriptor.InsertConstraint(&m_cnstr_dot);
}

void ChLinkRevoluteSpherical::ConstraintsBiReset()
{
  m_cnstr_dist.Set_b_i(0.0);
  m_cnstr_dot.Set_b_i(0.0);
}

void ChLinkRevoluteSpherical::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
  if (!IsActive())
    return;

  double cnstr_dist_violation = do_clamp ?
    ChMin(ChMax(factor * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp) :
    factor * (m_cur_dist - m_dist);

  double cnstr_dot_violation = do_clamp ?
    ChMin(ChMax(factor * m_cur_dot, -recovery_clamp), recovery_clamp) :
    factor * m_cur_dot;

  m_cnstr_dist.Set_b_i(m_cnstr_dist.Get_b_i() + cnstr_dist_violation);
  m_cnstr_dot.Set_b_i(m_cnstr_dot.Get_b_i() + cnstr_dot_violation);
}

void ChLinkRevoluteSpherical::ConstraintsLoadJacobians()
{
  // Nothing to do here. Jacobians were loaded in Update().
}


void ChLinkRevoluteSpherical::ConstraintsFetch_react(double factor)
{
  //// TODO
}


// -----------------------------------------------------------------------------
// Load and store multipliers (caching to allow warm starting)
// -----------------------------------------------------------------------------
void  ChLinkRevoluteSpherical::ConstraintsLiLoadSuggestedSpeedSolution()
{
  // Set multipliers to those cached at previous step.
  m_cnstr_dist.Set_l_i(m_cache_speed[0]);
  m_cnstr_dot.Set_l_i(m_cache_speed[1]);
}

void  ChLinkRevoluteSpherical::ConstraintsLiLoadSuggestedPositionSolution()
{
  // Set multipliers to those cached at previous step.
  m_cnstr_dist.Set_l_i(m_cache_pos[0]);
  m_cnstr_dot.Set_l_i(m_cache_pos[1]);
}

void  ChLinkRevoluteSpherical::ConstraintsLiFetchSuggestedSpeedSolution()
{
  // Cache current multipliers.
  m_cache_speed[0] = m_cnstr_dist.Get_l_i();
  m_cache_speed[1] = m_cnstr_dot.Get_l_i();
}

void  ChLinkRevoluteSpherical::ConstraintsLiFetchSuggestedPositionSolution()
{
  // Cache current multipliers.
  m_cache_pos[0] = m_cnstr_dist.Get_l_i();
  m_cache_pos[1] = m_cnstr_dot.Get_l_i();
}


} // END_OF_NAMESPACE____


