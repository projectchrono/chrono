//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "physics/ChLinkLinActuator.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkLinActuator> a_registration_ChLinkLinActuator;


ChLinkLinActuator::ChLinkLinActuator ()
{
  type = LNK_LINACTUATOR;     // initializes type

  dist_funct = ChSharedPtr<ChFunction>(new ChFunction_Const(0));
  mot_torque = ChSharedPtr<ChFunction>(new ChFunction_Recorder());
  mot_rot = ChSharedPtr<ChFunction>(new ChFunction_Recorder());

  learn = FALSE;
  offset = 0.1;

  mot_tau = 1.0;
  mot_eta = 1.0;
  mot_inertia = 0.0;

  // Mask: initialize our LinkMaskLF (lock formulation mask)
  // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
  ((ChLinkMaskLF*)mask)->SetLockMask(true, false, false,
                                     false, false, false, false);

  ChangedLinkMask();

  mot_rerot = mot_rerot_dt = mot_rerot_dtdt = 0;
}

ChLinkLinActuator::~ChLinkLinActuator ()
{
}

void ChLinkLinActuator::Copy(ChLinkLinActuator* source)
{
  // first copy the parent class data...
  ChLinkLock::Copy(source);

  // copy custom data:
  learn = source->learn;
  offset = source->offset;

  dist_funct = ChSharedPtr<ChFunction>(source->dist_funct->new_Duplicate());
  mot_torque = ChSharedPtr<ChFunction>(source->mot_torque->new_Duplicate());
  mot_rot = ChSharedPtr<ChFunction>(source->mot_rot->new_Duplicate());

  mot_tau = source->mot_tau;
  mot_eta = source->mot_eta;
  mot_inertia = source->mot_inertia;
}

ChLink* ChLinkLinActuator::new_Duplicate ()
{
  ChLinkLinActuator* m_l;
  m_l = new ChLinkLinActuator;  // inherited classes should write here: m_l = new MyInheritedLink;
  m_l->Copy(this);
  return (m_l);
}

void ChLinkLinActuator::Set_learn(int mset)
{
  if (mset)
    SetDisabled(true);  // ..just to show it as a green wireframe...
  else
    SetDisabled(false);

  if (mset)
    ((ChLinkMaskLF*)mask)->Constr_X().SetMode(CONSTRAINT_FREE);
  else
    ((ChLinkMaskLF*)mask)->Constr_X().SetMode(CONSTRAINT_LOCK);

  ChangedLinkMask();

  learn = mset;
  if (dist_funct->Get_Type() != FUNCT_RECORDER)
    dist_funct = ChSharedPtr<ChFunction>(new ChFunction_Recorder);
}

void ChLinkLinActuator::UpdateTime (double mytime)
{
  // First, inherit to parent class
  ChLinkLock::UpdateTime(mytime);

  // If LEARN MODE, just record motion
  if (learn == TRUE)
  {
    /*   do not change deltas, in free mode maybe that 'limit on X' changed them
    deltaC.pos = VNULL;
    deltaC_dt.pos = VNULL;
    deltaC_dtdt.pos = VNULL;
    deltaC.rot = QUNIT;
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
    */
    if (dist_funct->Get_Type() != FUNCT_RECORDER)
      dist_funct = ChSharedPtr<ChFunction>(new ChFunction_Recorder);

    // record point
    double rec_dist = Vlength(Vsub(marker1->GetAbsCoord().pos,
      marker2->GetAbsCoord().pos));
    rec_dist -= offset;
    dist_funct.StaticCastTo<ChFunction_Recorder>()->AddPoint(mytime, rec_dist, 1);  // (x,y,w)  x=t
  }

  // Move (well, rotate...) marker 2 to align it in actuator direction

  // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
  marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

  ChMatrix33<> ma;
  ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

  Vector absdist = Vsub(marker1->GetAbsCoord().pos,
    marker2->GetAbsCoord().pos);

  Vector mx = Vnorm(absdist);

  Vector my = ma.Get_A_Yaxis();
  if (Vequal(mx, my))
  {
    if (mx.x == 1.0)   my = VECT_Y;
    else                my = VECT_X;
  }
  Vector mz = Vnorm(Vcross(mx, my));
  my = Vnorm(Vcross(mz, mx));

  ma.Set_A_axis(mx, my, mz);

  Coordsys newmarkpos;
  ChVector<> oldpos = marker2->GetPos(); // backup to avoid numerical err.accumulation
  newmarkpos.pos = marker2->GetAbsCoord().pos;
  newmarkpos.rot = ma.Get_A_quaternion();
  marker2->Impose_Abs_Coord(newmarkpos);        //rotate "main" marker2 into tangent position (may add err.accumulation)
  marker2->SetPos(oldpos); // backup to avoid numerical err.accumulation

  if (learn == TRUE) return; // no need to go on further...--->>>>

  // imposed relative positions/speeds
  deltaC.pos = VNULL;
  deltaC.pos.x = dist_funct->Get_y(ChTime) + this->offset;        // distance is always on M2 'X' axis

  deltaC_dt.pos = VNULL;
  deltaC_dt.pos.x = dist_funct->Get_y_dx(ChTime); // distance speed

  deltaC_dtdt.pos = VNULL;
  deltaC_dtdt.pos.x = dist_funct->Get_y_dxdx(ChTime); // distance acceleration
  // add also the centripetal acceleration if distance vector's rotating,
  // as centripetal acc. of point sliding on a sphere surface.
  Vector tang_speed = GetRelM_dt().pos;
  tang_speed.x = 0; // only z-y coords in relative tang speed vector
  double len_absdist = Vlength(absdist);  // don't divide by zero
  if(len_absdist > 1E-6)
    deltaC_dtdt.pos.x -= pow(Vlength(tang_speed), 2) / Vlength(absdist);  // An = Adelta -(Vt^2 / r)

  deltaC.rot = QUNIT;             // no relative rotations imposed!
  deltaC_dt.rot = QNULL;
  deltaC_dtdt.rot = QNULL;

  // Compute motor variables
  //double m_rotation;
  //double m_torque;
  mot_rerot = (deltaC.pos.x - this->offset) / mot_tau;
  mot_rerot_dt = deltaC_dt.pos.x / mot_tau;
  mot_rerot_dtdt = deltaC_dtdt.pos.x / mot_tau;
  mot_retorque = mot_rerot_dtdt * mot_inertia + (this->react_force.x * mot_tau) / mot_eta;
  //  m_rotation = (deltaC.pos.x - this->offset) / mot_tau;
  //  m_torque =  (deltaC_dtdt.pos.x / mot_tau) * mot_inertia + (this->react_force.x * mot_tau) / mot_eta;

  if (mot_torque->Get_Type() != FUNCT_RECORDER)
    mot_torque = ChSharedPtr<ChFunction>(new ChFunction_Recorder);

  if (mot_rot->Get_Type() != FUNCT_RECORDER)
    mot_rot = ChSharedPtr<ChFunction>(new ChFunction_Recorder);

  mot_torque.StaticCastTo<ChFunction_Recorder>()->AddPoint(mytime, mot_retorque, 1);  // (x,y,w)  x=t
  mot_rot.StaticCastTo<ChFunction_Recorder>()->AddPoint(mytime, mot_rerot, 1);        // (x,y,w)  x=t
}


void ChLinkLinActuator::StreamOUT(ChStreamOutBinary& mstream)
{
  // class version number
  mstream.VersionWrite(1);
  // serialize parent class too
  ChLinkLock::StreamOUT(mstream);

  // stream out all member data
  mstream << learn;
  mstream << offset;
  mstream.AbstractWrite(dist_funct.get_ptr());
  mstream << mot_tau;
  mstream << mot_eta;
  mstream << mot_inertia;
  mstream.AbstractWrite(mot_rot.get_ptr());
  mstream.AbstractWrite(mot_torque.get_ptr());
}

void ChLinkLinActuator::StreamIN(ChStreamInBinary& mstream)
{
  // class version number
  int version = mstream.VersionRead();
  // deserialize parent class too
  ChLinkLock::StreamIN(mstream);

  // stream in all member data
  ChFunction* newfun;
  mstream >> learn;
  mstream >> offset;
  mstream.AbstractReadCreate(&newfun);      dist_funct = ChSharedPtr<ChFunction>(newfun);
  mstream >> mot_tau;
  mstream >> mot_eta;
  mstream >> mot_inertia;
  mstream.AbstractReadCreate(&newfun);      mot_rot = ChSharedPtr<ChFunction>(newfun);
  mstream.AbstractReadCreate(&newfun);      mot_torque = ChSharedPtr<ChFunction>(newfun);
}



} // END_OF_NAMESPACE____


