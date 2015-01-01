//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 Justin Madsen
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "physics/ChLinkRotSpring.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger)


namespace chrono
{
// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkRotSpring> a_registration_ChLinkRotSpring;


ChLinkRotSpring::ChLinkRotSpring ()
{
  spr_rest_rot = 0;
  spr_k = 100;
  spr_r = 5;
  spr_t = 0;

  mod_t_time = ChSharedPtr<ChFunction>(new ChFunction_Const(1));
  mod_k_d = ChSharedPtr<ChFunction>(new ChFunction_Const(1));
  mod_k_speed = ChSharedPtr<ChFunction>(new ChFunction_Const(1));
  mod_r_d = ChSharedPtr<ChFunction>(new ChFunction_Const(1));
  mod_r_speed = ChSharedPtr<ChFunction>(new ChFunction_Const(1));

  spr_react_T = 0.0;
}


ChLinkRotSpring::~ChLinkRotSpring ()
{
}


void ChLinkRotSpring::Copy(ChLinkRotSpring* source)
{
  // first copy the parent class data...
  ChLinkMarkers::Copy(source);

  // copy custom data:
  spr_rest_rot = source->spr_rest_rot;
  spr_t = source->spr_t;
  spr_k = source->spr_k;
  spr_r = source->spr_r;
  spr_react_T = source->spr_react_T;

  mod_t_time = ChSharedPtr<ChFunction>(source->mod_t_time->new_Duplicate());
  mod_k_d = ChSharedPtr<ChFunction>(source->mod_k_d->new_Duplicate());
  mod_k_speed = ChSharedPtr<ChFunction>(source->mod_k_speed->new_Duplicate());
  mod_r_d = ChSharedPtr<ChFunction>(source->mod_r_d->new_Duplicate());
  mod_r_speed = ChSharedPtr<ChFunction>(source->mod_r_speed->new_Duplicate());
}


ChLink* ChLinkRotSpring::new_Duplicate ()
{
  ChLinkRotSpring* m_l;
  m_l = new ChLinkRotSpring;  // inherited classes should write here: m_l = new MyInheritedLink;
  m_l->Copy(this);
  return (m_l);
}


void ChLinkRotSpring::Initialize(ChSharedPtr<ChBody> mbody1,           // first body to link
                                ChSharedPtr<ChBody> mbody2,           // second body to link
                                bool                pos_are_relative, // true: following posit. are considered relative to bodies. false: pos.are absolute
                                ChVector<>          mpos1,            // position of distance endpoint, for 1st body (rel. or abs., see flag above)
                                ChVector<>          mpos2,            // position of distance endpoint, for 2nd body (rel. or abs., see flag above) 
                                ChVector<>          m_rel_rot_axis,   // relative axis of rotation
                                bool                auto_rest_rot, 	  // if true, initializes the imposed rot as the current rotation angle between mpos1 and mpos2
                                double              rest_rot      	  // imposed rotation (no need to define, if auto_distance=true.)
                                )
{
  // initialize as a constraint with markers, create the two markers.
  ChLinkMarkers::Initialize(mbody1, mbody2, CSYSNORM);

  if (pos_are_relative)
  {
    marker1->Impose_Rel_Coord(ChCoordsys<>(mpos1, QUNIT));
    marker2->Impose_Rel_Coord(ChCoordsys<>(mpos2, QUNIT));
  }
  else
  {
    marker1->Impose_Abs_Coord(ChCoordsys<>(mpos1, QUNIT));
    marker2->Impose_Abs_Coord(ChCoordsys<>(mpos2, QUNIT));
  }

  // don't really need this
  ChVector<> AbsDist = marker1->GetAbsCoord().pos - marker2->GetAbsCoord().pos;
  dist = AbsDist.Length();

  // setup the axis of rotation
  relAxis = m_rel_rot_axis;

  // if specified, override the rest rotation angle
  spr_rest_rot = auto_rest_rot ? relAngle : rest_rot;
}


void ChLinkRotSpring::UpdateForces(double mytime)
{
  // Inherit force computation:
  // also base class can add its own forces.
  ChLinkMarkers::UpdateForces(mytime);

  spr_react_T = 0.0;
  Vector m_torque;
  double deform = Get_SpringDeform();

  spr_react_T = spr_t * mod_t_time->Get_y(ChTime);
  spr_react_T -= (spr_k * mod_k_d->Get_y(deform) * mod_k_speed->Get_y(Vdot(relWvel,relAxis))) * deform;
  spr_react_T -= (spr_r * mod_r_d->Get_y(deform) * mod_r_speed->Get_y( Vdot(relWvel,relAxis))) * Vdot(relWvel,relAxis);

  m_torque = relAxis * spr_react_T;

  C_torque = Vadd(C_torque, m_torque);
}


void ChLinkRotSpring::StreamOUT(ChStreamOutBinary& mstream)
{
  // Class version number
  mstream.VersionWrite(1);

  // Serialize parent class too
  ChLinkMarkers::StreamOUT(mstream);

  // Stream out all member data
  mstream << spr_rest_rot;
  mstream << spr_t;
  mstream << spr_k;
  mstream << spr_r;
  mstream.AbstractWrite(mod_t_time.get_ptr());
  mstream.AbstractWrite(mod_k_d.get_ptr());
  mstream.AbstractWrite(mod_k_speed.get_ptr());
  mstream.AbstractWrite(mod_r_d.get_ptr());
  mstream.AbstractWrite(mod_r_speed.get_ptr());
}


void ChLinkRotSpring::StreamIN(ChStreamInBinary& mstream)
{
  // Class version number
  int version = mstream.VersionRead();

  // Deserialize parent class
  ChLinkMarkers::StreamIN(mstream);

  // Stream in all member data
  ChFunction* newfun;
  mstream >> spr_rest_rot;
  mstream >> spr_t;
  mstream >> spr_k;
  mstream >> spr_r;
  mstream.AbstractReadCreate(&newfun);    mod_t_time  = ChSharedPtr<ChFunction>(newfun);
  mstream.AbstractReadCreate(&newfun);    mod_k_d     = ChSharedPtr<ChFunction>(newfun);
  mstream.AbstractReadCreate(&newfun);    mod_k_speed = ChSharedPtr<ChFunction>(newfun);
  mstream.AbstractReadCreate(&newfun);    mod_r_d     = ChSharedPtr<ChFunction>(newfun);
  mstream.AbstractReadCreate(&newfun);    mod_r_speed = ChSharedPtr<ChFunction>(newfun);
}


} // END_OF_NAMESPACE____


