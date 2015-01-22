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

#ifndef CHLINKROTSPRING_H
#define CHLINKROTSPRING_H


#include "physics/ChLinkMarkers.h"


namespace chrono
{

/// Rotational spring damper, applies a reaction torque, T=T(theta), to a rotational DOF between to bodies.
/// theta is the accumulated rotation angle m1 relative to m2.
class ChApi ChLinkRotSpring : public ChLinkMarkers {

  CH_RTTI(ChLinkRotSpring,ChLinkMarkers);

protected:
  double spr_rest_rot;	// initial angle, m1 relative to m2
  double spr_k;	// T=T(theta)
  double spr_r;	// T=T(theta_dt)
  double spr_t;	// T
  ChSharedPtr<ChFunction> mod_t_time;   // T(t)
  ChSharedPtr<ChFunction> mod_k_d;      // k(d)
  ChSharedPtr<ChFunction> mod_r_d;      // r(d)
  ChSharedPtr<ChFunction> mod_r_speed;  // k(speed)
  ChSharedPtr<ChFunction> mod_k_speed;  // r(speed)
  double spr_react_T;   // resulting torque / readonly

public:

  /// constructor sets default spring values and cosnt_functions, f(t)=1
  ChLinkRotSpring();
  virtual ~ChLinkRotSpring();

  
  /// Given the two bodies to be connected, the positions of the two anchor endpoints of the spring
  // and possibly the imposed rest length of the spring.
  void Initialize(ChSharedPtr<ChBody> mbody1,   ///< first body to link
                  ChSharedPtr<ChBody> mbody2,   ///< second body to link
                  bool pos_are_relative,        ///< true: following pos. are considered relative to bodies. false: pos.are absolute
                  ChVector<> mpos1,             ///< position of spring endpoint, for 1st body (rel. or abs., see flag above)
                  ChVector<> mpos2,             ///< position of spring endpoint, for 2nd body (rel. or abs., see flag above) 
                  ChVector<>  m_rel_rot_axis,   ///< relative axis of rotation
                  bool auto_rest_rot = true,    ///< if true, initializes the rest-length as the distance between mpos1 and mpos2
                  double rest_rot = 0           ///< imposed rest_length (no need to define, if auto_rest_length=true.)
                  );


  // Virtual functions

  /// Inherits, adds spring custom torque to C_torque. 
  virtual void UpdateForces(double mytime);

  virtual void Copy(ChLinkRotSpring* source);
  virtual ChLink* new_Duplicate();  // always return base link class pointer
  virtual int GetType() { return LNK_SPRING; }

  // STREAMING
  virtual void StreamIN(ChStreamInBinary& mstream);
  virtual void StreamOUT(ChStreamOutBinary& mstream);

  // Accessors
  double Get_SpringRestRot() const { return spr_rest_rot; }


  // TODO: dist to rot from ChMarkers
  double Get_SpringDeform()     const { return (dist - spr_rest_rot); }
  double Get_SpringRot()     const { return dist; }
  double Get_SpringRot_dt()   const { return dist_dt; }



  double Get_SpringK()          const { return spr_k; }
  double Get_SpringR()          const { return spr_r; }
  double Get_SpringT()          const { return spr_t; }
  double Get_SpringReact()      const { return spr_react_T; }

  ChSharedPtr<ChFunction> Get_mod_f_time()  const { return mod_t_time; }
  ChSharedPtr<ChFunction> Get_mod_k_d()     const { return mod_k_d; }
  ChSharedPtr<ChFunction> Get_mod_r_d()     const { return mod_r_d; }
  ChSharedPtr<ChFunction> Get_mod_k_speed() const { return mod_k_speed; }
  ChSharedPtr<ChFunction> Get_mod_r_speed() const { return mod_r_speed; }
    
  /// 1st spring endpoint, absolute coordinate system
  ChVector<> GetEndPoint1Abs() { return marker1->GetAbsCoord().pos; }
  ///1st spring endpoint, Body1 coordinate system
  ChVector<> GetEndPoint1Rel() { return marker1->GetPos(); }
   ///  2nd spring endpoint, Body2 coordinate system
  ChVector<> GetEndPoint2Rel() { return marker2->GetPos(); };
   /// 1st spring endpointc absolute coordinate system
  ChVector<> GetEndPoint2Abs() { return marker2->GetAbsCoord().pos; }

  // Modifiers
  void Set_SpringRestRot(double m_r) { spr_rest_rot = m_r; }
  void Set_SpringK(double m_r)          { spr_k = m_r; }
  void Set_SpringR(double m_r)          { spr_r = m_r; }
  void Set_SpringT(double m_r)          { spr_t = m_r; }

  void Set_mod_t_time(ChSharedPtr<ChFunction> mf)  { mod_t_time = mf; }
  void Set_mod_k_d(ChSharedPtr<ChFunction> mf)     { mod_k_d = mf; }
  void Set_mod_r_d(ChSharedPtr<ChFunction> mf)     { mod_r_d = mf; }
  void Set_mod_k_speed(ChSharedPtr<ChFunction> mf) { mod_k_speed = mf; }
  void Set_mod_r_speed(ChSharedPtr<ChFunction> mf) { mod_r_speed = mf; }

  /// 1st spring endpoint, Body1 coordinate system
  void SetEndPoint1Rel(const ChVector<>& mset) { marker1->Impose_Rel_Coord(ChCoordsys<>(mset, QUNIT)); }
  /// 1st spring endpoint, absolute coordinate system
  void SetEndPoint1Abs(ChVector<>& mset) { marker1->Impose_Abs_Coord(ChCoordsys<>(mset, QUNIT)); }
  /// 2nd spring endpoint, Body2 coordinate system
  void SetEndPoint2Rel(const ChVector<>& mset) { marker2->Impose_Rel_Coord(ChCoordsys<>(mset, QUNIT)); }
  /// 1st spring endpointc absolute coordinate system)
  void SetEndPoint2Abs(ChVector<>& mset) { marker2->Impose_Abs_Coord(ChCoordsys<>(mset, QUNIT)); }

};


} // end namespace chrono

#endif
