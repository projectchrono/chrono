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
// Authors: Justin Madsen
// =============================================================================
//
// Generates a track chain around the track system
//
// =============================================================================

#ifndef TRACKCHAIN_H
#define TRACKCHAIN_H

#include "subsys/ChApiSubsys.h"
#include "assets/ChTriangleMeshShape.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChBodyAuxRef.h"

#include "ModelDefs.h"

namespace chrono {

/// Functor class for a custom rotaional damper value
class ChFunction_CustomDamper : public ChFunction
{
public:
  ChFunction_CustomDamper(double R_const=1.0, double R_nonlin=0.5): m_R(R_const), m_R_nonlin(R_nonlin) {}
  ChFunction* new_Duplicate() {return new ChFunction_CustomDamper;} 

  double Get_y(double x)
  {
    // scale the nonlinear damping coefficient
    return m_R + m_R_nonlin * x;
  }

private:
  double m_R;   ///< constant damping coefficient
  double m_R_nonlin;  ///< nonlinear dapming coefficient
};


/// Generates and manages the track chain system, which is typically generated
/// with a single track shoe/pin geometry set.
class CH_SUBSYS_API TrackChain : public ChShared
{
public:
  
  /// constructor, default values are  for an M113 model
  TrackChain(const std::string& name, 
    VisualizationType::Enum vis = VisualizationType::Enum::Primitives,
    CollisionType::Enum collide = CollisionType::Enum::Primitives,
    size_t chainSys_idx = 0,
    double shoe_mass = 18.02,
    const ChVector<>& shoeIxx = ChVector<>(0.22, 0.25, 0.04) );

  ~TrackChain() {}

  /// Use the clearance to define a spherical envelope for each rolling element.
  /// An envelope is where the surface of the track chain will not penetrate.
  /// Start_loc should be somewhere between the idler and driveGear, e.g. the top of the chain.
  /// NOTE: control_points begin and end with the idler and driveGear
  void Initialize(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const std::vector<ChVector<> >& rolling_element_loc, ///< center of rolling element geometry, w.r.t chassis_REF
    const std::vector<double>& clearance,       ///< rolling element geometry clearance from center
     const std::vector<ChVector<> >& spin_axis,  ///< rolling element revolute joint DOF axis, w.r.t absolute c-ssy
    const ChVector<>& start_loc                 ///< where to place the middle of the first shoe, w.r.t. chassis_REF
    );
  
  /// handle to the shoe body
  ChSharedPtr<ChBody> GetShoeBody(size_t track_idx) const; 
  
  // get list to track chain body list
  std::vector<ChSharedPtr<ChBody> > GetShoeBody() const; 

  /// reaction force on pin constraint, relative coords
  const ChVector<> GetPinReactForce(size_t pin_idx);
  const ChVector<> GetPinReactTorque(size_t pin_idx);

  /// turn on damping friction in the shoe pins
  void Set_pin_friction(double damping_C, ///< damping coefficient for the rotational damper [N-m-s]
    bool use_custom_damper= false,  ///< use a non-constant damper modulus
    double damping_C_nonlin = 0);   ///< nonlinear portion of damper, used if use_custom_damper = true

  // accessors
  /// number of track shoes created/initialized
  double Get_numShoes() const { return m_numShoes; }

  /// damping coef. in the pin revolute constraint
  double Get_pin_damping() const { return m_damping_C;}

private:

  // private functions

  /// add visualization assets to the last added body
  void TrackChain::AddVisualization();

  /// add visualization assets to a specific track shoe body
  void AddVisualization(size_t track_idx,
    bool custom_texture = false,
    const std::string& tex_name = "none");

  /// add collision geometry to the last shoe added
  void AddCollisionGeometry(double mu = 0.6,
                            double mu_sliding = 0.5,
                            double mu_roll = 0,
                            double mu_spin = 0);

  /// add collision geometrey to a certain track shoe
  void AddCollisionGeometry(size_t track_idx,
                            double mu = 0.6,
                            double mu_sliding = 0.5,
                            double mu_roll = 0,
                            double mu_spin = 0);
   
  /// initialize shoe bodies by wrapping the track chain around the rolling elements.
  /// Define a string along which the chain is wrapper, using the input values.
  /// Note: start_loc_abs should be between the idler and sprockets
  void CreateChain(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const std::vector<ChFrame<> >& control_points_abs,
    const std::vector<ChFrame<> >& rolling_element_abs,
    const std::vector<double>& clearance,
    const ChVector<>& start_pos_abs );

  /// create the shoes along a line segment and associated curved section that define
  /// the collision envelope.
  void CreateShoes(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChVector<>& start_seg,
    const ChVector<>& end_seg,
    const ChVector<>& end_curve_seg,
    const ChVector<>& rolling_elem_center,
    double clearance);

  /// close the trackChain loop by connecting the end of the chain to the start.
  /// Absolute coordinates.
  void CreateShoes_closeChain(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChVector<>& start_seg,
    const ChVector<>& end_seg);

  /// check if the shoe placement meets the criteria for being aligned.
  /// sets the boolean, m_aligned_with_seg, and returns it also.
  bool check_shoe_aligned(const ChVector<>& pin2_pos_abs,
    const ChVector<>& start_seg_pos,
    const ChVector<>& end_seg_pos,
    const ChVector<>& seg_norm_axis);

    // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }
  const std::string& getCollisionFilename() const { return m_collisionFile; }

  // private variables
  std::vector<ChSharedPtr<ChBody> > m_shoes;  ///< handle to track shoes
  // std::vector<ChSharedPtr<ChBodyAuxRef> > m_shoes;  ///< handle to track shoes
  size_t m_numShoes;      ///< number of track shoe bodies added to m_shoes;

  std::vector<ChSharedPtr<ChLinkLockRevolute> > m_pins; ///< handles to inter-shoe pin joints
  std::vector<ChLinkForce*> m_pin_friction; ///< functions to apply inter-shoe pin friction
  double m_damping_C;        ///< shoe pin damping coef.
  bool m_use_custom_damper;  ///< use a nonlinear damping coefficient function?
  std::vector<ChSharedPtr<ChFunction_CustomDamper> > m_custom_dampers;
 
  double m_mass;         // mass per shoe
  ChVector<> m_inertia;  // inertia of a shoe

  VisualizationType::Enum m_vis;    // visual asset geometry type
  CollisionType::Enum m_collide;    // collision geometry type
  const size_t m_chainSys_idx; ///< if there are multiple chain systems 
  // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)
  

  const std::string m_collisionFile;	// collision geometry filename
  const std::string m_meshFile;  // visual geometry mesh filename
  const std::string m_meshName;  // name of the mesh
  
  // helper variable
  bool m_aligned_with_seg;  ///< when building track chain, was last shoe created exactly aligned with the envelope?

  //static values 
  static const ChVector<> m_COM;      // location of COM, relative to REF (e.g, geomtric center)

  static const ChVector<> m_shoe_box;
  static const double m_pin_width;
  // figure this from your collision mesh
  // e.g., vert. distance from COM to surface that collides with rolling elements
  static const double m_shoe_chain_Yoffset; // y - vertical shoe axis

  static const ChVector<> m_tooth_box;
  static const double m_tooth_COG_offset;
  static const double m_pin_dist;		  // linear distance between a shoe's two pin joint center
  static const double m_pin_radius;
  static const double m_pin_COG_offset;

};


} // end namespace chrono


#endif
