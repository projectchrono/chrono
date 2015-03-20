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
// Author: Justin Madsen
//
// Any custom collision callback classes go here.
// =============================================================================

#ifndef TRACKCOLLISIONCALLBACK_H
#define TRACKCOLLISIONCALLBACK_H

#include <cmath>

#include "physics/ChSystem.h"
#include "physics/ChContactContainer.h"
#include "core/ChHashTable.h"
#include "core/ChHashFunction.h"
// #include "collision/CHcModelBulletBody.h"
// #include "collision/ChCModelBulletParticle.h"

namespace chrono {

/// Gear seat geometry, # of teeth, pin size, spacing, offsets, etc. are parametrized
///   with this data container class, for the below custom collision between the 
///   sprocket gear seat and track shoe pins.
/// Default values for M113 gear/pin geometry.
class GearPinGeometry
{
public:
  GearPinGeometry(double gear_base_radius = 0.211,  ///< gear base circle radius
    double gear_pitch_radius = 0.267,   ///< center of the circle to define concave gear tooth base surface
    double gear_seat_width_max = 0.606, ///< max width of the gear seat, w.r.t. gear c-sys
    double gear_seat_width_min = 0.438, ///< min width of the gear seat, w.r.t. gear c-sys 
    ChVector<> tooth_mid_bar = ChVector<>(0.079815, 0.24719, 0.2712), ///< assume first seat bottom is directly above COG, then center of top of gear tooth is relative to gear c-sys
    double tooth_len = 0.013119,  ///< length of top of gear tooth, in local XY plane
    double tooth_width = 0.0840,  ///< width of top of gear tooth, in local Z plane
    size_t num_teeth = 10,        ///< number of gear teeth,
    double key_angle = 0,         ///< if the gear tooth base is not directly above the center
    double pin_radius = 0.0232,   ///< shoe pin radius
    double pin_width_max = 0.531, ///< max total pin width
    double pin_width_min = 0.38,  ///< min total pin width
    double pin_x_offset = -0.07581, ///< x-offset of pin from center of shoe c-sys, in shoe c-sys
    double pin_y_offset = 0         ///< y-offset of pin from center of shoe c-sys
    ): gear_base_radius(gear_base_radius),
    gear_pitch_radius(gear_pitch_radius),
    gear_concave_radius(gear_pitch_radius-gear_base_radius),
    gear_seat_width_max(gear_seat_width_max),
    gear_seat_width_min(gear_seat_width_min),
    gear_seat_width( 0.5*(gear_seat_width_max-gear_seat_width_min) ),
    tooth_mid_bar(tooth_mid_bar),
    tooth_len(tooth_len),
    tooth_width(tooth_width),
    num_teeth( num_teeth),
    key_angle(0),
    pin_radius(pin_radius),
    pin_width_max(pin_width_max),
    pin_width_min(pin_width_min),
    pin_width( 0.5*(pin_width_max-pin_width_min) ),
    pin_x_offset(pin_x_offset),
    pin_y_offset(pin_y_offset)
  {
    // make sure the geometric dimensons are valid
    assert(gear_seat_width_max - gear_seat_width_min > 0);
    assert(pin_width_max - pin_width_min > 0);
    assert(gear_pitch_radius - gear_base_radius > 0);
  }

  // gear geometry
  double gear_base_radius; 
  double gear_pitch_radius; 
  double gear_concave_radius; ///< radius of circle that defines concave gear tooth profile section
  double gear_seat_width_max;
  double gear_seat_width_min;
  double gear_seat_width;
  size_t num_teeth;
  double key_angle;

  // gear tooth geometry
  ChVector<> tooth_mid_bar;
  double tooth_len; // flat top surface of gear, length in XY plane (z-axis is gear rot axis)
  double tooth_width; // flat top surface of gear, width in Z plane

  // shoe pin geometry
  double pin_radius;
  double pin_width_max;
  double pin_width_min;
  double pin_width;
  double pin_x_offset;
  double pin_y_offset;
};


/// Concave geometry (gear tooth seat) cannot be exactly represented by default collision primitives,
///  nor can it be accurately modeled with a mesh or convex hull.
/// This custom collision checks the sprocket gear seat geometry with all the track shoe pins.
/// Gear seat geometry, # of teeth, pin size, spacing, offsets, etc. are parametrized thru class GearPinGeometry. 
/// Templated based on the type of collision engine you are using, e.g. DVI or DEM, (DVI default).
template <class ContactEngine = ChContactContainerBase>
class GearPinCollisionCallback : public ChSystem::ChCustomComputeCollisionCallback
{
	public:
  // all length units in meters
	GearPinCollisionCallback(const std::vector<ChSharedPtr<ChBody> >& shoes,
    ChSharedPtr<ChBody> gear_body,
    GearPinGeometry& geom,
    const double geom_envelope = 0.005,
    const int persistent_hashtable_dim = 1000
	) : m_shoes(shoes),
  m_gear(gear_body),
  m_geom(geom),
  m_envelope(geom_envelope),
  m_persistent_hashtable_dim(persistent_hashtable_dim),
  m_Ncontacts(0),
  m_NbroadPhasePassed(0),
  m_sum_Pz_contacts(0),
  m_sum_Nz_contacts(0)
  {

    // two endpoints of cylinder pin, w.r.t. shoe c-sys. 
    // SYMMETRIC ABOUT XY PLANE (e.g., check for contact for -z)
    // point 1 = inner, 2 = outer
		m_pin_pos_bar = ChVector<>(m_geom.pin_x_offset,
      m_geom.pin_y_offset,
      0.5*(m_geom.pin_width_min + m_geom.pin_width) );

    // two endpoints of  the seat cylinder (there are as many as gear teeth
    // SYMMETRIC ABOUT XY PLANE (e.g., check for contact for -z)
    // point 1 = inner, 2 = outer
    m_seat_pos_bar = ChVector<>(0,
      m_geom.gear_base_radius,
      0.5*(m_geom.gear_seat_width_min + m_geom.gear_seat_width) );

    // Gear broadphase collision shape is a cylinder that circumscribes corners
    //  of the tooth in the XY plane, z is gear rot axis.
    ChVector<> tooth_mid_XY_bar = m_geom.tooth_mid_bar;
    tooth_mid_XY_bar.z = 0;
    tooth_mid_XY_bar.x += 0.5*m_geom.tooth_len;
    m_bound_rad_Gear = tooth_mid_XY_bar.Length() + m_envelope;
    
    // Shoe pin broadphase collision shape is a cylinder at each pin COG
    // apply one to each side of the shoe body
    m_bound_rad_Pin = m_geom.pin_radius + m_envelope;

    // if the center of the pin and gear, in the XY gear c-sys, are less than this value, passes broad phase
    m_bound_broadphase = m_bound_rad_Gear + m_bound_rad_Pin; // includes the outward envelope

    // the bounding concave section needs to include an outward envelope, which reduces the concave 
    //  circle radius
    m_bound_gear_seat_rad = m_geom.gear_concave_radius - m_envelope;

		// alloc the hash table for persistent manifold of gear-cylinder contacts
		m_hashed_contacts = new ChHashTable<int, GearPinCacheContact>(m_persistent_hashtable_dim);

    // keep track of some persistence of contact info
    m_persistentContactSteps.resize(m_shoes.size(), 0);
    m_contactPrevStep.resize(m_shoes.size(), false);
	}

	~GearPinCollisionCallback()
	{ 
		if (m_hashed_contacts) delete m_hashed_contacts; m_hashed_contacts=0;

	}

  // data container for 6-DOF contact reaction cache
  class GearPinCacheContact{
	  public:
		  GearPinCacheContact()
			  {reactions_cache[0]=reactions_cache[1]=reactions_cache[2]=reactions_cache[3]=reactions_cache[4]=reactions_cache[5]=0; }

		  float reactions_cache[6]; // same structure as in btManifoldPoint for other types of contact
	}; 

	ChHashTable<int, GearPinCacheContact>* m_hashed_contacts;

 
  // check the hash table for persistent contact
  // vnGear_bar the surface normal of the gear at the contact point.
	void Found_GearPin_Contact(const ChVector<>& pGear_bar,
    const ChVector<>& pPin_bar, 
    const ChVector<>& vnGear_bar,
    const size_t shoeID) 
	{
    // see if this contact is in the hash table
    float* reaction_cache = 0;
		typename ChHashTable<int, GearPinCacheContact>::iterator cached = m_hashed_contacts->find(shoeID);
		if (cached == m_hashed_contacts->end())
      reaction_cache =  (m_hashed_contacts->insert(shoeID))->second.reactions_cache;
		else
			reaction_cache = cached->second.reactions_cache;
			 
    // fill the contact container with info
		collision::ChCollisionInfo mcont;
		mcont.modelA = m_gear->GetCollisionModel();
		mcont.modelB = m_shoes[shoeID]->GetCollisionModel();
    // passed in contact points, normal, w.r.t. gear c-sys
    mcont.vN = m_gear->GetRot().Rotate(vnGear_bar);
    mcont.vpA = m_gear->GetPos() + m_gear->GetRot().Rotate(pGear_bar);
    mcont.vpB = m_gear->GetPos() + m_gear->GetRot().Rotate(pPin_bar);
    mcont.distance = (mcont.vpA - mcont.vpB).Length();
    mcont.reaction_cache = reaction_cache;

    // increment the counter, add the contact
    m_Ncontacts++;
		(m_gear->GetSystem()->GetContactContainer())->AddContact(mcont);
	}

  // true when radial dist. from center of gear to pins on either side of shoe, in gear c-sys,
  //  is less than combined geometry bounding spheres (actually a bounding cylinder in 2D)
  // Input 
  bool BroadphasePassed(const ChVector<>& pin_gear_Pz_bar,
    const ChVector<>& pin_gear_Nz_bar) const
  {
    // only need to check radial distance from gear center, in gear c-sys
    ChVector<> pin_gear_XY_Pz(pin_gear_Pz_bar.x, pin_gear_Pz_bar.y, 0);
    ChVector<> pin_gear_XY_Nz(pin_gear_Nz_bar.x, pin_gear_Nz_bar.y, 0);

    if( pin_gear_XY_Pz.Length() <= m_bound_broadphase )
    {
      // GetLog() << "pin 1 is w/in collision envelope \n";
      return true;
    }
    if( pin_gear_XY_Nz.Length() <= m_bound_broadphase )
    {
      // GetLog() << "pin 2 is w/in collision envelope \n";
      return true;
    }
    // gear and pin bounding shapes don't intersect
    return false;
  }

  // true if in contact in the x-y plane, also calls the function to add the
  //  contact to the system.
  // Contact pts. use z- from pin_cen_bar. Normal is only in XY-bar plane
  bool eval2Dcontact(const ChVector<>& gear_seat_cen_bar,
    const ChVector<>& pin_cen_bar, 
    const size_t shoe_idx)
  {
    // find the center of the gear base circle, in XY-gear plane.
    ChVector<> pitch_circle_cenXY_bar = gear_seat_cen_bar;
    pitch_circle_cenXY_bar.z = 0;
    // center of the pitch circle should just be radially outward from the seat position, XY gear plane.
    pitch_circle_cenXY_bar *= (m_geom.gear_pitch_radius / m_geom.gear_base_radius);

    // vector from pitch circle pos to pin center, XY-gear plane
    ChVector<> r_pitch_pin_XY = pin_cen_bar - pitch_circle_cenXY_bar;
    r_pitch_pin_XY.z = 0;
    // gear center to gear seat center, XY-gear plane
    ChVector<> gear_seat_bar_XY = gear_seat_cen_bar;
    gear_seat_bar_XY.z = 0;

    // negative when pin center is radially inwards from the pitch circle center pos, the direction of imortance.
    double r1r2_dot = Vdot(r_pitch_pin_XY, gear_seat_bar_XY); 

    // true when the pin intersects with the semi-circle that is radially inwards from the pitch circle center position
    //  (relative to gear c-sys)
    if( r_pitch_pin_XY.Length() + m_bound_rad_Pin >= m_bound_gear_seat_rad && r1r2_dot < 0 )
    {
      // fill in contact info. 

      // contact points, XY-bar plane
      r_pitch_pin_XY.Normalize();
      // project the contact points on the gear and the pin onto their surfaces, respectively
      ChVector<> contact_pos_gear_bar = pitch_circle_cenXY_bar + r_pitch_pin_XY * m_geom.gear_concave_radius;
      ChVector<> contact_pos_pin_bar = pin_cen_bar + r_pitch_pin_XY * m_geom.pin_radius;
      // both contact points use pin z-coord, relative to gear
      contact_pos_gear_bar.z = pin_cen_bar.z;
      contact_pos_pin_bar.z = pin_cen_bar.z;

      // normal surface of the gear is only in XY-gear plane.
      ChVector<> contact_normal_Gear_bar = -r_pitch_pin_XY;
      contact_normal_Gear_bar.z = 0; // to be complete

      // add contact info to the system, in the format the collision engine expects
      Found_GearPin_Contact(contact_pos_gear_bar,
        contact_pos_pin_bar,
        contact_normal_Gear_bar,
        shoe_idx);

      return true;
    }
    else
    {
      return false;
    }
  }


  /// return true if there are contacts detected
  // centers w.r.t. global c-sys
  // be sure to pass centers as positive and negative z, w.r.t. gear c-sys
  bool NarrowPhase(const ChVector<>& gear_seat_cen_Pz,
    const ChVector<>& gear_seat_cen_Nz,
    const ChVector<>& pin_cen_Pz,
    const ChVector<>& pin_cen_Nz,
    const size_t shoe_idx)
  {

    // do narrow phase between this shoe and gear
    // find the closest gear to rotate the relate coordinates by the right angle
    size_t tooth_idx = Get_GearToothIdx(pin_cen_Pz);
    double rot_ang = tooth_idx * CH_C_2PI / m_geom.num_teeth;

    // rotate the relative pos. things w.r.t gear c-sys
    ChQuaternion<> rot_q = Q_from_AngAxis(rot_ang, VECT_Z);
    // ChQuaternion<> rot_q2 = Q_from_AngAxis(rot_ang + (CH_C_2PI / m_geom.num_teeth), VECT_Z ); // look at the next gear tooth
    ChFrame<> seat_frame(m_seat_pos_bar, QUNIT);

    // get the gear seat for this tooth seat
    ChVector<> gear_seat_cen_bar_Pz = (rot_q * seat_frame).GetPos();
    ChVector<> gear_seat_cen_bar_Nz = gear_seat_cen_bar_Pz;
    gear_seat_cen_bar_Nz.z *= -1;

    // get the pin centers in the gear c-sys, drop the out of plane (lateral) part.
    ChVector<> pin_cen_bar_Pz = m_gear->GetRot().RotateBack(pin_cen_Pz - m_gear->GetPos() );
    ChVector<> pin_cen_bar_Nz = m_gear->GetRot().RotateBack(pin_cen_Nz - m_gear->GetPos() );

    // do the x-y plane collision detection. 
    // Fill data, w.r.t. gear c-sys
    ChVector<> pGear_bar;
    ChVector<> pPin_bar;
    ChVector<> norm_onGear_bar;

    if( eval2Dcontact(gear_seat_cen_bar_Pz, pin_cen_bar_Pz, shoe_idx) )
    {
      // GetLog() << "\n narrow phase contact, positive z-side \n\n";

      // curious about Pz/Nz contacts overall
      m_sum_Pz_contacts++;
    }

    if( eval2Dcontact(gear_seat_cen_bar_Nz, pin_cen_bar_Nz, shoe_idx) )
    {
      // GetLog() << "\n narrow phase contact, negative z-side \n\n";

      // curious about Pz/Nz contacts overall
      m_sum_Nz_contacts++;
    }

    return true;
  }


  /// based on the distance between input global centers, find the dist. relative to the gear
  /// c-sys. Return which gear tooth to perform narrow-phase with
  size_t Get_GearToothIdx(const ChVector<>& pin_cen) const
  {
    ChVector<> gear_pin = pin_cen - m_gear->GetPos();  // global c-sys
    // transform to local gear c-sys
    gear_pin = (m_gear->GetRot()).RotateBack(gear_pin);
    // in local coords, can find the rotation angle in x-y plane, off the vertical y-axis
    double rot_ang = std::atan2(gear_pin.y,gear_pin.x) + 3.0 * CH_C_PI_2;
    double incr = chrono::CH_C_2PI / m_geom.num_teeth;
    size_t idx = std::floor( (rot_ang + 0.5*incr) / incr);
    return idx;
  }

  // callback function used each timestep
	virtual void PerformCustomCollision(ChSystem* msys)
	{
		CollisionGearPinFamily(msys);
		
	}

  // function implementation
	void CollisionGearPinFamily(ChSystem* msys)
	{
		// reset any per-step variables
    m_Ncontacts = 0;
    m_NbroadPhasePassed = 0;

		// for each shoe, see if any pins are in contact with the concave gear seat surface.
    for(size_t idx = 0; idx < m_shoes.size(); idx++)
		{
      // check pins on both sides of shoe for contact with the gear
      // broad-phase, is the distance between geometry bounding sphere centers 
      //  <= sum of bounding sphere?

      // put the Shoe bounding sphere at the center of the pin, symmetric about shoe relative z-axis
      ChVector<> pin_pos_Pz = m_shoes[idx]->GetPos()
        + m_shoes[idx]->GetRot().Rotate( m_pin_pos_bar );
      ChVector<> pin_pos_Nz = m_shoes[idx]->GetPos()
        + m_shoes[idx]->GetRot().Rotate( ChVector<>(m_pin_pos_bar.x, m_pin_pos_bar.y, -m_pin_pos_bar.z) );

      // convert to gear c-sys, to find the radial distance between centers
      ChVector<> pin_gear_bar_Pz = m_gear->GetRot().RotateBack(m_gear->GetPos() - pin_pos_Pz);
      ChVector<> pin_gear_bar_Nz = m_gear->GetRot().RotateBack(m_gear->GetPos() - pin_pos_Nz);


      // DEBUGGING
      if(idx == 9 && m_gear->GetSystem()->GetChTime() > 0.09)
        int arg = 2;




      // broad-phase passes?
      if( BroadphasePassed(pin_gear_bar_Pz, pin_gear_bar_Nz) )
      {
        // GetLog() << " \n\n Broadphase passed, time = " << msys->GetChTime() << "\n\n";
        m_NbroadPhasePassed++;
      
        // Narrow phase needs coords in global c-sys
        ChVector<> gear_seat_pos_Pz = m_gear->GetPos() 
          + m_gear->GetRot().Rotate( m_seat_pos_bar );
        // same as Pz_bar, negate z
        ChVector<> gear_seat_pos_Nz = m_gear->GetPos() 
          + m_gear->GetRot().Rotate( ChVector<>(m_seat_pos_bar.x, m_seat_pos_bar.y, -m_seat_pos_bar.z) );

        // narrow phase will add the contact if passed. All coords are in global c-sys
        bool passed_Narrow = NarrowPhase(gear_seat_pos_Pz, gear_seat_pos_Nz,
          pin_pos_Pz, pin_pos_Nz,
          idx);
  
      }
    }
  }

  bool Get_contactPrevStep(size_t idx) const { assert(idx<m_shoes.size()); return m_contactPrevStep[idx]; }

  // number of contacts detected this step
  int GetNcontacts() const { return m_Ncontacts; }

  /// time broadphase has passed this step
  int GetNbroadPhasePassed() const { return m_NbroadPhasePassed;}

  /// total times narrow phase passed on positive/negative z side
  int Get_sum_Pz_contacts() const { return m_sum_Pz_contacts; }

  int Get_sum_Nz_contacts() const { return m_sum_Nz_contacts; }

private:

  ChVector<> m_pin_pos_bar;  ///< center of pin cylinder on positive z-side, in shoe c-sys
  ChVector<> m_seat_pos_bar; ///< center of gear seat on pos. z side, in shoe c-sys

  // handles to bodies to check
  std::vector<ChSharedPtr<ChBody> > m_shoes;
  ChSharedPtr<ChBody> m_gear;
  const GearPinGeometry m_geom; ///< gear and pin geometry data
  const double m_envelope; 
  double m_bound_rad_Gear; ///< broadphase geometry bounding sphere radius for gear
  double m_bound_rad_Pin; ///< geometry bounding sphere circumscribes the outside circumference of the pins
  double m_bound_broadphase;  ///< total bounding radius
  double m_bound_gear_seat_rad;  ///< radius swept out by the concave gear seat section

  // following are used for determining if contacts are "persistent"
  // i.e., no liftoff once they are engaged with the sprocket. 1 per shoe
  std::vector<bool> m_contactPrevStep;  ///<  was the shoe body in contact with the gear last step? i.e., passed narrow phase last step?
  std::vector<size_t> m_persistentContactSteps;   ///< how many steps in a row was the pin in contact with the gear?

  // # of passed broad phase
  int m_NbroadPhasePassed;

  // # of passed narrow phase
  int m_Ncontacts;

  // hashtable
  size_t m_persistent_hashtable_dim;

  size_t m_sum_Pz_contacts; // curious about Pz/Nz contacts overall
  size_t m_sum_Nz_contacts;
};  // end class GearPinCollisionCallback


// scan through all the contacts, write ascii to console or file
class _contact_reporter : public ChReportContactCallback
{
public:

  // constructor requires user to specify to file or console
  _contact_reporter(ChStreamOutAscii& out_stream,
    bool write_to_console = true
    ): os(out_stream), to_console(write_to_console), contact_num(0) {}

  virtual bool ReportContactCallback(const ChVector<>& pA,
                                     const ChVector<>& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const float& mfriction,
                                     const ChVector<>& react_forces,
                                     const ChVector<>& react_torques,
                                     collision::ChCollisionModel* modA,
                                     collision::ChCollisionModel* modB)
  {
    // write to console
    if(to_console)
    {
      os << "\n ---- collision info, # : " << contact_num
        <<"\n body A, body B : " << modA->GetPhysicsItem()->GetName() << ", " << modB->GetPhysicsItem()->GetName()
        <<"\n pA : " << pA
        <<" pB : " << pB
        <<" norm : " << plane_coord.Get_A_Xaxis()
        <<" dist : " << distance
        <<"\n forces: " << react_forces;
    }
    else
    {
      // write to file
      ChVector<> x_hat = plane_coord.Get_A_Xaxis();
      os << modA->GetPhysicsItem()->GetName()
        <<","<< modB->GetPhysicsItem()->GetName()
        <<","<< pA.x <<","<< pA.y <<","<< pA.z 
        <<","<< pB.x <<","<< pB.y <<","<< pB.z
        <<","<< x_hat.x <<","<< x_hat.y <<","<< x_hat.z
        <<","<< distance
        <<","<< react_forces.x <<","<< react_forces.y <<","<< react_forces.z
        << "\n";
    }

    contact_num++;
    return true; // to continue scanning contacts
  }

  // can be console or file
  ChStreamOutAscii& os;
  bool to_console;
  int contact_num;  // this contact #
};  // end class _contact_reporter


}  // end namespace chrono

#endif
