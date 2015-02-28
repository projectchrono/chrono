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

#include "physics/ChSystem.h"
#include "core/ChHashTable.h"
#include "core/ChHashFunction.h"
// #include "collision/CHcModelBulletBody.h"
// #include "collision/ChCModelBulletParticle.h"

namespace chrono {

/// data container for the M113 gear/pin geometry.
const class GearPinGeometry
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
  double tooth_len;
  double tooth_width;

  // shoe pin geometry
  double pin_radius;
  double pin_width_max;
  double pin_width_min;
  double pin_width;
  double pin_x_offset;
  double pin_y_offset;
};

// Concave geometry (gear tooth seat) cannot be exactly represented by default collision primitives,
//  nor can it be accurately modeled with a mesh or convex hull.
// This custom collision checks the gear with all the track shoes
// I suppose since you can choose between DVI and DEM contact, might as well
// template this so we can add to either contact container type
template <class ContactEngine = ChContactContainer>
class GearPinCollisionCallback : public ChSystem::ChCustomComputeCollisionCallback
{
	public:
  /// all length units in meters
	GearPinCollisionCallback(const std::vector<ChSharedPtr<ChBody> >& shoes,
    ChSharedPtr<ChBody> gear_body,
    GearPinGeometry& geom,
    int persistent_hashtable_dim = 1000
	) : m_shoes(shoes),
  m_gear(gear_body),
  m_geom(geom),
  m_persistent_hashtable_dim(persistent_hashtable_dim),
  m_Ncontacts(0)
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

  
  class GearPinCacheContact{
	  public:
		  GearPinCacheContact()
			  {reactions_cache.resize(6,0);}

		  std::vector<float> reactions_cache; // same structure as in btManifoldPoint for other types of contact
	}; 

	ChHashTable<int, GearPinCacheContact>* m_hashed_contacts;

  
  // check the hash table for persistent contact
	void Found_GearPin_Contact(ChSharedPtr<ChBody> gear,
    ChSharedPtr<ChBody> shoe,
    const int shoeID, 
    const ChVector<>& pGear_bar,
    const ChVector<>& pPin_bar, 
    const ChVector<>& vnGear_bar) 
    // ChHashTable<int, GearPinCacheContact>* mhash)
	{
		std::vector<float> reaction_cache;

    // see if this contact is in the hash table
		ChHashTable<int, GearPinCacheContact>::iterator cached = m_hashed_contacts->find(shoeID);
		if (cached == m_hashed_contacts->end())
      reaction_cache =  (m_hashed_contacts->insert(shoeID))->second.reactions_cache;
		else
			reaction_cache = cached->second.reactions_cache;
			 
    // fill the contact container with info
		collision::ChCollisionInfo mcont;
		mcont.modelA = gear->GetCollisionModel();
		mcont.modelB = shoe->GetCollisionModel();
    // passed in contact points, normal, w.r.t. gear c-sys
    mcont.vN = gear->GetRot().Rotate(vnGear_bar);
    mcont.vpA = gear->GetPos() + gear->GetRot().Rotate(pGear_bar);
    mcont.vpB = gear->GetPos() + gear->GetRot().Rotate(pPin_bar);
    mcont.distance = (mcont.vpA - mcont.vpB).Length();
    mcont.reaction_cache = &(reaction_cache[0]);

    // increment the counter, add the contact
    m_Ncontacts++;
		((ContactEngine*)(gear->GetSystem()->GetContactContainer()))->AddContact(mcont);
	}

  /// return true if the bounding spheres of specified centers intersect
  // centers w.r.t. global c-sys
  // be sure to pass in the pin centers as positive and negative z, w.r.t. gear c-sys
  bool BroadphasePassed(const ChVector<>& gear_cen_Pz,
    const ChVector<>& gear_cen_Nz,
    const ChVector<>& pin_cen_Pz,
    const ChVector<>& pin_cen_Nz) const
  {
    // Gear bounding sphere circumscribes tips/edges of the tooth
    // apply one to each side of the sprocket
    double bound_rad_Gear = std::sqrt( std::pow(m_geom.tooth_mid_bar.Length(),2) + std::pow(m_geom.tooth_len*0.5,2) );
    
    // Shoe bounding sphere circumscribes the outside circumference of the pins
    double bound_rad_Pin = ChVector<>(m_geom.pin_radius,
      m_geom.pin_radius, 
      (m_geom.pin_width_max-m_geom.pin_width_min)/4.0).Length();

    // check both sides for contact
    // broad-phase, is the distance between centers <= sum of bounding sphere?
    return ( ( (gear_cen_Pz - pin_cen_Pz).Length() <= (bound_rad_Gear + bound_rad_Pin) ||
      (gear_cen_Nz - pin_cen_Nz).Length() <= (bound_rad_Gear + bound_rad_Pin) )? true : false );

  }

  /// true if in contact in the x-y plane.
  // fills the contact info and normal on pin, since the direction is defined by the concave section
  //  all relative to gear c-sys. Contact pts. use z- from pin_cen_bar. Normal is only in XY-bar plane
  bool eval2Dcontact(const ChVector<>& gear_seat_cen_bar,
    const ChVector<>& pin_cen_bar, 
    ChVector<>& contact_pos_gear_bar,
    ChVector<>& contact_pos_pin_bar,
    ChVector<>& contact_normal_onGear_bar)
  {
    // find the center of the gear base circle, in XY-gear plane.
    ChVector<> pitch_circle_cenXY_bar = gear_seat_cen_bar;
    pitch_circle_cenXY_bar.z = 0;
    pitch_circle_cenXY_bar *= (m_geom.gear_pitch_radius / m_geom.gear_base_radius);

    // vector from circle center to pin center, XY-gear plane
    ChVector<> r_pin_circleXY = pin_cen_bar - pitch_circle_cenXY_bar;
    r_pin_circleXY.z = 0;

    // in the XY gear plane, pin is in contact when dist from pitch_circle_center to pin center + radius > pitch_circle_radius
    if( r_pin_circleXY.Length() + m_geom.pin_radius > m_geom.gear_concave_radius )
    {
      // fill in contact info

      // XY normalized direction between pin center, concave circle center
      // points towards gear surface
      ChVector<> r_cpXY_hat = r_pin_circleXY / (r_pin_circleXY.Length() );

      // contact points, XY-bar plane
      contact_pos_gear_bar = pitch_circle_cenXY_bar + r_cpXY_hat * m_geom.gear_concave_radius;
      contact_pos_pin_bar = pin_cen_bar + r_cpXY_hat * m_geom.pin_radius;
      // both contact points use pin z-coord, relative to gear
      contact_pos_gear_bar.z = pin_cen_bar.z;
      contact_pos_pin_bar.z = pin_cen_bar.z;

      // normal dir is only in XY-gear plane, gear is way more stiff than pin
      contact_normal_onGear_bar = r_cpXY_hat;
      contact_normal_onGear_bar.z = 0; // to be complete
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
    const ChSharedPtr<ChBody> gear,
    const size_t shoe_idx)
  {

    // do narrow phase between this shoe and gear
    // find the closest gear to rotate the relate coordinates by the right angle
    size_t tooth_idx = Get_GearToothIdx(gear_seat_cen_Pz, pin_cen_Pz, gear->GetRot() );
    double rot_ang = CH_C_PI / m_geom.num_teeth + tooth_idx * (CH_C_PI / m_geom.num_teeth);

    // rotate the relative pos. things w.r.t gear c-sys
    ChQuaternion<> rot_q = Q_from_AngAxis(rot_ang, VECT_Z);
    // ChQuaternion<> rot_q2 = Q_from_AngAxis(rot_ang + (CH_C_2PI / m_geom.num_teeth), VECT_Z ); // look at the next gear tooth
    ChFrame<> seat_frame(m_seat_pos_bar, QUNIT);

    // get the gear seat for this tooth seat
    ChVector<> gear_seat_cen_bar_Pz = (rot_q * seat_frame).GetPos();
    ChVector<> gear_seat_cen_bar_Nz = -gear_seat_cen_bar_Pz;

    // get the pin centers in the gear c-sys, drop the out of plane (lateral) part.
    ChVector<> pin_cen_bar_Pz = gear->GetRot().RotateBack(pin_cen_Pz - gear->GetPos() );
    ChVector<> pin_cen_bar_Nz = gear->GetRot().RotateBack(pin_cen_Nz - gear->GetPos() );

    // do the x-y plane collision detection. 
    // Fill data, w.r.t. gear c-sys
    ChVector<> pGear_bar;
    ChVector<> pPin_bar;
    ChVector<> norm_onGear_bar;

    if( eval2Dcontact(gear_seat_cen_bar_Pz, pin_cen_bar_Pz,
      pGear_bar, pPin_bar, norm_onGear_bar) )
    {
      GetLog() << "\n narrow phase contact, positive z-side \n\n";


      Found_GearPin_Contact(gear, m_shoes[shoe_idx], shoe_idx, 
        pGear_bar, pPin_bar,
        norm_onGear_bar);
    }

    if( eval2Dcontact(gear_seat_cen_bar_Nz, pin_cen_bar_Nz, 
      pGear_bar, pPin_bar, norm_onGear_bar) )
    {
      GetLog() << "\n narrow phase contact, negative z-side \n\n";

      Found_GearPin_Contact(gear, m_shoes[shoe_idx], shoe_idx,
        pGear_bar, pPin_bar, norm_onGear_bar);
    }

    return true;
  }


  /// based on the distance between input global centers, find the dist. relative to the gear
  /// c-sys. Return which gear tooth to perform narrow-phase with
  size_t Get_GearToothIdx(const ChVector<>& gear_cen,
    const ChVector<>& pin_cen,
    const ChQuaternion<>& gear_rot) const
  {
    ChVector<> len = pin_cen - gear_cen;  // global c-sys
    // transform to local coords
    len = gear_rot.RotateBack(len);
    // in local coords, can find the rotation angle in x-y plane
    double rot_ang = std::atan2(len.y,len.x);
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
    // ChHashTable<int, GearPinCacheContact>* mhash)
	{
		//GetLog() << "hash statistics: loading=" << hashed_contacts->loading() << "   size=" << hashed_contacts->size() <<"\n";
		// look thru the shoe list, see if any pins are in contact with the concave gear seat surface.
    for(size_t idx = 0; idx < m_shoes.size(); idx++)
		{
      // global c-sys
      ChVector<> shoe_pos = m_shoes[idx]->GetPos();
      ChVector<> gear_seat_pos_Pz = m_gear->GetPos() 
        + m_gear->GetRot().Rotate( m_seat_pos_bar );
      // same as Pz_bar, negate z
      ChVector<> gear_seat_pos_Nz = m_gear->GetPos() 
        + m_gear->GetRot().Rotate( ChVector<>(m_seat_pos_bar.x, m_seat_pos_bar.y, -m_seat_pos_bar.z) );

      // put the Shoe bounding sphere at the center of the pin, on the positive and negative z-dir, w.r.t. gear c-sys
      // TODO: relative to the shoe c-sys, is the pin -x or +x dir ??
      ChVector<> pin_pos_Pz = shoe_pos 
        + m_shoes[idx]->GetRot().Rotate( m_pin_pos_bar );
      ChVector<> pin_pos_Nz = shoe_pos 
        + m_shoes[idx]->GetRot().Rotate( ChVector<>(m_pin_pos_bar.x, m_pin_pos_bar.y, -m_pin_pos_bar.z) );

      // broad-phase passes?
      if( BroadphasePassed(gear_seat_pos_Pz, gear_seat_pos_Nz,
        pin_pos_Pz, pin_pos_Nz) )
      {
        GetLog() << " \n\n Broadphase passed, time = " << msys->GetChTime() << "\n\n";
        bool passed_Narrow = NarrowPhase(gear_seat_pos_Pz, gear_seat_pos_Nz,
          pin_pos_Pz, pin_pos_Nz,
          m_gear, idx);
  
      }
    }
  }

  bool Get_contactPrevStep(size_t idx) const { assert(idx<m_shoes.size()); return m_contactPrevStep[idx]; }

  // other virtuals

  // number of contacts detected
  int GetNcontacts_GearPin() const { return m_Ncontacts; }


private:

  ChVector<> m_pin_pos_bar;  ///< center of pin cylinder on positive z-side, in shoe c-sys
  ChVector<> m_seat_pos_bar; ///< center of gear seat on pos. z side, in shoe c-sys

  // handles to bodies to check
  const std::vector<ChSharedPtr<ChBody> > m_shoes;
  const ChSharedPtr<ChBody> m_gear;
  const GearPinGeometry m_geom; ///< gear and pin geometry data

  // following are used for determining if contacts are "persistent"
  // i.e., no liftoff once they are engaged with the sprocket. 1 per shoe
  std::vector<bool> m_contactPrevStep;  ///<  was the shoe body in contact with the gear last step? i.e., passed narrow phase last step?
  std::vector<size_t> m_persistentContactSteps;   ///< how many steps in a row was the pin in contact with the gear?

  int m_Ncontacts;

  // hashtable
  size_t m_persistent_hashtable_dim;

};  // end class GearPinCollisionCallback


}  // end namespace chrono

#endif