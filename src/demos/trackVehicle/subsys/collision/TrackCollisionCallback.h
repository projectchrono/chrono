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
    double gear_pitch_radius = 0.267, ///< center of the circle that uses gear_tooth_radius to define gear tooth surface
    double gear_seat_width_max = 0.626, ///< max width of the gear seat, w.r.t. gear c-sys
    double gear_seat_width_min = 0.458, ///< min width of the gear seat, w.r.t. gear c-sys 
    ChVector<> tooth_mid_bar = ChVector<>(0.079815, 0.24719, 0.2712), ///< assume first seat bottom is directly above COG, then center of top of gear tooth is relative to gear c-sys
    double tooth_len = 0.013119,  ///< length of top of gear tooth, in local XY plane
    double tooth_width = 0.0840,  ///< width of top of gear tooth, in local Z plane
    size_t num_teeth = 10.0,      ///< number of gear teeth
    double pin_radius = 0.0232,   ///< shoe pin radius
    double pin_width_max = 0.531,  ///< max total pin width
    double pin_width_min = 0.38,  ///< min total pin width
    double pin_x_offset = -0.07581, ///< x-offset of pin from center of shoe c-sys, in shoe c-sys
    double pin_y_offset = 0        ///< y-offset of pin from center of shoe c-sys
    ): gear_base_radius(gear_base_radius),
    gear_pitch_radius(gear_pitch_radius),
    gear_tooth_radius(gear_pitch_radius-gear_base_radius),
    gear_seat_width_max(gear_seat_width_max),
    gear_seat_width_min(gear_seat_width_min),
    gear_seat_width( (gear_seat_width_max-gear_seat_width_min)/2.0 ),
    tooth_mid_bar(tooth_mid_bar),
    tooth_len(tooth_len),
    tooth_width(tooth_width),
    num_teeth( num_teeth),
    pin_radius(pin_radius),
    pin_width_max(pin_width_max),
    pin_width_min(pin_width_min),
    pin_width( (pin_width_max-pin_width_min)/2.0 ),
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
  double gear_tooth_radius;
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
template <class ContactEngine>
class GearPinCollisionCallback : public ChSystem::ChCustomComputeCollisionCallback
{
	public:
  /// all length units in meters
	GearPinCollisionCallback(std::vector<ChSharedPtr<ChBody> >& shoes,
    ChSharedPtr<ChBody>& gear_body,
    const GearPinGeometry& geom,
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
      m_geom.pin_width_min/2.0);

    // two endpoints of  the seat cylinder (there are as many as gear teeth
    // SYMMETRIC ABOUT XY PLANE (e.g., check for contact for -z)
    // point 1 = inner, 2 = outer
    m_seat_pos_bar = ChVector<>(0,
      m_geom.gear_base_radius,
      (m_geom.gear_seat_width_min + m_geom.gear_seat_width)/2.0 );
   
		// alloc the hash table for persistent manifold of gear-cylinder contacts
		hashed_contacts = new ChHashTable<int, GearPinCacheContact>(persistent_hashtable_dim);

    // keep track of some persistence of contact info
    m_persistentContactSteps.resize(m_shoes.size(), 0);
    m_contactPrevStep.resize(m_shoes.size(), false);
	}

	~GearPinCollisionCallback()
	{ 
		if (hashed_contacts) delete hashed_contacts; hashed_contacts=0;

	}

    
  class GearPinCacheContact{
	  public:
		  GearPinCacheContact()
			  {reactions_cache.resize(6,0);}

		  std::vector<float> reactions_cache; // same structure as in btManifoldPoint for other types of contact
	}; 

	ChHashTable<int, GearPinCacheContact>* hashed_contacts;

  // check the hash table for persistent contact
	void Found_GearPin_Contact(ChSharedPtr<ChBody>& gear, ChSharedPtr<ChBody> shoe,
    int shoeID, 
    ChVector<> pa, ChVector<> pb, 
    ChVector<> vn, 
    double mdist, 
    ChHashTable<int, GearPinCacheContact>* mhash)
	{
		std::vector<float> mreaction_cache;

		ChHashTable<int, GearPinCacheContact>::iterator mcached = mhash->find(shoeID);
		if (mcached == mhash->end())
      mreaction_cache =  (mhash->insert(shoeID))->second.reactions_cache;
		else
			mreaction_cache = mcached->second.reactions_cache;
			 
    // fill the contact container with info
		collision::ChCollisionInfo mcont;
		mcont.modelA = gear->GetCollisionModel();
		mcont.modelB = shoe->GetCollisionModel();
		mcont.vN = vn;
		mcont.vpA = pa;
		mcont.vpB = pb;
    mcont.distance = mdist;
    mcont.reaction_cache = &(mreaction_cache[0]);

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
    const ChVector<>& pin_cen_Nz)
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


  /// return true if there are contacts detected
  // centers w.r.t. global c-sys
  // be sure to pass centers as positive and negative z, w.r.t. gear c-sys
  bool NarrowPhase(const ChVector<>& gear_cen_Pz,
    const ChVector<>& gear_cen_Nz,
    const ChVector<>& pin_cen_Pz,
    const ChVector<>& pin_cen_Nz,
    const ChQuaternion<>& gear_rot)
  {

    // do narrow phase between this shoe and gear
    // find the closest gear to rotate the relate coordinates by the right angle
    size_t tooth_idx = Get_GearToothIdx(gear_cen_Pz, pin_cen_Pz, gear_rot);
    double rot_ang = CH_C_PI / m_geom.num_teeth + tooth_idx * (CH_C_PI / m_geom.num_teeth);

    // rotate the relative pos. things w.r.t gear c-sys
    ChQuaternion<> rot_q = Q_from_AngAxis(rot_ang, VECT_Z);
    ChQuaternion<> rot_q2 = Q_from_AngAxis(rot_ang + (CH_C_2PI / m_geom.num_teeth), VECT_Z ); // look at the next gear tooth
    ChFrame<> seat_frame(m_seat_pos_bar, QUNIT);

    ChVector<> m_seat_pos_bar_tooth = (rot_q * seat_frame).GetPos();
    ChVector<> m_tooth_cen_pos_bar_A = (rot_q * seat_frame).GetPos();
    ChVector<> m_tooth_cen_pos_bar_B = (rot_q2 * seat_frame).GetPos();

    return true;
  }
  /// based on the distance between input global centers, find the dist. relative to the gear
  /// c-sys. Return which gear tooth to perform narrow-phase with
  size_t Get_GearToothIdx(const ChVector<>& gear_cen,
    const ChVector<>& pin_cen,
    const ChQuaternion<>& gear_rot)
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
		CollisionGearPinFamily(msys, m_gear, m_shoes, this->hashed_contacts);
		
	}

  // function implementation
	void CollisionGearPinFamily(ChSystem* msys, ChSharedPtr<ChBody> gear,
    std::vector<ChSharedPtr<ChBody> > shoes,
    ChHashTable<int, GearPinCacheContact>* mhash)
	{
		//GetLog() << "hash statistics: loading=" << hashed_contacts->loading() << "   size=" << hashed_contacts->size() <<"\n";
		// look thru the shoe list, see if any pins are in contact with the concave gear seat surface.
    for(size_t idx = 0; idx < m_shoes.size(); idx++)
		{
      // global c-sys
      ChVector<> shoe_pos = m_shoes[idx]->GetPos();
      ChVector<> gear_pos_Pz = m_gear->GetPos() 
        + gear->GetRot().Rotate( m_seat_pos_bar );
      ChVector<> gear_pos_Nz = m_gear->GetPos() 
        + gear->GetRot().Rotate( -m_seat_pos_bar );

      // put the Shoe bounding sphere at the center of the pin, on the positive and negative z-dir, w.r.t. gear c-sys
      // TODO: relative to the shoe c-sys, is the pin -x or +x dir ??
      ChVector<> pin_pos_Pz = shoe_pos 
        + shoes[idx]->GetRot().Rotate( m_pin_pos_bar );
      ChVector<> pin_pos_Nz = shoe_pos 
        + shoes[idx]->GetRot().Rotate( -m_pin_pos_bar );

      // broad-phase passes?
      if( BroadphasePassed(gear_pos_Pz, gear_pos_Nz, pin_pos_Pz, pin_pos_Nz) )
      {

        if( NarrowPhase(gear_pos_Pz, gear_pos_Nz, pin_pos_Pz, pin_pos_Nz, gear->GetRot() ) )
          GetLog () << " fascinating .... \n\n\n";



        /*
			 
			  // Case 1: is the cylinder pin colliding with the base of the gear profile (concave cylinder)?
			  if (mpen_cyl<envelope)
			  {
				  ChVector<> drad = Vnorm(vrad);
				  ChVector<> pa(drad*vessell_rad); 
				  pa.y=vspherepos.y;
				  ChVector<> pb(drad*(mrad + sphere_rad));
				  pb.y=vspherepos.y;
				  ChVector<> vn(-drad);
				  FoundReactorParticleContact(vessell_body,family, ip,pa,pb,vn,-mpen_cyl,mhash);
			  }
			
        */


			}
		  else
      {
        // not within the bounding geometry, so no contact

      }
    }
  }

  bool Get_contactPrevStep(size_t idx) const { assert(idx<m_shoes.size()); return m_contactPrevStep[idx]; }

  // other virtuals

  // number of contacts detected
  int GetNcontacts_GearPin() { return m_Ncontacts; }


private:

  ChVector<> m_pin_pos_bar;  ///< center of pin cylinder on positive z-side, in shoe c-sys
  ChVector<> m_seat_pos_bar; ///< center of gear seat on pos. z side, in shoe c-sys

  // handles to bodies to check
  std::vector<ChSharedPtr<ChBody> >& m_shoes;
  ChSharedPtr<ChBody>& m_gear;
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