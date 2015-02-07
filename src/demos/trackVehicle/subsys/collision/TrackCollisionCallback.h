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
class GearPinGeometry : public ChShared
{
public:
  GearPinGeometry(double gear_base_radius = 0.211,  ///< gear base circle radius
    double gear_pitch_radius = 0.267, ///< center of the circle that uses gear_tooth_radius to define gear tooth surface
    double gear_circle_angle = 0.3,   ///< angle from the axis of symmetric the gear base circle is swept
    double gear_seat_width_max = 0.626, ///< max width of the gear seat, w.r.t. gear c-sys
    double gear_seat_width_min = 0.458, ///< min width of the gear seat, w.r.t. gear c-sys 
    ChVector<> tooth_mid_bar = ChVector<>(0.079815, 0.24719, 0.2712), ///< assume first seat bottom is directly above COG, then center of top of gear tooth is relative to local c-sys
    double tooth_len = 0.013119,  ///< length of top of gear tooth, in local XY plane
    double tooth_width = 0.0840,  ///< width of top of gear tooth, in local Z plane
    size_t num_teeth = 10.0,      ///< number of gear teeth
    double key_angle = 0.0,       ///< if the bottom of the tooth profile is not directly above the COG of the gear, rotation angle in rads
    double pin_radius = 0.0232,   ///< shoe pin radius
    double pin_width_max = 0.531,  ///< max total pin width
    double pin_width_min = 0.38,  ///< min total pin width
    double pin_x_offset = -0.07581, ///< x-offset of pin from center of shoe c-sys, in shoe c-sys
    double pin_y_offset = 0        ///< y-offset of pin from center of shoe c-sys
    ): m_gear_base_radius(gear_base_radius),
    m_gear_pitch_radius(gear_pitch_radius),
    m_gear_tooth_radius(gear_pitch_radius-gear_base_radius),
    m_gear_circle_angle(gear_circle_angle),
    m_gear_seat_width_max(gear_seat_width_max),
    m_gear_seat_width_min(gear_seat_width_min),
    m_tooth_mid_bar(tooth_mid_bar),
    m_tooth_len(tooth_len),
    m_tooth_width(tooth_width),
    m_num_teeth( num_teeth),
    m_key_angle(key_angle),
    m_pin_radius(pin_radius),
    m_pin_width_max(pin_width_max),
    m_pin_width_min(pin_width_min),
    m_pin_x_offset(pin_x_offset),
    m_pin_y_offset(pin_y_offset)
  {
    // make sure the geometric dimensons are valid
    assert(m_gear_seat_width_max - m_gear_seat_width_min > 0);
    assert(m_pin_width_max - m_pin_width_min > 0);
    assert(m_gear_pitch_radius - m_gear_base_radius > 0);
  }

  // gear geometry
  const double m_gear_base_radius; 
  const double m_gear_pitch_radius; 
  const double m_gear_tooth_radius;
  const double m_gear_circle_angle;
  const double m_gear_seat_width_max;
  const double m_gear_seat_width_min;
  const size_t m_num_teeth;
  const double m_key_angle;

  // gear tooth geometry
  const ChVector<> m_tooth_mid_bar;
  const double m_tooth_len;
  const double m_tooth_width;

  // shoe pin geometry
  const double m_pin_radius;
  const double m_pin_width_max;
  const double m_pin_width_min;
  const double m_pin_x_offset;
  const double m_pin_y_offset;
};

// Concave geometry (gear tooth seat) cannot be exactly represented by default collision primitives,
//  nor can it be accurately modeled with a mesh or convex hull.
// This custom collision checks the gear with all the track shoes
class GearPinCollisionCallback : public ChSystem::ChCustomComputeCollisionCallback
{
	public:
  /// all length units in meters
	GearPinCollisionCallback(std::vector<ChSharedPtr<ChBody>>& shoes,
    ChSharedPtr<ChBody>& gear_body,
    ChSharedPtr<GearPinGeometry> geom,
    int persistent_hashtable_dim = 1000
	) : m_shoes(shoes),
  m_gear(gear_body),
  m_geom(geom),
  m_persistent_hashtable_dim(persistent_hashtable_dim)
	{

    // two endpoints of cylinder pin, w.r.t. shoe c-sys. 
    // SYMMETRIC ABOUT XY PLANE (e.g., check for contact for -z)
    // point 1 = inner, 2 = outer
		m_p1_bar = ChVector<>(m_geom->m_pin_x_offset,
      m_geom->m_pin_y_offset,
      m_geom->m_pin_width_min/2.0);
    m_p2_bar = ChVector<>(m_geom->m_pin_x_offset,
      m_geom->m_pin_y_offset,
      m_geom->m_pin_width_max/2.0);

    // two endpoints of  the seat cylinder (there are as many as gear teeth
    // SYMMETRIC ABOUT XY PLANE (e.g., check for contact for -z)
    // point 1 = inner, 2 = outer
    m_seat1_bar.clear();
    m_seat2_bar.clear();
    m_seat1_bar.resize(m_geom->m_num_teeth);
    m_seat2_bar.resize(m_geom->m_num_teeth);
    for(int t_idx = 0; t_idx < m_geom->m_num_teeth; t_idx++)
    {
      m_seat1_bar[t_idx] = ChVector<>(m_geom->m_gear_base_radius*sin(m_geom->m_key_angle),
        m_geom->m_gear_base_radius*cos(m_geom->m_key_angle),
        m_geom->m_gear_seat_width_min/2.0);
      m_seat2_bar[t_idx] = ChVector<>(m_geom->m_gear_base_radius*sin(m_geom->m_key_angle),
        m_geom->m_gear_base_radius*cos(m_geom->m_key_angle),
        m_geom->m_gear_seat_width_max/2.0);
    }

		// alloc the hash table for persistent manifold of gear-cylinder contacts
		hashed_contacts = new ChHashTable<int, ReactCachedContact >(persistent_hashtable_dim);
	}

	~GearPinCollisionCallback()
	{ 
		if (hashed_contacts) delete hashed_contacts; hashed_contacts=0;

	}

    
  class ReactCachedContact{
	  public:
		  ReactCachedContact()
			  {reactions_cache[0]=reactions_cache[1]=reactions_cache[2]=reactions_cache[3]=reactions_cache[4]=reactions_cache[5]=0;}
		  float reactions_cache[6]; // same structure as in btManifoldPoint for other types of contact
	}; 

	ChHashTable<int, ReactCachedContact>* hashed_contacts;

  // check the hash table for persistent contact
	void Found_GearPin_Contact(ChSharedPtr<ChBody>& gear, ChSharedPtr<ChBody> shoe,
    int shoeID, 
    ChVector<> pa, ChVector<> pb, 
    ChVector<> vn, 
    double mdist, 
    ChHashTable<int, ReactCachedContact>* mhash)
	{
		float* mreaction_cache=0;

		ChHashTable<int, ReactCachedContact>::iterator mcached = mhash->find(shoeID);
		if (mcached == mhash->end())
			mreaction_cache=(mhash->insert(shoeID))->second.reactions_cache;
		else
			mreaction_cache=mcached->second.reactions_cache;
			 
    // fill the contact container with info
		collision::ChCollisionInfo mcont;
		mcont.modelA = gear->GetCollisionModel();
		mcont.modelB = shoe->GetCollisionModel();
		mcont.vN = vn;
		mcont.vpA = pa;
		mcont.vpB = pb;
    mcont.distance = mdist;
    mcont.reaction_cache = mreaction_cache;

		gear->GetSystem()->GetContactContainer();//->AddContact(mcont);
	}

	virtual void PerformCustomCollision(ChSystem* msys)
	{
		CollisionReactorFamily(msys, m_gear, m_shoes, this->hashed_contacts);
		
	}

	virtual void CollisionReactorFamily(ChSystem* msys, ChSharedPtr<ChBody> gear,
    std::vector<ChSharedPtr<ChBody>> shoes,
    ChHashTable<int, ReactCachedContact>* mhash)
	{
		//GetLog() << "hash statistics: loading=" << hashed_contacts->loading() << "   size=" << hashed_contacts->size() <<"\n";
		for(int t_idx = 0; t_idx < m_shoes.size(); t_idx++)
		{

      ChVector<> shoe_pos = m_shoes[t_idx]->GetPos();
      ChVector<> gear_pos = m_gear->GetPos();

      // broad-phase, is the distance between centers > sum of bounding sphere?
      if( (gear_pos - shoe_pos).Length() <= (m_geom->m_gear_pitch_radius + ChVector<>(m_geom->m_pin_x_offset + m_geom->m_pin_radius,
        m_geom->m_pin_y_offset, 
        m_geom->m_pin_width_max/2.0).Length()) )
      {

        // do narrow phase between this shoe and gear
        



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

        // Case 2: is the cylinder pin colliding with the top of the gear tooth plane?

        // Case 3: are the sides of the shoe colliding with the inner plane of the gear?

        // Case 4: do the pin ends collide with anything?


			}
		  else
      {
        // not within the bounding geometry, so no contact

      }
    }
  }


private:

  ChVector<> m_p1_bar;
  ChVector<> m_p2_bar;
  std::vector<ChVector<>> m_seat1_bar;
  std::vector<ChVector<>> m_seat2_bar;

  // handles to bodies to check
  std::vector<ChSharedPtr<ChBody>>& m_shoes;
  ChSharedPtr<ChBody>& m_gear;
  ChSharedPtr<GearPinGeometry>& m_geom;

  // hashtable
  size_t m_persistent_hashtable_dim;

};  // end class GearPinCollisionCallback


}  // end namespace chrono

#endif