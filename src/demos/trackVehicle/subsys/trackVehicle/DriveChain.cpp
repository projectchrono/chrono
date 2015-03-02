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
// Chain drive.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"

#include "core/ChFileutils.h"
#include "physics/ChGlobal.h"
#include "physics/ChContactContainer.h"

#include "DriveChain.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"
// custom collision detection classes
#include "subsys/collision/TrackCollisionCallback.h"


// to help write vectors and quats to file by overloading stringstream

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
// idler, relative to gear/chassis
const ChVector<> DriveChain::m_idlerPos(-2.1904, -0.1443, 0.2447); // relative to local csys
const ChQuaternion<> DriveChain::m_idlerRot(QUNIT);


/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
DriveChain::DriveChain(const std::string& name,
                       VisualizationType::Enum gearVis,
                       CollisionType::Enum gearCollide,
                       size_t num_idlers,
                       size_t num_rollers,
                       double gear_mass,
                       const ChVector<>& gear_inertia
): ChTrackVehicle(name, gearVis, gearCollide, gear_mass, gear_inertia, 1),
  m_num_rollers(num_rollers),
  m_num_idlers(num_idlers)
{
  // ---------------------------------------------------------------------------
  // Init any debugging, reporter variables
  m_SG_info.resize(2, ChVector<>() );
  m_SG_is_persistentContact_set.resize(2,false);
  m_SG_PosRel.resize(2, ChVector<>());
  m_SG_PosAbs.resize(2, ChVector<>());
  m_SG_Fn.resize(2, ChVector<>());
  


  // setup the chassis body    
  m_chassis->SetIdentifier(0);
  // basic body info. Not relevant since it's fixed.
  m_chassis->SetFrame_COG_to_REF(ChFrame<>() );
  // chassis is fixed to ground
  m_chassis->SetBodyFixed(true);

  // set any non-initialized base-class variables here
  // doesn't matter for the chassis, since no visuals used
  m_meshName = "na";
  m_meshFile = "none";
  m_chassisBoxSize = ChVector<>(2.0, 0.6, 0.75);
  
  // --------------------------------------------------------------------------
  // BUILD THE SUBSYSTEMS
  // drive gear, inherits drivechain's visual and collision types, mass, inertia, etc.
  m_gear = ChSharedPtr<DriveGear>(new DriveGear("drive gear",
    m_vis,
    m_collide,
    0,
    gear_mass,
    gear_inertia) );

 // idlers
  m_idlers.clear();
  m_idlers.resize(m_num_idlers);
  double idler_mass = 100.0; // 429.6
  ChVector<> idler_Ixx(gear_inertia);    // 12.55, 12.55, 14.7
  double tensioner_K = 40e3;
  double tensioner_C = tensioner_K * 0.08;
  m_idlers[0] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler",
    VisualizationType::Mesh,
    // VisualizationType::Primitives,
    CollisionType::Primitives,
    0,
    idler_mass,
    idler_Ixx,
    tensioner_K,
    tensioner_C) );

  // track chain system
  double shoe_mass = 18.03/4.0; // 18.03
  ChVector<> shoe_Ixx(0.22/4.0, 0.25/4.0, 0.04/4.0);  // 0.22, 0.25, 0.04
  m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain",
    VisualizationType::CompoundPrimitives,
    CollisionType::Primitives,
    0,
    shoe_mass,
    shoe_Ixx) );

  // create the powertrain, connect transmission shaft directly to gear shaft
  m_ptrains[0] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain ") );

  // support rollers, if any
  m_rollers.clear();
  m_rollers.resize(m_num_rollers);
  double roller_mass = 50.0;

  for(int j = 0; j < m_num_rollers; j++)
  {
    double roller_r = m_rollers[j]->GetRadius();
    double roller_w = m_rollers[j]->GetWidth();
    // assume constant density cylinder
    ChVector<> roller_Ixx = roller_mass * ChVector<>((3.0*roller_r*roller_r + roller_w*roller_w)/12.0,
      (3.0*roller_r*roller_r + roller_w*roller_w)/12.0,
      roller_r*roller_r/2.0);
    GetLog() << " I just manually calculated inertia, uh-oh \n\n Ixx = " << roller_Ixx << "\n";

    std::stringstream sr_s;
    sr_s << "support roller " << j;
    m_rollers[j] = ChSharedPtr<SupportRoller>(new SupportRoller(sr_s.str(),
      VisualizationType::Primitives,
      CollisionType::Primitives,
      0,
      roller_mass,
      roller_Ixx) );
  }

  if(m_num_idlers > 1)
  {
    // for now, just create 1 more idler
    m_idlers[1] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler 2",
    VisualizationType::Mesh,
    // VisualizationType::Primitives,
    CollisionType::Primitives,
    0,
    idler_mass,
    idler_Ixx) );
  }
}


DriveChain::~DriveChain()
{
  if(m_ownsSystem)
    delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void DriveChain::Initialize(const ChCoordsys<>& gear_Csys)
{
  // initialize the drive gear, idler and track chain
  double idler_preload = 10000;
  // m_idlerPosRel = m_idlerPos;
  m_idlerPosRel = ChVector<>(-2, -0.10, 0);
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(gear_Csys.pos, gear_Csys.rot));
  
  // Create list of the center location of the rolling elements and their clearance.
  // Clearance is a sphere shaped envelope at each center location, where it can
  //  be guaranteed that the track chain geometry will not penetrate the sphere.
  std::vector<ChVector<> > rolling_elem_locs; // w.r.t. chassis ref. frame
  std::vector<double> clearance;  // 1 per rolling elem  
  std::vector<ChVector<> > rolling_elem_spin_axis; /// w.r.t. abs. frame
  

  // initialize 1 of each of the following subsystems.
  // will use the chassis ref frame to do the transforms, since the TrackSystem
  // local ref. frame has same rot (just difference in position)
  // NOTE: move Gear Init() to after TrackChain Init(), but still add Gear info to list first.

  // drive sprocket is First added to the lists passed into TrackChain Init()
  rolling_elem_locs.push_back(ChVector<>() );
  clearance.push_back(m_gear->GetRadius() );
  rolling_elem_spin_axis.push_back(m_gear->GetBody()->GetRot().GetZaxis() );

  // initialize the support rollers.
  // Usually use 2, so spacing is based on that
  double spacing = m_idlerPosRel.Length();
  for(int r_idx = 0; r_idx < m_num_rollers; r_idx++)
  {
    ChVector<> roller_loc = m_chassis->GetPos();
    roller_loc.y = -0.8;
    roller_loc.x = gear_Csys.pos.x - 0.5 - r_idx*spacing/2.0;

    m_rollers[r_idx]->Initialize(m_chassis,
      m_chassis->GetFrame_REF_to_abs(),
      ChCoordsys<>(roller_loc, QUNIT) );

    // add to the points passed into the track chain
    rolling_elem_locs.push_back( roller_loc );
    clearance.push_back(m_rollers[r_idx]->GetRadius() );
    rolling_elem_spin_axis.push_back( m_rollers[r_idx]->GetBody()->GetRot().GetZaxis() );
  }

  // init the idler last
  m_idlers[0]->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    ChCoordsys<>(m_idlerPosRel, Q_from_AngAxis(CH_C_PI, VECT_Z) ),
    idler_preload);

  // add to the lists passed into the track chain Init()
  rolling_elem_locs.push_back(m_idlerPosRel );
  clearance.push_back(m_idlers[0]->GetRadius() );
  rolling_elem_spin_axis.push_back(m_idlers[0]->GetBody()->GetRot().GetZaxis() );

  // when there's only 2 rolling elements, the above locations will repeat
  // the first rolling element at the front and end of the vector.
  // So, just use the mid-point between the first two rolling elements.

  ChVector<> start_pos;
  if( clearance.size() < 3 )
  {
    start_pos = (rolling_elem_locs[0] + rolling_elem_locs[1])/2.0;
    start_pos.y += (clearance[0] + clearance[1])/2.0;
  }
  else
  {
    start_pos = (rolling_elem_locs.front() + rolling_elem_locs.back())/2.0;
    start_pos.y += (clearance.front() + clearance.back() )/2.0;
  }
  start_pos.x += 0.04;
  // NOTE: start_pos needs to somewhere on the top length of chain.
  // Rolling_elem_locs MUST be ordered from front-most to the rear, 
  //  w.r.t. the chassis x-dir, so chain links created in a clockwise direction.
  m_chain->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    rolling_elem_locs, clearance,
    rolling_elem_spin_axis,
    start_pos );
  
  // can set pin friction between adjoining shoes by activing damping on the DOF
  // m_chain->Set_pin_friction(2.0); // [N-m-sec] ???

  // now, init the gear
  m_gear->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    ChCoordsys<>(),
    m_chain->GetShoeBody() );

  // initialize the powertrain, drivelines
  m_ptrains[0]->Initialize(m_chassis, m_gear->GetAxle() );

  // extra idlers?
  if(m_num_idlers > 1)
  {
    // init this idler, and position it so
    // it snaps into place
    double preload_2 = 60000;
    double idler2_xoff = -1.5;
    double idler2_yoff = -2.1;
    double rotation_ang = CH_C_PI_4;

    // get the absolute c-sys of the gear, and set position relative to that point.
    ChCoordsys<> idler2_csys(m_gear->GetBody()->GetPos(), m_gear->GetBody()->GetRot());
    idler2_csys.pos.x += idler2_xoff;
    idler2_csys.pos.y += idler2_yoff;
    // rotation should set the -x dir 
    idler2_csys.rot = Q_from_AngAxis(rotation_ang, VECT_Z);

    // set the idler relative to the chassis/gear origin
    m_idlers[1]->Initialize(m_chassis, 
      m_chassis->GetFrame_REF_to_abs(),
      idler2_csys,
      preload_2);

  }
}

void DriveChain::Update(double time,
                        double throttle,
                        double braking)
{
  // update left and right powertrains, with the new left and right throttle/shaftspeed
  m_ptrains[0]->Update(time, throttle, GetDriveshaftSpeed(0) );

}

void DriveChain::Advance(double step)
{
  double t = 0;
  m_system->SetIterLCPmaxItersStab(100);
  m_system->SetIterLCPmaxItersSpeed(150);
  double settlePhaseA = 0.05;
  double settlePhaseB = 0.1;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( m_system->GetChTime() < settlePhaseA )
    {
      m_system->SetIterLCPmaxItersStab(80);
      m_system->SetIterLCPmaxItersSpeed(100);
    } else if ( m_system->GetChTime() < settlePhaseB )
    {
      m_system->SetIterLCPmaxItersStab(100);
      m_system->SetIterLCPmaxItersSpeed(150);
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


// call the chain function to update the constant damping coef.
void DriveChain::SetShoePinDamping(double damping)
{
  m_chain->Set_pin_friction(damping);
}


// get some data about the gear and its normal and friction contact forces each timestep.
// info = (max, avg, stdev = sigma)
// returns: number of contacts for the gear body.
int DriveChain::reportGearContact(ChVector<>& Fn_info, ChVector<>& Ft_info)
{
  // scans the contacts, reports the desired info about the gear, including:
  //  max, sum and variance of both normal and friction forces
  class _gear_report_contact : public ChReportContactCallback
  {
  public:

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
      // does this collision include the gear?
      if( modA->GetFamily() == (int)CollisionFam::Gear || modB->GetFamily() == (int)CollisionFam::Gear)
      {
        // don't count collisions w/ 0 normal force
        if( react_forces.x > 0 )
        {
          // average value of norm, friction forces from last time scanned contact
          double Fn_bar_n1 = 0;
          double Ft_bar_n1 = 0;
          if(m_num_gear_contacts > 0)
          {
            Fn_bar_n1 = m_Fn_sum / m_num_gear_contacts;
            Ft_bar_n1 = m_Ft_sum / m_num_gear_contacts;
          }
          // increment the data for this contact
          m_num_gear_contacts++;
          // incrase normal force
          m_Fn_sum += react_forces.x;
          // current friction magnitude in local c-syss
          double Ft = ChVector<>(0, react_forces.y, react_forces.z).Length();
          m_Ft_sum += Ft; 
        
          // update max normal and force magnitude
          if( react_forces.x > m_Fn_max )
            m_Fn_max = react_forces.x;
          if(Ft > m_Ft_max)
            m_Ft_max = Ft;

          // compute some statistics. 
          // update normal force avg, variance
          double Fn_bar = Fn_bar_n1 + (react_forces.x - Fn_bar_n1)/m_num_gear_contacts;
          // Fn_var is from last step, e.g. Fn_var_n1
          double Fn_sig2 = (m_Fn_var*(m_num_gear_contacts-1) + (react_forces.x - Fn_bar_n1)*(react_forces.x - Fn_bar)) /m_num_gear_contacts;
          // with updated average and variance, update the data
          m_Fn_var = Fn_sig2;

          // similarly for the friction forces
          double Ft_bar = Ft_bar_n1 + (Ft - Ft_bar_n1)/m_num_gear_contacts;
          // like the normal force, the Ft_var has not been updated this step, so it is actually Ft_var_n1
          double Ft_sig2 = ((m_num_gear_contacts-1)*m_Ft_var + (Ft - Ft_bar_n1)*(Ft - Ft_bar)) / m_num_gear_contacts;
          // with  updated variance, update the data
          m_Ft_var = Ft_sig2;
        }
      }
      return true; // to continue scanning contacts
    }

    // NOTE: must initialize these before use!
    // relevant gear info
    int m_num_gear_contacts;
    double m_Fn_max;    // max normal force
    double m_Fn_sum;    // reaction normal force sum
    double m_Fn_var;    // reaction normal force population variance, sigma_n^2

    double m_Ft_max;  // tangent/friction force max
    double m_Ft_sum;  // friction force sum
    double m_Ft_var;  // friction force variance, sigma_t^2

  };

  // setup the reporter, init any variables
  _gear_report_contact reporter;
  reporter.m_num_gear_contacts = 0;
  reporter.m_Fn_max = 0;
  reporter.m_Fn_sum = 0;
  reporter.m_Fn_var = 0;
  reporter.m_Ft_max = 0;
  reporter.m_Ft_sum = 0;
  reporter.m_Ft_var = 0;

  // pass the reporter callback to the system
  m_system->GetContactContainer()->ReportAllContacts(&reporter);

  // set any data here from the reporter, and return # of bodies in contact with the gear
  double Fn_avg = (reporter.m_num_gear_contacts > 0) ? reporter.m_Fn_sum/reporter.m_num_gear_contacts: 0;
  Fn_info = ChVector<>(reporter.m_Fn_max,
    Fn_avg,
    std::sqrt( reporter.m_Fn_var) );

  double Ft_avg = (reporter.m_num_gear_contacts > 0) ? reporter.m_Ft_sum/reporter.m_num_gear_contacts: 0;
  Ft_info = ChVector<>(reporter.m_Ft_max,
    Ft_avg,
    std::sqrt(reporter.m_Ft_var) );

  return reporter.m_num_gear_contacts;
}

// Get some data about the gear and its normal and friction contact forces with the specified shoe each timestep.
// Returns: total number of contacts between the specified shoe body and gear body
// Sets non-const inputs with contact persistence info, # of contacts, Fn, Ft statistics.
// SG_info = (Num_contacts, t_persist, t_persist_max)
// Force_mag_info = (Fn, Ft, 0)
// PosRel, VRel = relative pos, vel of contact point, relative to gear c-sys
// NormDirRel = tracked contact normal dir., w.r.t. gear c-sys
int DriveChain::reportShoeGearContact(const std::string& shoe_name,
                                      std::vector<ChVector<> >& SG_info,
                                      std::vector<ChVector<> >& Force_mag_info,
                                      std::vector<ChVector<> >& PosRel_contact,
                                      std::vector<ChVector<> >& VRel_contact,
                                      std::vector<ChVector<> >& NormDirRel_contact)
{
  
  // scans the contacts, reports the desired info about any shoe 0 and gear collision pairs
  class _shoeGear_report_contact : public ChReportContactCallback
  {
  public:

    // get the location of the contact, on the specified body type, with specified body COG pos in global coords.
    // If cannot find the collision type for either modA or modB family, just use modB
    const ChVector<> getContactLocRel(const ChVector<>& pA,
                                      const ChVector<>& pB,
                                      collision::ChCollisionModel* modA,
                                      collision::ChCollisionModel* modB,
                                      CollisionFam::Enum fam)
    {
      ChVector<> pos_abs = ChVector<>();
      ChFrame<> body_frame = ChFrame<>();
      // is the collision body A?
      if(modA->GetFamily() == (int)fam)
      {
        // A is the gear body
        pos_abs = pA - ((ChBody*)modA->GetPhysicsItem())->GetPos(); // abs dist. of contact point from the gear center
        body_frame.SetRot( ((ChBody*)modA->GetPhysicsItem())->GetRot() );
      }
      else
      {
        // assume gear is body B
        pos_abs = pB - ((ChBody*)modB->GetPhysicsItem())->GetPos(); // abs dist. of contact point from the gear center
        body_frame.SetRot( ((ChBody*)modB->GetPhysicsItem())->GetRot() );
      }

      // get the distance from gear center to contact point on the gear, in gear c-sys
      ChVector<> loc_rel =  body_frame.TransformParentToLocal(pos_abs);
      return loc_rel;
    }

    // get the normal direction of the contact normal, on the specified body type,
    //  relative to the body c-sys
    // If cannot find the collision type for either modA or modB family, just use modB
    // plane_coord has normal w.r.t. modA
    const ChVector<> getContactNormDirRel(const ChVector<> normDir_abs,
                                      collision::ChCollisionModel* modA,
                                      collision::ChCollisionModel* modB,
                                      CollisionFam::Enum fam)
    {
      // TODO: does the normal direction switch depending on which collision model ends up being = to fam ???
      ChFrame<> body_frame = ChFrame<>(); // should end up being the gear body orientation
      ChVector<> normDir_rel = ChVector<>();  // relative to gear c-sys
       // is the collision body A?
      if(modA->GetFamily() == (int)fam)
      {
        body_frame.SetRot( ((ChBody*)modA->GetPhysicsItem())->GetRot() );
        normDir_rel = body_frame.TransformParentToLocal(normDir_abs);
      }
      else
      {
        // assume collision is body B
        body_frame.SetRot( ((ChBody*)modB->GetPhysicsItem())->GetRot() );
        normDir_rel = body_frame.TransformParentToLocal(-normDir_abs);
      }
      
      return normDir_rel;
    }

    // is this contact point on the gear body close enough to the one we're tracking?
    // NOTE: per_step_len_max needs to be set depending on how much the contact point can
    //    logically move between the two times you called this function.
    bool is_PosRel_match(const ChVector<>& test_PosRel,
      int idx,
      double per_step_len_max = 0.01 ///< relative location of contact on gear body is close enough to tracked contact to be used as the one we follow
      )
    {
      if( (m_PosRel[idx] - test_PosRel).Length() < per_step_len_max )
      {
        return true;
      }

      return false;
    }

    // based on specified Collision fam, set the contact data for the specified persistent contact index:
    // m_Fn
    // m_Ft
    // m_NormDirRel
    // m_PosAbs
    // mNormDirAbs
    void SetPeristentContactInfo(const ChVector<>& pA,
      const ChVector<>& pB,
      const ChMatrix33<>& plane_coord,
      const ChVector<>& react_forces,
      collision::ChCollisionModel* modA,
      collision::ChCollisionModel* modB,
      CollisionFam::Enum fam,
      int idx)
    {
      m_Fn[idx] = react_forces.x;
      m_Ft[idx] = ChVector<>(0,react_forces.y,react_forces.z).Length();
      if( modA->GetFamily() == (int)fam)
      {
        // gear body is A, normal will point in the right direction
        m_NormDirRel[idx] = getContactNormDirRel(plane_coord.Get_A_Xaxis(),modA,modB,
          CollisionFam::Gear);
        m_PosAbs[idx] = pA;
        m_NormDirAbs[idx] = plane_coord.Get_A_Xaxis();
      }
      else {
        // gear body is B, switch normal dir
        m_NormDirRel[idx] = getContactNormDirRel(-(plane_coord.Get_A_Xaxis()),modA,modB,
          CollisionFam::Gear);
        m_PosAbs[idx] = pB;
        m_NormDirAbs[idx] = -(plane_coord.Get_A_Xaxis());

      }
    }

    // add the absolute location, absolute normal force vector for the specified
    // collisionFam, for all the contacts between the shoe and gear
    void AddContactInfo_all(const ChVector<>& pA,
                        const ChVector<>& pB,
                        const ChMatrix33<>& plane_coord,
                        const ChVector<>& react_forces,
                        collision::ChCollisionModel* modA,
                        collision::ChCollisionModel* modB,
                        CollisionFam::Enum fam)
    {
      if( modA->GetFamily() == (int)fam)
      {
        // gear body is A, normal will point in the right direction
        m_ContactPos_all.push_back(pA);
        m_ContactFn_all.push_back( plane_coord.Get_A_Xaxis()*react_forces.x);
      }
      else {
        // fam body is B
        m_ContactPos_all.push_back(pB);
        m_ContactFn_all.push_back( -(plane_coord.Get_A_Xaxis())*react_forces.x);

      }
    }

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
      // if in contact with the gear, and the other body is the specified shoe
      if( (modA->GetFamily() == (int)CollisionFam::Gear && modB->GetPhysicsItem()->GetNameString() == m_shoe_name )
        || (modB->GetFamily() == (int)CollisionFam::Gear && modA->GetPhysicsItem()->GetNameString() == m_shoe_name ) )
      {
        // don't count collisions w/ normal force = 0
        if( react_forces.x > 0 )
        {
          // this is a non-zero contact force between the shoe and the gear
          m_num_contacts++;

          // get the relative location of the contact point on the gear body
          ChVector<> gearpt_PosRel = getContactLocRel(pA,pB,modA,modB,
            CollisionFam::Gear);

          // positive z-relative position will be index 0, else index 1.
          int index = (gearpt_PosRel.z > 0) ? 0 : 1;
          // count the number of contats on each side
          m_num_contacts_side[index]++;
          // see if the relative position of the contact point has been set on this side of the gear
          if( !m_is_persistentContact_set[index])
          {
            // set the persistent contact info on this side of the gear
            m_PosRel[index] = gearpt_PosRel;
            // use this relative position next time
            m_is_persistentContact_set[index] = true;

            // set some other useful data
            // get the normal force direction relative to the gear
            SetPeristentContactInfo(pA,pB, plane_coord, react_forces, modA, modB, 
              CollisionFam::Gear,
              index);

            if(0)
            {
              GetLog () << " \n\n ***********     setting the gear contact point at time : " << modA->GetPhysicsItem()->GetSystem()->GetChTime()
                << " , collision index: " << index
                << "\n rel pos : " << m_PosRel[index] 
                << "\n";
            }   
          }
          else
          {
            // relative loc has already been set. 
            // See if this relative contact position is close to already set location.
            // If within some relative distance, assume this is the same contact from last step.
            if( is_PosRel_match(gearpt_PosRel, index) )
            {
              // increment the total time this contact has been persistent
              m_t_persist[index] += modA->GetPhysicsItem()->GetSystem()->GetStep();
              // see if the time is larger than the last max
              if( m_t_persist[index] > m_t_persist_max[index])
                m_t_persist_max[index] = m_t_persist[index];

              // set the pos, vel data
              // velocity is the relative distance between the contact pts between time steps, w.r.t. gear c-sys
              m_VelRel[index] = (gearpt_PosRel - m_PosRel[index]) / modA->GetPhysicsItem()->GetSystem()->GetStep();
              // done using last step relative position of contact pt on gear body, update for current contact
              m_PosRel[index] = gearpt_PosRel;

              // set the other data
              SetPeristentContactInfo(pA,pB, plane_coord, react_forces, modA, modB, 
                CollisionFam::Gear,
                index);

            }
            else
            {
              // this is a different the point on the shoe than the one tracked,
              // want to be able to plot it in Irrlicht
              AddContactInfo_all(pA,pB,plane_coord,react_forces,modA,modB,
                CollisionFam::Gear);
            }
          }
        }
      }

      return true; // to continue scanning contacts
    }

    // NOTE: must initialize 
    // relevant gear info
    std::string m_shoe_name;      // string name for the shoe to check for collision with 
    int m_num_contacts;  // contacts this step

    // NOTE: for this type of gear/shoe pin contact, two pins on each shoe are in contact with
    //      the gear. EXPECT to have symmetric contact behavior, and a single contact point,
    //      where the contact force magnitude and relative direction vector vary smootly through time.
    // So, track two contacts for the shoe/gear contact, one w/ positive z "Pz", the other negative z, "Nz"
    std::vector<double> m_t_persist;     // time the contacts have been in persistent contact
    std::vector<double> m_t_persist_max; // max time the shoe stayed in contact with the gear
    std::vector<int> m_num_contacts_side;
    std::vector<ChVector<> > m_PosRel;  // position of a contact to follow, relative to gear c-sys
    std::vector<ChVector<> > m_PosAbs;
    std::vector<ChVector<> > m_VelRel;  // velocity of contact followed, relative to gear c-sys
    std::vector<ChVector<> > m_NormDirRel;  // relative norm. dir of contact
    std::vector<ChVector<> > m_NormDirAbs;
    std::vector<bool> m_is_persistentContact_set; // has a contact point to follow been chosen? if yes, check to see, else write to above coord

    std::vector<double> m_Fn;  // normal force this step
    std::vector<double> m_Ft;  // max friction force

    // all other contacts
    int m_num_shoeGear_contacts_Pz; // # of contacts with z-positive, relative to gear c-sys
    int m_num_shoeGear_contacts_Nz; // # of contacts with z-negative, relative to gear c-sys
    std::vector<ChVector<> > m_ContactPos_all; // abs. coords, for irrlicht
    std::vector<ChVector<> > m_ContactFn_all;  // abs. coords, for irrlicht vis

  };

  // setup the reporter, init any variables that are not history dependent
  _shoeGear_report_contact reporter;
  reporter.m_shoe_name = shoe_name;
  reporter.m_num_contacts = 0;
  reporter.m_num_contacts_side.resize(2, 0);
  reporter.m_PosRel.resize(2, ChVector<>());
  reporter.m_PosAbs.resize(2, ChVector<>() );
  reporter.m_VelRel.resize(2, ChVector<>() );
  reporter.m_NormDirRel.resize(2, ChVector<>() );
  reporter.m_NormDirAbs.resize(2, ChVector<>() );
  // normal, friction force magnitude
  reporter.m_Fn.resize(2, 0); // normal reaction force
  reporter.m_Ft.resize(2, 0); // friction reaction force

  // ----------------------------------
  // history dependent variables.
  // SG_info = (Num_contacts, t_persist, t_persist_max), carry over timers.
  reporter.m_t_persist.resize(2,0);
  reporter.m_t_persist_max.resize(2,0);
  for(int i = 0; i < 2; i++)
  {
    reporter.m_t_persist[i] = m_SG_info[i].y;
    reporter.m_t_persist_max[i] = m_SG_info[i].z;
  }
  // track a single shoe pin/gear contact point relative pos,vel., if it has been set, for each side of the gear.
  reporter.m_is_persistentContact_set.resize(2,0);
  for(int pc = 0; pc < 2; pc++)
  {
    reporter.m_is_persistentContact_set[pc] = m_SG_is_persistentContact_set[pc];
    if(reporter.m_is_persistentContact_set[pc])
      reporter.m_PosRel[pc] = m_SG_PosRel[pc];
    else
    {
      // when not set (between contact events), keep the z-coord (so the contact doesn't swap to the other side)
      reporter.m_PosRel[pc] = ChVector<>(0, 0, m_SG_PosRel[pc].z);
      // reporter.m_PosRel = ChVector<>();
    }
  }
  // to be complete
  reporter.m_ContactPos_all.clear();
  reporter.m_ContactFn_all.clear();

  // pass the reporter callback to the system
  m_system->GetContactContainer()->ReportAllContacts(&reporter);
  
  // fill the output data vectors. MaKe sure they are empty first
  SG_info.clear();
  Force_mag_info.clear();
  PosRel_contact.clear();
  VRel_contact.clear();
  NormDirRel_contact.clear();
  // fill both persistent contacts
  for(int rc = 0; rc < 2; rc++)
  {
    // process any history dependent values after reporter is called
    // reset persistent time counter when no contacts
    if(reporter.m_Fn[rc] == 0)
    {
      reporter.m_t_persist[rc] = 0;
      // allow the relative position of the shoe-gear contact to track to reset
      // reporter.m_PosRel = ChVector<>();
      reporter.m_is_persistentContact_set[rc] = false;
    }

    // set any data here from the reporter,
    SG_info.push_back( ChVector<>(reporter.m_num_contacts_side[rc],
      reporter.m_t_persist[rc],
      reporter.m_t_persist_max[rc]) );

    Force_mag_info.push_back( ChVector<>(reporter.m_Fn[rc],
      reporter.m_Ft[rc],
      0 ) );

    // set loc, vel, norm. force dir. of contact point, relative to gear csys
    PosRel_contact.push_back( reporter.m_PosRel[rc]);
    VRel_contact.push_back( reporter.m_VelRel[rc]);
    NormDirRel_contact.push_back( reporter.m_NormDirRel[rc]);

    
    // finally, set any data that should persist to the next time step to the ChainSystem.
    m_SG_Fn.push_back( reporter.m_NormDirAbs[rc] * reporter.m_Fn[rc]);

    // all of these should be vector copy/assign, moved outside the for loop
    /*
    m_SG_info[rc] = SG_info[rc];
    m_SG_is_persistentContact_set[rc] = reporter.m_is_persistentContact_set[rc];
    m_SG_PosRel[rc] = reporter.m_PosRel[rc];

    // set any aother desired info, e.g.  for plotting the contact point and normal dir.
    m_SG_PosAbs[rc] = reporter.m_PosAbs[rc];
    m_SG_Fn[rc] = reporter.m_NormDirAbs[rc] * reporter.m_Fn[rc];

    m_SG_ContactPos_all[rc] = reporter.m_ContactPos_all[rc];
    m_SG_ContactFn_all[rc] = reporter.m_ContactFn_all[rc];
    */
  }

  // finally, set any data that should persist to the next time step to the ChainSystem.
  m_SG_numContacts = reporter.m_num_contacts;
  m_SG_info = SG_info;
  m_SG_is_persistentContact_set = reporter.m_is_persistentContact_set;
  m_SG_PosRel = reporter.m_PosRel;

  // set any aother desired info, e.g.  for plotting the contact point and normal dir.
  m_SG_PosAbs = reporter.m_PosAbs;

  m_SG_ContactPos_all = reporter.m_ContactPos_all;
  m_SG_ContactFn_all = reporter.m_ContactFn_all;

  //  # of contacts between specified shoe and gear body
  return reporter.m_num_contacts;
}



// Log constraint violations
// -----------------------------------------------------------------------------
void DriveChain::LogConstraintViolations(bool include_chain)
{
  GetLog().SetNumFormat("%16.4e");

  // Report constraint violations for the gear revolute joint
  GetLog() << "\n---- Gear constraint violations\n\n";
  m_gear->LogConstraintViolations();

  // report violations for idler constraints
  for(int id = 0; id < m_num_idlers; id++)
  {
    GetLog() << "\n---- idler # " << id << " constraint violations\n\n";
    m_idlers[id]->LogConstraintViolations();
  }

  // violations of the roller revolute joints
  for(int roller = 0; roller < m_num_rollers; roller++)
  {
    GetLog() << "\n---- Roller # " << roller << " constrain violations\n\n";
    m_rollers[roller]->LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");
}

// Write constraint violations of subsystems, in order, to the ostraem
// -----------------------------------------------------------------------------
void DriveChain::SaveConstraintViolations(bool include_chain)
{
  if( !m_log_file_exists ) 
  {
    std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
  }
  double t = m_system->GetChTime();
  // call the subsystems in the same order as the headers are set up
  std::stringstream ss_g;
  ss_g << t;
  m_gear->SaveConstraintViolations(ss_g);
  ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str(), std::ios::app);
  ofileGCV << ss_g.str().c_str();

  // report violations for idler constraints
  for(int id = 0; id < m_num_idlers; id++)
  {
    std::stringstream ss_id;
    ss_id << t;
    m_idlers[id]->SaveConstraintViolations(ss_id);
    ChStreamOutAsciiFile ofileICV(m_filename_ICV[id].c_str(), std::ios::app);
    ofileICV << ss_id.str().c_str();
  }

  // violations of the roller revolute joints
  for(int roller = 0; roller < m_num_rollers; roller++)
  {
    std::stringstream ss_r;
    ss_r << t;
    m_rollers[roller]->SaveConstraintViolations(ss_r);
    ChStreamOutAsciiFile ofileRCV(m_filename_RCV[roller].c_str(), std::ios::app);
    ofileRCV << ss_r.str().c_str();
  }

}

// write output


// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void DriveChain::Setup_log_to_file(int what,
                               const std::string& filename,
                               const std::string& data_dirname)
{
  m_save_log_to_file = false;
  m_log_what_to_file = what;

  // create the directory for the data files
  if(ChFileutils::MakeDirectory(data_dirname.c_str()) < 0) {
    std::cout << "Error creating directory " << data_dirname << std::endl;
  }
  m_log_file_name = data_dirname + "/" + filename;

  // has the chain been created?
  if(m_chain)
  {
    // have the chain, the last subsystem created, been initialized?
    if(m_chain->Get_numShoes() )
    {
      m_save_log_to_file = true;
      GetLog() << " SAVING OUTPUT DATA TO FILE: \n " << filename.c_str() << "\n";
      create_fileHeaders(what);
      m_log_file_exists = true;

      // write the system heirarchy and ChSystem data also
      GetLog() << " SAVING model heirarchy and ChSystem details \n";
      ChStreamOutAsciiFile ofile_hier( (m_log_file_name +"_Heirarchy.csv").c_str() );
      m_system->ShowHierarchy(ofile_hier); 
      ChStreamOutAsciiFile ofile_system( (m_log_file_name +"_ChSystem.csv").c_str() );
      m_system->StreamOUT( ofile_system );

    }
    else
    {
      GetLog() << " no shoes were initialized, not saving data ";
    }
  }
  else
  {
    GetLog() << " chain subsystem not created yet, not saving data";
  }

  // initialize the rig input values to zero
}


void DriveChain::Log_to_console(int console_what)
{
  GetLog().SetNumFormat("%10.2f");

  if (console_what & DBG_FIRSTSHOE)
  {
    // first shoe COG state data, and first pin force/torque
    GetLog() << "\n---- shoe 0 : " << m_chain->GetShoeBody(0)->GetName() 
      << "\n COG Pos [m] : "  <<  m_chain->GetShoeBody(0)->GetPos() 
      << "\n COG Vel [m/s] : "  <<  m_chain->GetShoeBody(0)->GetPos_dt() 
      << "\n COG Acc [m/s2] : "  <<  m_chain->GetShoeBody(0)->GetPos_dtdt()
      << "\n COG omega [rad/s] : "  <<  m_chain->GetShoeBody(0)->GetRot_dt() 
      << "\n";

    // shoe pin tension
    GetLog() << " pin 0 reaction force [N] : "  <<  m_chain->GetPinReactForce(0) << "\n";
    GetLog() << "pin 0 reaction torque [N-m] : "  <<  m_chain->GetPinReactTorque(0) << "\n";

    // shoe - gear contact details
    if(1)
    {
      //  specify some collision info with the gear
      std::vector<ChVector<> > sg_info;  // output data set
      std::vector<ChVector<> > Force_mag_info;  // contact forces, (Fn, Ft, 0),
      std::vector<ChVector<> > Ft_info;  // per step friction contact force statistics
      std::vector<ChVector<> > PosRel_contact; // location of contact point, relative to gear c-sysss
      std::vector<ChVector<> > VRel_contact; // velocity of contact point, relative to gear c-sys
      std::vector<ChVector<> > NormDirRel_contact; // tracked contact normal dir., w.r.t. gear c-sys
      // sg_info = (Num_contacts, t_persist, t_persist_max)
      reportShoeGearContact(m_chain->GetShoeBody(0)->GetNameString(),
        sg_info,
        Force_mag_info,
        PosRel_contact,
        VRel_contact,
        NormDirRel_contact);

      GetLog() << "\n ---- Shoe - gear contact info, INDEX 0 :"
        <<"\n (# contacts, time_persist, t_persist_max) : " << sg_info[0]
        <<"\n force magnitude, (Fn, Ft, 0) : " << Force_mag_info[0]
        <<"\n contact point rel pos : " << PosRel_contact[0]
        <<"\n contact point rel vel : " << VRel_contact[0]
        <<"\n contact point rel norm. dir : " << NormDirRel_contact[0]
        <<"\n";
    }
  }

  if (console_what & DBG_GEAR)
  {
    // gear state data, contact info
    GetLog() << "\n---- Gear : " << m_gear->GetBody()->GetName() 
      << "\n COG Pos [m] : " << m_gear->GetBody()->GetPos() 
      << "\n COG Vel [m/s] : " << m_gear->GetBody()->GetPos_dt() 
      << "\n COG omega [rad/s] : " << m_gear->GetBody()->GetRot_dt().Q_to_NasaAngles()
      << "\n";

     // find what's in contact with the gear by processing all collisions with a special callback function
    ChVector<> Fn_info = ChVector<>();
    ChVector<> Ft_info = ChVector<>();
    // info is: (max, avg., variance)
    int num_gear_contacts = reportGearContact(Fn_info, Ft_info);

    GetLog() << "\n     Gear Contact info"
      << "\n # of contacts : " << num_gear_contacts
      << "\n normal (max, avg, variance) : " << Fn_info
      << "\n tangent (max, avg, variance) : " << Ft_info
      <<"\n";

  }

  if (console_what & DBG_IDLER)
  {
    GetLog() << "\n---- Idler : " << m_idlers[0]->GetBody()->GetName() 
      <<"\n COG Pos [m] : " << m_idlers[0]->GetBody()->GetPos() 
      <<"\n COG Vel [m/s] : " << m_idlers[0]->GetBody()->GetPos_dt()
      <<"\n COG omega [rad/s] : " << m_idlers[0]->GetBody()->GetRot_dt().Q_to_NasaAngles()
      <<"\n spring react F [N] : " << m_idlers[0]->GetSpringForce()
      <<"\n react from K [N] : " << m_idlers[0]->Get_SpringReact_Deform()
      <<"\n react from C [N] : " << m_idlers[0]->Get_SpringReact_Deform_dt();
  }

  if (console_what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations(false);
  }

  if (console_what & DBG_PTRAIN)
  {
    GetLog() << "\n ---- powertrain \n throttle : " << m_ptrains[0]->GetThrottle()
      <<"\n motor speed [RPM] : " << m_ptrains[0]->GetMotorSpeed() * 60.0 / (CH_C_2PI)
      <<"\n motor torque [N-m] : " << m_ptrains[0]->GetMotorTorque()
      <<"\n output torque [N-m : " << m_ptrains[0]->GetOutputTorque()
      <<"\n";
  
  }

  if(console_what & DBG_COLLISIONCALLBACK)
  {
    GetLog() << "\n ---- collision callback info :"
      << "\n Contacts this step: " << m_gear->GetCollisionCallback()->GetNcontacts()
      << "\n Broadphase passed this step: " << m_gear->GetCollisionCallback()->GetNbroadPhasePassed()
      << "\n Sum contacts, +z side: " << m_gear->GetCollisionCallback()->Get_sum_Pz_contacts()
      << "\n Sum contacts, -z side: " << m_gear->GetCollisionCallback()->Get_sum_Nz_contacts()
      <<"\n";
  }

  GetLog().SetNumFormat("%g");

}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void DriveChain::Log_to_file()
{
  // told to save the data?
  if( m_save_log_to_file )
  {
    if( !m_log_file_exists ) 
    {
      std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
    }
    // open the file to append
    // open the data file for writing the header

    // write the simulation time
    double t = m_system->GetChTime();

    // python pandas expects csv w/ no whitespace
    if( m_log_what_to_file & DBG_FIRSTSHOE )
    {
      std::stringstream ss;
      // time,x,y,z,vx,vy,vz,ax,ay,az,wx,wy,wz,fx,fy,fz
      ss << t <<","<< m_chain->GetShoeBody(0)->GetPos() 
        <<","<<  m_chain->GetShoeBody(0)->GetPos_dt() 
        <<","<<  m_chain->GetShoeBody(0)->GetPos_dtdt()
        <<","<<  m_chain->GetShoeBody(0)->GetWvel_loc()
        <<","<<  m_chain->GetPinReactForce(0)
        <<","<< m_chain->GetPinReactTorque(0)
        <<"\n";
      // open the file for appending, write the data.
      ChStreamOutAsciiFile ofile(m_filename_DBG_FIRSTSHOE.c_str(), std::ios::app);
      ofile << ss.str().c_str();

      // second file, to specify some collision info with the gear
      double num_contacts = 0;
      std::vector<ChVector<> > sg_info;  // output data set
      std::vector<ChVector<> > Force_mag_info;  // per step contact force magnitude, (Fn, Ft, 0)
      std::vector<ChVector<> > PosRel_contact; // location of a contact point relative to the gear c-sys
      std::vector<ChVector<> > VRel_contact;  // follow the vel. of a contact point relative to the gear c-sys
      std::vector<ChVector<> > NormDirRel_contact; // tracked contact normal dir., w.r.t. gear c-sys
      // sg_info = (Num_contacts, t_persist, t_persist_max)
      num_contacts = reportShoeGearContact(m_chain->GetShoeBody(0)->GetNameString(),
        sg_info,
        Force_mag_info,
        PosRel_contact,
        VRel_contact,
        NormDirRel_contact);

      // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
      // "time,Ncontacts,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,VzRelP,normDirRelxP,normDirRelyP,normDirRelzP
      //  ,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,normDirRelxN,normDirRelyN,normDirRelzN "
    std::stringstream ss_sg;
      ss_sg << t <<"," << num_contacts
        <<"," << sg_info[0]
        <<","<< Force_mag_info[0].x
        <<","<< Force_mag_info[0].y
        <<","<< PosRel_contact[0]
        <<","<< VRel_contact[0]
        <<","<< NormDirRel_contact[0]
        <<"," << sg_info[1]
        <<","<< Force_mag_info[1].x
        <<","<< Force_mag_info[1].y
        <<","<< PosRel_contact[1]
        <<","<< VRel_contact[1]
        <<","<< NormDirRel_contact[1]
        <<"\n";
      ChStreamOutAsciiFile ofile_shoeGear(m_filename_DBG_shoeGear.c_str(), std::ios::app);
      ofile_shoeGear << ss_sg.str().c_str();
    }
    if (m_log_what_to_file & DBG_GEAR)
    {
      std::stringstream ss_g;
      // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz
      ss_g << t <<","<< m_gear->GetBody()->GetPos() 
        <<","<< m_gear->GetBody()->GetPos_dt() 
        <<","<< m_gear->GetBody()->GetWvel_loc()
        <<"\n";
      ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str(), std::ios::app);
      ofileDBG_GEAR << ss_g.str().c_str();

      // second file, for the specific contact info
      std::stringstream ss_gc;

      // find what's in contact with the gear by processing all collisions with a special callback function
      ChVector<> Fn_info = ChVector<>();
      ChVector<> Ft_info = ChVector<>();
      // info is: (max, avg., variance)
      int num_gear_contacts = reportGearContact(Fn_info, Ft_info);
      // time,Ncontacts,FnMax,FnAvg,FnVar,FtMax,FtAvg,FtVar
      ss_gc << t <<","<< num_gear_contacts
        <<","<< Fn_info
        <<","<< std::sqrt(Fn_info.z)
        <<","<< Ft_info
        <<","<< std::sqrt(Ft_info.z)
        <<"\n";
      ChStreamOutAsciiFile ofileDBG_GEAR_CONTACT(m_filename_DBG_GEAR_CONTACT.c_str(), std::ios::app);
      ofileDBG_GEAR_CONTACT << ss_gc.str().c_str();
    }

    if (m_log_what_to_file & DBG_IDLER)
    {
      std::stringstream ss_id;
      // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c
      ss_id << t <<","<< m_idlers[0]->GetBody()->GetPos()
        <<","<< m_idlers[0]->GetBody()->GetPos_dt()
        <<","<< m_idlers[0]->GetBody()->GetWvel_loc()
        <<","<< m_idlers[0]->GetSpringForce()
        <<","<< m_idlers[0]->Get_SpringReact_Deform()
        <<","<< m_idlers[0]->Get_SpringReact_Deform_dt()
        <<"\n";
      ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.c_str(), std::ios::app);
      ofileDBG_IDLER << ss_id.str().c_str();
    }

    if (m_log_what_to_file & DBG_CONSTRAINTS)
    {
      // Report constraint violations for all joints
      SaveConstraintViolations();
    }
    
    if (m_log_what_to_file & DBG_PTRAIN)
    {
      std::stringstream ss_pt;
      // motor speed, mot torque, out torque
      ss_pt << t <<","<< m_ptrains[0]->GetThrottle()
        <<","<< m_ptrains[0]->GetMotorSpeed() * 60.0 / (CH_C_2PI) // RPM
        <<","<< m_ptrains[0]->GetMotorTorque()
        <<","<< m_ptrains[0]->GetOutputTorque()
        <<"\n";
      ChStreamOutAsciiFile ofilePT(m_filename_DBG_PTRAIN.c_str(), std::ios::app);
      ofilePT << ss_pt.str().c_str();
    }

    if( m_log_what_to_file & DBG_COLLISIONCALLBACK)
    {
      std::stringstream ss_cc;
      // report # of contacts detected this step between shoe pins # gear.
      // time,Ncontacts,Nbroadphase,NcPz,NcNz
      ss_cc << t <<","<< m_gear->GetCollisionCallback()->GetNcontacts()
        <<","<< m_gear->GetCollisionCallback()->GetNbroadPhasePassed()
        <<","<< m_gear->GetCollisionCallback()->Get_sum_Pz_contacts()
        <<","<< m_gear->GetCollisionCallback()->Get_sum_Nz_contacts()
        <<"\n";
    }

  }
}



void DriveChain::create_fileHeaders(int what)
{
  GetLog() << " ------ Output Data ------------ \n\n";

  if(what & DBG_FIRSTSHOE)
  {
    // Shoe 0 : S0, Pin0: P0
    m_filename_DBG_FIRSTSHOE = m_log_file_name+"_shoe0.csv";
    ChStreamOutAsciiFile ofileDBG_FIRSTSHOE(m_filename_DBG_FIRSTSHOE.c_str());
    std::stringstream ss;
    ss << "time,x,y,z,Vx,Vy,Vz,Ax,Ay,Az,Wx,Wy,Wz,Fx,Fy,Fz,Tx,Ty,Tz\n";
    ofileDBG_FIRSTSHOE << ss.str().c_str();
    GetLog() << " writing to file: " << m_filename_DBG_FIRSTSHOE << "\n         data: " << ss.str().c_str() <<"\n";

    // report the contact with the gear in a second file
    m_filename_DBG_shoeGear = m_log_file_name+"_shoe0GearContact.csv";
    ChStreamOutAsciiFile ofileDBG_shoeGear(m_filename_DBG_shoeGear.c_str());
    std::stringstream ss_sg;
    // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
    ss_sg << "time,Ncontacts,NcontactsP,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,VzRelP,normDirRelxP,normDirRelyP,normDirRelzP"
      << ",NcontactsN,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,normDirRelxN,normDirRelyN,normDirRelzN\n";
    ofileDBG_shoeGear << ss_sg.str().c_str();
  }

  if(what & DBG_GEAR)
  {
    m_filename_DBG_GEAR = m_log_file_name+"_gear.csv";
    ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str());
    std::stringstream ss_g;
    ss_g << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz\n";
    ofileDBG_GEAR << ss_g.str().c_str();
    GetLog() << " writing to file: " << m_filename_DBG_GEAR << "\n          data: " << ss_g.str().c_str() <<"\n";

    // report on some specific collision info in a separate file
    m_filename_DBG_GEAR_CONTACT = m_log_file_name+"_gearContact.csv";
    ChStreamOutAsciiFile ofileDBG_GEAR_CONTACT(m_filename_DBG_GEAR_CONTACT.c_str());
    std::stringstream ss_gc;
    ss_gc << "time,Ncontacts,FnMax,FnAvg,FnVar,FnSig,FtMax,FtAvg,FtVar,FtSig\n";
    ofileDBG_GEAR_CONTACT << ss_gc.str().c_str();
    GetLog() << " writing to file : " << m_filename_DBG_GEAR_CONTACT << "\n          data: "<<ss_gc.str().c_str() <<"\n";
  }

  if(what & DBG_IDLER)
  {
    m_filename_DBG_IDLER = m_log_file_name+"_idler.csv";
    ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.c_str());
    std::stringstream ss_id;
    ss_id << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c\n";
    ofileDBG_IDLER << ss_id.str().c_str();
    GetLog() << " writing to file: " << m_filename_DBG_IDLER << "\n          data:" << ss_id.str().c_str() <<"\n";
  }

  // write the data for each subsystem's constraint violation
  if(what & DBG_CONSTRAINTS)
  {
    // in the same order as listed in the header
    m_filename_GCV = m_log_file_name+"_GearCV.csv";
    ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str());
    std::stringstream ss_gCV;
    ss_gCV << m_gear->getFileHeader_ConstraintViolations(0);
    ofileGCV << ss_gCV.str().c_str();
    GetLog() << " writing to file: " << m_filename_GCV << "\n          data: " << ss_gCV.str().c_str() <<"\n";

    for(int id = 0; id < m_num_idlers; id++)
    {
      std::stringstream ss_iCV;
      ss_iCV << m_log_file_name << "_idler" << id << "CV.csv";
      m_filename_ICV.push_back(ss_iCV.str().c_str());
      ChStreamOutAsciiFile ofileICV(m_filename_ICV.back().c_str());
      std::stringstream ss_header;
      ss_header << m_idlers[id]->getFileHeader_ConstraintViolations(id);
      ofileICV << ss_header.str().c_str();
      GetLog() << " writing to file: " << m_filename_ICV[id] << "\n          data: " << ss_header.str().c_str() <<"\n";
    }

    // violations of the roller revolute joints
    for(int roller = 0; roller < m_num_rollers; roller++)
    {
      std::stringstream ss_rCV;
      ss_rCV << m_log_file_name << "_roller" << roller << "CV.csv";
      m_filename_RCV.push_back(ss_rCV.str());
      ChStreamOutAsciiFile ofileRCV(m_filename_RCV.back().c_str());
      std::stringstream ss_header;
      ofileRCV << ss_rCV.str().c_str();
      GetLog() << " writing to file: " << m_filename_RCV[roller] 
      << "\n         data: " << ss_header.str().c_str() <<"\n";
    }
  }

  // write powertrian data
  if(what & DBG_PTRAIN)
  {
    m_filename_DBG_PTRAIN = m_log_file_name+"_ptrain.csv";
    ChStreamOutAsciiFile ofileDBG_PTRAIN(m_filename_DBG_PTRAIN.c_str());
    std::stringstream ss_pt;
    ss_pt << "time,throttle,motSpeed,motT,outT\n";
    ofileDBG_PTRAIN << ss_pt.str().c_str();
    GetLog() << " writing to file: " << m_filename_DBG_PTRAIN 
      << "\n          data: " << ss_pt.str().c_str() <<"\n";
  }

  // write broadphase, narrow phase contact info
  if(what & DBG_COLLISIONCALLBACK)
  {
    m_filename_DBG_COLLISIONCALLBACK = m_log_file_name+"_Ccallback.csv";
    ChStreamOutAsciiFile ofileDBG_COLLISIONCALLBACK(m_filename_DBG_COLLISIONCALLBACK.c_str());
    std::stringstream ss_cc;
    ss_cc << "time,Ncontacts,Nbroadphase,NcPz,NcNz\n";
    ofileDBG_COLLISIONCALLBACK << ss_cc.str().c_str();
    GetLog() << " writing to file: " << m_filename_DBG_COLLISIONCALLBACK 
      << "\n     data: " << ss_cc.str().c_str() <<"\n";

  }
}


} // end namespace chrono
