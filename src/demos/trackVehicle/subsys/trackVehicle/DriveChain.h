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
// Use some track vehicle components to model a drive gear, attached to a powertrain
// via a driveline. Wrap a chain around the drive gear and idler, to which an
// inertial load will be applied.
// It would be easy to turn this into a motorcycle dynamometer.
//
// =============================================================================

#ifndef DRIVECHAIN_H
#define DRIVECHAIN_H

#include "core/ChCoordsys.h"
// #include "physics/ChSystem.h"
// #include "ModelDefs.h"

#include "subsys/base/ChTrackVehicle.h"
#include "subsys/driveGear/DriveGear.h"
#include "subsys/idler/IdlerSimple.h"
#include "subsys/trackChain/TrackChain.h"
#include "subsys/powertrain/TrackPowertrain.h"
#include "subsys/idler/SupportRoller.h"

namespace chrono {

/// Model a static DriveChain, with a powered Gear and
/// an idler attached to some large inertial load
class CH_SUBSYS_API DriveChain : public ChTrackVehicle
{
public:

  /// chassis is static, so use idler info to create the gear body, instead.
  DriveChain(const std::string& name,
    VisualizationType chassisVis = VisualizationType::PRIMITIVES,
    CollisionType chassisCollide = CollisionType::PRIMITIVES,
    size_t num_idlers = 1,
    size_t num_rollers = 1,
    double gear_mass = 100.0,
    const ChVector<>& gear_inertia = ChVector<>(12.22/4.0, 12.22/4.0, 13.87/4.0) );

  ~DriveChain();
  
  /// Initialize the drive chain system
  virtual void Initialize(const ChCoordsys<>& gear_Csys);  ///< [in] initial config of gear
  
  /// Update the vehicle with the new settings for throttle and brake
  virtual void Update(double time,
              double throttle,
              double braking);

  /// Advance the vehicle (and the ChSystem)
  virtual void Advance(double step);

  /// set the pin friction as a damping value
  virtual void SetShoePinDamping(double damping);

  /// log the constraint violations, w/ and w/o chain body, to either the console or a file
  virtual void LogConstraintViolations(bool include_chain = false);

  /// save the constraint violations, w/ and w/o chain body, to the 
  virtual void SaveConstraintViolations(bool include_chain = false);

  virtual void Log_to_console(int console_what);

  /// Log data to file.
  /// Data types AND filename to be saved should already set in Setup_log_to_file() 
  virtual void Log_to_file();

  /// setup class to save the log to a file for python postprocessing.
  /// Usage: call after construction & Initialize(), else no data is saved.
  virtual void Setup_log_to_file(int what,
                     const std::string& out_filename,
                     const std::string& data_dirname);

  // ---------------------------------------------------------------------------
  // Accessors
  virtual double GetShoePinDamping() const {return m_chain->Get_pin_damping(); }

  /// Get the angular speed of the driveshaft.
  virtual double GetDriveshaftSpeed(size_t idx) const { return m_gear->GetAxle()->GetPos_dt(); }

  /// pointer to the powertrain
  virtual const ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const { return m_ptrains[idx]; }

  /// vehicle's driveshaft body.
  const ChSharedPtr<ChShaft> GetDriveshaft(size_t idx) const {return m_gear->GetAxle(); }

  /// current value of the integration step size for the vehicle system.
  double GetStepsize() const { return m_stepsize; }

  /// just return the COG of the gear
  const ChCoordsys<> GetLocalDriverCoordsys() const { return ChCoordsys<>(m_chassis->GetPos(), m_chassis->GetRot()); }

  /// set the powertrain drive mode
  void SetDriveMode(TrackPowertrain::DriveMode mode) { m_ptrains[0]->SetDriveMode(mode); }

  /// vehicle speed, doesn't matter
  double GetVehicleSpeed() const { return 0; }
  int GetNum_Engines() const { return 1;}

protected:

  /// create files with headers for all specified output data types.
  /// File format is .csv, for easy reading into python pandas scripts for data analysis
  void create_fileHeaders(int what);

  /// info set is: (max, avg, stdev = sigma) for x,y,z vector data.
  /// returns total number of contacts with the gear this step.
  int reportGearContact(ChVector<>& Fn_info,
    ChVector<>& Ft_info);

  /// for a given shoe body name, scan all collisions and report collision time data.
  /// SG_info = (num_contacts, t_persist, t_persist_max)
  //  Fn_info, Ft_info = (max, avg, stdev = sigma)
  /// returns # of contacts between the gear and shoe body
  int reportShoeGearContact(const std::string& shoe_name,
    ChVector<>& SG_info,
    ChVector<>& Fn_info,
    ChVector<>& Ft_info);

  // private variables
  // <ChBodyAuxRef> m_chassis   in base class
  ChSharedPtr<DriveGear> m_gear;  		///< drive gear
  std::vector<ChSharedPtr<IdlerSimple>>	m_idlers;	///< idler wheel
  size_t m_num_idlers;  ///< number of idlers to create
  ChSharedPtr<TrackChain> m_chain;    ///< chain

  ChVector<> m_idlerPosRel;	///< position of idler COG relative to local c-sys
  




  // ChSharedPtr<TrackPowertrain>  m_ptrain;  ///< powertrain system






  std::vector<ChSharedPtr<SupportRoller>> m_rollers;  ///< passive support rollers
  size_t m_num_rollers;
	
  // I/O stuff
  std::string m_filename_DBG_FIRSTSHOE;     // write to this file, first shoe/pin info
  std::string m_filename_DBG_shoeGear;      // info about gear/shoe contact
  std::string m_filename_DBG_GEAR;          // write to this file, gear body
  std::string m_filename_DBG_GEAR_CONTACT;  // specific info about collisions with gear
  std::string m_filename_DBG_IDLER;         // to to this file, idler body
  std::string m_filename_GCV;               // write to this file, gear constraint violation
  std::vector<std::string> m_filename_ICV;  // write to this file, idler constraint violation
  std::vector<std::string> m_filename_RCV;  // write to this file, roller constraint violation
  std::string m_filename_DBG_PTRAIN;        // write to this file, ptrain info

  // static variables. hard-coded for now
  static const ChVector<> m_idlerPos; // relative to chassis frame, which is the same as the gear's (initially)
  static const ChQuaternion<> m_idlerRot; 


  ChSharedPtr<IdlerSimple> m_idler2;


  friend std::ostream & operator<< (std::ostream &out, const ChVector<double>& vect);
  friend std::ostream & operator << (std::ostream &out, const ChQuaternion<double>& q);
};


} // end namespace chrono


#endif
