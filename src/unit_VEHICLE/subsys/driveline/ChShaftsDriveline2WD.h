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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// 2WD driveline model template based on ChShaft objects. This template can be
// used to model either a FWD or a RWD driveline.
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_2WD_H
#define CH_SHAFTS_DRIVELINE_2WD_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChDriveline.h"
#include "subsys/ChVehicle.h"

#include "physics/ChShaftsGear.h" 
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsMotor.h"
#include "physics/ChShaftsTorque.h"

namespace chrono {

// Forward reference
class ChVehicle;


class CH_SUBSYS_API ChShaftsDriveline2WD : public ChDriveline
{
public:

  ChShaftsDriveline2WD(ChVehicle* car);

  ~ChShaftsDriveline2WD() {}

  void SetMotorBlockDirection(const ChVector<>& dir) { m_dir_motor_block = dir; }
  void SetAxleDirection(const ChVector<>& dir)       { m_dir_axle = dir; }

  /// To be called after creation, to create all the wrapped ChShaft objects 
  /// and their constraints, torques etc. 
  void Initialize(ChSharedPtr<ChBody>  chassis,
                  ChSharedPtr<ChShaft> axle_L,
                  ChSharedPtr<ChShaft> axle_R);

  virtual double GetWheelTorque(ChWheelId which) const;

protected:

  /// Inertias of the component ChShaft objects.
  virtual double GetDriveshaftInertia() const = 0;
  virtual double GetDifferentialBoxInertia() const = 0;

  /// Gear ratios.
  virtual double GetConicalGearRatio() const = 0;
  virtual double GetDifferentialRatio() const = 0;

private:

  ChSharedPtr<ChShaftsGearboxAngled>    m_conicalgear;
  ChSharedPtr<ChShaft>                  m_differentialbox;
  ChSharedPtr<ChShaftsPlanetary>        m_differential;

  ChVector<> m_dir_motor_block;
  ChVector<> m_dir_axle;

  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
