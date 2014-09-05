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
// 4WD driveline model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_4WD_H
#define CH_SHAFTS_DRIVELINE_4WD_H

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


class CH_SUBSYS_API ChShaftsDriveline4WD : public ChDriveline
{
public:

  ChShaftsDriveline4WD(ChVehicle* car);

  ~ChShaftsDriveline4WD() {}

  void SetMotorBlockDirection(const ChVector<>& dir) { m_dir_motor_block = dir; }
  void SetAxleDirection(const ChVector<>& dir)       { m_dir_axle = dir; }

  /// To be called after creation, to create all the wrapped ChShaft objects 
  /// and their constraints, torques etc. 
  void Initialize(ChSharedPtr<ChBody>  chassis,
                  ChSharedPtr<ChShaft> axle_front_L,
                  ChSharedPtr<ChShaft> axle_front_R,
                  ChSharedPtr<ChShaft> axle_rear_L,
                  ChSharedPtr<ChShaft> axle_rear_R);

  virtual double GetWheelTorque(ChWheelId which) const;

protected:

  /// Inertias of the component ChShaft objects.
  virtual double GetDriveshaftInertia() const = 0;
  virtual double GetCentralDifferentialBoxInertia() const = 0;
  virtual double GetToFrontDiffShaftInertia() const = 0;
  virtual double GetToRearDiffShaftInertia() const = 0;
  virtual double GetRearDifferentialBoxInertia() const = 0;
  virtual double GetFrontDifferentialBoxInertia() const = 0;

  /// Gear ratios.
  virtual double GetRearConicalGearRatio() const = 0;
  virtual double GetFrontConicalGearRatio() const = 0;
  virtual double GetRearDifferentialRatio() const = 0;
  virtual double GetFrontDifferentialRatio() const = 0;
  virtual double GetCentralDifferentialRatio() const = 0;

private:

  ChSharedPtr<ChShaftsPlanetary>        m_central_differential;
  ChSharedPtr<ChShaft>                  m_front_shaft;
  ChSharedPtr<ChShaft>                  m_rear_shaft;
  ChSharedPtr<ChShaftsGearboxAngled>    m_rear_conicalgear;
  ChSharedPtr<ChShaftsPlanetary>        m_rear_differential;
  ChSharedPtr<ChShaft>                  m_rear_differentialbox;
  ChSharedPtr<ChShaftsGearboxAngled>    m_front_conicalgear;
  ChSharedPtr<ChShaftsPlanetary>        m_front_differential;
  ChSharedPtr<ChShaft>                  m_front_differentialbox;

  ChVector<> m_dir_motor_block;
  ChVector<> m_dir_axle;

  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
