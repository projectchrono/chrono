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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for LuGre tire model
//
// =============================================================================

#ifndef CH_LUGRETIRE_H
#define CH_LUGRETIRE_H

#include <vector>

#include "physics/ChBody.h"

#include "chrono_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {

///
/// Tire model based on LuGre friction model.
///
class CH_VEHICLE_API ChLugreTire : public ChTire
{
public:

  ChLugreTire(
    const std::string& name,     ///< [in] name of this tire system
    const ChTerrain&   terrain   ///< [in] reference to the terrain system
    );

  virtual ~ChLugreTire() {}

  /// Initialize this tire system.
  void Initialize();

  /// Initialize this tire system and enable visualization of the discs.
  void Initialize(
    ChSharedPtr<ChBody> wheel   ///< handle to the associated wheel body
    );

  /// Get the tire force and moment.
  /// This represents the output from this tire system that is passed to the
  /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
  /// to the appropriate suspension subsystem which applies it as an external
  /// force one the wheel body.
  virtual ChTireForce GetTireForce() const { return m_tireForce; }

  /// Update the state of this tire system at the current time.
  /// The tire system is provided the current state of its associated wheel.
  virtual void Update(
    double               time,          ///< [in] current time
    const ChWheelState&  wheel_state    ///< [in] current state of associated wheel body
    );

  /// Advance the state of this tire by the specified time step.
  virtual void Advance(double step);

  /// Set the value of the integration step size for the underlying dynamics.
  void SetStepsize(double val) { m_stepsize = val; }

  /// Get the current value of the integration step size.
  double GetStepsize() const { return m_stepsize; }

protected:

  /// Return the number of discs used to model this tire.
  virtual int getNumDiscs() const = 0;

  /// Return the tire radius.
  virtual double getRadius() const = 0;

  /// Return the laterla disc locations.
  /// These locations are relative to the tire center.
  virtual const double* getDiscLocations() const = 0;

  /// Return the vertical tire stiffness (for normal force calculation).
  virtual double getNormalStiffness() const = 0;
  /// Return the vertical tire damping coefficient (for normal force calculation).
  virtual double getNormalDamping() const = 0;

  /// Set the parameters in the LuGre friction model.
  virtual void SetLugreParams() = 0;

  /// Lugre friction model parameters (longitudinal/lateral)
  double   m_sigma0[2];    ///<
  double   m_sigma1[2];    ///<
  double   m_sigma2[2];    ///<
  double   m_Fc[2];        ///<
  double   m_Fs[2];        ///<
  double   m_vs[2];        ///< Stribeck velocity

private:

  struct DiscContactData {
    bool         in_contact;     // true if disc in contact with terrain
    ChCoordsys<> frame;          // contact frame (x: long, y: lat, z: normal)
    ChVector<>   vel;            // relative velocity expressed in contact frame
    double       normal_force;   // magnitude of normal contact force
    double       ode_coef_a[2];  // ODE coefficients:  z' = a + b * z
    double       ode_coef_b[2];  //   (longitudinal/lateral)
  };

  struct DiscState {
    double       z0;             // longitudinal direction
    double       z1;             // lateral direction
  };

  double   m_stepsize;

  ChTireForce                  m_tireForce;
  std::vector<DiscContactData> m_data;
  std::vector<DiscState>       m_state;

};


} // end namespace chrono


#endif
