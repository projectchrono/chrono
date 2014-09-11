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

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"

namespace chrono {

class CH_SUBSYS_API ChLugreTire : public ChTire {
public:
  ChLugreTire(const ChTerrain& terrain);
  virtual ~ChLugreTire() {}

  void Initialize();

  virtual ChTireForce GetTireForce() const { return m_tireForce; }

  virtual void Update(double              time,
                      const ChBodyState&  wheel_state);
  virtual void Advance(double step);

  void SetStepsize(double val) { m_stepsize = val; }
  double GetStepsize() const { return m_stepsize; }

protected:
  virtual int getNumDiscs() const = 0;
  virtual double getRadius() const = 0;
  virtual const double* getDiscLocations() const = 0;

  virtual double getNormalStiffness() const = 0;
  virtual double getNormalDamping() const = 0;

  virtual void SetLugreParams() = 0;

  double   m_stepsize;

  // Lugre friction model parameters (longitudinal/lateral)
  double   m_sigma0[2];
  double   m_sigma1[2];
  double   m_sigma2[2];
  double   m_Fc[2];
  double   m_Fs[2];
  double   m_vs[2];

private:
  struct DiscContactData {
    bool         in_contact;     // true if disc in contact with terrain
    ChCoordsys<> frame;          // contact frame (x: long, y: lat, z: normal)
    ChVector<>   vel;            // relative velocity expressed in contact frame
    double       normal_force;   // magnitude of normal contact force
    double       ode_coef_a[2];  // ODE coefficients:  z' = z + b * z
    double       ode_coef_b[2];  //   (longitudinal/lateral)
  };

  struct DiscState {
    double       z0;             // longitudinal direction
    double       z1;             // lateral direction
  };

  ChTireForce                  m_tireForce;
  std::vector<DiscContactData> m_data;
  std::vector<DiscState>       m_state;

};


} // end namespace chrono


#endif
