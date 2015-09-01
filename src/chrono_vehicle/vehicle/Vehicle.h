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
// Authors: Radu Serban
// =============================================================================
//
// Vehicle model constructed from a JSON specification file
//
// =============================================================================

#ifndef VEHICLE_H
#define VEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "chrono_vehicle/ChVehicle.h"

namespace chrono {

class CH_VEHICLE_API Vehicle : public ChVehicle
{
public:

  Vehicle(const std::string& filename);

  Vehicle(ChSystem*          system,
          const std::string& filename);

  ~Vehicle() {}

  virtual int GetNumberAxles() const { return m_num_axles; }

  virtual ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  virtual void Initialize(const ChCoordsys<>& chassisPos);

  bool UseVisualizationMesh() const          { return m_chassisUseMesh; }
  const std::string& GetMeshFilename() const { return m_chassisMeshFile; }
  const std::string& GetMeshName() const     { return m_chassisMeshName; }

private:

  void Create(const std::string& filename);

  void LoadSteering(const std::string& filename, int which);
  void LoadDriveline(const std::string& filename);
  void LoadSuspension(const std::string& filename, int axle);
  void LoadAntirollbar(const std::string& filename);
  void LoadWheel(const std::string& filename, int axle, int side);
  void LoadBrake(const std::string& filename, int axle, int side);

private:

  int                      m_num_axles;         // number of axles for this vehicle
  std::vector<ChVector<> > m_suspLocations;     // locations of the suspensions relative to chassis
  std::vector<int>         m_suspSteering;      // indexes of steering subsystems (-1 indicates a non-steered suspension)

  std::vector<ChVector<> > m_arbLocations;      // locations of the antirollbar subsystems relative to chassis
  std::vector<int>         m_arbSuspension;     // indexes of steering subsystems

  int                          m_num_strs;      // number of steering subsystems
  std::vector<ChVector<> >     m_strLocations;  // locations of the steering subsystems relative to chassis
  std::vector<ChQuaternion<> > m_strRotations;  // orientations of the steering subsystems relative to chassis

  std::vector<int> m_driven_susp;               // indexes of the driven suspensions

  bool        m_chassisUseMesh;                 // true if using a mesh for chassis visualization
  std::string m_chassisMeshName;                // name of the chassis visualization mesh
  std::string m_chassisMeshFile;                // name of the Waveform file with the chassis mesh

  double     m_chassisMass;                     // chassis mass
  ChVector<> m_chassisCOM;                      // location of the chassis COM in the chassis reference frame
  ChVector<> m_chassisInertia;                  // moments of inertia of the chassis

  ChCoordsys<> m_driverCsys;                    // driver position and orientation relative to chassis
};


} // end namespace chrono


#endif
