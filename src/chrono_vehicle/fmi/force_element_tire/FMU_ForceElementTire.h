// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Co-simulation FMU encapsulating a "force element" (handling) tire system.
//
// The wrapped Chrono::Vehicle tire model is defined through a JSON specification
// file which is assumed to define a tire of ChForceElementTire type.
//
// This tire FMU must be co-simulated with a vehicle system which provides
// the current wheel state (of type WheelState) and a terrain system which provides
// local terrain information (height, normal, and coefficient of friction) at a
// single query point.
//
// This tire FMU defines continuous output variables for:
//   - wheel tire/terrain load (of type TerrainForce)
//   - location of the terrain query point (of type ChVector)
//
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <array>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "chrono_fmi/ChFmuToolsExport.h"

class FmuComponent : public chrono::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID);
    ~FmuComponent() {}

    /// Advance dynamics.
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  private:
    virtual void _enterInitializationMode() override;
    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    void CreateTire();
    void SynchronizeTire(double time);
    void CalculateTireOutputs();

    /// Local terrain system to intermediate tire FMU datab exchange.
    /// This represents a locally-flat terrain patch, with the plane updated at each synchronization time.
    class LocalTerrain : public chrono::vehicle::ChTerrain {
      public:
        LocalTerrain() : height(0), normal(chrono::ChVector<>(0, 0, 1)), mu(0.8) {}
        virtual double GetHeight(const chrono::ChVector<>& loc) const override { return height; }
        virtual chrono::ChVector<> GetNormal(const chrono::ChVector<>& loc) const override { return normal; }
        virtual float GetCoefficientFriction(const chrono::ChVector<>& loc) const override { return (float)mu; }
        double height;
        chrono::ChVector<> normal;
        double mu;
    };

    std::shared_ptr<chrono::vehicle::ChTire> tire;    ///< underlying tire
    std::shared_ptr<chrono::vehicle::ChWheel> wheel;  ///< associated wheel
    LocalTerrain terrain;                             ///< associated terrain object
    chrono::ChSystemSMC sys;                          ///< containing system

    // FMU parameters
    std::string tire_JSON;  ///< JSON tire specification file
    double step_size;       ///< integration step size

    // FMU inputs and outputs (vehicle side)
    chrono::vehicle::WheelState wheel_state;   ///< state of associated wheel (input)
    chrono::vehicle::TerrainForce wheel_load;  ///< tire loads on associated wheel (output)

    // FMU inputs and outputs (terrain side)
    chrono::ChVector<> query_point;
    double terrain_height;
    chrono::ChVector<> terrain_normal;
    double terrain_mu;
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
