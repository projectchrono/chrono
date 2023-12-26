#pragma once

// #define FMI2_FUNCTION_PREFIX MyModel_
#include <FmuToolsExport.h>
#include <string>
#include <vector>
#include <array>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

class FmuComponent : public FmuComponentBase {
  public:
    FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID);
    virtual ~FmuComponent() {}

    /// Advance dynamics
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  protected:
    virtual void _enterInitializationMode() override;
    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    void CreateVehicle();
    void ConfigureSystem();
    void SynchronizeVehicle(double time);
    void CalculateVehicleOutputs();

    std::shared_ptr<chrono::vehicle::WheeledVehicle> vehicle;

    // JSON specification files
    std::string vehicle_JSON;
    std::string tire_JSON;
    std::string engine_JSON;
    std::string transmission_JSON;

    fmi2Boolean system_SMC = true;

    chrono::ChVector<> init_loc;
    double init_yaw;

    double step_size;

    // Enable/disable run-time visualization
    fmi2Boolean vis = false;

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vissys;
#endif

    // FMU inputs
    chrono::vehicle::DriverInputs driver_inputs;
    std::array<double, 4> terrain_height;
    std::array<chrono::ChVector<>, 4> terrain_normal;

    // FMU outputs

    //// TODO
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
