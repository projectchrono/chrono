#ifndef CH_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_VEHICLE_VISUAL_SYSTEM_VSG_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChVSGGuiDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"

namespace chrono {
namespace vehicle {
class CH_VEHICLE_API ChVehicleVisualSystemVSG : public ChVehicleVisualSystem, public vsg3d::ChVisualSystemVSG {
  public:
    /// Construct a vehicle Irrlicht visualization system
    ChVehicleVisualSystemVSG();
    ~ChVehicleVisualSystemVSG();

    virtual void Initialize() override;

    /// Attach a vehicle to this VSG vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

    void AttachGuiDriver(ChVSGGuiDriver* driver);

    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    void Advance(double step);

    /// Update information related to driver inputs.
    virtual void Synchronize(const std::string& msg, const DriverInputs& driver_inputs) override;

    void IncreaseVehicleSpeed();
    void DecreaseVehicleSpeed();
    void SteeringLeft();
    void SteeringRight();
    void SteeringCenter();
    void ReleasePedals();

protected:
    ChVSGGuiDriver* m_guiDriver;

    friend class ChVSGGuiDriver;
};

}  // namespace vehicle
}  // namespace chrono
#endif