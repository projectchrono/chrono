#ifndef CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vehicle/utils/ChVehicleVisualSystemVSG.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"

namespace chrono {
    namespace vehicle {
    class CH_VEHICLE_API ChWheeledVehicleVisualSystemVSG : public ChVehicleVisualSystemVSG {
        public:
            ChWheeledVehicleVisualSystemVSG();
            ~ChWheeledVehicleVisualSystemVSG() {};

        /// Attach a vehicle to this Irrlicht vehicle visualization system.
        virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;
    private:
        ChWheeledVehicle* m_wvehicle;
    };
    }
}

#endif