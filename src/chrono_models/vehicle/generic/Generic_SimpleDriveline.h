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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple driveline model for a single axle open differential.
//
// =============================================================================

#ifndef CH_GENERIC_SIMPLE_DRIVELINE_H
#define CH_GENERIC_SIMPLE_DRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/ChDriveline.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {
    class CH_MODELS_API Generic_SimpleDriveline : public ChDriveline {
    public:
        Generic_SimpleDriveline(const std::string& name);
        virtual ~Generic_SimpleDriveline() {}

        /// Return the number of driven axles.
        virtual int GetNumDrivenAxles() const final override { return 1; }

        /// Initialize the driveline subsystem.
        /// This function connects this driveline subsystem to the axles of the
        /// specified suspension subsystems.
        virtual void Initialize(std::shared_ptr<ChBody> chassis,      ///< handle to the chassis body
            const ChSuspensionList& suspensions,  ///< list of all vehicle suspension subsystems
            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
            ) override;

        /// Get the angular speed of the driveshaft.
        /// This represents the output from the driveline subsystem that is passed to
        /// the powertrain system.
        virtual double GetDriveshaftSpeed() const override;

        /// Update the driveline subsystem: apply the specified motor torque.
        /// This represents the input to the driveline subsystem from the powertrain
        /// system.
        virtual void Synchronize(double torque) override;

        /// Get the motor torque to be applied to the specified wheel.
        virtual double GetWheelTorque(const WheelID& wheel_id) const override;

    private:
        static const double m_conicalgear_ratio;

        std::shared_ptr<ChShaft> m_driven_left;
        std::shared_ptr<ChShaft> m_driven_right;
    };

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
