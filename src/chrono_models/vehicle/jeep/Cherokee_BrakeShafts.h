//
// Created by Rainer Gericke on 16.04.24.
//

#ifndef CHRONO_CHEROKEE_BRAKESHAFTS_H
#define CHRONO_CHEROKEE_BRAKESHAFTS_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Shafts-based HMMWV brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API Cherokee_BrakeShafts : public ChBrakeShafts {
  public:
    Cherokee_BrakeShafts(const std::string& name);
    ~Cherokee_BrakeShafts() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }
    virtual double GetShaftInertia() override { return m_shaft_inertia; }

  private:
    static const double m_maxtorque;
    static const double m_shaft_inertia;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono


#endif  // CHRONO_CHEROKEE_BRAKESHAFTS_H
