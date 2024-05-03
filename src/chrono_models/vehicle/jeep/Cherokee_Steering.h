//
// Created by Rainer Gericke on 18.04.24.
//

#ifndef CHRONO_CHEROKEE_STEERING_H
#define CHRONO_CHEROKEE_STEERING_H
#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_fmtv
/// @{

/// RotaryArm steering subsystem for the FMTV vehicles.
class CH_MODELS_API Cherokee_Steering : public ChRotaryArm {
  public:
    Cherokee_Steering(const std::string& name);
    ~Cherokee_Steering() {}

    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector3d& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector3d& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const ChVector3d getLocation(PointId which) override;
    virtual const ChVector3d getDirection(DirectionId which) override;

  private:
    static const double m_pitmanArmMass;

    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const ChVector3d m_pitmanArmInertiaMoments;
    static const ChVector3d m_pitmanArmInertiaProducts;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono


#endif  // CHRONO_CHEROKEE_STEERING_H
