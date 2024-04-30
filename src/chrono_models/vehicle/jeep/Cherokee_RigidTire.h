//
// Created by Rainer Gericke on 20.04.24.
//

#ifndef CHRONO_CHEROKEE_RIGIDTIRE_H
#define CHRONO_CHEROKEE_RIGIDTIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Rigid tire model for the Jeep Cherokee vehicle.
class CH_MODELS_API Cherokee_RigidTire : public ChRigidTire {
  public:
    Cherokee_RigidTire(const std::string& name, bool use_mesh = false);
    ~Cherokee_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

  private:
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector3d m_inertia;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;

    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_models_cherokee

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_RIGIDTIRE_H
