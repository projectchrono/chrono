//
// Created by Rainer Gericke on 16.04.24.
//

#ifndef CHRONO_CHEROKEE_TMEASYTIRE_H
#define CHRONO_CHEROKEE_TMEASYTIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// TMeasy tire model for the Jeep Cherokee.
class CH_MODELS_API Cherokee_TMeasyTire : public ChTMeasyTire {
  public:
    Cherokee_TMeasyTire(const std::string& name);
    ~Cherokee_TMeasyTire() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMeasyParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const double m_mass;
    static const ChVector3d m_inertia;

    ChFunctionInterp m_stiffnessMap;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_models_cherokee

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_TMEASYTIRE_H
