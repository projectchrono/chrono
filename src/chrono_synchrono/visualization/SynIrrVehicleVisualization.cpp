#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

namespace chrono {
namespace synchrono {

SynIrrVehicleVisualization::SynIrrVehicleVisualization(std::shared_ptr<ChDriver> driver, double render_step_size)
    : SynVisualization(SynVisualization::IRRLICHT),
      m_driver(driver),
      m_render_step_size(render_step_size),
      m_render_steps(std::ceil(render_step_size / STEP_SIZE)),
      m_step_number(0) {}

void SynIrrVehicleVisualization::InitializeAsDefaultChaseCamera(std::shared_ptr<SynVehicle> vehicle,
                                                                double following_distance,
                                                                std::wstring window_title) {
    m_app = chrono_types::make_shared<ChWheeledVehicleIrrApp>(&vehicle->GetVehicle(), window_title.c_str());
    m_app->SetSkyBox();
    m_app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                            130);
    m_app->SetTimestep(STEP_SIZE);
    m_app->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), following_distance, 0.5);
}

void SynIrrVehicleVisualization::InitializeAsDefaultTrackedChaseCamera(std::shared_ptr<SynTrackedVehicle> vehicle,
                                                                double following_distance,
                                                                std::wstring window_title) {
    m_app = chrono_types::make_shared<ChTrackedVehicleIrrApp>(&vehicle->GetVehicle(), window_title.c_str());
    m_app->SetSkyBox();
    m_app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                            130);
    m_app->SetTimestep(STEP_SIZE);
    m_app->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), following_distance, 0.5);
}

void SynIrrVehicleVisualization::AttachIrrApp(std::shared_ptr<ChVehicleIrrApp> app) {
    if (m_app != nullptr)
        std::cout << "SynIrrVehicleVisualization::AttachIrrApp: m_app is already initialized. Updating anyways..."
                  << std::endl;
    m_app = app;
}

void SynIrrVehicleVisualization::Initialize() {}

void SynIrrVehicleVisualization::Update(double step) {
    if (m_should_initialize) {
        m_app->AssetBindAll();
        m_app->AssetUpdateAll();
        m_should_initialize = false;
    }

    if (m_visualize && !m_app->GetDevice()->run())
        exit(-1);

    if (m_step_number % m_render_steps == 0) {
        m_app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        m_app->DrawAll();
        m_app->EndScene();
        if (m_save) {
            char filename[100];
            sprintf(filename, "%s%03d.jpg", m_save_path.c_str(), m_render_frame++);
            m_app->WriteImageToFile(filename);
        }
    }

    m_app->Synchronize("", m_driver->GetInputs());

    m_app->Advance(step);

    m_step_number++;
}

}  // namespace synchrono
}  // namespace chrono
