#include "ChVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

class VehAppKeyboardHandler : public vsg::Inherit<vsg::Visitor, VehAppKeyboardHandler> {
  public:
    VehAppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}

    void SetParams(vsg::ref_ptr<ChVehicleVisualSystemVSG::StateParams> params, ChVehicleVisualSystemVSG* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }

    void apply(vsg::KeyPressEvent& keyPress) override {
            if (keyPress.keyBase == 'a' || keyPress.keyModified == 'a') {
                // terminate process
                m_appPtr->SteeringLeft();
            }
            if (keyPress.keyBase == 'w' || keyPress.keyModified == 'w') {
                // terminate process
                m_appPtr->IncreaseVehicleSpeed();
            }
            if (keyPress.keyBase == 's' || keyPress.keyModified == 's') {
                // terminate process
                m_appPtr->DecreaseVehicleSpeed();
            }
            if (keyPress.keyBase == 'd' || keyPress.keyModified == 'd') {
                // terminate process
                m_appPtr->SteeringRight();
            }
            if (keyPress.keyBase == 'c' || keyPress.keyModified == 'c') {
                // terminate process
                m_appPtr->SteeringCenter();
            }
            if (keyPress.keyBase == 'r' || keyPress.keyModified == 'r') {
                // terminate process
                m_appPtr->ReleasePedals();
            }
    }

  private:
    vsg::observer_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<ChVehicleVisualSystemVSG::StateParams> _params;
    ChVehicleVisualSystemVSG* m_appPtr;
};

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() : ChVisualSystemVSG(), m_guiDriver(nullptr) {
    m_params->showVehicleState = true;
}

ChVehicleVisualSystemVSG::~ChVehicleVisualSystemVSG() {}

void ChVehicleVisualSystemVSG::Initialize() {
    ChVisualSystemVSG::Initialize();
    // add keyboard handler
    auto veh_kbHandler = VehAppKeyboardHandler::create(m_viewer);
    veh_kbHandler->SetParams(m_params, this);
    m_viewer->addEventHandler(veh_kbHandler);
}

void ChVehicleVisualSystemVSG::AttachVehicle(vehicle::ChVehicle* vehicle) {
    ChVehicleVisualSystem::AttachVehicle(vehicle);
}

void ChVehicleVisualSystemVSG::AttachGuiDriver(ChVSGGuiDriver* driver) {
    m_guiDriver = driver;
}

void ChVehicleVisualSystemVSG::Synchronize(const std::string& msg, const DriverInputs& driver_inputs) {
    if (m_vehicle) {
        m_params->vehicleSpeed = m_vehicle->GetSpeed();
        m_params->steering = driver_inputs.m_steering;
        m_params->throttle = driver_inputs.m_throttle;
        m_params->braking = driver_inputs.m_braking;
    } else {
        m_params->vehicleSpeed = 0.0;
        m_params->steering = 0.0;
        m_params->throttle = 0.0;
        m_params->braking = 0.0;
    }
}

void ChVehicleVisualSystemVSG::Advance(double step) {
    // Update the ChChaseCamera: take as many integration steps as needed to
    // exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera->Update(h);
        t += h;
    }

    // Update the Irrlicht camera
    ChVector<> cam_pos = m_camera->GetCameraPos();
    ChVector<> cam_target = m_camera->GetTargetPos();
    m_vsg_cameraEye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_vsg_cameraTarget.set(cam_target.x(), cam_target.y(), cam_target.z());
    m_lookAt->eye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_lookAt->center.set(cam_target.x(), cam_target.y(), cam_target.z());
}

void ChVehicleVisualSystemVSG::IncreaseVehicleSpeed() {
    //GetLog() << "Speed+\n";
    if (m_guiDriver) {
        m_guiDriver->IncreaseThrottle();
    }
}

void ChVehicleVisualSystemVSG::DecreaseVehicleSpeed() {
    //GetLog() << "Speed-\n";
    if (m_guiDriver) {
        m_guiDriver->DecreaseThrottle();
    }
}

void ChVehicleVisualSystemVSG::SteeringLeft() {
    //GetLog() << "Left\n";
    if (m_guiDriver) {
        m_guiDriver->SteeringLeft();
    }
}

void ChVehicleVisualSystemVSG::SteeringRight() {
    //GetLog() << "Right\n";
    if (m_guiDriver) {
        m_guiDriver->SteeringRight();
    }
}

void ChVehicleVisualSystemVSG::SteeringCenter() {
    //GetLog() << "Center\n";
    if (m_guiDriver) {
        m_guiDriver->SteeringCenter();
    }
}

void ChVehicleVisualSystemVSG::ReleasePedals() {
    //GetLog() << "Release\n";
    if (m_guiDriver) {
        m_guiDriver->ReleasePedals();
    }
}

}  // namespace vehicle
}  // namespace chrono