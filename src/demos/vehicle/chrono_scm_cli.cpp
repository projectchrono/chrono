// Chrono SCM CLI: evaluates wheel contact forces via SCM terrain.
// Reads JSON queries from stdin, writes JSON results to stdout.
// Built as part of the Chrono project CMake.
//
// Usage: echo '[{"wheel_id":"FL","x":2.5,"y":2.5,"z":0.15,"load":3000}]' | chrono_scm_cli

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/core/ChVector3.h"
#include <iostream>
#include <sstream>
#include <cmath>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::collision;

int main() {
    // Read JSON from stdin
    std::stringstream buffer;
    buffer << std::cin.rdbuf();
    std::string input = buffer.str();

    // Minimal JSON parsing (avoid nlohmann dependency)
    std::cout << "[";
    bool first = true;

    // Parse: [{"wheel_id":"...","x":...,"y":...,"z":...,"load":...},...]
    size_t pos = input.find('{');
    while (pos != std::string::npos) {
        size_t end = input.find('}', pos);
        if (end == std::string::npos) break;
        std::string obj = input.substr(pos + 1, end - pos - 1);

        // Extract fields
        std::string wheel_id = "?";
        double x = 0, y = 0, z = 0, load = 0;
        auto extract = [&](const std::string& key) -> double {
            size_t kp = obj.find("\"" + key + "\":");
            if (kp == std::string::npos) return 0;
            kp = obj.find(":", kp) + 1;
            while (kp < obj.size() && (obj[kp] == ' ' || obj[kp] == '\t')) kp++;
            return std::stod(obj.substr(kp));
        };
        auto extract_str = [&](const std::string& key) -> std::string {
            size_t kp = obj.find("\"" + key + "\":\"");
            if (kp == std::string::npos) return "?";
            kp += key.size() + 4;
            size_t ke = obj.find("\"", kp);
            return obj.substr(kp, ke - kp);
        };

        wheel_id = extract_str("wheel_id");
        x = extract("x");
        y = extract("y");
        z = extract("z");
        load = extract("load");

        // Set up Chrono system with SCM terrain
        ChSystemNSC system;
        system.SetNumThreads(1);
        system.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

        auto coll = chrono_types::make_shared<ChCollisionSystemBullet>();
        system.SetCollisionSystem(coll);

        SCMTerrain scm(&system);
        scm.SetSoilParameters(2e6, 0, 1.1, 50000, std::tan(CH_PI * 20.0 / 180.0), 0.01, 5e7, 0);
        scm.Initialize(5.0, 5.0, 0.1);

        auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(x, y, z));
        body->SetMass(0.5);
        body->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));
        auto sphere = chrono_types::make_shared<ChCollisionShapeSphere>(mat, 0.02);
        body->AddCollisionShape(sphere);
        system.AddBody(body);

        // Step
        for (int i = 0; i < 40; i++) {
            body->AccumulateForce(0, ChVector3d(0, 0, -load), body->GetPos(), false);
            system.DoStepDynamics(1e-4);
        }

        // Extract
        ChVector3d force(0, 0, 0), torque(0, 0, 0);
        bool has_contact = scm.GetContactForceBody(body, force, torque);
        double sinkage = std::max(0.0, z - body->GetPos().z());

        if (!first) std::cout << ",";
        first = false;
        std::cout << "{"
                  << "\"wheel_id\":\"" << wheel_id << "\","
                  << "\"sinkage_m\":" << sinkage << ","
                  << "\"traction_force_n\":" << (has_contact ? force.x() : 0.0) << ","
                  << "\"rolling_resistance_n\":0.0,"
                  << "\"lateral_force_n\":" << (has_contact ? force.y() : 0.0) << ","
                  << "\"slip_ratio\":0.0,"
                  << "\"ground_pressure_pa\":" << (has_contact ? -force.z() / 1e-4 : 0.0) << ","
                  << "\"contact_area_m2\":0.0"
                  << "}";

        system.RemoveBody(body);
        pos = input.find('{', end + 1);
    }

    std::cout << "]" << std::endl;
    return 0;
}
