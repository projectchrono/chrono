// Utility classes and functions for Viper wheel simulations

#include <string>
#include <fstream>
#include <vector>
#include <cmath>

#include "chrono/core/ChVector3.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"

/// Viper wheel for use in a ChWheelTestRig.
class ViperRigWheel : public chrono::vehicle::ChWheelTestRig::WheelAssembly {
  public:
    ViperRigWheel(chrono::ChSystem& system);

    void SetRadius(double radius) { m_radius = radius; }
    void SetWidth(double width) { m_width = width; }
    void SetGrouserHeight(double height) { m_grouser_height = height; }
    void SetGrouserWidth(double width) { m_grouser_width = width; }
    void SetNumGrousers(int num_grousers) { m_num_grousers = num_grousers; }
    void SetMass(double mass) { m_mass = mass; }
    void SetInertia(const chrono::ChVector3d& inertia) { m_inertia = inertia; }

    virtual double GetRadius() const override { return m_radius + m_grouser_height; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetMass() const override { return m_mass; }
    virtual std::shared_ptr<chrono::ChBody> GetHub() const override { return m_wheel; }

    virtual void Initialize(const chrono::ChFramed& frame, bool fixed, double step_size, chrono::VisualizationType vis_type) override {
        m_wheel->SetPos(frame.GetPos());
        m_wheel->SetRot(frame.GetRot());
        m_wheel->SetFixed(fixed);
    }

#ifdef CHRONO_CRM
    virtual void AddFSIBodies(chrono::vehicle::CRMTerrain& terrain, double spacing) override;

    // Create wheel BCE markers.
    std::vector<chrono::ChVector3d> CreateBCE(double spacing, bool cartesian = false) const;
    static std::vector<chrono::ChVector3d> CreateBCE(double radius, double width, double grouser_height, double grouser_width, int num_grousers, double spacing, bool cartesian);
#endif

    // Save Viper wheel mesh in current configuration to a Paraview VTK file.
    void WriteVTK(const std::string& filename);
    static void WriteVTK(const std::string& filename, chrono::ChTriangleMeshConnected& mesh, const chrono::ChFrame<>& frame);

  private:
    std::shared_ptr<chrono::ChBody> m_wheel;

    double m_radius;
    double m_width;
    double m_grouser_height;
    double m_grouser_width;
    int m_num_grousers;
    double m_mass;
    chrono::ChVector3d m_inertia;
    std::shared_ptr<chrono::ChTriangleMeshConnected> m_trimesh;
};

// -----------------------------------------------------------------------------

ViperRigWheel::ViperRigWheel(chrono::ChSystem& system)
    : chrono::vehicle::ChWheelTestRig::WheelAssembly(system), m_radius(0.225), m_width(0.2), m_grouser_height(0.01), m_grouser_width(0.005), m_num_grousers(24) {
    // Create trimesh
    std::string mesh_filename = chrono::GetChronoDataFile("robot/viper/obj/nasa_viper_wheel.obj");

    m_trimesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    m_trimesh->LoadWavefrontMesh(mesh_filename, false, true);
    m_trimesh->RepairDuplicateVertices(1e-9);

    // Calculate inertia properties
    double density = 1500;
    chrono::ChVector3d cog;
    chrono::ChMatrix33<> inertia;
    chrono::ChMatrix33<> principal_inertia_rot;
    m_trimesh->ComputeMassProperties(true, m_mass, cog, inertia);
    chrono::ChInertiaUtils::PrincipalInertia(inertia, m_inertia, principal_inertia_rot);
    m_mass *= density;
    m_inertia *= density;

    // Create wheel body
    m_wheel = chrono_types::make_shared<chrono::ChBody>();
    m_wheel->SetMass(m_mass);
    m_wheel->SetInertia(inertia);
    system.AddBody(m_wheel);

    // Visualization shape
    auto vis_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    vis_shape->SetMesh(m_trimesh);
    vis_shape->SetName("ViperWheel");
    vis_shape->SetMutable(false);
    m_wheel->AddVisualShape(vis_shape);

    // Contact material and collision shape
    chrono::ChContactMaterialData mat_info;
    auto material = mat_info.CreateMaterial(system.GetContactMethod());
    auto ct_shape = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(material, m_trimesh, false, false, m_grouser_width / 2);
    m_wheel->AddCollisionShape(ct_shape);
    m_wheel->EnableCollision(true);
}

#ifdef CHRONO_CRM

void ViperRigWheel::AddFSIBodies(chrono::vehicle::CRMTerrain& terrain, double spacing) {
    auto bce = CreateBCE(spacing);
    terrain.GetFsiSystemSPH()->AddFsiBody(m_wheel, bce, chrono::ChFramed(), false);
}

std::vector<chrono::ChVector3d> ViperRigWheel::CreateBCE(double spacing, bool cartesian) const {
    return CreateBCE(m_radius, m_width - spacing, m_grouser_height, m_grouser_width, m_num_grousers, spacing, cartesian);
}

std::vector<chrono::ChVector3d> ViperRigWheel::CreateBCE(double radius, double width, double grouser_height, double grouser_width, int num_grousers, double spacing, bool cartesian) {
    std::vector<chrono::ChVector3d> bce;

    int num_layers = (int)std::floor(1.00001 * width / spacing) + 1;

    int numr = (int)std::floor(1.00001 * radius / spacing);
    int numr_g = (int)std::floor(1.00001 * grouser_height / spacing);
    int numw_g = (int)std::floor(1.00001 * grouser_width / spacing) + 1;

    for (size_t si = 0; si < num_layers; si++) {
        double s = -0.5 * width + spacing * si;
        if (cartesian)
            for (double x = -radius; x <= radius; x += spacing) {
                for (double y = -radius; y <= radius; y += spacing) {
                    if (x * x + y * y <= radius * radius)
                        bce.push_back(chrono::ChVector3d(x, s, y));
                }
            }
        else {
            chrono::ChVector3d center(0, s, 0);
            bce.push_back(chrono::ChVector3d(0, s, 0));

            // wheel
            for (size_t ir = 0; ir < numr; ir++) {
                double r = spacing + ir * spacing;
                int numTheta = (int)std::floor(2 * 3.1415 * r / spacing);
                for (size_t t = 0; t < numTheta; t++) {
                    double teta = t * 2 * 3.1415 / numTheta;
                    chrono::ChVector3d BCE_Pos_local(r * std::cos(teta), 0, r * std::sin(teta));
                    BCE_Pos_local += center;
                    bce.push_back(BCE_Pos_local);
                }
            }

            // grouser
            for (size_t ir_g = 0; ir_g < numr_g; ir_g++) {
                double r = 0.5 * spacing + ir_g * spacing + radius;
                for (size_t t = 0; t < num_grousers; t++) {
                    for (size_t iw_g = 0; iw_g < numw_g; iw_g++) {
                        double teta = t * 2 * 3.1415 / num_grousers + iw_g * spacing / radius;
                        chrono::ChVector3d BCE_Pos_local(r * std::cos(teta), 0, r * std::sin(teta));
                        BCE_Pos_local += center;
                        bce.push_back(BCE_Pos_local);
                    }
                }
            }
        }
    }

    return bce;
}

#endif

void ViperRigWheel::WriteVTK(const std::string& filename) {
    WriteVTK(filename, *m_trimesh, m_wheel->GetFrameRefToAbs());
}

void ViperRigWheel::WriteVTK(const std::string& filename, chrono::ChTriangleMeshConnected& mesh, const chrono::ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);

    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertices().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertices()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }

    outf.close();
}
