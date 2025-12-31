// Utility classes and functions for Viper wheel simulations

#include <string>
#include <fstream>
#include <vector>
#include <cmath>

#include "chrono/core/ChVector3.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

// -----------------------------------------------------------------------------

// Dummy Viper wheel subsystem (needed to attach a ChTire).
class DummyViperWheel : public chrono::vehicle::ChWheel {
  public:
    DummyViperWheel() : chrono::vehicle::ChWheel("tire_wheel"), m_inertia(chrono::ChVector3d(0)) {}
    virtual double GetWheelMass() const override { return 0; }
    virtual const chrono::ChVector3d& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return 1; }
    virtual double GetWidth() const override { return 1; }

  private:
    chrono::ChVector3d m_inertia;
};

// -----------------------------------------------------------------------------

// Viper rigid tire.
class ViperTire : public chrono::vehicle::ChRigidTire {
  public:
    ViperTire()
        : chrono::vehicle::ChRigidTire("viper_tire"),
          m_radius(0.225),
          m_width(0.2),
          m_grouser_height(0.01),
          m_grouser_width(0.005),
          m_num_grousers(24) {
        SetMeshFile(chrono::GetChronoDataFile("robot/viper/obj/nasa_viper_wheel.obj"));
    }

    void SetRadius(double radius) { m_radius = radius; }
    void SetWidth(double width) { m_width = width; }
    void SetGrouserHeight(double height) { m_grouser_height = height; }
    void SetGrouserWidth(double width) { m_grouser_width = width; }
    void SetNumGrousers(int num_grousers) { m_num_grousers = num_grousers; }

    void SetMeshFile(const std::string& filename, double scale = 1) {
        // Create trimesh
        m_trimesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
        m_trimesh->LoadWavefrontMesh(filename, false, true);
        m_trimesh->Transform(chrono::ChVector3d(0, 0, 0), chrono::ChMatrix33<>(scale));
        m_trimesh->RepairDuplicateVertexes(1e-9);

        // Calculate inertia properties
        double density = 1500;
        chrono::ChVector3d cog;
        chrono::ChMatrix33<> inertia;
        chrono::ChMatrix33<> principal_inertia_rot;
        m_trimesh->ComputeMassProperties(true, m_mass, cog, inertia);
        chrono::ChInertiaUtils::PrincipalInertia(inertia, m_inertia, principal_inertia_rot);
        m_mass *= density;
        m_inertia *= density;

        // Use contact mesh
        SetContactMesh(filename, m_grouser_width / 2);

        std::cout << "Viper wheel mass:    " << m_mass << std::endl;
        std::cout << "Viper wheel inertia: " << m_inertia << std::endl;
    }

    void SetMass(double mass) { m_mass = mass; }
    void SetInertia(const chrono::ChVector3d& inertia) { m_inertia = inertia; }

    // Create BCE markers on Viper wheel with grousers.
    std::vector<chrono::ChVector3d> CreateBCE(double spacing, bool cartesian = false) const {
        std::vector<chrono::ChVector3d> bce;

        double width = m_width - spacing;
        int num_layers = (int)std::floor(1.00001 * width / spacing) + 1;

        int numr = (int)std::floor(1.00001 * m_radius / spacing);
        int numr_g = (int)std::floor(1.00001 * m_grouser_height / spacing);
        int numw_g = (int)std::floor(1.00001 * m_grouser_width / spacing) + 1;

        for (size_t si = 0; si < num_layers; si++) {
            double s = -0.5 * width + spacing * si;
            if (cartesian)
                for (double x = -m_radius; x <= m_radius; x += spacing) {
                    for (double y = -m_radius; y <= m_radius; y += spacing) {
                        if (x * x + y * y <= m_radius * m_radius)
                            bce.push_back(chrono::ChVector3d(x, s, y));
                    }
                }
            else {
                chrono::ChVector3d center(0, s, 0);
                bce.push_back(chrono::ChVector3d(0, s, 0));

                // wheel
                for (size_t ir = 0; ir < numr; ir++) {
                    double r = spacing + ir * spacing;
                    int numTheta = (int)std::floor(chrono::CH_2PI * r / spacing);
                    for (size_t t = 0; t < numTheta; t++) {
                        double teta = t * chrono::CH_2PI / numTheta;
                        chrono::ChVector3d pos(r * std::cos(teta), 0, r * std::sin(teta));
                        pos += center;
                        bce.push_back(pos);
                    }
                }

                // grouser
                for (size_t ir_g = 0; ir_g < numr_g; ir_g++) {
                    double r = m_radius + 0.5 * spacing + ir_g * spacing;
                    for (size_t t = 0; t < m_num_grousers; t++) {
                        for (size_t iw_g = 0; iw_g < numw_g; iw_g++) {
                            double teta = t * chrono::CH_2PI / m_num_grousers + (iw_g - numw_g / 2) * spacing / m_radius;
                            chrono::ChVector3d pos(r * std::cos(teta), 0, r * std::sin(teta));
                            pos += center;
                            bce.push_back(pos);
                        }
                    }
                }
            }
        }

        return bce;
    }

    // Save Viper wheel mesh in current configuration to a Paraview VTK file.
    void WriteVTK(const std::string& filename) {
        const auto& frame = GetWheel()->GetSpindle()->GetFrameRefToAbs();

        std::ofstream outf;
        outf.open(filename);

        outf << "# vtk DataFile Version 2.0" << std::endl;
        outf << "VTK from simulation" << std::endl;
        outf << "ASCII" << std::endl;
        outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
        outf << "POINTS " << m_trimesh->GetCoordsVertices().size() << " "
             << "float" << std::endl;
        for (auto& v : m_trimesh->GetCoordsVertices()) {
            auto w = frame.TransformPointLocalToParent(v);
            outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
        }
        auto nf = m_trimesh->GetIndicesVertexes().size();
        outf << "CELLS " << nf << " " << 4 * nf << std::endl;
        for (auto& f : m_trimesh->GetIndicesVertexes()) {
            outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
        }
        outf << "CELL_TYPES " << nf << std::endl;
        for (int i = 0; i < nf; i++) {
            outf << "5 " << std::endl;
        }

        outf.close();
    }

  private:
    virtual double GetRadius() const override { return m_radius + m_grouser_height; }
    virtual double GetWidth() const override { return m_width; }

    virtual double GetTireMass() const override { return m_mass; }
    virtual chrono::ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual void CreateContactMaterial(chrono::ChContactMethod contact_method) override {
        chrono::ChContactMaterialData mat_info;
        m_material = mat_info.CreateMaterial(contact_method);
    }

    virtual void AddVisualizationAssets(chrono::VisualizationType vis) override {
        m_trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        m_trimesh_shape->SetMesh(m_trimesh);
        m_trimesh_shape->SetName("ViperWheel");
        m_trimesh_shape->SetMutable(false);
        m_wheel->GetSpindle()->AddVisualShape(m_trimesh_shape);
    }

    virtual void RemoveVisualizationAssets() override {
        chrono::vehicle::ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
        chrono::vehicle::ChRigidTire::RemoveVisualizationAssets();
    }

    double m_radius;
    double m_width;
    double m_grouser_height;
    double m_grouser_width;
    int m_num_grousers;
    double m_mass;
    chrono::ChVector3d m_inertia;
    std::shared_ptr<chrono::ChTriangleMeshConnected> m_trimesh;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> m_trimesh_shape;
};

// -----------------------------------------------------------------------------

// Custom callback for Viper tire BCE marker generation.
class ViperTireBCE : public chrono::vehicle::ChTireTestRig::WheelBCECreationCallback {
  public:
    ViperTireBCE(std::shared_ptr<ViperTire> tire, double spacing) { m_bce = tire->CreateBCE(spacing); }
    virtual std::vector<chrono::ChVector3d> GetMarkers() {
        std::cout << "Viper wheel -- num BCEs: " << m_bce.size() << std::endl;
        return m_bce;
    }

  private:
    std::vector<chrono::ChVector3d> m_bce;
};

// -----------------------------------------------------------------------------

// Save wheel mesh to Paraview VTK file
void WriteWheelVTK(const std::string& filename, chrono::ChTriangleMeshConnected& mesh, const chrono::ChFrame<>& frame) {
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
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

// -----------------------------------------------------------------------------

// Create BCE markers on Viper wheel.
std::vector<chrono::ChVector3d> CreateWheelBCE(double wheel_rad,
                                               double wheel_w,
                                               double gro_h,
                                               double gro_w,
                                               int gro_num,
                                               double spacing,
                                               bool cartesian) {
    std::vector<chrono::ChVector3d> bce;

    int num_layers = (int)std::floor(1.00001 * wheel_w / spacing) + 1;
    for (size_t si = 0; si < num_layers; si++) {
        double s = -0.5 * wheel_w + spacing * si;

        if (cartesian)
            for (double x = -wheel_rad; x <= wheel_rad; x += spacing) {
                for (double y = -wheel_rad; y <= wheel_rad; y += spacing) {
                    if (x * x + y * y <= wheel_rad * wheel_rad)
                        bce.push_back(chrono::ChVector3d(x, s, y));
                }
            }
        else {
            chrono::ChVector3d center(0, s, 0);
            bce.push_back(chrono::ChVector3d(0, s, 0));

            // wheel
            int numr = (int)std::floor(1.00001 * wheel_rad / spacing);
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
            int numr_g = (int)std::floor(1.00001 * gro_h / spacing);
            int numw_g = (int)std::floor(1.00001 * gro_w / spacing) + 1;
            for (size_t ir_g = 0; ir_g < numr_g; ir_g++) {
                double r = 0.5 * spacing + ir_g * spacing + wheel_rad;
                for (size_t t = 0; t < gro_num; t++) {
                    for (size_t iw_g = 0; iw_g < numw_g; iw_g++) {
                        double teta = t * 2 * 3.1415 / gro_num + iw_g * spacing / wheel_rad;
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
