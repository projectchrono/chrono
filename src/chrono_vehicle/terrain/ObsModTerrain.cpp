// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Simple trench or mound shaped obstacle as needed for NRMM
//  The obstacle is defined by three parameters
//  - aa:        approach angle 180 deg = flat, aa < 180 deg = mound, aa > 180 deg = trench
//  - length:    obstacle length for mound, base length for trench
//  - obsheight: allways >= 0, obstacle height for mound, obstacle depth for trench
//
// =============================================================================

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/terrain/ObsModTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

ObsModTerrain::ObsModTerrain(ChSystem* system,
                             double height,
                             float friction,
                             double aa,
                             double obslength,
                             double obsheight)
    : m_height(height),
      m_width(6.0),
      m_aa(aa),
      m_friction(friction),
      m_obslength(obslength),
      m_obsheight(obsheight),
      m_xmin(0.0),
      m_nx(4),
      m_ny(4) {
    m_x.resize(m_nx);
    m_y.resize(m_ny);
    m_y[0] = -m_width / 2.0;
    m_y[1] = -m_width / 2.0 + 1.0;
    m_y[2] = m_width / 2.0 - 1.0;
    m_y[3] = m_width / 2.0;
    m_ymin = m_y[0];
    m_ymax = m_y[3];
    m_Q = ChMatrixDynamic<>::Zero(m_nx, m_ny);
    double ramp_angle = 0.0;
    double ramp_length = 1.0;

    if ((m_aa > 175.0) && (m_aa < 185.0)) {
        // flat terrain
        ramp_length = 1.0;
        m_x[0] = m_xmin;
        m_x[1] = m_x[0] + ramp_length;
        m_x[2] = m_x[1] + m_obslength;
        m_x[3] = m_x[2] + ramp_length;
        m_xmax = m_x[3];
        // m_Q is already set up
    } else if (aa <= 175.0) {
        // mound obstacle
        ramp_angle = CH_DEG_TO_RAD * abs(m_aa - 180.0);
        ramp_length = m_obsheight / std::tan(ramp_angle);
        m_x[0] = m_xmin;
        m_x[1] = m_x[0] + ramp_length;
        m_x[2] = m_xmin + m_obslength - ramp_length;
        m_x[3] = m_xmin + m_obslength;
        if (2.0 * ramp_length > m_obslength) {
            m_obslength = 0.1 + 2.0 * ramp_length;
            m_x[0] = m_xmin;
            m_x[1] = m_x[0] + ramp_length;
            m_x[2] = m_xmin + m_obslength - ramp_length;
            m_x[3] = m_xmin + m_obslength;
        }
        m_xmax = m_x[3];
        m_Q(1, 1) = m_obsheight;
        m_Q(2, 1) = m_obsheight;
        m_Q(2, 2) = m_obsheight;
        m_Q(1, 2) = m_obsheight;
    } else {
        // trench obstacle
        ramp_angle = CH_DEG_TO_RAD * abs(m_aa - 180.0);
        ramp_length = m_obsheight / std::tan(ramp_angle);
        if (m_obslength <= 0.0) {
            m_obslength = 0.1;
        }
        m_x[0] = m_xmin;
        m_x[1] = m_x[0] + ramp_length;
        m_x[2] = m_x[1] + m_obslength;
        m_x[3] = m_x[2] + ramp_length;
        m_xmax = m_x[3];
        m_Q(1, 1) = -m_obsheight;
        m_Q(2, 1) = -m_obsheight;
        m_Q(2, 2) = -m_obsheight;
        m_Q(1, 2) = -m_obsheight;
    }

    // ground body, carries the graphic assets
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector3d(0, 0, 0));
    m_ground->SetFixed(true);
    m_ground->EnableCollision(false);

    m_ground->AddVisualModel(chrono_types::make_shared<ChVisualModel>());

    system->Add(m_ground);
}

double ObsModTerrain::GetEffLength() {
    return std::hypot(m_x[1] - m_x[0], m_Q(1, 1) - m_Q(0, 1)) + std::hypot(m_x[2] - m_x[1], m_Q(2, 1) - m_Q(1, 1)) +
           std::hypot(m_x[3] - m_x[2], m_Q(3, 1) - m_Q(2, 1));
}

double ObsModTerrain::GetHeight(const ChVector3d& loc) const {
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    if ((loc_ISO.x() > m_xmin && loc_ISO.x() < m_xmax) && (loc_ISO.y() > m_ymin && loc_ISO.y() < m_ymax)) {
        // interpolation needed
        int ix1 = -1;
        int ix2 = -1;
        for (size_t i = 0; i < m_nx - 1; i++) {
            if (loc_ISO.x() >= m_x[i] && loc_ISO.x() < m_x[i + 1]) {
                ix1 = int(i);
                ix2 = int(i + 1);
                break;
            }
        }
        if (ix1 == -1) {
            std::cerr << "x intervall?\n";
        }
        int jy1 = -1;
        int jy2 = -1;
        for (size_t j = 0; j < m_ny - 1; j++) {
            if (loc_ISO.y() >= m_y[j] && loc_ISO.y() < m_y[j + 1]) {
                jy1 = int(j);
                jy2 = int(j + 1);
                break;
            }
        }
        if (jy1 == -1) {
            std::cerr << "y intervall?\n";
        }
        double x = loc_ISO.x();
        double y = loc_ISO.y();
        return m_height + BilinearInterpolation(m_Q(ix1, jy1), m_Q(ix1, jy2), m_Q(ix2, jy1), m_Q(ix2, jy2), m_x[ix1],
                                                m_x[ix2], m_y[jy1], m_y[jy2], x, y);
    } else {
        return m_height;
    }
}

ChVector3d ObsModTerrain::GetNormal(const ChVector3d& loc) const {
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    // to avoid 'jumping' of the normal vector, we take this smoothing approach
    const double delta = 0.05;
    double z0, zfront, zleft;
    z0 = GetHeight(loc);
    zfront = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector3d(delta, 0, 0)));
    zleft = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector3d(0, delta, 0)));
    ChVector3d p0(loc_ISO.x(), loc_ISO.y(), z0);
    ChVector3d pfront(loc_ISO.x() + delta, loc_ISO.y(), zfront);
    ChVector3d pleft(loc_ISO.x(), loc_ISO.y() + delta, zleft);
    ChVector3d normal_ISO;
    ChVector3d r1, r2;
    r1 = pfront - p0;
    r2 = pleft - p0;
    normal_ISO = Vcross(r1, r2);
    if (normal_ISO.z() <= 0.0) {
        std::cerr << "Fatal: wrong surface normal!" << std::endl;
        throw std::runtime_error("Fatal: wrong surface normal!");
    }
    ChVector3d normal = ChWorldFrame::FromISO(normal_ISO);
    normal.Normalize();

    return normal;
}

float ObsModTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : m_friction;
}

void ObsModTerrain::Initialize(ObsModTerrain::VisualisationType vType) {
    switch (vType) {
        case ObsModTerrain::VisualisationType::NONE:
            break;
        case ObsModTerrain::VisualisationType::MESH:
            GenerateMesh();
            break;
    }
    if (m_collision_mesh)
        SetupCollision();
}

void ObsModTerrain::GenerateMesh() {
    m_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    auto& coords = m_mesh->GetCoordsVertices();
    auto& indices = m_mesh->GetIndicesVertexes();
    auto& normals = m_mesh->GetCoordsNormals();
    auto& normidx = m_mesh->GetIndicesNormals();

    for (size_t i = 0; i < m_nx; i++) {
        double x = m_x[i];
        for (size_t j = 0; j < m_y.size(); j++) {
            double y = m_y[j];
            double z = m_Q(i, j);
            coords.push_back(ChWorldFrame::FromISO(ChVector3d(x, y, z)));
            normals.push_back(ChWorldFrame::FromISO(GetNormal(ChVector3d(x, y, z))));
        }
    }
    // Define the faces
    for (int i = 0; i < m_nx - 1; i++) {
        int ofs = (int)m_y.size() * i;
        for (int j = 0; j < m_y.size() - 1; j++) {
            indices.push_back(ChVector3i(j + ofs, j + (int)m_y.size() + ofs, j + 1 + ofs));
            indices.push_back(ChVector3i(j + 1 + ofs, j + (int)m_y.size() + ofs, j + 1 + (int)m_y.size() + ofs));
            normidx.push_back(ChVector3i(j + ofs, j + (int)m_y.size() + ofs, j + 1 + ofs));
            normidx.push_back(ChVector3i(j + 1 + ofs, j + (int)m_y.size() + ofs, j + 1 + (int)m_y.size() + ofs));
        }
    }
    auto vmesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vmesh->SetMesh(m_mesh);
    vmesh->SetMutable(false);
    vmesh->SetName("ISO_track");
    vmesh->SetColor(ChColor(0.6f, 0.6f, 0.8f));
    m_ground->AddVisualShape(vmesh);
}

void ObsModTerrain::EnableCollisionMesh(std::shared_ptr<ChContactMaterial> material,
                                        double length,
                                        double sweep_sphere_radius) {
    m_material = material;
    m_start_length = length;
    m_sweep_sphere_radius = sweep_sphere_radius;
    m_collision_mesh = true;
}

void ObsModTerrain::SetupCollision() {
    GenerateMesh();

    m_ground->EnableCollision(true);

    auto ct_shape =
        chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_material, m_mesh, true, false, m_sweep_sphere_radius);
    m_ground->AddCollisionShape(ct_shape);

    if (m_start_length > 0) {
        double thickness = 1;
        ChVector3d loc(-m_start_length / 2, 0, m_height - thickness / 2);
        auto ct_box = chrono_types::make_shared<ChCollisionShapeBox>(m_material, m_start_length, m_width, thickness);
        m_ground->AddCollisionShape(ct_box, ChFrame<>(loc, QUNIT));

        auto vis_box = chrono_types::make_shared<ChVisualShapeBox>(m_start_length, m_width, thickness);
        m_ground->AddVisualShape(vis_box, ChFrame<>(loc));

        // we also need an end plate here
        double end_length = 10.0;
        ChVector3d loc2(GetXObstacleEnd() + end_length / 2, 0, m_height - thickness / 2);
        auto ct_box2 = chrono_types::make_shared<ChCollisionShapeBox>(m_material, end_length, m_width, thickness);
        m_ground->AddCollisionShape(ct_box2, ChFrame<>(loc2, QUNIT));

        auto vis_box2 = chrono_types::make_shared<ChVisualShapeBox>(end_length, m_width, thickness);
        m_ground->AddVisualShape(vis_box2, ChFrame<>(loc2));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
