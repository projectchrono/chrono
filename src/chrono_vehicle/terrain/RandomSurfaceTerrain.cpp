// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
// Terrain object representing an uneven area with controlled roughness.
//
// By default, this class implements a terrain modeled as an infinite horizontal plane at a specified height with the
// uneven terrain lane of specified length and width starting at the origin. This type of terrain can be used in
// conjunction with tire models that perform their own collision detection (e.g. ChPacejkaTire, ChFiala, and
// ChLugreTire).
//
// Alternatively, this terrain type can be represented as a collision mesh representing the uneven lane with an
// optional flat starting lane.  This terrain type can be used with vehicles that require contact for the
// vehicle-terrain interaction (wheeled vehicle with rigid tires or tracked vehicles).
// ===================================================================================================================

#include <random>
#include <cmath>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineSegment.h"

#include "chrono/assets/ChLineShape.h"

namespace chrono {
namespace vehicle {

using namespace chrono::geometry;
using namespace Eigen;

RandomSurfaceTerrain::RandomSurfaceTerrain(ChSystem* system, double length, double width, double height, float friction)
    : m_length(length),
      m_width(width),
      m_height(height),
      m_friction(friction),
      m_rms(0.0),
      m_dx(0.1),
      m_lambda_max(20.0),
      m_collision_mesh(false),
      m_start_length(0),
      m_sweep_sphere_radius(0) {
    m_xmin = 0.0;
    m_xmax = floor(length);
    m_ymin = -ceil(width / 2.0);
    m_ymax = ceil(width / 2.0);
    m_nx = static_cast<int>(m_xmax / m_dx + 1);
    m_ny = 7;
    m_y.resize(m_ny);
    m_y[0] = m_ymin;
    m_y[1] = m_ymin + 0.2;
    m_y[2] = -0.2;
    m_y[3] = 0.0;
    m_y[4] = 0.2;
    m_y[5] = m_ymax - 0.2;
    m_y[6] = m_ymax;
    m_f_fft_min = 1.0 / m_length;
    m_f_fft_max = 10.0;
    double dtmax = m_dx;
    int p = static_cast<int>(std::ceil(-std::log(m_f_fft_min * dtmax) / std::log(2.0)));
    m_Nfft = 2 << p;

    // ground body, carries the graphic assets
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(false);
    m_ground->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
    system->Add(m_ground);

    m_curve_left_name = "leftContour";
    m_curve_left_name = "rightContour";
    
    // class limits for unevenness
    m_classLimits.resize(9);
    m_classLimits << 0.0, 2e-6, 8e-6, 32e-6, 128e-6, 512e-6, 2048e-6, 8192e-6, 16384e-6;
}

void RandomSurfaceTerrain::ApplyAmplitudes() {
    m_rms = 0.0;
    for (int i = 0; i < m_nx; i++) {
        double x = m_dx * double(i);
        double A_left = CalculateAmplitudeLeft(x);
        double A_right = CalculateAmplitudeRight(x);
        m_Q(i, 1) = A_right;
        m_Q(i, 2) = A_right;
        m_Q(i, 4) = A_left;
        m_Q(i, 5) = A_left;
        m_rms += A_right * A_right + A_left * A_left;
    }
    m_rms = std::sqrt(m_rms / (2.0 * m_nx));
    m_iri = 2.21 * std::sqrt(m_unevenness * 1e6) *
            std::exp(-0.356 * (m_waviness - 2.0) + 0.13 * std::pow(m_waviness - 2.0, 2));
}

double RandomSurfaceTerrain::GetHeight(const ChVector<>& loc) const {
    ChVector<> loc_ISO = ChWorldFrame::ToISO(loc);
    if (loc_ISO.x() < m_xmin || loc_ISO.x() > m_xmax)
        return m_height;
    if (loc_ISO.y() < m_ymin || loc_ISO.y() > m_ymax)
        return m_height;
    int ix = (std::abs(loc_ISO.x() - m_xmax) > 1e-6) ? static_cast<int>((loc_ISO.x() - m_xmin) / m_dx) : m_nx - 2;
    int iy = -1;
    for (int i = 0; i < m_ny - 1; i++) {
        if (loc_ISO.y() >= m_y[i] && loc_ISO.y() <= m_y[i + 1]) {
            iy = i;
            break;
        }
    }
    return m_height + m_a0(ix, iy) + m_a1(ix, iy) * loc.x() + m_a2(ix, iy) * loc.y() + m_a3(ix, iy) * loc.x() * loc.y();
}

ChVector<> RandomSurfaceTerrain::GetNormal(const ChVector<>& loc) const {
    ChVector<> loc_ISO = ChWorldFrame::ToISO(loc);
    // to avoid 'jumping' of the normal vector, we take this smoothing approach
    const double delta = 0.05;
    double z0, zfront, zleft;
    z0 = GetHeight(loc);
    zfront = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector<>(delta, 0, 0)));
    zleft = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector<>(0, delta, 0)));
    ChVector<> p0(loc_ISO.x(), loc_ISO.y(), z0);
    ChVector<> pfront(loc_ISO.x() + delta, loc_ISO.y(), zfront);
    ChVector<> pleft(loc_ISO.x(), loc_ISO.y() + delta, zleft);
    ChVector<> normal_ISO;
    ChVector<> r1, r2;
    r1 = pfront - p0;
    r2 = pleft - p0;
    normal_ISO = Vcross(r1, r2);
    if (normal_ISO.z() <= 0.0) {
        GetLog() << "Fatal: wrong surface normal!\n";
        exit(99);
    }
    ChVector<> normal = ChWorldFrame::FromISO(normal_ISO);
    normal.Normalize();

    return normal;
}

float RandomSurfaceTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : m_friction;
}

void RandomSurfaceTerrain::GenerateSurfaceCanonical(double unevenness, double waviness) {
    m_unevenness = ChClamp(unevenness, 1.0e-6, m_classLimits[7]);
    m_waviness = waviness;
    m_Q = ChMatrixDynamic<>::Zero(m_nx, m_ny);
    CalculateSpectralCoefficients(m_unevenness, m_waviness);
    ApplyAmplitudes();
    m_a0 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a1 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a2 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a3 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    CalculatePolynomialCoefficients();
}

void RandomSurfaceTerrain::GenerateSurfaceCanonicalCorr(double unevenness,
                                                        double vehicleTrackWidth,
                                                        double omega_p,
                                                        double p,
                                                        double waviness,
                                                        double a) {
    m_waviness = waviness;
    m_Q = ChMatrixDynamic<>::Zero(m_nx, m_ny);
    CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, omega_p, p, m_waviness, a);
    ApplyAmplitudes();
    m_a0 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a1 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a2 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a3 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    CalculatePolynomialCoefficients();
}

void RandomSurfaceTerrain::GenerateSurfaceFromPreset(SurfaceType theSurface, double vehicleTrackWidth) {
    m_Q = ChMatrixDynamic<>::Zero(m_nx, m_ny);
    switch (theSurface) {
        case SurfaceType::FLAT:
            // that's all
            break;
        case SurfaceType::ISO8608_A_NOCORR:
            m_unevenness = 1.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_B_NOCORR:
            m_unevenness = 4.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_C_NOCORR:
            m_unevenness = 16.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_D_NOCORR:
            m_unevenness = 64.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_E_NOCORR:
            m_unevenness = 256.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_F_NOCORR:
            m_unevenness = 1024.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_G_NOCORR:
            m_unevenness = 4096.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_H_NOCORR:
            m_unevenness = 16384.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficients(m_unevenness, m_waviness);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_A_CORR:
            m_unevenness = 1.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 8.5, 3.5, m_waviness, 1.0);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_B_CORR:
            m_unevenness = 4.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 5.3, 2.2, m_waviness, 1.0);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_C_CORR:
            m_unevenness = 16.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 3.0, 1.3, m_waviness, 1.0);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_D_CORR:
            m_unevenness = 64.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 1.6, 0.8, m_waviness, 1.0);
            ApplyAmplitudes();
            break;
        case SurfaceType::ISO8608_E_CORR:
            m_unevenness = 256.0e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 0.8, 0.5, m_waviness, 1.0);
            ApplyAmplitudes();
            break;
        case SurfaceType::MAJOR_ROAD_CONCRETE:
            m_unevenness = 3.7e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 0.96, 0.47, m_waviness, 0.96);
            ApplyAmplitudes();
            break;
        case SurfaceType::MAJOR_ROAD_ASPHALTIC_CONCRETE:
            m_unevenness = 2.6e-6;
            m_waviness = 2.1;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 0.73, 0.45, m_waviness, 0.6);
            ApplyAmplitudes();
            break;
        case SurfaceType::MAIN_ROAD_ASPHALTIC_CONCRETE_ON_PAVEMENT:
            m_unevenness = 2.6e-6;
            m_waviness = 2.3;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 1.53, 0.45, m_waviness, 0.56);
            ApplyAmplitudes();
            break;
        case SurfaceType::MAIN_ROAD_ASPHALTIC_CONCRETE:
            m_unevenness = 5e-6;
            m_waviness = 2.2;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 3.3, 0.88, m_waviness, 0.97);
            ApplyAmplitudes();
            break;
        case SurfaceType::TRACK_TILED_CONCRETE_PAVEMENT:
            m_unevenness = 10e-6;
            m_waviness = 2.0;
            CalculateSpectralCoefficientsCorr(m_unevenness, vehicleTrackWidth, 0.99, 0.5, m_waviness, 0.94);
            ApplyAmplitudes();
            break;
    }
    m_a0 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a1 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a2 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    m_a3 = ChMatrixDynamic<>::Zero(m_nx - 1, m_ny - 1);
    CalculatePolynomialCoefficients();
}

void RandomSurfaceTerrain::CalculatePolynomialCoefficients() {
    // m_Q was set up before!
    for (int i = 0; i < m_nx - 1; i++) {
        for (int j = 0; j < m_ny - 1; j++) {
            double Q11 = m_Q(i, j);
            double Q12 = m_Q(i, j + 1);
            double Q21 = m_Q(i + 1, j);
            double Q22 = m_Q(i + 1, j + 1);
            double x1 = m_dx * double(i);
            double x2 = m_dx * double(i + 1);
            double y1 = m_y[j];
            double y2 = m_y[j + 1];

            ChMatrixDynamic<> r(4, 1);
            r << Q11, Q12, Q21, Q22;
            ChMatrixDynamic<> K(4, 4);
            K << 1, x1, y1, x1 * y1, 1, x1, y2, x1 * y2, 1, x2, y1, x2 * y1, 1, x2, y2, x2 * y2;
            ChMatrixDynamic<> a = K.fullPivLu().solve(r);
            m_a0(i, j) = a(0);
            m_a1(i, j) = a(1);
            m_a2(i, j) = a(2);
            m_a3(i, j) = a(3);
        }
    }
}

void RandomSurfaceTerrain::CalculateSpectralCoefficients(double Phi_h0, double waviness) {
    const double w0 = 1.0;
    for (int i = 1; i < m_Nfft; i++) {
        double f = m_f_fft_min * i;
        if (f > m_f_fft_max)
            break;
        double Lbd = 1.0 / f;
        if (Lbd > m_lambda_max)
            continue;
        double w = CH_C_2PI * f;
        double ck = std::sqrt(Phi_h0 * std::pow(w / w0, -waviness) * m_f_fft_min);
        m_ck.push_back(ck);
        m_wfft.push_back(w);
    }
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, CH_C_2PI);

    m_phase_left.resize(m_ck.size());
    for (int i = 0; i < m_ck.size(); i++) {
        m_phase_left[i] = distribution(generator);
    }
    m_phase_right.resize(m_ck.size());
    for (int i = 0; i < m_ck.size(); i++) {
        m_phase_right[i] = distribution(generator);
    }
}

void RandomSurfaceTerrain::CalculateSpectralCoefficientsCorr(double Phi_h0,
                                                             double trackWidth,
                                                             double omega_p,
                                                             double p,
                                                             double waviness,
                                                             double a) {
    const double w0 = 1.0;
    for (int i = 1; i < m_Nfft; i++) {
        double f = m_f_fft_min * double(i);
        if (f > m_f_fft_max)
            break;
        double Lbd = 1.0 / f;
        if (Lbd > 20.0)
            continue;
        double w = CH_C_2PI * f;
        double ck = std::sqrt(Phi_h0 * std::pow(w / w0, -waviness) * m_f_fft_min);
        m_ck.push_back(ck);
        m_wfft.push_back(w);
    }
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, CH_C_2PI);

    m_phase_left.resize(m_ck.size());
    for (int i = 0; i < m_ck.size(); i++) {
        m_phase_left[i] = distribution(generator);
    }
    m_phase_right.resize(m_ck.size());
    for (int i = 0; i < m_ck.size(); i++) {
        m_phase_right[i] = distribution(generator);
    }
    // consider correlation by blending the phase angles via coherence function
    for (int i = 0; i < m_wfft.size(); i++) {
        double coh = Coherence(m_wfft[i], trackWidth, omega_p, p, waviness, a);
        double ph_left = m_phase_left[i];
        double ph_right = m_phase_right[i];
        m_phase_right[i] = ph_left * coh + ph_right * (1.0 - coh);
    }
}

double RandomSurfaceTerrain::CalculateAmplitudeLeft(double x) {
    double A = 0.0;
    for (int i = 0; i < m_ck.size(); i++) {
        A += m_ck[i] * cos(m_wfft[i] * x + m_phase_left[i]);
    }
    double fade = ChSineStep(x, 0, 0, 10.0, 1.0) * ChSineStep(x, m_xmax - 10.0, 1.0, m_xmax, 0.0);
    return 2.0 * A * fade;
}

double RandomSurfaceTerrain::CalculateAmplitudeRight(double x) {
    double A = 0.0;
    for (int i = 0; i < m_ck.size(); i++) {
        A += m_ck[i] * cos(m_wfft[i] * x + m_phase_right[i]);
    }
    double fade = ChSineStep(x, 0, 0, 10.0, 1.0) * ChSineStep(x, m_xmax - 10.0, 1.0, m_xmax, 0.0);
    return 2.0 * A * fade;
}

double RandomSurfaceTerrain::Coherence(double omega,
                                       double trackWidth,
                                       double omega_p,
                                       double p,
                                       double waviness,
                                       double a) {
    return std::pow(1.0 + std::pow(omega * std::pow(trackWidth, a) / omega_p, waviness), -p);
}

void RandomSurfaceTerrain::GenerateCurves() {
    std::vector<ChVector<>> pl, pr;
    int np = static_cast<int>(m_xmax / m_dx);

    int j_left = 1;
    int j_right = 5;
    double yl = m_y[j_left];
    double yr = m_y[j_right];
    for (int i = 0; i < np; i++) {
        double x = m_xmin + i * m_dx;
        double zl = m_Q(i, j_left);
        double zr = m_Q(i, j_right);
        pl.push_back(ChWorldFrame::FromISO(ChVector<>(x, yl, zl)));
        pr.push_back(ChWorldFrame::FromISO(ChVector<>(x, yr, zr)));
    }
    // Create the two road boundary Bezier curves
    m_road_left = chrono_types::make_shared<ChBezierCurve>(pl);
    m_road_right = chrono_types::make_shared<ChBezierCurve>(pr);
}

// very simple normal calculation based on GetNormal()
void RandomSurfaceTerrain::GenerateMesh() {
    if (m_mesh)
        return;

    m_mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    auto& coords = m_mesh->getCoordsVertices();
    auto& indices = m_mesh->getIndicesVertexes();
    auto& normals = m_mesh->getCoordsNormals();
    auto& normidx = m_mesh->getIndicesNormals();

    for (int i = 0; i < m_nx; i++) {
        double x = m_dx * double(i);
        for (int j = 0; j < m_y.size(); j++) {
            double y = m_y[j];
            double z = m_Q(i, j);
            coords.push_back(ChWorldFrame::FromISO(ChVector<>(x, y, z)));
            normals.push_back(ChWorldFrame::FromISO(GetNormal(ChVector<>(x, y, z))));
        }
    }
    // Define the faces
    for (int i = 0; i < m_nx - 1; i++) {
        int ysize = static_cast<int>(m_y.size());
        for (int j = 0; j < m_y.size() - 1; j++) {
            indices.push_back(ChVector<int>(j + 0 + ysize * i, j + ysize + ysize * i, j + 1 + ysize * i));
            indices.push_back(ChVector<int>(j + 1 + ysize * i, j + ysize + ysize * i, j + 1 + ysize + ysize * i));
            normidx.push_back(ChVector<int>(j + 0 + ysize * i, j + ysize + ysize * i, j + 1 + ysize * i));
            normidx.push_back(ChVector<int>(j + 1 + ysize * i, j + ysize + ysize * i, j + 1 + ysize + ysize * i));
        }
    }
}

void RandomSurfaceTerrain::SetupVisualization(RandomSurfaceTerrain::VisualisationType vType) {
    switch (vType) {
        case RandomSurfaceTerrain::VisualisationType::NONE:
            break;

        case RandomSurfaceTerrain::VisualisationType::LINES: {
            GenerateCurves();

            auto np = m_road_left->getNumPoints();
            unsigned int num_render_points = std::max<unsigned int>(static_cast<unsigned int>(3 * np), 400);
            auto bezier_line_left = chrono_types::make_shared<geometry::ChLineBezier>(m_road_left);
            auto bezier_asset_left = chrono_types::make_shared<ChLineShape>();
            bezier_asset_left->SetLineGeometry(bezier_line_left);
            bezier_asset_left->SetNumRenderPoints(num_render_points);
            bezier_asset_left->SetName(m_curve_left_name);
            m_ground->AddVisualShape(bezier_asset_left);

            auto bezier_line_right = chrono_types::make_shared<geometry::ChLineBezier>(m_road_right);
            auto bezier_asset_right = chrono_types::make_shared<ChLineShape>();
            bezier_asset_right->SetLineGeometry(bezier_line_right);
            bezier_asset_right->SetNumRenderPoints(num_render_points);
            bezier_asset_right->SetName(m_curve_right_name);
            m_ground->AddVisualShape(bezier_asset_right);

            break;
        }

        case RandomSurfaceTerrain::VisualisationType::MESH: {
            GenerateMesh();

            auto vmesh = chrono_types::make_shared<ChTriangleMeshShape>();
            vmesh->SetMesh(m_mesh);
            vmesh->SetMutable(false);
            vmesh->SetName("ISO_track");
            m_ground->AddVisualShape(vmesh);

            break;
        }
    }
}

void RandomSurfaceTerrain::EnableCollisionMesh(std::shared_ptr<ChMaterialSurface> material,
                                               double length,
                                               double sweep_sphere_radius) {
    m_material = material;
    m_start_length = length;
    m_sweep_sphere_radius = sweep_sphere_radius;
    m_collision_mesh = true;
}

void RandomSurfaceTerrain::SetupCollision() {
    GenerateMesh();

    m_ground->SetCollide(true);
    m_ground->GetCollisionModel()->ClearModel();

    m_ground->GetCollisionModel()->AddTriangleMesh(m_material, m_mesh, true, false, VNULL, ChMatrix33<>(1),
                                                   m_sweep_sphere_radius);

    if (m_start_length > 0) {
        double thickness = 1;
        ChVector<> loc(-m_start_length / 2, 0, m_height - thickness / 2);
        m_ground->GetCollisionModel()->AddBox(m_material, 0.5 * m_start_length, 0.5 * m_width, 0.5 * thickness, loc);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(m_start_length, m_width, thickness));
        m_ground->AddVisualShape(box, ChFrame<>(loc));
    }

    m_ground->GetCollisionModel()->BuildModel();
}

void RandomSurfaceTerrain::Initialize(RandomSurfaceTerrain::SurfaceType surfType,
                                      double vehicleTrackWidth,
                                      RandomSurfaceTerrain::VisualisationType vType) {
    GenerateSurfaceFromPreset(surfType, vehicleTrackWidth);
    SetupVisualization(vType);
    if (m_collision_mesh)
        SetupCollision();
}

void RandomSurfaceTerrain::Initialize(double iri,
                                      double vehicleTrackWidth,
                                      bool considerCorrelation,
                                      RandomSurfaceTerrain::VisualisationType vType) {
    double unevenness = std::pow(iri / 2.21, 2) * 1.0e-6;
    double waviness = 2.0;
    Initialize(unevenness, waviness, vehicleTrackWidth, considerCorrelation, vType);
}

void RandomSurfaceTerrain::Initialize(double unevenness,
                                      double waviness,
                                      double vehicleTrackWidth,
                                      bool considerCorrelation,
                                      RandomSurfaceTerrain::VisualisationType vType) {
    double omega_p = 0.0;
    double p = 0.0;
    const double a = 1.0;

    // we better don't try to go beyond ISO class H
    unevenness = ChClamp(unevenness, 1.0e-6, m_classLimits[7]);

    int class_index = -1;
    for (int i = 0; i < m_classLimits.size() - 1; i++) {
        if (unevenness >= m_classLimits[i] && unevenness <= m_classLimits[i + 1]) {
            class_index = i;
            break;
        }
    }
    switch (class_index) {
        // class A
        case 0:
            omega_p = 8.5;
            p = 3.5;
            if (considerCorrelation) {
                GenerateSurfaceCanonicalCorr(m_unevenness, vehicleTrackWidth, omega_p, p, m_waviness, a);
            } else {
                GenerateSurfaceCanonical(unevenness, waviness);
            }
            break;
        // class B
        case 1:
            omega_p = 5.3;
            p = 2.2;
            if (considerCorrelation) {
                GenerateSurfaceCanonicalCorr(m_unevenness, vehicleTrackWidth, omega_p, p, m_waviness, a);
            } else {
                GenerateSurfaceCanonical(unevenness, waviness);
            }
            break;
        // class C
        case 2:
            omega_p = 3.0;
            p = 1.3;
            if (considerCorrelation) {
                GenerateSurfaceCanonicalCorr(m_unevenness, vehicleTrackWidth, omega_p, p, m_waviness, a);
            } else {
                GenerateSurfaceCanonical(unevenness, waviness);
            }
            break;
        // class D
        case 3:
            omega_p = 1.6;
            p = 0.8;
            if (considerCorrelation) {
                GenerateSurfaceCanonicalCorr(m_unevenness, vehicleTrackWidth, omega_p, p, m_waviness, a);
            } else {
                GenerateSurfaceCanonical(unevenness, waviness);
            }
            break;
        // class E
        case 4:
            omega_p = 0.8;
            p = 0.5;
            if (considerCorrelation) {
                GenerateSurfaceCanonicalCorr(m_unevenness, vehicleTrackWidth, omega_p, p, m_waviness, a);
            } else {
                GenerateSurfaceCanonical(unevenness, waviness);
            }
            break;
        // class F
        case 5:
            GenerateSurfaceCanonical(unevenness, waviness);
            break;
        // class G
        case 6:
            GenerateSurfaceCanonical(unevenness, waviness);
            break;
        // class H
        case 7:
            GenerateSurfaceCanonical(unevenness, waviness);
            break;
    }

    SetupVisualization(vType);
    if (m_collision_mesh)
        SetupCollision();
}

}  // end namespace vehicle
}  // end namespace chrono
