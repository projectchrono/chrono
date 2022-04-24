// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// A driver model that combines a path steering controller and a speed controller.
// The controller adjusts the steering input to follow the prescribed path.
// The output also adjusts throttle and braking inputs in order to maintain a
// varying speed that depends on the curvature of the road.
//
// This implementation is based on the following paper:
// BEST, M.C., 2012. A simple realistic driver model. Presented at:
// AVEC `12: The 11th International Symposium on Advanced Vehicle Control,
// 9th-12th September 2012, Seoul, Korea.
//
// The path to be followed is specified as a ChBezierCurve object and the the
// original definition points are extracted automatically.
// Open and closed course definitions can be handled.
// The ChBezier is still used for visualization.
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChMathematics.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChHumanDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ChHumanDriver::ChHumanDriver(ChVehicle& vehicle,
                             std::shared_ptr<ChBezierCurve> path,
                             const std::string& path_name,
                             bool isClosedPath,
                             double road_width,
                             double max_wheel_turn_angle,
                             double axle_space)
    : ChDriver(vehicle),
      m_path(path),
      m_pathName(path_name),
      m_isClosedPath(isClosedPath),
      m_road_width(road_width),
      m_Tp(0.5),
      m_Klat(0.1),
      m_Kug(0),
      m_Klong(0.1),
      m_Kplus(0.1),
      m_Kminus(0.1),
      m_u0(10.0),
      m_umax(30.0),
      m_uthres(2.0),
      m_target(0, 0, 0),
      m_sentinel(0, 0, 0),
      m_idx_curr(0),
      m_i_curr(0),
      m_j_curr(0),
      m_delta(0.0),
      m_delta_max(max_wheel_turn_angle),
      m_L(axle_space),
      m_run_once(true),
      m_distance(0.0),
      m_travel_time(0.0),
      m_speed_max(1.0e-99),
      m_speed_min(1.0e99),
      m_left_acc(0),
      m_right_acc(0) {
    Create();
}

ChHumanDriver::ChHumanDriver(const std::string& filename,
                             ChVehicle& vehicle,
                             std::shared_ptr<ChBezierCurve> path,
                             const std::string& path_name,
                             bool isClosedPath,
                             double road_width,
                             double max_wheel_turn_angle,
                             double axle_space)
    : ChDriver(vehicle),
      m_path(path),
      m_pathName(path_name),
      m_isClosedPath(isClosedPath),
      m_road_width(road_width),
      m_Tp(0.5),
      m_Klat(0.1),
      m_Kug(0),
      m_Klong(0.1),
      m_Kplus(0.1),
      m_Kminus(0.1),
      m_u0(10.0),
      m_umax(30.0),
      m_uthres(2.0),
      m_target(0, 0, 0),
      m_sentinel(0, 0, 0),
      m_idx_curr(0),
      m_i_curr(0),
      m_j_curr(0),
      m_delta(0.0),
      m_delta_max(max_wheel_turn_angle),
      m_L(axle_space),
      m_run_once(true),
      m_distance(0.0),
      m_travel_time(0.0),
      m_speed_max(1.0e-99),
      m_speed_min(1.0e99),
      m_left_acc(0),
      m_right_acc(0) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    if (d.HasMember("Preview Time")) {
        m_Tp = d["Preview Time"].GetDouble();
        GetLog() << "Preview Time read from JSON file. Tp = " << m_Tp << " secs\n";
    } else {
        GetLog() << "Caution: I am using the default value for Preview Time Tp = " << m_Tp << " secs\n";
    }

    if (d.HasMember("Speed Range")) {
        if (d["Speed Range"].HasMember("MinSpeed")) {
            m_u0 = d["Speed Range"]["MinSpeed"].GetDouble();
            GetLog() << "Minimal speed U0 read from JSON file. U0 = " << m_u0 << " m/s\n";
        } else {
            GetLog() << "Caution: I am using the default value for U0 = " << m_u0 << " m/s\n";
        }
        if (d["Speed Range"].HasMember("MaxSpeed")) {
            m_umax = d["Speed Range"]["MaxSpeed"].GetDouble();
            GetLog() << "Minimal speed Umax read from JSON file. Umax = " << m_umax << " m/s\n";
        } else {
            GetLog() << "Caution: I am using the default value for Umax = " << m_umax << " m/s\n";
        }
    } else {
        GetLog() << "Caution: I am using the default values for U0 = " << m_u0 << " m/s  and  Umax = " << m_umax
                 << " m/s\n";
    }

    if (d.HasMember("Lateral Gains")) {
        if (d["Lateral Gains"].HasMember("Klat")) {
            m_Klat = d["Lateral Gains"]["Klat"].GetDouble();
            GetLog() << "Lateral gain Klat read from JSON file. Klat = " << m_Klat << "\n";
        } else {
            GetLog() << "Caution: I am using the default value for Klat = " << m_Klat << "\n";
        }
        if (d["Lateral Gains"].HasMember("Kug")) {
            m_Kug = d["Lateral Gains"]["Kug"].GetDouble();
            GetLog() << "Lateral gain Kug read from JSON file. Kug = " << m_Kug << " deg/g\n";
        } else {
            GetLog() << "Caution: I am using the default value for Kug = " << m_Kug << " deg/g\n";
        }
    } else {
        GetLog() << "Caution: I am using the default values for Klat= " << m_Klat << "  and  Kug = " << m_Kug
                 << " deg/g\n";
    }

    if (d.HasMember("Longitudinal Gains")) {
        if (d["Longitudinal Gains"].HasMember("Klong")) {
            m_Klong = d["Longitudinal Gains"]["Klong"].GetDouble();
            GetLog() << "Longitudinal gain Klong read from JSON file. Klong = " << m_Klong << " m/s\n";
        } else {
            GetLog() << "Caution: I am using the default value for Klong = " << m_Klong << " \n";
        }
        if (d["Longitudinal Gains"].HasMember("Kplus")) {
            m_Kplus = d["Longitudinal Gains"]["Kplus"].GetDouble();
            GetLog() << "Longitudinal gain Kplus read from JSON file. Kplus = " << m_Kplus << " m/s\n";
        } else {
            GetLog() << "Caution: I am using the default value for Kplus = " << m_Kplus << " \n";
        }
        if (d["Longitudinal Gains"].HasMember("Kminus")) {
            m_Kminus = d["Longitudinal Gains"]["Kminus"].GetDouble();
            GetLog() << "Longitudinal gain Kminus read from JSON file. Kminus = " << m_Kminus << " m/s\n";
        } else {
            GetLog() << "Caution: I am using the default value for Kminus = " << m_Kminus << " \n";
        }
    } else {
        GetLog() << "Caution: I am using the default values for Klong= " << m_Klong << "  ,  Kplus = " << m_Kplus
                 << "  ,  Kminus = " << m_Kminus << " \n";
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    Create();
}

void ChHumanDriver::Create() {
    // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);
    auto num_points = static_cast<unsigned int>(m_path->getNumPoints());
    auto path_asset = chrono_types::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(m_path));
    path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    road->AddAsset(path_asset);
}

void ChHumanDriver::Advance(double step) {  // distance in front of the vehicle.
    const double g = 9.81;

    if (m_run_once) {
        m_UIntegrator.Config(step);
        m_acc_filter.Config(4, step, 2.0);
        m_run_once = false;
    }

    auto& chassis_frame = m_vehicle.GetChassisBody()->GetFrame_REF_to_abs();  // chassis ref-to-world frame
    auto& chassis_rot = chassis_frame.GetRot();                               // chassis ref-to-world rotation
    double u = m_vehicle.GetSpeed();                                          // vehicle speed

    m_distance = m_UIntegrator.Filter(u);
    m_travel_time += step;
    if (u > m_speed_max) {
        m_speed_max = u;
    }
    if (u < m_speed_min) {
        m_speed_min = u;
    }

    double acc = m_acc_filter.Filter(m_vehicle.GetPointAcceleration(ChVector<>(0, 0, 0)).y());
    if (acc > m_left_acc) {
        m_left_acc = acc;
    }
    if (acc < m_right_acc) {
        m_right_acc = acc;
    }

    // Calculate unit vector pointing to the yaw center
    ChVector<> n_g = chassis_rot.GetYaxis();                // vehicle left direction (ISO frame)
    ChWorldFrame::Project(n_g);                             // projected onto horizontal plane (world frame)
    n_g.Normalize();                                        // normalized

    // Calculate unit vector in the vehicle forward direction
    ChVector<> t_g = chassis_rot.GetXaxis();                // vehicle forward direction (ISO frame)
    ChWorldFrame::Project(t_g);                             // projected onto horizontal plane (world frame)
    t_g.Normalize();                                        // normalized

    double R = 0;
    double ut = u > m_uthres ? u : m_uthres;
    double factor = ut * m_Tp;
    if (m_delta == 0.0) {
        m_sentinel = chassis_frame.TransformPointLocalToParent(factor * ChWorldFrame::Forward());
    } else {
        // m_Kug is in [°/g]
        R = (m_L + CH_C_DEG_TO_RAD * m_Kug * ut * ut / g) / m_delta;
        double theta = ut * m_Tp / R;
        ChMatrix33<> RM(theta, ChWorldFrame::Vertical());
        m_sentinel = chassis_frame.TransformPointLocalToParent(factor * ChWorldFrame::Forward()) + R * (n_g - RM * n_g);
    }

    ChVector<> Pt = m_sentinel - m_S_l[m_idx_curr];
    double rt = m_R_l[m_idx_curr].Length();

    double t = std::abs(Pt.Dot(m_R_lu[m_idx_curr]));
    if (t >= rt) {
        while (t > rt) {
            m_idx_curr++;
            if (m_isClosedPath) {
                if (m_idx_curr == m_S_l.size()) {
                    m_idx_curr = 0;
                }
                Pt = m_sentinel - m_S_l[m_idx_curr];
                rt = m_R_l[m_idx_curr].Length();
                t = std::abs(Pt.Dot(m_R_lu[m_idx_curr]));
            } else {
                if (m_idx_curr == m_S_l.size()) {
                    m_idx_curr = m_S_l.size() - 1;
                }
                Pt = m_sentinel - m_S_l[m_idx_curr];
                rt = m_R_l[m_idx_curr].Length();
                t = std::abs(Pt.Dot(m_R_lu[m_idx_curr]));
                break;
            }
        }
    }

    m_target = m_S_l[m_idx_curr] + t * m_R_lu[m_idx_curr];

    m_i_curr = m_idx_curr;
    m_j_curr = m_idx_curr;

    ChVector<> n_lu = m_R_lu[m_idx_curr] % ChWorldFrame::Vertical();  // cross product

    double err = Pt.Dot(n_lu);

    if (u > m_uthres) {
        m_delta = ChClamp(m_delta + m_Klat * err, -m_delta_max, m_delta_max);
        // set steering value
    }
    m_steering = m_delta / m_delta_max;

    // define field of view angle of the driver +-10 deg
    const double ny = 10.0 * CH_C_DEG_TO_RAD;
    // which speed to choose?
    // search intervalls for longitudinal controller
    double d_long = 0;
    bool i_restrict = false;
    bool j_restrict = false;
    ChVector<> Li_G;
    ChVector<> Lip_G;
    ChVector<> Rj_G;
    ChVector<> Rjp_G;
    ChVector<> r_rv = t_g + tan(ny) * n_g;
    ChVector<> r_lv = t_g - tan(ny) * n_g;
    while (!i_restrict && !j_restrict) {
        size_t i_next = GetNextI();
        size_t j_next = GetNextJ();
        Li_G = m_Li[m_i_curr] - m_sentinel;
        Lip_G = m_Li[i_next] - m_sentinel;
        Rj_G = m_Rj[m_j_curr] - m_sentinel;
        Rjp_G = m_Rj[j_next] - m_sentinel;

        bool C1 = ChWorldFrame::Height(Li_G.Cross(Lip_G)) > 0.0;
        bool C2 = ChWorldFrame::Height(Rj_G.Cross(Rjp_G)) < 0.0;
        bool C3 = ChWorldFrame::Height(Lip_G.Cross(Rjp_G)) > 0.0;
        bool C4I = (ChWorldFrame::Height(Li_G.Cross(r_rv)) > 0.0) && (ChWorldFrame::Height(Lip_G.Cross(r_rv)) <= 0.0);
        bool C4J = (ChWorldFrame::Height(Rj_G.Cross(r_lv)) < 0.0) && (ChWorldFrame::Height(Rjp_G.Cross(r_lv)) >= 0.0);

        d_long = std::max(Li_G.Length(), Rj_G.Length());
        if ((C1 || C2) && C3) {
            if (C1 && !C4I && !i_restrict) {
                m_i_curr = i_next;
            } else {
                d_long = Li_G.Length();
                i_restrict = true;
            }
            if (C2 && !C4J && !j_restrict) {
                m_j_curr = j_next;
            } else {
                d_long = Rj_G.Length();
                j_restrict = true;
            }
        } else {
            break;
        }
    }
    double udem = std::min(m_Klong * d_long + m_u0, m_umax);

    double tau = 0;
    if (udem < u) {
        // u is too high, brake!
        tau = m_Kminus * (udem - u);
    } else {
        // u is too low, accelerate!
        tau = m_Kplus * (udem - u);
    }
    if (tau >= 0.0) {
        m_throttle = ChClamp(std::abs(tau), 0.0, 1.0);
        m_braking = 0.0;
    } else {
        m_braking = ChClamp(std::abs(tau), 0.0, 1.0);
        m_throttle = 0.0;
    }
}

void ChHumanDriver::Initialize() {
    // Information about the drive course and the road borders
    size_t np = m_path->getNumPoints();
    m_S_l.resize(np);
    m_R_l.resize(np);
    m_R_lu.resize(np);

    m_Li.resize(np);
    m_Rj.resize(np);

    if (m_isClosedPath) {
        for (size_t i = 0; i < np; i++) {
            m_S_l[i] = m_path->getPoint(i);
        }
        for (size_t i = 0; i < np - 1; i++) {
            m_R_l[i] = m_S_l[i + 1] - m_S_l[i];
            m_R_lu[i] = m_R_l[i];
            m_R_lu[i].Normalize();
            m_Li[i] = m_S_l[i] + 0.5 * m_road_width * m_R_lu[i].Cross(ChWorldFrame::Vertical());
            m_Rj[i] = m_S_l[i] - 0.5 * m_road_width * m_R_lu[i].Cross(ChWorldFrame::Vertical());
        }
        // connect the last point to the first point
        m_R_l[np - 1] = m_S_l[0] - m_S_l[np - 1];
        m_R_lu[np - 1] = m_R_l[np - 1];
        m_R_lu[np - 1].Normalize();
        m_Li[np - 1] = m_S_l[np - 1] + 0.5 * m_road_width * m_R_lu[np - 1].Cross(ChWorldFrame::Vertical());
        m_Rj[np - 1] = m_S_l[np - 1] - 0.5 * m_road_width * m_R_lu[np - 1].Cross(ChWorldFrame::Vertical());
    } else {
        for (size_t i = 0; i < np; i++) {
            m_S_l[i] = m_path->getPoint(i);
        }
        for (size_t i = 0; i < np - 1; i++) {
            m_R_l[i] = m_S_l[i + 1] - m_S_l[i];
            m_R_lu[i] = m_R_l[i];
            m_R_lu[i].Normalize();
            m_Li[i] = m_S_l[i] - 0.5 * m_road_width * m_R_lu[i].Cross(ChWorldFrame::Vertical());
            m_Rj[i] = m_S_l[i] + 0.5 * m_road_width * m_R_lu[i].Cross(ChWorldFrame::Vertical());
        }
        m_R_l[np - 1] = m_S_l[np - 1] - m_S_l[np - 2];
        m_R_lu[np - 1] = m_R_l[np - 1];
        m_R_lu[np - 1].Normalize();
        m_Li[np - 1] = m_S_l[np - 1] - 0.5 * m_road_width * m_R_lu[np - 1].Cross(ChWorldFrame::Vertical());
        m_Rj[np - 1] = m_S_l[np - 1] + 0.5 * m_road_width * m_R_lu[np - 1].Cross(ChWorldFrame::Vertical());
    }
}

void ChHumanDriver::SetSpeedRange(double u0, double umax) {
    m_u0 = std::abs(u0);
    m_umax = std::abs(umax);
    if (m_u0 <= 5 || m_umax <= m_u0) {
        m_u0 = 10;
        m_umax = 30.0;
    }
}

size_t ChHumanDriver::GetNextI() {
    // what is next possible index I?
    if (m_isClosedPath) {
        if (m_i_curr < m_Li.size() - 1) {
            // just the next one
            return m_i_curr + 1;
        } else {
            // next point is the starting point
            return 0;
        }
    } else {
        if (m_i_curr < m_Li.size() - 2) {
            // just the next one
            return m_i_curr + 1;
        } else {
            // next point is the last point
            return m_Li.size() - 1;
        }
    }
}

size_t ChHumanDriver::GetNextJ() {
    if (m_isClosedPath) {
        if (m_j_curr < m_Rj.size() - 1) {
            return m_j_curr + 1;
        } else
            return 0;
    } else {
        if (m_j_curr < m_Rj.size() - 2) {
            return m_j_curr + 1;
        } else {
            return m_Rj.size() - 1;
        }
    }
}

void ChHumanDriver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_path, m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

}  // namespace vehicle
}  // namespace chrono
