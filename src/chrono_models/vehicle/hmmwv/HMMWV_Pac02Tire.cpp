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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// HMMWV PAC02 tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_Pac02Tire::m_mass = 37.6;
const ChVector<> HMMWV_Pac02Tire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_Pac02Tire::m_meshName = "hmmwv_tire_POV_geom";
const std::string HMMWV_Pac02Tire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Pac02Tire::HMMWV_Pac02Tire(const std::string& name) : ChPac02Tire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Pac02Tire::SetPac02Params() {
    m_measured_side = LEFT;
    m_use_mode = 4;
    // Parameter Set Converted from an Adams Tire file, gained by scaling
    m_PacScal.lfz0 = 0.59;
    m_PacScal.ltr = 0.8;

    m_PacCoeff.R0 = 0.46482;
    m_PacCoeff.width = 0.3175;
    m_PacCoeff.FzNomin = 35000;
    m_PacCoeff.Cz = 420810;
    m_PacCoeff.Kz = 398;

    // longitudinal parameters
    m_PacCoeff.pcx1 = 1.7204;        // Shape factor Cfx for longitudinal force
    m_PacCoeff.pdx1 = 0.77751;       // Longitudinal friction Mux at Fznom
    m_PacCoeff.pdx2 = -0.24431;      // Variation of friction Mux with load
    m_PacCoeff.pdx3 = -0.00015908;   // Variation of friction Mux with camber
    m_PacCoeff.pex1 = 0.46659;       // Longitudinal curvature Efx at Fznom
    m_PacCoeff.pex2 = 0.393;         // Variation of curvature Efx with load
    m_PacCoeff.pex3 = 0.076024;      // Variation of curvature Efx with load squared
    m_PacCoeff.pex4 = 2.6509e-006;   // Factor in curvature Efx while driving
    m_PacCoeff.pkx1 = 14.848;        // Longitudinal slip stiffness Kfx/Fz at Fznom
    m_PacCoeff.pkx2 = -9.8161;       // Variation of slip stiffness Kfx/Fz with load
    m_PacCoeff.pkx3 = 0.15818;       // Exponent in slip stiffness Kfx/Fz with load
    m_PacCoeff.phx1 = -0.00088873;   // Horizontal shift Shx at Fznom
    m_PacCoeff.phx2 = -0.00067818;   // Variation of shift Shx with load
    m_PacCoeff.pvx1 = -5.5714e-007;  // Vertical shift Svx/Fz at Fznom
    m_PacCoeff.pvx2 = 6.2972e-006;   // Variation of shift Svx/Fz with load
    m_PacCoeff.rbx1 = 11.13;         // Slope factor for combined slip Fx reduction
    m_PacCoeff.rbx2 = -12.494;       // Variation of slope Fx reduction with kappa
    m_PacCoeff.rbx1 = 0.97505;       // Shape factor for combined slip Fx reduction
    m_PacCoeff.rex1 = -0.37196;      // Curvature factor of combined Fx
    m_PacCoeff.rex2 = 0.0017379;     // Curvature factor of combined Fx with load
    m_PacCoeff.rhx1 = 0.0045181;     // Shift factor for combined slip Fx reduction
    m_PacCoeff.ptx1 = 1.5;           // Relaxation length SigKap0/Fz at Fznom
    m_PacCoeff.ptx2 = 1.4;           // Variation of SigKap0/Fz with load
    m_PacCoeff.ptx3 = 1;             // Variation of SigKap0/Fz with exponent of load
    m_PacCoeff.ptx4 = 0.1;           // Low speed damping

    // lateral coefficients
    m_PacCoeff.pcy1 = 1.5874;        // Shape factor Cfy for lateral forces
    m_PacCoeff.pdy1 = 0.73957;       // Lateral friction Muy
    m_PacCoeff.pdy2 = -0.075004;     // Variation of friction Muy with load
    m_PacCoeff.pdy3 = -8.0362;       // Variation of friction Muy with squared camber
    m_PacCoeff.pey1 = 0.37562;       // Lateral curvature Efy at Fznom
    m_PacCoeff.pey2 = -0.069325;     // Variation of curvature Efy with load
    m_PacCoeff.pey3 = 0.29168;       // Zero order camber dependency of curvature Efy
    m_PacCoeff.pey4 = 11.559;        // Variation of curvature Efy with camber
    m_PacCoeff.pky1 = -10.289;       // Maximum value of stiffness Kfy/Fznom
    m_PacCoeff.pky2 = 3.3343;        // Load at which Kfy reaches maximum value
    m_PacCoeff.pky3 = -0.25732;      // Variation of Kfy/Fznom with camber
    m_PacCoeff.phy1 = 0.0056509;     // Horizontal shift Shy at Fznom
    m_PacCoeff.phy2 = -0.0020257;    // Variation of shift Shy with load
    m_PacCoeff.phy3 = -0.038716;     // Variation of shift Shy with camber
    m_PacCoeff.pvy1 = 0.015216;      // Vertical shift in Svy/Fz at Fznom
    m_PacCoeff.pvy2 = -0.010365;     // Variation of shift Svy/Fz with load
    m_PacCoeff.pvy3 = -0.31373;      // Variation of shift Svy/Fz with camber
    m_PacCoeff.pvy4 = -0.055766;     // Variation of shift Svy/Fz with camber and load
    m_PacCoeff.rby1 = 13.271;        // Slope factor for combined Fy reduction
    m_PacCoeff.rby2 = 5.2405;        // Variation of slope Fy reduction with alpha
    m_PacCoeff.rby3 = 1.1547e-005;   // Shift term for alpha in slope Fy reduction
    m_PacCoeff.rcy1 = 1.01;          // Shape factor for combined Fy reduction
    m_PacCoeff.rey1 = 0.010513;      // Curvature factor of combined Fy
    m_PacCoeff.rey2 = 5.9816e-005;   // Curvature factor of combined Fy with load
    m_PacCoeff.rhy1 = 0.028005;      // Shift factor for combined Fy reduction
    m_PacCoeff.rhy2 = -4.8794e-005;  // Shift factor for combined Fy reduction with load
    m_PacCoeff.rvy1 = 0.0066878;     // Kappa induced side force Svyk/Muy*Fz at Fznom
    m_PacCoeff.rvy2 = -0.042813;     // Variation of Svyk/Muy*Fz with load
    m_PacCoeff.rvy3 = -0.16227;      // Variation of Svyk/Muy*Fz with camber
    m_PacCoeff.rvy4 = -0.019796;     // Variation of Svyk/Muy*Fz with alpha
    m_PacCoeff.rvy5 = 1.9;           // Variation of Svyk/Muy*Fz with kappa
    m_PacCoeff.rvy6 = -7.8097;       // Variation of Svyk/Muy*Fz with atan(kappa)
    m_PacCoeff.pty1 = 1.2;           // Peak value of relaxation length SigAlp0/R0
    m_PacCoeff.pty2 = 2.5;           // Value of Fz/Fznom where SigAlp0 is extreme

    // overturning coefficients
    m_PacCoeff.qsx1 = 0;  // Lateral force induced overturning moment
    m_PacCoeff.qsx2 = 0;  // Camber induced overturning couple
    m_PacCoeff.qsx3 = 0;  // Fy induced overturning couple

    // rolling resistance
    m_PacCoeff.qsy1 = 0.015;

    // aligning coefficients
    m_PacCoeff.qbz1 = 5.8978;      // Trail slope factor for trail Bpt at Fznom
    m_PacCoeff.qbz2 = -0.1535;     // Variation of slope Bpt with load
    m_PacCoeff.qbz3 = -2.0052;     // Variation of slope Bpt with load squared
    m_PacCoeff.qbz4 = 0.62731;     // Variation of slope Bpt with camber
    m_PacCoeff.qbz5 = -0.92709;    // Variation of slope Bpt with absolute camber
    m_PacCoeff.qbz9 = 10.637;      // Slope factor Br of residual torque Mzr
    m_PacCoeff.qbz10 = 0;          // Slope factor Br of residual torque Mzr
    m_PacCoeff.qcz1 = 1.4982;      // Shape factor Cpt for pneumatic trail
    m_PacCoeff.qdz1 = 0.085549;    // Peak trail Dpt" = Dpt*(Fz/Fznom*R0)
    m_PacCoeff.qdz2 = -0.025298;   // Variation of peak Dpt" with load
    m_PacCoeff.qdz3 = 0.21486;     // Variation of peak Dpt" with camber
    m_PacCoeff.qdz4 = -3.9098;     // Variation of peak Dpt" with camber squared
    m_PacCoeff.qdz6 = -0.0013373;  // Peak residual torque Dmr" = Dmr/(Fz*R0)
    m_PacCoeff.qdz7 = 0.0013869;   // Variation of peak factor Dmr" with load
    m_PacCoeff.qdz8 = -0.053513;   // Variation of peak factor Dmr" with camber
    m_PacCoeff.qdz9 = 0.025817;    // Variation of peak factor Dmr" with camber and load
    m_PacCoeff.qez1 = -0.0084519;  // Trail curvature Ept at Fznom
    m_PacCoeff.qez2 = 0.0097389;   // Variation of curvature Ept with load
    m_PacCoeff.qez3 = 0;           // Variation of curvature Ept with load squared
    m_PacCoeff.qez4 = 4.3583;      // Variation of curvature Ept with sign of Alpha-t
    m_PacCoeff.qez5 = -645.04;     // Variation of Ept with camber and sign Alpha-t
    m_PacCoeff.qhz1 = 0.0085657;   // Trail horizontal shift Sht at Fznom
    m_PacCoeff.qhz2 = -0.0042922;  // Variation of shift Sht with load
    m_PacCoeff.qhz3 = 0.14763;     // Variation of shift Sht with camber
    m_PacCoeff.qhz4 = -0.29999;    // Variation of shift Sht with camber and load
    m_PacCoeff.ssz1 = -0.019408;   // Nominal value of s/R0: effect of Fx on Mz
    m_PacCoeff.ssz2 = 0.025786;    // Variation of distance s/R0 with Fy/Fznom
    m_PacCoeff.ssz3 = 0.31908;     // Variation of distance s/R0 with camber
    m_PacCoeff.ssz4 = -0.50765;    // Variation of distance s/R0 with load and camber
    m_PacCoeff.qtz1 = 0;           // Gyration torque constant
    m_PacCoeff.mbelt = 0;          // Belt mass of the wheel

    // load the vertical stiffness table
    m_vert_map.AddPoint(0.00, 0);
    m_vert_map.AddPoint(0.01, 2830.0);
    m_vert_map.AddPoint(0.02, 6212.0);
    m_vert_map.AddPoint(0.03, 10146.0);
    m_vert_map.AddPoint(0.04, 14632.0);
    m_vert_map.AddPoint(0.05, 19670.0);
    m_vert_map.AddPoint(0.06, 25260.0);
    m_vert_map.AddPoint(0.07, 31402.0);
    m_vert_map.AddPoint(0.08, 38096.0);
    m_vert_map.AddPoint(0.09, 45342.0);
    m_vert_map.AddPoint(0.10, 53140.0);
}

double HMMWV_Pac02Tire::GetNormalStiffnessForce(double depth) const {
    return m_vert_map.Get_y(depth);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void HMMWV_Pac02Tire::RemoveVisualizationAssets() {
    ChPac02Tire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_Pac02Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
