// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2017 projectchrono.org
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
// Template for the "Tire Model made Easy"
//
// Ref: Georg Rill, "Road Vehicle Dynamics - Fundamentals and Modelling",
//          @2012 CRC Press, ISBN 978-1-4398-3898-3
//      Georg Rill, "An Engineer's Guess On Tyre Model Parameter Made Possible With TMeasy",
//          https://hps.hs-regensburg.de/rig39165/Rill_Tyre_Coll_2015.pdf
//
// This implementation does not include transient slip state modifications.
// No parking slip calculations.
//
// Changes:
// 2017-12-21 - There is a simple form of contact smoothing now. It works on flat
//			    terrain as well.
//			  - The parameter estimation routines have changed, know you have the
//				option to use either the load index or the load force as input.
//
// =============================================================================

#ifndef CH_TMEASYTIRE
#define CH_TMEASYTIRE

#include <vector>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// TMeasy tire model.
class CH_VEHICLE_API ChTMeasyTire : public ChTire {
  public:
    ChTMeasyTire(const std::string& name  ///< [in] name of this tire system
    );

    virtual ~ChTMeasyTire() {}

    /// Initialize this tire system.
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< [in] associated wheel body
                            VehicleSide side                ///< [in] left/right vehicle side
                            ) override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TerrainForce GetTireForce() const override { return m_tireforce; }

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_tireforce; }

    /// Update the state of this tire system at the current time.
    /// The tire system is provided the current state of its associated wheel.
    virtual void Synchronize(double time,                    ///< [in] current time
                             const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                             const ChTerrain& terrain        ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    /// Set the value of the integration step size for the underlying dynamics.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the current value of the integration step size.
    double GetStepsize() const { return m_stepsize; }

    /// Get the width of the tire.
    double GetWidth() const { return m_width; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return m_width; }

    /// Get the tire slip angle.
    virtual double GetSlipAngle() const override { return m_states.cp_side_slip; }

    /// Get the tire longitudinal slip.
    virtual double GetLongitudinalSlip() const override { return m_states.cp_long_slip; }

    /// Get the longitudinal slip used in the TMeasy model (expressed as a percentage).
    double GetKappa() const { return m_kappa; }

    /// Get the slip angle used in Pac89 (expressed in degrees).
    double GetAlpha() const { return m_alpha; }

    /// Get the camber angle used in the TMeasy model (expressed in degrees).
    double GetGamma() { return m_gamma; }

    /// Get Max. Tire Load from Load Index (LI) in N [0:279]
    static double GetTireMaxLoad(unsigned int li);

    /// Guess Tire Parameters from characteristic truck tire parameter pattern (Ratio = 80%)
    void GuessTruck80Par(unsigned int li,        ///< tire load index
                         double tireWidth,       ///< tire width [m]
                         double ratio,           ///< use 0.75 meaning 75%
                         double rimDia,          ///< rim diameter [m]
                         double pinfl_li = 1.0,  ///< inflation pressure at load index
                         double pinfl_use = 1.0  ///< inflation pressure in this configuration
    );

    void GuessTruck80Par(double loadForce,       ///< tire nominal load force [N]
                         double tireWidth,       ///< tire width [m]
                         double ratio,           ///< use 0.75 meaning 75%
                         double rimDia,          ///< rim diameter [m]
                         double pinfl_li = 1.0,  ///< inflation pressure at load index
                         double pinfl_use = 1.0  ///< inflation pressure in this configuration
    );

    /// Guess Tire Parameters from characteristic passenger car tire parameter pattern (Ratio = 70%)
    void GuessPassCar70Par(unsigned int li,        ///< tire load index
                           double tireWidth,       ///< tire width [m]
                           double ratio,           ///< use 0.75 meaning 75%
                           double rimDia,          ///< rim diameter [m]
                           double pinfl_li = 1.0,  ///< inflation pressure at load index
                           double pinfl_use = 1.0  ///< inflation pressure in this configuration
    );
    void GuessPassCar70Par(double loadForce,       ///< tire nominal load force [N]
                           double tireWidth,       ///< tire width [m]
                           double ratio,           ///< use 0.75 meaning 75%
                           double rimDia,          ///< rim diameter [m]
                           double pinfl_li = 1.0,  ///< inflation pressure at load index
                           double pinfl_use = 1.0  ///< inflation pressure in this configuration
    );

	// Set vertical tire stiffness as linear function by coefficient [N/m]
	void SetVerticalStiffness(double Cz) { SetVerticalStiffness(Cz,Cz); };	
	// Set vertical tire stiffness as nonlinear function by coefficients at nominal load 1 [N/m]
	// and nominal load 2 [N/m]
	void SetVerticalStiffness(double Cz1, double Cz2);
	// Set vertical tire stiffness as nonlinear function by calculation from tire test data (least squares)
    void SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc);

    /// Generate basic tire plots.
    /// This function creates a Gnuplot script file with the specified name.
    void WritePlots(const std::string& plFileName, const std::string& plTireFormat);

	/// Get the tire deflection
	virtual double GetDeflection() const override { return m_data.depth; }
	
	/// Simple parameter consistency test
	bool CheckParameters();
	
  protected:
    /// Perform disc-terrain collision detection considering the curvature of the road
    /// surface. The surface normal is calculated based on 4 different height values below
    /// the wheel center. The effective height is calculated as average value of the four
    /// height values.
    /// This utility function checks for contact between a disc of specified
    /// radius with given position and orientation (specified as the location of
    /// its center and a unit vector normal to the disc plane) and the terrain
    /// system associated with this tire. It returns true if the disc contacts the
    /// terrain and false otherwise.  If contact occurs, it returns a coordinate
    /// system with the Z axis along the contact normal and the X axis along the
    /// "rolling" direction, as well as a positive penetration depth (i.e. the
    /// height below the terrain of the lowest point on the disc).
    bool disc_terrain_contact_3d(
        const ChTerrain& terrain,       ///< [in] reference to terrain system
        const ChVector<>& disc_center,  ///< [in] global location of the disc center
        const ChVector<>& disc_normal,  ///< [in] disc normal, expressed in the global frame
        double disc_radius,             ///< [in] disc radius
        ChCoordsys<>& contact,          ///< [out] contact coordinate system (relative to the global frame)
        double& depth                   ///< [out] penetration depth (positive if contact occurred)
    );

    /// Return the vertical tire stiffness contribution to the normal force.
    double GetNormalStiffnessForce(double depth);

    /// Return the vertical tire damping contribution to the normal force.
    double GetNormalDampingForce(double depth, double velocity);

    /// Set the parameters in the TMeasy model.
    virtual void SetTMeasyParams() = 0;

    double m_kappa;  ///< longitudinal slip (percentage)
    double m_alpha;  ///< slip angle (degrees)
    double m_gamma;  ///< camber angle (degrees)

    double m_gamma_limit;  ///< limit camber angle (degrees)

    /// TMeasy tire model parameters
    double m_unloaded_radius;
    double m_width;
    double m_rolling_resistance;  // double m_lateral_stiffness (actually unused)
    double m_mu;                  // local friction coefficient of the road

	double m_a1;				  // polynomial coefficient a1 in cz = a1 + 2.0*a2 * deflection
	double m_a2;				  // polynomial coefficient a2 in cz = a1 + 2.0*a2 * deflection
	
    VehicleSide m_measured_side;

    typedef struct {
        double pn;

        double mu_0;     // local friction coefficient of the road for given parameters
        double cz;       // stiffness, may vary with the vertical force
        double dz;       // linear damping coefficient

        double dfx0_pn, dfx0_p2n;
        double fxm_pn, fxm_p2n;
        double sxm_pn, sxm_p2n;
        double fxs_pn, fxs_p2n;
        double sxs_pn, sxs_p2n;

        double dfy0_pn, dfy0_p2n;
        double fym_pn, fym_p2n;
        double sym_pn, sym_p2n;
        double fys_pn, fys_p2n;
        double sys_pn, sys_p2n;

        double nto0_pn, nto0_p2n;
        double synto0_pn, synto0_p2n;
        double syntoE_pn, syntoE_p2n;
    } TMeasyCoeff;

    TMeasyCoeff m_TMeasyCoeff;

    // linear Interpolation
    double InterpL(double fz, double w1, double w2) { return w1 + (w2 - w1) * (fz / m_TMeasyCoeff.pn - 1.0); };
    // quadratic Interpolation
    double InterpQ(double fz, double w1, double w2) {
        return (fz / m_TMeasyCoeff.pn) * (2.0 * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (fz / m_TMeasyCoeff.pn));
    };
    double RampSignum(double inval);

  private:
    double m_stepsize;

	void UpdateVerticalStiffness();
	
	std::vector<double> 	m_tire_test_defl;	// set, when test data are used for vertical 			
	std::vector<double> 	m_tire_test_frc;	// stiffness calculation
	
    void tmxy_combined(double& f, double& fos, double s, double df0, double sm, double fm, double ss, double fs);
    double tmy_tireoff(double sy, double nto0, double synto0, double syntoE);

    struct ContactData {
        bool in_contact;      // true if disc in contact with terrain
        ChCoordsys<> frame;   // contact frame (x: long, y: lat, z: normal)
        ChVector<> vel;       // relative velocity expressed in contact frame
        double normal_force;  // magnitude of normal contact force
        double depth;         // penetration depth
    };

    struct TireStates {
        double cp_long_slip;     // Contact Path - Longitudinal Slip State (Kappa)
        double cp_side_slip;     // Contact Path - Side Slip State (Alpha)
        double vx;               // Longitudinal speed
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis
        double R_eff;            // Effective Radius
        ChVector<> disc_normal;  //(temporary for debug)
    };

    ContactData m_data;
    TireStates m_states;

    TerrainForce m_tireforce;

    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
    std::shared_ptr<ChTexture> m_texture;          ///< visualization texture asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
