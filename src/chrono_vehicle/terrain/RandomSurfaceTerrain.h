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
//
// Simple flat horizontal terrain (infinite x-y extent) with an uneven area.
// The uneven two-track surface is modeled by means of standard PSD spectra as
// defined in the ISO 8608 standard. ISO 8608 defines 8 classes of uneven
// roads all can be selected from preset configurations if the two tracks
// shall be uncorrelated or independend. Tracks like this are used for
// intensive vehicle testing, because they apply a high load to the vehicle
// structure.
//
// If normal road surface conditions are prefered, correlated tracks are the
// better choice. From test we know that the measured cohereny is a function of
//  - vehicle track width
//  - signal wavelength
//  - unevenness
//  - curve parameters omega_p, p and a.
// ISO classes from A to E can be selected with correlation activated.
// For classes F to H no correlated variants can be selected. This due to
// the lack of test results with real road surfaces. As a tendency we can see
// in the measured coherence its vanishing with increasing unevenness of the
// road. That could mean uncorrelated F to H signals are realistic enough.
//
// We use the standard PSD function Phi = Phi_h0*(omega/omega0)^(-w)
// where Phi_h0 is the unevenness and w is the waviness. ISO 8608 uses only w = 2.
// omega is the spatial natural frequency. omega0 is defined to 1 rad/m.
//
// The transformation to spatial cordinates is done by inverse Fourier transform.
// Care has been taken, that the signal does not repeat inside the demanded x
// interval by selecting sufficient spectral coefficients.
//
// The measured coherence function:
//  Xsi = (1.0 + (omega * trackWidth^a / omega_p)^waviness)^(-p)
// Xsi is in the interval [0,1].
//
// To complete the conversion process from PSD to Fourier series, it is necessary
// to generate a set of random phase angles in the range [0,2Pi] for every track.
// We must use a uniform random number for this to avoid 'monster waves'.
// Every track gets is own set of phase angles. If correlated signals are demanded
// the phase angle sets left and right are blended whith the coherence function.
//
// For practical work it is necessary to limit the used wavelengths. The minimal
// useful wavelength is 0.3 m. This is roughly the contact patch length of the tire.
// Shorter wavelengths wouldn't give better results (smoothing of the tire) but
// increase the amount of data.
//
// The maximal wavelength must also be limited. The disadvantage of the PSD
// formula is the ability to increase the amplitude to infinity with decresing
// spatial natural frequency. This can lead to enormous height differences
// between left and right track that would tip the vehicle. For this reason
// we limit the wavelength maximum to 20 m. As a side effect the RMS value this
// class optionally reports is the so called 'Detrended RMS' from the WES method
// for vehicle ride quality (6 Watt method). WES proposed to use 60 ft high pass
// filter before calculating the RMS.
//
// Three different initilizer methods can be used: a preset based one, an
// unevenness/waviness based one, and an IRI based one. The second one allows the
// usage of non-ISO wavinesses. IRI must be given in [mm/m] to generate useful
// results.
//
// References:
//
// Löhe K., Zehelein Th.: "Modellierung und Parametrierung stochastischer Fahrbahnunebenheiten mit Hub-, Wank-, Nick-,
// und Verspannanteil", Conference Paper 2005
//
// Quarz V.: "Die Generierung von Fahrbahnstörungen für vorgegebene Spektraldichen mit Hilfe orthogonaler Funktionen",
// Diss. 2004
//
// ISO 8608: "Mechanical Vibration - Road surface profiles - Reporting of measured data", 2016
//
// Kropac O., Mucka P.: "Estimation of road waviness using the IRI algorithm", Strojnicky Casopis 55, 2004
//
// Mitschke M., Wallentowitz H.: "Dynamik der Kraftfahrzeuge", Springer 2014
//
// ===================================================================================================================

#ifndef RANDOM_SURFACE_TERRAIN_H
#define RANDOM_SURFACE_TERRAIN_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/core/ChBezierCurve.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Terrain object representing an uneven area with controlled roughness.
///
/// By default, this class implements a terrain modeled as an infinite horizontal plane at a specified height with the
/// uneven terrain lane of specified length and width starting at the origin. This type of terrain can be used in
/// conjunction with tire models that perform their own collision detection (e.g., ChTMeasy ChPac89, ChPac02, ChFiala).
///
/// Alternatively, this terrain type can be represented as a collision mesh representing the uneven lane with an
/// optional flat starting lane.  This terrain type can be used with vehicles that require contact for the
/// vehicle-terrain interaction (wheeled vehicle with rigid tires or tracked vehicles).
class CH_VEHICLE_API RandomSurfaceTerrain : public ChTerrain {
  public:
    enum class SurfaceType {
        FLAT,
        ISO8608_A_NOCORR,
        ISO8608_B_NOCORR,
        ISO8608_C_NOCORR,
        ISO8608_D_NOCORR,
        ISO8608_E_NOCORR,
        ISO8608_F_NOCORR,
        ISO8608_G_NOCORR,
        ISO8608_H_NOCORR,
        ISO8608_A_CORR,
        ISO8608_B_CORR,
        ISO8608_C_CORR,
        ISO8608_D_CORR,
        ISO8608_E_CORR,
        MAJOR_ROAD_ASPHALTIC_CONCRETE,
        MAJOR_ROAD_CONCRETE,
        MAIN_ROAD_ASPHALTIC_CONCRETE_ON_PAVEMENT,
        MAIN_ROAD_ASPHALTIC_CONCRETE,
        TRACK_TILED_CONCRETE_PAVEMENT
    };

    enum class VisualisationType { NONE, LINES, MESH };

    RandomSurfaceTerrain(ChSystem* system,       ///< [in] pointer to the containing multibody system
                         double length = 300.0,  ///< [in] uneven terrain length
                         double width = 5.0,     ///< [in] uneven terrain width
                         double height = 0.0,    ///< [in] terrain height offset
                         float friction = 0.8f   ///< [in] terrain coefficient of friction
    );

    ~RandomSurfaceTerrain() {}

    /// Get the terrain height below the specified location.
    /// Returns the constant value passed at construction.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    /// Returns a constant unit vector along the vertical axis.
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For RandomSurfaceTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified at construction.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

    /// Get the (detrended) root mean square of the tracks, height offset is not considered [m]
    double GetRMS() { return m_rms; }

    /// Get the International Roughness Index, estimated from unevenness and waviness [m/km]
    double GetIRI() { return m_iri; }

    /// Enable creation of a collision mesh and enable collision (default: no collision mesh).
    /// Optionally (length > 0), create a flat lane of given length positioned before the uneven portion.
    /// The specified radius (default 0) is used as a "mesh thickness" to improve robustness of the collision detection.
    /// Note that this function must be called before Initialize().
    void EnableCollisionMesh(std::shared_ptr<ChMaterialSurface> material,
                             double length = 0,
                             double sweep_sphere_radius = 0);

    /// Select a road surface from presets, ISO 8608 and literature
    void Initialize(RandomSurfaceTerrain::SurfaceType surfType = RandomSurfaceTerrain::SurfaceType::FLAT,
                    double vehicleTrackWidth = 2.0,
                    RandomSurfaceTerrain::VisualisationType vType = RandomSurfaceTerrain::VisualisationType::MESH);
    /// Directly generate a road surface from unevenness and waviness, optional consideration of
    /// correlation, if unevenness fits between ISO classes A to E
    void Initialize(double unevenness,
                    double waviness = 2.0,
                    double vehicleTrackWidth = 2.0,
                    bool considerCorrelation = true,
                    RandomSurfaceTerrain::VisualisationType vType = RandomSurfaceTerrain::VisualisationType::MESH);
    /// Directly generate a road surface from International Roughness Index [mm/m], optional consideration of
    /// correlation, if unevenness fits between ISO classes A to E
    void Initialize(double iri,
                    double vehicleTrackWidth = 2.0,
                    bool considerCorrelation = true,
                    RandomSurfaceTerrain::VisualisationType vType = RandomSurfaceTerrain::VisualisationType::MESH);

  private:
    double m_unevenness;
    double m_waviness;
    double m_rms;                      ///< (detrended) root mean square of the uneven tracks
    double m_iri;                      ///< International Roughness Index estimated from unevenness and waviness
    std::shared_ptr<ChBody> m_ground;  ///< ground body
    std::shared_ptr<ChBezierCurve> m_road_left;                 ///< curve for left road boundary
    std::shared_ptr<ChBezierCurve> m_road_right;                ///< curve for right road boundary
    std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;  ///< mesh for visualization/export

    std::string m_curve_left_name;
    std::string m_curve_right_name;

    double m_length;          ///< length of the uneven surface
    double m_width;           ///< width of the uneven surface
    double m_height;          ///< overall terrain height offset
    float m_friction;         ///< contact coefficient of friction
    double m_xmin;            ///< x coordinate where the uneven surface begins
    double m_xmax;            ///< x coordinate where the uneven surface ends
    double m_ymin;            ///< y coordinate where the uneven surface begins
    double m_ymax;            ///< y coordinate where the uneven surface ends
    double m_dx;              ///< length of a single surface patch
    int m_nx;                 ///< number of points in x-direction
    int m_ny;                 ///< number of points in y-direction
    std::vector<double> m_y;  ///< hold the unequally spaced y values

    ChMatrixDynamic<> m_Q;   ///< matrix of uneven height values
    ChMatrixDynamic<> m_a0;  ///< polynom coefficients f(x,y) = a0 + a1*x + a2*y + a3*x*y
    ChMatrixDynamic<> m_a1;
    ChMatrixDynamic<> m_a2;
    ChMatrixDynamic<> m_a3;
    ChVectorDynamic<> m_classLimits;

    double m_lambda_max;  ///< maximal spatial wavelength

    double m_f_fft_min;
    double m_f_fft_max;

    int m_Nfft;                  ///< number of fft coefficients to avoid periodicity in x[xmin...xmax]
    std::vector<double> m_ck;    ///< Fourier coefficients
    std::vector<double> m_wfft;  ///< natural freqencies for iFFT

    ChVectorDynamic<> m_phase_left;
    ChVectorDynamic<> m_phase_right;

    std::shared_ptr<ChMaterialSurface> m_material;
    bool m_collision_mesh;
    double m_start_length;
    double m_sweep_sphere_radius;

    void GenerateSurfaceFromPreset(SurfaceType theSurface, double vehicleTrackWidth = 2.0);
    void GenerateSurfaceCanonical(double unevenness, double waviness = 2.0);
    void GenerateSurfaceCanonicalCorr(double unevenness,
                                      double vehicleTrackWidth,
                                      double omega_p,
                                      double p,
                                      double waviness,
                                      double a);
    void CalculatePolynomialCoefficients();
    void CalculateSpectralCoefficients(double Phi_h0, double waviness = 2.0);
    void CalculateSpectralCoefficientsCorr(double Phi_h0,
                                           double trackWidth,
                                           double omega_p,
                                           double p,
                                           double waviness = 2.0,
                                           double a = 1.0);
    double CalculateAmplitudeLeft(double x);
    double CalculateAmplitudeRight(double x);
    double Coherence(double omega, double trackWidth, double omega_p, double p, double waviness = 2.0, double a = 1.0);
    void ApplyAmplitudes();

    void GenerateCurves();
    void GenerateMesh();

    void SetupVisualization(RandomSurfaceTerrain::VisualisationType vType);
    void SetupCollision();
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
