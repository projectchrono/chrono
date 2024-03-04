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
// Authors: Radu Serban
// =============================================================================
//
// Utility functions to facilitate the creation of mixtures of granular material
// using different samplers for initial positions, geometry shapes, material
// properties, etc.
//
// =============================================================================

#ifndef CH_UTILS_GENERATORS_H
#define CH_UTILS_GENERATORS_H

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsSamplers.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Enumeration of various geometric shapes available for mixtures.
enum class MixtureType { SPHERE, ELLIPSOID, BOX, CYLINDER, CONE, CAPSULE };

// Forward declarations
class Generator;
class MixtureIngredient;

/// Encapsulation of an ingredient of one of the supported types in a mixture.
/// Such an object defines size, mass properties and material properties for, all of which can be constant for all
/// mixture components of this type or else obtained from associated truncated normal distributions. In addition, a
/// mixture ingredient defines the ratio of this particular type in the containing mixture.
class ChApi MixtureIngredient {
  public:
    MixtureIngredient(Generator* generator, MixtureType type, double ratio);
    ~MixtureIngredient();

    void setDefaultMaterial(std::shared_ptr<ChMaterialSurface> mat);
    void setDefaultDensity(double density);
    void setDefaultSize(const ChVector<>& size);

    void setDistributionFriction(float friction_mean, float friction_stddev, float friction_min, float friction_max);
    void setDistributionCohesion(float cohesion_mean, float cohesion_stddev, float cohesion_min, float cohesion_max);
    void setDistributionYoung(float young_mean, float young_stddev, float young_min, float young_max);
    void setDistributionPoisson(float poisson_mean, float poisson_stddev, float poisson_min, float poisson_max);
    void setDistributionRestitution(float restitution_mean,
                                    float restitution_stddev,
                                    float restitution_min,
                                    float restitution_max);
    void setDistributionDensity(double density_mean, double density_stddev, double density_min, double density_max);
    void setDistributionSize(double size_mean,
                             double size_stddev,
                             const ChVector<>& size_min,
                             const ChVector<>& size_max);

    /// Class to be used as a callback interface for some user-defined action to be taken each
    /// time the generator creates and adds a body based on this mixture ingredient to the system.
    class ChApi AddBodyCallback {
      public:
        virtual ~AddBodyCallback() {}

        /// Callback used to process bodies as they are created and added to the system.
        virtual void OnAddBody(std::shared_ptr<ChBody> body) = 0;
    };

    /// Specify a callback object to be used each time a body is generated using this
    /// mixture ingredient specification.
    void RegisterAddBodyCallback(std::shared_ptr<AddBodyCallback> callback) { add_body_callback = callback; }

  private:
    void freeMaterialDist();
    ChVector<> getSize();
    double getDensity();
    void calcGeometricProps(const ChVector<>& size, double& volume, ChVector<>& gyration);
    double calcMinSeparation();

    void setMaterialProperties(std::shared_ptr<ChMaterialSurfaceNSC> mat);
    void setMaterialProperties(std::shared_ptr<ChMaterialSurfaceSMC> mat);

    Generator* m_generator;

    MixtureType m_type;
    double m_ratio;
    double m_cumRatio;

    std::shared_ptr<ChMaterialSurfaceNSC> m_defMaterialNSC;
    std::shared_ptr<ChMaterialSurfaceSMC> m_defMaterialSMC;

    float m_minFriction, m_maxFriction;
    float m_minCohesion, m_maxCohesion;
    float m_minYoung, m_maxYoung;
    float m_minPoisson, m_maxPoisson;
    float m_minRestitution, m_maxRestitution;
    std::normal_distribution<float>* m_frictionDist;
    std::normal_distribution<float>* m_cohesionDist;
    std::normal_distribution<float>* m_youngDist;
    std::normal_distribution<float>* m_poissonDist;
    std::normal_distribution<float>* m_restitutionDist;

    double m_defDensity;
    double m_minDensity, m_maxDensity;
    std::normal_distribution<>* m_densityDist;

    ChVector<> m_defSize;
    ChVector<> m_minSize, m_maxSize;
    std::normal_distribution<>* m_sizeDist;

    std::shared_ptr<AddBodyCallback> add_body_callback;

    friend class Generator;
};

/// Provides functionality for generating sets of bodies with positions drawn from a specified sampler and various
/// mixture properties. Bodies can be generated in different bounding volumes (boxes or cylinders) which can be
/// degenerate (to a rectangle or circle, repsectively).
class ChApi Generator {
  public:
    typedef Types<double>::PointVector PointVector;

    Generator(ChSystem* system);
    ~Generator();

    /// Add a new mixture ingredient of the specified type and in the given ratio (note that the ratios are normalized
    /// before creating bodies).
    std::shared_ptr<MixtureIngredient> AddMixtureIngredient(MixtureType type, double ratio);

    /// Get/Set the identifier that will be assigned to the next body.
    /// Identifiers are incremented for successively created bodies.
    int getBodyIdentifier() const { return m_crtBodyId; }
    void setBodyIdentifier(int id) { m_crtBodyId = id; }

    /// Create bodies, according to the current mixture setup, with initial positions given by the specified sampler in
    /// the box domain specified by 'pos' and 'hdims'. Optionally, a constant initial linear velocity can be set for all
    /// created bodies.
    void CreateObjectsBox(Sampler<double>& sampler,
                          const ChVector<>& pos,
                          const ChVector<>& hdims,
                          const ChVector<>& vel = ChVector<>(0, 0, 0));

    /// Create bodies, according to the current mixture setup, with initial positions on a uniform grid with given
    /// separations (in x,y,z directions) in the box domain specified by 'pos' and 'hdims'. Optionally, a constant
    /// initial linear velocity can be set for all created bodies.
    void CreateObjectsBox(const ChVector<>& dist,
                          const ChVector<>& pos,
                          const ChVector<>& hdims,
                          const ChVector<>& vel = ChVector<>(0, 0, 0));

    /// Create bodies, according to the current mixture setup, with initial positions given by the specified sampler in
    /// the X-aligned cylinder domain specified by 'pos', 'radius' and 'halfHeight'. Optionally, a constant initial
    /// linear velocity can be set for all created bodies.
    void CreateObjectsCylinderX(Sampler<double>& sampler,
                                const ChVector<>& pos,
                                float radius,
                                float halfHeight,
                                const ChVector<>& vel = ChVector<>(0, 0, 0));

    /// Create bodies, according to the current mixture setup, with initial positions given by the specified sampler in
    /// the Y-aligned cylinder domain specified by 'pos', 'radius' and 'halfHeight'. Optionally, a constant initial
    /// linear velocity can be set for all created bodies.
    void CreateObjectsCylinderY(Sampler<double>& sampler,
                                const ChVector<>& pos,
                                float radius,
                                float halfHeight,
                                const ChVector<>& vel = ChVector<>(0, 0, 0));

    /// Create bodies, according to the current mixture setup, with initial positions given by the specified sampler in
    /// the Z-aligned cylinder domain specified by 'pos', 'radius' and 'halfHeight'. Optionally, a constant initial
    /// linear velocity can be set for all created bodies.
    void CreateObjectsCylinderZ(Sampler<double>& sampler,
                                const ChVector<>& pos,
                                float radius,
                                float halfHeight,
                                const ChVector<>& vel = ChVector<>(0, 0, 0));

    /// Create bodies, according to the current mixture setup, with initial positions given by the specified sampler in
    /// the spherical domain specified by 'pos' and 'radius'. Optionally, a constant initial linear velocity can be set
    /// for all created bodies.
    void CreateObjectsSphere(Sampler<double>& sampler,
                             const ChVector<>& pos,
                             float radius,
                             const ChVector<>& vel = ChVector<>(0, 0, 0));

    /// Class to be used as a callback interface for user-defined filtering of initial positions.
    class ChApi CreateObjectsCallback {
      public:
        virtual ~CreateObjectsCallback() {}

        /// Callback used to process the initial position points generated by the underlying sampler.
        /// The provided vector of boolean flags is set to all 'true'. Set the i-th entry to 'false'
        /// if a body should not be created at the i-th position.
        virtual void OnCreateObjects(
            const PointVectorD& points,  ///< vector of positions generated by the sampler
            std::vector<bool>& flags     ///< change to 'false' for positions where a body should not be generated
            ) = 0;
    };

    /// Specify a callback object to be used before creating the bodies. The OnCreateObjects() method of the
    /// provided callback object will be called before object creation, allowing the user to filter the points
    /// at which bodies will be initialized.
    void RegisterCreateObjectsCallback(std::shared_ptr<CreateObjectsCallback> callback) { m_callback = callback; }

    /// Write information about the bodies created so far to the specified file (CSV format).
    void writeObjectInfo(const std::string& filename);

    unsigned int getTotalNumBodies() const { return m_totalNumBodies; }
    double getTotalMass() const { return m_totalMass; }
    double getTotalVolume() const { return m_totalVolume; }

  private:
    struct BodyInfo {
        BodyInfo(MixtureType t, double density, const ChVector<>& size, const std::shared_ptr<ChBody>& b)
            : m_type(t), m_density(density), m_size(size), m_body(b) {}
        MixtureType m_type;
        double m_density;
        ChVector<> m_size;
        std::shared_ptr<ChBody> m_body;
    };

    void normalizeMixture();
    int selectIngredient();
    double calcMinSeparation(double sep);
    ChVector<> calcMinSeparation(const ChVector<>& sep);
    void createObjects(const PointVector& points, const ChVector<>& vel);

    ChSystem* m_system;

    std::uniform_real_distribution<> m_mixDist;

    std::vector<std::shared_ptr<MixtureIngredient>> m_mixture;
    std::vector<BodyInfo> m_bodies;
    unsigned int m_totalNumBodies;
    double m_totalMass;
    double m_totalVolume;

    std::shared_ptr<CreateObjectsCallback> m_callback;

    int m_crtBodyId;

    friend class MixtureIngredient;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
