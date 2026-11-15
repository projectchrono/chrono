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
// Authors: Radu Serban, Hammad Mazhar, Arman Pazouki
// =============================================================================
//
// =============================================================================

#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChCapsule.h"
#include "chrono/geometry/ChCone.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/geometry/ChEllipsoid.h"
#include "chrono/geometry/ChSphere.h"

namespace chrono {
namespace utils {

// =============================================================================
// Implementation of ChMixtureIngredient class
// =============================================================================

// Constructor: create a new mixture ingredient for the specified generator, of
// a given type and with a specified ratio in the final mixture.
// This constructor is private as a new mixture ingredient can only be created
// through a generator.
ChMixtureIngredient::ChMixtureIngredient(ChGenerator* generator, MixtureType type, double ratio)
    : m_generator(generator),
      m_type(type),
      m_ratio(ratio),
      m_cumRatio(0),
      m_defMaterialNSC(chrono_types::make_shared<ChContactMaterialNSC>()),
      m_defMaterialSMC(chrono_types::make_shared<ChContactMaterialSMC>()),
      m_defDensity(1),
      m_defSize(ChVector3d(1, 1, 1)),
      m_frictionDist(nullptr),
      m_cohesionDist(nullptr),
      m_youngDist(nullptr),
      m_poissonDist(nullptr),
      m_restitutionDist(nullptr),
      m_densityDist(nullptr),
      m_sizeDist(nullptr),
      add_body_callback(nullptr) {}

// Destructor:: free the various distribution associated with this ingredient
ChMixtureIngredient::~ChMixtureIngredient() {
    FreeMaterialDist();
    delete m_densityDist;
    delete m_sizeDist;
}

// Set constant material properties for all objects based on this ingredient.
void ChMixtureIngredient::SetDefaultMaterial(std::shared_ptr<ChContactMaterial> mat) {
    assert(mat->GetContactMethod() == m_generator->m_system->GetContactMethod());

    if (mat->GetContactMethod() == ChContactMethod::NSC) {
        m_defMaterialNSC = std::static_pointer_cast<ChContactMaterialNSC>(mat);
    } else {
        m_defMaterialSMC = std::static_pointer_cast<ChContactMaterialSMC>(mat);
    }

    FreeMaterialDist();
}

void ChMixtureIngredient::SetDefaultDensity(double density) {
    m_defDensity = density;
    delete m_densityDist;
    m_densityDist = nullptr;
}

void ChMixtureIngredient::SetDefaultSize(const ChVector3d& size) {
    m_defSize = size;
    delete m_sizeDist;
    m_sizeDist = nullptr;
}

// Functions to set parameters of truncated normal distributions for the various
// attributes of an ingredient.  Properties of objects created based on this
// ingredient will be obtained by drawing from the corresponding distribution.
// If no distribution is specified for a particular property, the default
// constant value is used.
void ChMixtureIngredient::SetDistributionFriction(float friction_mean,
                                                  float friction_stddev,
                                                  float friction_min,
                                                  float friction_max) {
    m_frictionDist = new std::normal_distribution<float>(friction_mean, friction_stddev);
    m_minFriction = friction_min;
    m_maxFriction = friction_max;
}

void ChMixtureIngredient::SetDistributionCohesion(float cohesion_mean,
                                                  float cohesion_stddev,
                                                  float cohesion_min,
                                                  float cohesion_max) {
    m_cohesionDist = new std::normal_distribution<float>(cohesion_mean, cohesion_stddev);
    m_minCohesion = cohesion_min;
    m_maxCohesion = cohesion_max;
}

void ChMixtureIngredient::SetDistributionYoung(float young_mean, float young_stddev, float young_min, float young_max) {
    m_youngDist = new std::normal_distribution<float>(young_mean, young_stddev);
    m_minYoung = young_min;
    m_maxYoung = young_max;
}

void ChMixtureIngredient::SetDistributionPoisson(float poisson_mean,
                                                 float poisson_stddev,
                                                 float poisson_min,
                                                 float poisson_max) {
    m_poissonDist = new std::normal_distribution<float>(poisson_mean, poisson_stddev);
    m_minPoisson = poisson_min;
    m_maxPoisson = poisson_max;
}

void ChMixtureIngredient::SetDistributionRestitution(float restitution_mean,
                                                     float restitution_stddev,
                                                     float restitution_min,
                                                     float restitution_max) {
    m_restitutionDist = new std::normal_distribution<float>(restitution_mean, restitution_stddev);
    m_minRestitution = restitution_min;
    m_maxRestitution = restitution_max;
}

void ChMixtureIngredient::SetDistributionDensity(double density_mean,
                                                 double density_stddev,
                                                 double density_min,
                                                 double density_max) {
    m_densityDist = new std::normal_distribution<double>(density_mean, density_stddev);
    m_minDensity = density_min;
    m_maxDensity = density_max;
}

void ChMixtureIngredient::SetDistributionSize(double size_mean,
                                              double size_stddev,
                                              const ChVector3d& size_min,
                                              const ChVector3d& size_max) {
    m_sizeDist = new std::normal_distribution<double>(size_mean, size_stddev);
    m_minSize = size_min;
    m_maxSize = size_max;
}

// Utility function to delete all distributions associated with this ingredient.
void ChMixtureIngredient::FreeMaterialDist() {
    delete m_frictionDist;
    delete m_cohesionDist;
    delete m_youngDist;
    delete m_poissonDist;
    delete m_restitutionDist;

    m_frictionDist = nullptr;
    m_cohesionDist = nullptr;
    m_youngDist = nullptr;
    m_poissonDist = nullptr;
    m_restitutionDist = nullptr;
}

// Modify the specified NSC material surface based on attributes of this ingredient.
void ChMixtureIngredient::SetMaterialProperties(std::shared_ptr<ChContactMaterialNSC> mat) {
    // Copy properties from the default material.
    *mat = *m_defMaterialNSC;

    // If using distributions for any of the supported properties, override those.
    if (m_frictionDist)
        mat->SetFriction(sampleTruncatedDist<float>(*m_frictionDist, m_minFriction, m_maxFriction));

    if (m_cohesionDist)
        mat->SetCohesion(sampleTruncatedDist<float>(*m_cohesionDist, m_minCohesion, m_maxCohesion));
}

// Modify the specified SMC material surface based on attributes of this ingredient.
void ChMixtureIngredient::SetMaterialProperties(std::shared_ptr<ChContactMaterialSMC> mat) {
    // Copy properties from the default material.
    *mat = *m_defMaterialSMC;

    // If using distributions for any of the supported properties, override those.
    if (m_youngDist)
        mat->SetYoungModulus(sampleTruncatedDist<float>(*m_youngDist, m_minYoung, m_maxYoung));

    if (m_poissonDist)
        mat->SetPoissonRatio(sampleTruncatedDist<float>(*m_poissonDist, m_minPoisson, m_maxPoisson));

    if (m_frictionDist)
        mat->SetFriction(sampleTruncatedDist<float>(*m_frictionDist, m_minFriction, m_maxFriction));

    if (m_restitutionDist)
        mat->SetRestitution(sampleTruncatedDist<float>(*m_restitutionDist, m_minRestitution, m_maxRestitution));

    if (m_cohesionDist)
        mat->SetAdhesion(sampleTruncatedDist<float>(*m_cohesionDist, m_minCohesion, m_maxCohesion));
}

// Return a size for an object created based on attributes of this ingredient.
ChVector3d ChMixtureIngredient::GetSize() {
    if (m_sizeDist)
        return ChVector3d(sampleTruncatedDist<double>(*m_sizeDist, m_minSize.x(), m_maxSize.x()),
                          sampleTruncatedDist<double>(*m_sizeDist, m_minSize.y(), m_maxSize.y()),
                          sampleTruncatedDist<double>(*m_sizeDist, m_minSize.z(), m_maxSize.z()));

    return m_defSize;
}

// Return a density for an object created based on attributes of this ingredient.
double ChMixtureIngredient::GetDensity() {
    if (m_densityDist)
        return sampleTruncatedDist<double>(*m_densityDist, m_minDensity, m_maxDensity);

    return m_defDensity;
}

// Calculate the volume and gyration tensor of an object of given size created
// from this ingredient type.
// The dimensions passed here must match those of the collision shape created in ChGenerator::CreateObjects.
void ChMixtureIngredient::CalcGeometricProps(const ChVector3d& size, double& volume, ChVector3d& gyration) {
    switch (m_type) {
        case MixtureType::SPHERE:
            volume = ChSphere::CalcVolume(size.x());
            gyration = ChSphere::CalcGyration(size.x()).diagonal();
            break;
        case MixtureType::ELLIPSOID:
            volume = ChEllipsoid::CalcVolume(size * 2);
            gyration = ChEllipsoid::CalcGyration(size * 2).diagonal();
            break;
        case MixtureType::BOX:
            volume = ChBox::CalcVolume(size * 2);
            gyration = ChBox::CalcGyration(size * 2).diagonal();
            break;
        case MixtureType::CYLINDER:
            volume = ChCylinder::CalcVolume(size.x(), size.y());
            gyration = ChCylinder::CalcGyration(size.x(), size.y()).diagonal();
            break;
        case MixtureType::CONE:
            volume = ChCone::CalcVolume(size.x(), size.z());
            gyration = ChCone::CalcGyration(size.x(), size.z()).diagonal();
            break;
        case MixtureType::CAPSULE:
            volume = ChCapsule::CalcVolume(size.x(), size.z());
            gyration = ChCapsule::CalcGyration(size.x(), size.z()).diagonal();
            break;
    }
}

// Calculate the radius of a sphere that bounds any object created based on
// attributes of this ingredient.
double ChMixtureIngredient::CalcBoundingSphereRadius() {
    const ChVector3d& size = m_sizeDist ? m_maxSize : m_defSize;

    switch (m_type) {
        case MixtureType::SPHERE:
            return ChSphere::CalcBoundingSphereRadius(size.x());
        case MixtureType::ELLIPSOID:
            return ChEllipsoid::CalcBoundingSphereRadius(size * 2);
        case MixtureType::BOX:
            return ChBox::CalcBoundingSphereRadius(size * 2);
        case MixtureType::CYLINDER:
            return ChCylinder::CalcBoundingSphereRadius(size.x(), size.y());
        case MixtureType::CONE:
            return ChCone::CalcBoundingSphereRadius(size.x(), size.z());
        case MixtureType::CAPSULE:
            return ChCapsule::CalcBoundingSphereRadius(size.x(), size.z());
    }

    return 0;
}

// Calculate the dimensions of an axis-aligned box that bounds any object
// created based on attributes of this ingredient (in its initial orientation).
ChVector3d ChMixtureIngredient::CalcBoundingBoxSize() {
    const ChVector3d& size = m_sizeDist ? m_maxSize : m_defSize;

    switch (m_type) {
        case MixtureType::SPHERE:
            return ChSphere::CalcBoundingBox(size.x()).Size();
        case MixtureType::ELLIPSOID:
            return ChEllipsoid::CalcBoundingBox(size * 2).Size();
        case MixtureType::BOX:
            return ChBox::CalcBoundingBox(size * 2).Size();
        case MixtureType::CYLINDER:
            return ChCylinder::CalcBoundingBox(size.x(), size.y()).Size();
        case MixtureType::CONE:
            return ChCone::CalcBoundingBox(size.x(), size.z()).Size();
        case MixtureType::CAPSULE:
            return ChCapsule::CalcBoundingBox(size.x(), size.z()).Size();
    }

    return VNULL;
}

// =============================================================================
// Implementation of ChGenerator class
// =============================================================================

// Constructor: create a generator for the specified system.
ChGenerator::ChGenerator(ChSystem* system)
    : m_system(system), m_mixDist(0, 1), m_start_tag(0), m_totalNumBodies(0), m_totalMass(0), m_totalVolume(0) {}

// Destructor
ChGenerator::~ChGenerator() {}

// Add a new ingredient to the current mixture by specifying its type
// and the ratio in the final mixture. A smart pointer to the new
// mixture ingredient is returned to allow modifying its properties.
std::shared_ptr<ChMixtureIngredient> ChGenerator::AddMixtureIngredient(MixtureType type, double ratio) {
    m_mixture.push_back(chrono_types::make_shared<ChMixtureIngredient>(this, type, ratio));
    return m_mixture.back();
}

// Create objects in a box domain using the given point sampler and separation distance and the current mixture
// settings. The types of objects created are selected randomly with probability proportional to the ratio of that
// ingredient in the mixture.
void ChGenerator::CreateObjectsBox(ChSampler<double>& sampler,
                                   const ChVector3d& pos,
                                   const ChVector3d& hdims,
                                   const ChVector3d& vel) {
    // Normalize the mixture ratios
    NormalizeMixture();

    // Generate the object locations
    double sep = sampler.GetSeparation();
    if (m_system->GetContactMethod() == ChContactMethod::SMC)
        sep = CalcMinSeparation(sep);
    sampler.SetSeparation(sep);

    PointVector points = sampler.SampleBox(pos, hdims);
    CreateObjects(points, vel);
}

// Create objects in a box domain sampled on a regular grid with
// specified separation distances and using the current mixture settings.
// The types of objects created are selected randomly with probability
// proportional to the ratio of that ingredient in the mixture.
void ChGenerator::CreateObjectsBox(const ChVector3d& dist,
                                   const ChVector3d& pos,
                                   const ChVector3d& hdims,
                                   const ChVector3d& vel) {
    // Normalize the mixture ratios
    NormalizeMixture();

    // When using SMC, make sure there is no shape overlap.
    ChVector3d distv;
    if (m_system->GetContactMethod() == ChContactMethod::SMC)
        distv = CalcMinSeparation(dist);
    else
        distv = dist;

    // Generate the object locations
    ChGridSampler<> sampler(distv);
    PointVector points;
    points = sampler.SampleBox(pos, hdims);

    CreateObjects(points, vel);
}

// Create objects in a cylindrical domain using the specified type of point
// sampler and separation distance and the current mixture settings.
// The types of objects created are selected randomly with probability
// proportional to the ratio of that ingredient in the mixture.
void ChGenerator::CreateObjectsCylinderX(ChSampler<double>& sampler,
                                         const ChVector3d& pos,
                                         float radius,
                                         float halfHeight,
                                         const ChVector3d& vel) {
    // Normalize the mixture ratios
    NormalizeMixture();

    // Generate the object locations
    double sep = sampler.GetSeparation();
    if (m_system->GetContactMethod() == ChContactMethod::SMC)
        sep = CalcMinSeparation(sep);
    sampler.SetSeparation(sep);

    PointVector points = sampler.SampleCylinderX(pos, radius, halfHeight);
    CreateObjects(points, vel);
}

void ChGenerator::CreateObjectsCylinderY(ChSampler<double>& sampler,
                                         const ChVector3d& pos,
                                         float radius,
                                         float halfHeight,
                                         const ChVector3d& vel) {
    // Normalize the mixture ratios
    NormalizeMixture();

    // Generate the object locations
    double sep = sampler.GetSeparation();
    if (m_system->GetContactMethod() == ChContactMethod::SMC)
        sep = CalcMinSeparation(sep);
    sampler.SetSeparation(sep);

    PointVector points = sampler.SampleCylinderY(pos, radius, halfHeight);
    CreateObjects(points, vel);
}

void ChGenerator::CreateObjectsCylinderZ(ChSampler<double>& sampler,
                                         const ChVector3d& pos,
                                         float radius,
                                         float halfHeight,
                                         const ChVector3d& vel) {
    // Normalize the mixture ratios
    NormalizeMixture();

    // Generate the object locations
    double sep = sampler.GetSeparation();
    if (m_system->GetContactMethod() == ChContactMethod::SMC)
        sep = CalcMinSeparation(sep);
    sampler.SetSeparation(sep);

    PointVector points = sampler.SampleCylinderZ(pos, radius, halfHeight);
    CreateObjects(points, vel);
}

void ChGenerator::CreateObjectsSphere(ChSampler<double>& sampler,
                                      const ChVector3d& pos,
                                      float radius,
                                      const ChVector3d& vel) {
    // Normalize the mixture ratios
    NormalizeMixture();

    // Generate the object locations
    double sep = sampler.GetSeparation();
    if (m_system->GetContactMethod() == ChContactMethod::SMC)
        sep = CalcMinSeparation(sep);
    sampler.SetSeparation(sep);

    PointVector points = sampler.SampleSphere(pos, radius);
    CreateObjects(points, vel);
}

// Normalize the mixture ratios (so that their sum is 1) and calculate
// the exclusive scan of these ratios.
void ChGenerator::NormalizeMixture() {
    if (m_mixture.empty()) {
        AddMixtureIngredient(MixtureType::SPHERE, 1);
        return;
    }

    double sum = 0;

    for (int i = 0; i < m_mixture.size(); i++)
        sum += m_mixture[i]->m_ratio;

    for (int i = 0; i < m_mixture.size(); i++) {
        m_mixture[i]->m_ratio /= sum;
        if (i == 0)
            m_mixture[i]->m_cumRatio = 0;
        else
            m_mixture[i]->m_cumRatio = m_mixture[i - 1]->m_cumRatio + m_mixture[i - 1]->m_ratio;
    }
}

// Select one of the mixture types with probability proportional to that
// type's ratio in the mixture.
int ChGenerator::SelectIngredient() {
    double val = m_mixDist(rengine());

    for (int i = (int)m_mixture.size() - 1; i >= 0; i--) {
        if (val > m_mixture[i]->m_cumRatio)
            return i;
    }

    return 0;
}

// Calculate the minimum distance between sampled points to ensure contact shapes never overlap.
// Points can be in any direction relative to each other, so use the diameter of the largest bounding sphere.
double ChGenerator::CalcMinSeparation(double sep) {
    for (int i = 0; i < m_mixture.size(); i++)
        sep = std::max(sep, 2 * m_mixture[i]->CalcBoundingSphereRadius());

    return sep;
}

// Calculate the minimum grid spacing to ensure contact shapes never overlap.
// Shapes at neighboring grid points are separated along at least one grid direction, so it is enough for the spacing
// in each direction to be at least the size of the largest bounding box in that direction.
ChVector3d ChGenerator::CalcMinSeparation(const ChVector3d& sep) {
    ChVector3d res = sep;

    for (int i = 0; i < m_mixture.size(); i++) {
        ChVector3d mix_size = m_mixture[i]->CalcBoundingBoxSize();
        res.x() = std::max(res.x(), mix_size.x());
        res.y() = std::max(res.y(), mix_size.y());
        res.z() = std::max(res.z(), mix_size.z());
    }

    return res;
}

// Create objects at the specified locations using the current mixture settings.
void ChGenerator::CreateObjects(const PointVector& points, const ChVector3d& vel) {
    bool check = false;
    std::vector<bool> flags;
    if (m_callback) {
        flags.resize(points.size(), true);
        m_callback->OnCreateObjects(points, flags);
        check = true;
    }

    for (int i = 0; i < points.size(); i++) {
        if (check && !flags[i])
            continue;

        // Select the type of object to be created.
        int index = SelectIngredient();

        // Create a contact material consistent with the associated system and modify it based on attributes of the
        // current ingredient.
        std::shared_ptr<ChContactMaterial> mat;
        switch (m_system->GetContactMethod()) {
            case ChContactMethod::NSC: {
                auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
                m_mixture[index]->SetMaterialProperties(matNSC);
                mat = matNSC;
                break;
            }
            case ChContactMethod::SMC: {
                auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
                m_mixture[index]->SetMaterialProperties(matSMC);
                mat = matSMC;
                break;
            }
        }

        // Create the body (with appropriate collision model, consistent with the associated system)
        auto body = chrono_types::make_shared<ChBody>();

        // Set identifier
        body->SetTag(m_start_tag++);

        // Set position and orientation
        body->SetPos(points[i]);
        body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        body->SetPosDt(vel);
        body->SetFixed(false);
        body->EnableCollision(true);

        // Get size and density; calculate geometric properties
        ChVector3d size = m_mixture[index]->GetSize();
        double density = m_mixture[index]->GetDensity();
        double volume;
        ChVector3d gyration;
        m_mixture[index]->CalcGeometricProps(size, volume, gyration);
        double mass = density * volume;

        // Set mass properties
        body->SetMass(mass);
        body->SetInertiaXX(mass * gyration);
        m_totalMass += mass;
        m_totalVolume += volume;

        // Add collision geometry (dimensions must match those used in ChMixtureIngredient::CalcGeometricProps)
        switch (m_mixture[index]->m_type) {
            case MixtureType::SPHERE:
                AddSphereGeometry(body.get(), mat, size.x());
                break;
            case MixtureType::ELLIPSOID:
                AddEllipsoidGeometry(body.get(), mat, size * 2);
                break;
            case MixtureType::BOX:
                AddBoxGeometry(body.get(), mat, size * 2);
                break;
            case MixtureType::CYLINDER:
                AddCylinderGeometry(body.get(), mat, size.x(), size.y());
                break;
            case MixtureType::CONE:
                AddConeGeometry(body.get(), mat, size.x(), size.z());
                break;
            case MixtureType::CAPSULE:
                AddCapsuleGeometry(body.get(), mat, size.x(), size.z());
                break;
        }

        // Attach the body to the system and append to list of generated bodies.
        m_system->AddBody(body);

        // If the callback pointer is set, call the function with the body pointer
        if (m_mixture[index]->add_body_callback) {
            m_mixture[index]->add_body_callback->OnAddBody(body);
        }

        m_bodies.push_back(BodyInfo(m_mixture[index]->m_type, density, size, body));
    }

    m_totalNumBodies += (unsigned int)points.size();
}

// Write body information to a CSV file
void ChGenerator::writeObjectInfo(const std::string& filename) {
    ChWriterCSV csv;

    for (int i = 0; i < m_bodies.size(); i++) {
        csv << static_cast<int>(m_bodies[i].m_type);
        csv << m_bodies[i].m_body->GetPos() << m_bodies[i].m_size;
        csv << m_bodies[i].m_density << m_bodies[i].m_body->GetMass();

        //// RADU: write collision shape information, including contact material properties?

        csv << std::endl;
    }

    csv.WriteToFile(filename);
}

}  // namespace utils
}  // namespace chrono
