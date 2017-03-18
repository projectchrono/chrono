// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

namespace chrono {
namespace utils {

// =============================================================================
// Implementation of MixtureIngredient class
// =============================================================================

// Constructor: create a new mixture ingredient for the specified generator, of
// a given type and with a specified ratio in the final mixture.
// This constructor is private as a new mixture ingredient can only be created
// through a generator.
MixtureIngredient::MixtureIngredient(Generator* generator, MixtureType type, double ratio)
    : m_generator(generator),
      m_type(type),
      m_ratio(ratio),
      m_cumRatio(0),
      m_defMaterialDVI(std::make_shared<ChMaterialSurface>()),
      m_defMaterialDEM(std::make_shared<ChMaterialSurfaceDEM>()),
      m_defDensity(1),
      m_defSize(ChVector<>(1, 1, 1)),
      m_frictionDist(nullptr),
      m_cohesionDist(nullptr),
      m_youngDist(nullptr),
      m_poissonDist(nullptr),
      m_restitutionDist(nullptr),
      m_densityDist(nullptr),
      m_sizeDist(nullptr),
      callback_post_creation(nullptr) {}

// Destructor:: free the various distribution associated with this ingredient
MixtureIngredient::~MixtureIngredient() {
    freeMaterialDist();
    delete m_densityDist;
    delete m_sizeDist;
}

// Set constant material properties for all objects based on this ingredient.
void MixtureIngredient::setDefaultMaterial(std::shared_ptr<ChMaterialSurfaceBase> mat) {
    assert(mat->GetContactMethod() == m_generator->m_system->GetContactMethod());

    if (mat->GetContactMethod() == ChMaterialSurfaceBase::DVI) {
        m_defMaterialDVI = std::static_pointer_cast<ChMaterialSurface>(mat);
    } else {
        m_defMaterialDEM = std::static_pointer_cast<ChMaterialSurfaceDEM>(mat);
    }

    freeMaterialDist();
}

void MixtureIngredient::setDefaultDensity(double density) {
    m_defDensity = density;
    delete m_densityDist;
    m_densityDist = nullptr;
}

void MixtureIngredient::setDefaultSize(const ChVector<>& size) {
    m_defSize = size;
    delete m_sizeDist;
    m_sizeDist = nullptr;
}

// Functions to set parameters of truncated normal distributions for the various
// attributes of an ingredient.  Properties of objects created based on this
// ingredient will be obtained by drawing from the corresponding distribution.
// If no distribution is specified for a particular property, the default
// constant value is used.
void MixtureIngredient::setDistributionFriction(float friction_mean,
                                                float friction_stddev,
                                                float friction_min,
                                                float friction_max) {
    m_frictionDist = new std::normal_distribution<float>(friction_mean, friction_stddev);
    m_minFriction = friction_min;
    m_maxFriction = friction_max;
}

void MixtureIngredient::setDistributionCohesion(float cohesion_mean,
                                                float cohesion_stddev,
                                                float cohesion_min,
                                                float cohesion_max) {
    m_cohesionDist = new std::normal_distribution<float>(cohesion_mean, cohesion_stddev);
    m_minCohesion = cohesion_min;
    m_maxCohesion = cohesion_max;
}

void MixtureIngredient::setDistributionYoung(float young_mean, float young_stddev, float young_min, float young_max) {
    m_youngDist = new std::normal_distribution<float>(young_mean, young_stddev);
    m_minYoung = young_min;
    m_maxYoung = young_max;
}

void MixtureIngredient::setDistributionPoisson(float poisson_mean,
                                               float poisson_stddev,
                                               float poisson_min,
                                               float poisson_max) {
    m_poissonDist = new std::normal_distribution<float>(poisson_mean, poisson_stddev);
    m_minPoisson = poisson_min;
    m_maxPoisson = poisson_max;
}

void MixtureIngredient::setDistributionRestitution(float restitution_mean,
                                                   float restitution_stddev,
                                                   float restitution_min,
                                                   float restitution_max) {
    m_restitutionDist = new std::normal_distribution<float>(restitution_mean, restitution_stddev);
    m_minRestitution = restitution_min;
    m_maxRestitution = restitution_max;
}

void MixtureIngredient::setDistributionDensity(double density_mean,
                                               double density_stddev,
                                               double density_min,
                                               double density_max) {
    m_densityDist = new std::normal_distribution<double>(density_mean, density_stddev);
    m_minDensity = density_min;
    m_maxDensity = density_max;
}

void MixtureIngredient::setDistributionSize(double size_mean,
                                            double size_stddev,
                                            const ChVector<>& size_min,
                                            const ChVector<>& size_max) {
    m_sizeDist = new std::normal_distribution<double>(size_mean, size_stddev);
    m_minSize = size_min;
    m_maxSize = size_max;
}

// Utility function to delete all distributions associated with this ingredient.
void MixtureIngredient::freeMaterialDist() {
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

// Modify the specified DVI material surface based on attributes of this ingredient.
void MixtureIngredient::setMaterialProperties(std::shared_ptr<ChMaterialSurface> mat) {
    // Copy properties from the default material.
    *mat = *m_defMaterialDVI;

    // If using distributions for any of the supported properties, override those.
    if (m_frictionDist)
        mat->SetFriction(sampleTruncatedDist<float>(*m_frictionDist, m_minFriction, m_maxFriction));

    if (m_cohesionDist)
        mat->SetCohesion(sampleTruncatedDist<float>(*m_cohesionDist, m_minCohesion, m_maxCohesion));
}

// Modify the specified DEM material surface based on attributes of this ingredient.
void MixtureIngredient::setMaterialProperties(std::shared_ptr<ChMaterialSurfaceDEM> mat) {
    // Copy properties from the default material.
    *mat = *m_defMaterialDEM;

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
ChVector<> MixtureIngredient::getSize() {
    if (m_sizeDist)
        return ChVector<>(sampleTruncatedDist<double>(*m_sizeDist, m_minSize.x(), m_maxSize.x()),
                          sampleTruncatedDist<double>(*m_sizeDist, m_minSize.y(), m_maxSize.y()),
                          sampleTruncatedDist<double>(*m_sizeDist, m_minSize.z(), m_maxSize.z()));

    return m_defSize;
}

// Return a density for an object created based on attributes of this ingredient.
double MixtureIngredient::getDensity() {
    if (m_densityDist)
        return sampleTruncatedDist<double>(*m_densityDist, m_minDensity, m_maxDensity);

    return m_defDensity;
}

// Calculate the volume and gyration tensor of an object of given size created
// from this ingredient type.
void MixtureIngredient::calcGeometricProps(const ChVector<>& size, double& volume, ChVector<>& gyration) {
    switch (m_type) {
        case SPHERE:
            volume = CalcSphereVolume(size.x());
            gyration = CalcSphereGyration(size.x()).Get_Diag();
            break;
        case ELLIPSOID:
            volume = CalcEllipsoidVolume(size);
            gyration = CalcEllipsoidGyration(size).Get_Diag();
            break;
        case BOX:
            volume = CalcBoxVolume(size);
            gyration = CalcBoxGyration(size).Get_Diag();
            break;
        case CYLINDER:
            volume = CalcCylinderVolume(size.x(), size.y());
            gyration = CalcCylinderGyration(size.x(), size.y()).Get_Diag();
            break;
        case CONE:
            volume = CalcConeVolume(size.x(), size.y());
            gyration = CalcConeGyration(size.x(), size.y()).Get_Diag();
            break;
        case BISPHERE:
            volume = CalcBiSphereVolume(size.x(), size.y());
            gyration = CalcBiSphereGyration(size.x(), size.y()).Get_Diag();
            break;
        case CAPSULE:
            volume = CalcCapsuleVolume(size.x(), size.y());
            gyration = CalcCapsuleGyration(size.x(), size.y()).Get_Diag();
            break;
        case ROUNDEDCYLINDER:
            volume = CalcRoundedCylinderVolume(size.x(), size.y(), size.z());
            gyration = CalcRoundedCylinderGyration(size.x(), size.y(), size.z()).Get_Diag();
            break;
    }
}

// Calculate a necessary minimum separation based on the largest possible
// dimension of an object created based on attributes of this ingredient.
double MixtureIngredient::calcMinSeparation() {
    if (m_sizeDist)
        return std::max(m_maxSize.x(), std::max(m_maxSize.y(), m_maxSize.z()));

    return std::max(m_defSize.x(), std::max(m_defSize.y(), m_defSize.z()));
}

// =============================================================================
// Implementation of Generator class
// =============================================================================

// Constructor: create a generator for the specified system.
Generator::Generator(ChSystem* system)
    : m_system(system), m_mixDist(0, 1), m_crtBodyId(0), m_totalNumBodies(0), m_totalMass(0), m_totalVolume(0) {
}

// Destructor
Generator::~Generator() {
}

// Add a new ingredient to the current mixture by specifying its type
// and the ratio in the final mixture. A smart pointer to the new
// mixture ingredient is returned to allow modifying its properties.
std::shared_ptr<MixtureIngredient> Generator::AddMixtureIngredient(MixtureType type, double ratio) {
    m_mixture.push_back(std::make_shared<MixtureIngredient>(this, type, ratio));
    return m_mixture.back();
}

// Create objects in a box domain using the specified type of point
// sampler and separation distance and the current mixture settings.
// The types of objects created are selected randomly with probability
// proportional to the ratio of that ingredient in the mixture.
void Generator::createObjectsBox(SamplingType sType,
                                 double dist,
                                 const ChVector<>& pos,
                                 const ChVector<>& hdims,
                                 const ChVector<>& vel) {
    // Normalize the mixture ratios
    normalizeMixture();

    // Generate the object locations
    if (m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM)
        dist = calcMinSeparation(dist);

    PointVector points;
    switch (sType) {
        case REGULAR_GRID: {
            GridSampler<> sampler(dist);
            points = sampler.SampleBox(pos, hdims);
        } break;
        case POISSON_DISK: {
            PDSampler<> sampler(dist);
            points = sampler.SampleBox(pos, hdims);
        } break;
        case HCP_PACK: {
            HCPSampler<> sampler(dist);
            points = sampler.SampleBox(pos, hdims);
        } break;
    }

    createObjects(points, vel);
}

// Create objects in a box domain sampled on a regular grid with
// specified separation distances and using the current mixture settings.
// The types of objects created are selected randomly with probability
// proportional to the ratio of that ingredient in the mixture.
void Generator::createObjectsBox(const ChVector<>& dist,
                                 const ChVector<>& pos,
                                 const ChVector<>& hdims,
                                 const ChVector<>& vel) {
    // Normalize the mixture ratios
    normalizeMixture();

    // When using DEM, make sure there is no shape overlap.
    ChVector<> distv;
    if (m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM)
        distv = calcMinSeparation(dist);
    else
        distv = dist;

    // Generate the object locations
    GridSampler<> sampler(distv);
    PointVector points;
    points = sampler.SampleBox(pos, hdims);

    createObjects(points, vel);
}

// Create objects in a cylindrical domain using the specified type of point
// sampler and separation distance and the current mixture settings.
// The types of objects created are selected randomly with probability
// proportional to the ratio of that ingredient in the mixture.
void Generator::createObjectsCylinderX(SamplingType sType,
                                       double dist,
                                       const ChVector<>& pos,
                                       float radius,
                                       float halfHeight,
                                       const ChVector<>& vel) {
    // Normalize the mixture ratios
    normalizeMixture();

    // Generate the object locations
    if (m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM)
        dist = calcMinSeparation(dist);

    PointVector points;
    switch (sType) {
        case REGULAR_GRID: {
            GridSampler<> sampler(dist);
            points = sampler.SampleCylinderX(pos, radius, halfHeight);
        } break;
        case POISSON_DISK: {
            PDSampler<> sampler(dist);
            points = sampler.SampleCylinderX(pos, radius, halfHeight);
        } break;
        case HCP_PACK: {
            HCPSampler<> sampler(dist);
            points = sampler.SampleCylinderX(pos, radius, halfHeight);
        } break;
    }

    createObjects(points, vel);
}

void Generator::createObjectsCylinderY(SamplingType sType,
                                       double dist,
                                       const ChVector<>& pos,
                                       float radius,
                                       float halfHeight,
                                       const ChVector<>& vel) {
    // Normalize the mixture ratios
    normalizeMixture();

    // Generate the object locations
    if (m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM)
        dist = calcMinSeparation(dist);

    PointVector points;
    switch (sType) {
        case REGULAR_GRID: {
            GridSampler<> sampler(dist);
            points = sampler.SampleCylinderY(pos, radius, halfHeight);
        } break;
        case POISSON_DISK: {
            PDSampler<> sampler(dist);
            points = sampler.SampleCylinderY(pos, radius, halfHeight);
        } break;
        case HCP_PACK: {
            HCPSampler<> sampler(dist);
            points = sampler.SampleCylinderY(pos, radius, halfHeight);
        } break;
    }

    createObjects(points, vel);
}

void Generator::createObjectsCylinderZ(SamplingType sType,
                                       double dist,
                                       const ChVector<>& pos,
                                       float radius,
                                       float halfHeight,
                                       const ChVector<>& vel) {
    // Normalize the mixture ratios
    normalizeMixture();

    // Generate the object locations
    if (m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM)
        dist = calcMinSeparation(dist);

    PointVector points;
    switch (sType) {
        case REGULAR_GRID: {
            GridSampler<> sampler(dist);
            points = sampler.SampleCylinderZ(pos, radius, halfHeight);
        } break;
        case POISSON_DISK: {
            PDSampler<> sampler(dist);
            points = sampler.SampleCylinderZ(pos, radius, halfHeight);
        } break;
        case HCP_PACK: {
            HCPSampler<> sampler(dist);
            points = sampler.SampleCylinderZ(pos, radius, halfHeight);
        } break;
    }

    createObjects(points, vel);
}

// Normalize the mixture ratios (so that their sum is 1) and calculate
// the exclusive scan of these ratios.
void Generator::normalizeMixture() {
    if (m_mixture.empty()) {
        AddMixtureIngredient(SPHERE, 1);
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
int Generator::selectIngredient() {
    double val = m_mixDist(rengine());

    for (int i = (int)m_mixture.size() - 1; i >= 0; i--) {
        if (val > m_mixture[i]->m_cumRatio)
            return i;
    }

    return 0;
}

// Calculate the minimum distance to ensure contact shapes never overlap.
double Generator::calcMinSeparation(double sep) {
    for (int i = 0; i < m_mixture.size(); i++)
        sep = std::max(sep, m_mixture[i]->calcMinSeparation());

    return sep;
}

ChVector<> Generator::calcMinSeparation(const ChVector<>& sep) {
    ChVector<> res;

    for (int i = 0; i < m_mixture.size(); i++) {
        double mix_sep = m_mixture[i]->calcMinSeparation();
        res.x() = std::max(sep.x(), mix_sep);
        res.y() = std::max(sep.y(), mix_sep);
        res.z() = std::max(sep.z(), mix_sep);
    }

    return res;
}

// Create objects at the specified locations using the current mixture settings.
void Generator::createObjects(const PointVector& points, const ChVector<>& vel) {
    for (int i = 0; i < points.size(); i++) {
        // Select the type of object to be created.
        int index = selectIngredient();

        // Create the body (with appropriate contact method and collision model, consistent
        // with the associated system) and set contact material
        ChBody* body = m_system->NewBody();

        switch (m_system->GetContactMethod()) {
            case ChMaterialSurfaceBase::DVI:
                m_mixture[index]->setMaterialProperties(body->GetMaterialSurface());
                break;
            case ChMaterialSurfaceBase::DEM:
                m_mixture[index]->setMaterialProperties(body->GetMaterialSurfaceDEM());
                break;
        }

        // Set identifier
        body->SetIdentifier(m_crtBodyId++);

        // Set position and orientation
        body->SetPos(points[i]);
        body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        body->SetPos_dt(vel);
        body->SetBodyFixed(false);
        body->SetCollide(true);

        // Get size and density; calculate geometric properties
        ChVector<> size = m_mixture[index]->getSize();
        double density = m_mixture[index]->getDensity();
        double volume;
        ChVector<> gyration;
        m_mixture[index]->calcGeometricProps(size, volume, gyration);
        double mass = density * volume;

        // Set mass properties
        body->SetMass(mass);
        body->SetInertiaXX(mass * gyration);
        m_totalMass += mass;
        m_totalVolume += volume;

        // Add collision geometry
        body->GetCollisionModel()->ClearModel();

        switch (m_mixture[index]->m_type) {
            case SPHERE:
                AddSphereGeometry(body, size.x());
                break;
            case ELLIPSOID:
                AddEllipsoidGeometry(body, size);
                break;
            case BOX:
                AddBoxGeometry(body, size);
                break;
            case CYLINDER:
                AddCylinderGeometry(body, size.x(), size.y());
                break;
            case CONE:
                AddConeGeometry(body, size.x(), size.y());
                break;
            case BISPHERE:
            	AddBiSphereGeometry(body, size.x(), size.y());
                break;
            case CAPSULE:
                AddCapsuleGeometry(body, size.x(), size.y());
                break;
            case ROUNDEDCYLINDER:
                AddRoundedCylinderGeometry(body, size.x(), size.y(), size.z());
                break;
        }

        body->GetCollisionModel()->BuildModel();

        // Attach the body to the system and append to list of generated bodies.
        std::shared_ptr<ChBody> bodyPtr(body);

        m_system->AddBody(bodyPtr);

        // If the callback pointer is set, call the function with the body pointer
        if (m_mixture[index]->callback_post_creation) {
            m_mixture[index]->callback_post_creation->PostCreation(bodyPtr);
        }

        m_bodies.push_back(BodyInfo(m_mixture[index]->m_type, density, size, bodyPtr));
    }

    m_totalNumBodies += (unsigned int)points.size();
}

// Write body information to a CSV file
void Generator::writeObjectInfo(const std::string& filename) {
    CSV_writer csv;

    for (int i = 0; i < m_bodies.size(); i++) {
        csv << m_bodies[i].m_type;
        csv << m_bodies[i].m_body->GetPos() << m_bodies[i].m_size;
        csv << m_bodies[i].m_density << m_bodies[i].m_body->GetMass();

        switch (m_system->GetContactMethod()) {
            case ChMaterialSurfaceBase::DVI: {
                std::shared_ptr<ChMaterialSurface> mat = m_bodies[i].m_body->GetMaterialSurface();
                csv << mat->GetSfriction() << mat->GetCohesion();
            } break;
            case ChMaterialSurfaceBase::DEM: {
                std::shared_ptr<ChMaterialSurfaceDEM> mat = m_bodies[i].m_body->GetMaterialSurfaceDEM();
                csv << mat->GetYoungModulus() << mat->GetPoissonRatio() << mat->GetSfriction() << mat->GetRestitution();
            } break;
        }

        csv << std::endl;
    }

    csv.write_to_file(filename);
}

}  // namespace utils
}  // namespace chrono
