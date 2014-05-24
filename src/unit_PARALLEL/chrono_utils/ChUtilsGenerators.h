#ifndef CH_UTILS_GENERATORS_H
#define CH_UTILS_GENERATORS_H

#include <random>
#include <cmath>
#include <vector>
#include <utility>
#include <string>

#include "core/ChSmartpointers.h"
#include "core/ChVector.h"
#include "core/ChQuaternion.h"

#include "physics/ChSystem.h"
#include "physics/ChSystemDEM.h"
#include "physics/ChBody.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChMaterialSurface.h"
#include "physics/ChMaterialSurfaceDEM.h"

#include "chrono_parallel/ChSystemParallel.h"

#include "chrono_utils/ChApiUtils.h"
#include "chrono_utils/ChUtilsCommon.h"
#include "chrono_utils/ChUtilsSamplers.h"
#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsInputOutput.h"


namespace chrono {
namespace utils {


// Enumeration of various geometric shapes
enum MixtureType {
	SPHERE,
	ELLIPSOID,
	BOX//,
	//CYLINDER,
	//CONE
};


// Forward declarations
class Generator;
class MixtureIngredient;


// Shortcut for a smart pointer to MixtureIngredient
typedef ChSmartPtr<MixtureIngredient>   MixtureIngredientPtr;


// -------------------------------------------------------------------------------
// MixtureIngredient
//
// This class encapsulates an ingredient of one of the supported types in a
// mixture. Such an object is defined size, mass properties and material
// properties, all of which can be constant for all mixture components of this
// type or else obtained from associated truncated normal distributions.
// In addition, a mixture ingredient defines the ratio of this particular type in
// the containing mixture.
// -------------------------------------------------------------------------------
class CH_UTILS_API MixtureIngredient {
public:
	~MixtureIngredient();

	void setDefaultMaterialDVI(const ChSharedPtr<ChMaterialSurface>& mat);
	void setDefaultMaterialDEM(const ChSharedPtr<ChMaterialSurfaceDEM>& mat);
	void setDefaultDensity(double density);
	void setDefaultSize(const ChVector<>& size);

	void setDistributionFriction(float friction_mean, float friction_stddev, float friction_min, float friction_max);
	void setDistributionCohesion(float cohesion_mean, float cohesion_stddev, float cohesion_min, float cohesion_max);
	void setDistributionYoung(float young_mean, float young_stddev, float young_min, float young_max);
	void setDistributionPoisson(float poisson_mean, float poisson_stddev, float poisson_min, float poisson_max);
	void setDistributionDissipation(float dissipation_mean, float dissipation_stddev, float dissipation_min, float dissipation_max);
	void setDistributionDensity(double density_mean, double density_stddev, double density_min, double density_max);
	void setDistributionSize(double size_mean, double size_stddev, const ChVector<>& size_min, const ChVector<>& size_max);

private:
	MixtureIngredient(Generator* generator, MixtureType type, double ratio);

	void                               freeMaterialDist();
	ChVector<>                         getSize();
	double                             getDensity();
	void                               calcGeometricProps(const ChVector<>& size, double& volume, ChVector<>& gyration);
	double                             calcMinSeparation();

	void                               setMaterialProperties(ChSharedPtr<ChMaterialSurface>& mat);
	void                               setMaterialProperties(ChSharedPtr<ChMaterialSurfaceDEM>& mat);

	Generator*                         m_generator;

	MixtureType                        m_type;
	double                             m_ratio;
	double                             m_cumRatio;

	ChSharedPtr<ChMaterialSurface>     m_defMaterialDVI;
	ChSharedPtr<ChMaterialSurfaceDEM>  m_defMaterialDEM;

	float                              m_minFriction, m_maxFriction;
	float                              m_minCohesion, m_maxCohesion;
	float                              m_minYoung, m_maxYoung;
	float                              m_minPoisson, m_maxPoisson;
	float                              m_minDissipation, m_maxDissipation;
	std::normal_distribution<float>*   m_frictionDist;
	std::normal_distribution<float>*   m_cohesionDist;
	std::normal_distribution<float>*   m_youngDist;
	std::normal_distribution<float>*   m_poissonDist;
	std::normal_distribution<float>*   m_dissipationDist;

	double                             m_defDensity;
	double                             m_minDensity, m_maxDensity;
	std::normal_distribution<>*        m_densityDist;

	ChVector<>                         m_defSize;
	ChVector<>                         m_minSize, m_maxSize;
	std::normal_distribution<>*        m_sizeDist;

friend class Generator;
};


// -------------------------------------------------------------------------------
// Generator
//
// This class encapsulates functionality for generating sets of bodies with
// positions drawn from a specified sampler and various mixture properties.
// -------------------------------------------------------------------------------
class CH_UTILS_API Generator {
public:
	typedef Types<double>::PointVector PointVector;

	Generator(ChSystem* system);
	~Generator();

	// Add a new mixture ingredient of the specified type and in the given ratio
	// (note that the ratios are normalized before creating bodies)
	MixtureIngredientPtr& AddMixtureIngredient(MixtureType type, double ratio);

	// Get/Set the identifier that will be assigned to the next body.
	// Identifiers are incremented for successively created bodies.
	int  getBodyIdentifier() const   {return m_crtBodyId;}
	void setBodyIdentifier(int id)   {m_crtBodyId = id;}

	// Create bodies, according to the current mixture setup, with initial positions
	// given by the specified sampler in the box domain specified by 'pos' and 'hdims'.
	// Optionally, a constant inital linear velocity can be set for all created bodies.
	void createObjectsBox(SamplingType sType, double dist,
	                      const ChVector<>& pos, const ChVector<>& hdims,
	                      const ChVector<>& vel = ChVector<>(0, 0, 0));

	// Create bodies, according to the current mixture setup, with initial positions
	// given by the specified sampler in the cylinder domain specified by 'pos', 'radius'
	// and 'halfHeight'.
	// Optionally, a constant inital linear velocity can be set for all created bodies.
	void createObjectsCylinderX(SamplingType sType, double dist,
	                            const ChVector<>& pos, float radius, float halfHeight,
	                            const ChVector<>& vel = ChVector<>(0, 0, 0));
	void createObjectsCylinderY(SamplingType sType, double dist,
	                            const ChVector<>& pos, float radius, float halfHeight,
	                            const ChVector<>& vel = ChVector<>(0, 0, 0));
	void createObjectsCylinderZ(SamplingType sType, double dist,
	                            const ChVector<>& pos, float radius, float halfHeight,
	                            const ChVector<>& vel = ChVector<>(0, 0, 0));


	// Write information about the bodies created so far to the specified file (CSV format)
	void writeObjectInfo(const std::string& filename);

	int    getTotalNumBodies() const {return m_totalNumBodies;}
	double getTotalMass() const      {return m_totalMass;}
	double getTotalVolume() const    {return m_totalVolume;}

private:
	struct BodyInfo {
		BodyInfo(MixtureType t, double density, const ChVector<>& size, const ChSharedPtr<ChBody>& b)
		:	m_type(t),
			m_density(density),
			m_size(size),
			m_body(b)
		{}
		MixtureType           m_type;
		double                m_density;
		ChVector<>            m_size;
		ChSharedPtr<ChBody>   m_body;
	};

	void    normalizeMixture();
	int     selectIngredient();
	double  calcMinSeparation(double sep);
	void    createObjects(const PointVector& points,
	                      const ChVector<>&  vel);

	ChSystem*                          m_system;
	SystemType                         m_sysType;

	std::uniform_real_distribution<>   m_mixDist;

	std::vector<MixtureIngredientPtr>  m_mixture;
	std::vector<BodyInfo>              m_bodies;
	int                                m_totalNumBodies;
	double                             m_totalMass;
	double                             m_totalVolume;

	int                                m_crtBodyId;

friend class  MixtureIngredient;
};


} // end namespace utils
} // end namespace chrono


#endif
