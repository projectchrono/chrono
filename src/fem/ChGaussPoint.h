#include "physics/ChApidll.h"
#include "core/ChMath.h"
#include "lcp/ChLcpSystemDescriptor.h"
//#include "fem/ChNodeFEMbase.h"
//#include "fem/ChContinuumMaterial.h"
//#include "fem/ChGaussIntegrationRule.h"
#include "fem/ChTensors.h"

namespace chrono
{
namespace fem
{

class ChGaussPoint
{
private:
		/// Number.
    int number;
		/// Reference to parent integration rule
    //ChIntegrationRule *irule;
	    /// Local (natural) coordinates of the point
    ChVector<> LocalCoordinates;
	    /// Absolute point coordinates
    ChVector<> *coordinates;
	    /// Integration weight
    double weight;

public:		
	
		/// Matrix of partial derivatives: to obtain strain & stress
	ChMatrixDynamic<> *MatrB;
	    /// Strain tensor
    ChStrainTensor<> Strain;
	    /// Stress tensor
    ChStressTensor<> Stress;


			/// Constructor; n number of the gauss point, coordinates of the point, w weight
    ChGaussPoint(/*ChGaussIntegrationRule *ir,*/ int n, ChVector<> *coord, double w);
			/// Destructor
	virtual ~ChGaussPoint();

			/// Returns local coordinates
	ChVector<> GetLocalCoordinates() { return LocalCoordinates; }
	void setLocalCoordinates(ChVector<> &c) { LocalCoordinates = c; }

			/// Returns absolute coordinates
    ChVector<> GetCoordinates() { if ( coordinates ) { return * coordinates; } else { return LocalCoordinates; } }
    void SetCoordinates(ChVector<> &c)
		{ 
			if ( coordinates ) { * coordinates = c; } else { coordinates = new ChVector<>(c); } 
		}

		    /// Returns  integration weight of receiver
    virtual double GetWeight() { return weight; }
    void setWeight(double w) { weight = w; }

		    /// Returns number of the point
    int GetNumber() { return number; }
    
			/// Returns corresponding integration rule to receiver
//    ChIntegrationRule *giveIntegrationRule() { return irule; }
   

};



}//__end mamespace fem
}//__end namespace chrono