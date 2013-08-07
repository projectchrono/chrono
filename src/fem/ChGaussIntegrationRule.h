#include "fem/ChGaussPoint.h"

namespace chrono
{
namespace fem
{

	
/// Class for the management of the Gauss Quadrature in 1D, 2D or 3D space.
/// Integration is done over the canonical interval (-1,...,+1), so the position of
/// each gauss point is expressed in terms of natural coordinates.
/// Input: number of gauss points, pointer to the vector 'GpVector'. 
/// The vector will be resized according to the number of points required.

class  ChGaussIntegrationRule
{
private:
	

public:
	ChGaussIntegrationRule();
	~ChGaussIntegrationRule();
	
	virtual void SetIntOnLine(int nPoints, std::vector<ChGaussPoint*>* GpVector){};
	virtual void SetIntOnTriangle(int nPoints, std::vector<ChGaussPoint*>* GpVector);
	virtual void SetIntOnSquare(int nPoints, std::vector<ChGaussPoint*>* GpVector);
	virtual void SetIntOnCube(int nPoints, std::vector<ChGaussPoint*>* GpVector);

};



}//__end mamespace fem
}//__end namespace chrono