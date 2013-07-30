#include "fem/ChGaussPoint.h"

namespace chrono
{
namespace fem
{

class  ChGaussIntegrationRule
{
private:
	

public:
	ChGaussIntegrationRule();
	~ChGaussIntegrationRule();
	
	virtual void SetIntOnLine(int nPoints, std::vector<ChGaussPoint*>* GpVector){};
	virtual void SetIntOnTriangle(int nPoints, std::vector<ChGaussPoint*>* GpVector){};
	virtual void SetIntOnSquare(int nPoints, std::vector<ChGaussPoint*>* GpVector){};
	virtual void SetIntOnCube(int nPoints, std::vector<ChGaussPoint*>* GpVector);

};



}//__end mamespace fem
}//__end namespace chrono