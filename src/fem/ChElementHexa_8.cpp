#include "fem/ChElementHexa_8.h"


namespace chrono
{
namespace fem
{



ChElementHexa_8::ChElementHexa_8()
{
	nodes.resize(8);
	MatrB.resize(8);		//standard: 8 integration points
	MatrB[0].Resize(6,24);
	MatrB[1].Resize(6,24);
	MatrB[2].Resize(6,24);
	MatrB[3].Resize(6,24);
	MatrB[4].Resize(6,24);
	MatrB[5].Resize(6,24);
	MatrB[6].Resize(6,24);
	MatrB[7].Resize(6,24);
	StiffnessMatrix.Resize(24,24);
	this->ir = new ChGaussIntegrationRule;
	this->SetDefaultIntegrationRule();
}


ChElementHexa_8::~ChElementHexa_8()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
