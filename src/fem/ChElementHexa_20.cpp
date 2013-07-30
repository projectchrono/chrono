#include "fem/ChElementHexa_20.h"


namespace chrono
{
namespace fem
{



ChElementHexa_20::ChElementHexa_20()
{
	nodes.resize(20);
	MatrB.resize(27);		//standard: 8 integration points
	MatrB[0].Resize(6,60);
	MatrB[1].Resize(6,60);
	MatrB[2].Resize(6,60);
	MatrB[3].Resize(6,60);
	MatrB[4].Resize(6,60);
	MatrB[5].Resize(6,60);
	MatrB[6].Resize(6,60);
	MatrB[7].Resize(6,60);
	this->StiffnessMatrix.Resize(60,60);
	this->ir = new ChGaussIntegrationRule;
	this->SetDefaultIntegrationRule();
}


ChElementHexa_20::~ChElementHexa_20()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
