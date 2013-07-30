#include "fem/ChElementHexa_20.h"


namespace chrono
{
namespace fem
{



ChElementHexa_20::ChElementHexa_20()
{
	nodes.resize(20);
	this->StiffnessMatrix.Resize(60,60);
	this->ir = new ChGaussIntegrationRule;
	this->SetDefaultIntegrationRule();
}


ChElementHexa_20::~ChElementHexa_20()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
