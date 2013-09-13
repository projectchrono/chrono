#include "ChElementHexa_8.h"


namespace chrono
{
namespace fem
{



ChElementHexa_8::ChElementHexa_8()
{
	nodes.resize(8);
	StiffnessMatrix.Resize(24,24);
	this->ir = new ChGaussIntegrationRule;
	this->SetDefaultIntegrationRule();
}


ChElementHexa_8::~ChElementHexa_8()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
