#include "fem\ChElementTetra_4.h"


namespace chrono
{
namespace fem
{



ChElementTetra_4::ChElementTetra_4()
{

	nodes.resize(4);
	this->MatrB.Resize(6,12);
	this->StiffnessMatrix.Resize(12,12);
}


ChElementTetra_4::~ChElementTetra_4()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
