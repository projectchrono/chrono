#include "ChElementTetra_10.h"


namespace chrono
{
namespace fem
{



ChElementTetra_10::ChElementTetra_10()
{
	nodes.resize(10);
	MatrB.resize(4);		//standard: 4 integration points
	MatrB[0].Resize(6,30);
	MatrB[1].Resize(6,30);
	MatrB[2].Resize(6,30);
	MatrB[3].Resize(6,30);
	this->StiffnessMatrix.Resize(30,30);
}


ChElementTetra_10::~ChElementTetra_10()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
