#include "fem\ChElementGeneric.h"

namespace chrono{
	namespace fem{

/// Class for all 3-Dimensional elements 

class ChApi ChElement3D : public ChElementGeneric
{
protected:
	double Volume;

public:
	/// Computes the volume of the element (and stores the value in this->volume)
	double ComputeVolume(){return 1;};
	double GetVolume() {return Volume;}


};




class ChApi ChTetrahedron : public ChElement3D
{
protected:
	
public:
	int ID;

};





	}//___end of namespace fem___
}//___end of namespace chrono___