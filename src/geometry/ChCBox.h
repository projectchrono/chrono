#ifndef CHC_BOX_H
#define CHC_BOX_H

//////////////////////////////////////////////////
//  
//   ChCBox.h
// 
//   Header for the box geometry, used for collision
//   detection an other stuff.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <math.h>

#include "geometry/ChCGeometry.h"


namespace chrono 
{
namespace geometry 
{


#define CH_GEOCLASS_BOX   3

///
/// A box.
/// Geometric object for collisions and such.
///

class ChBox : public ChGeometry{

							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBox,ChGeometry);

public:

		//
		// CONSTRUCTORS
		//

	ChBox() 
				{
					Pos= VNULL;
					Size = VNULL;
					Rot.Set33Identity();
				};
					
			/// Build from pos, rotation, size
	ChBox(Vector& mpos, ChMatrix33<>& mrot, Vector& msize) 
				{
					Pos = mpos; Size = msize;
					Rot.CopyFromMatrix(mrot);
				}

			/// Build from first corner and three other neighbouring corners
	ChBox(Vector& mC0, Vector& mC1,Vector& mC2,Vector& mC3);


	ChBox(ChBox & source)
				{
					Copy(&source);
				}

	void Copy (ChBox* source) 
				{
					Pos = source->Pos;
					Size = source->Size;
					Rot.CopyFromMatrix(*source->GetRotm());
				};

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChBox(); 
					mgeo->Copy(this); return mgeo;
				};

		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_BOX;};

	virtual void GetBoundingBox(double& xmin, double& xmax, 
					    double& ymin, double& ymax, 
						double& zmin, double& zmax, 
						ChMatrix33<>* bbRot = NULL);
	
			/// Computes the baricenter of the box
	virtual Vector Baricenter() {return Pos;};

			/// Computes the covariance matrix for the box
	virtual void CovarianceMatrix(ChMatrix33<>& C);

			/// Evaluate position in cube volume
	virtual void Evaluate(Vector& pos, 
						const double parU, 
						const double parV = 0., 
						const double parW = 0.);

			/// This is a solid
	virtual int GetManifoldDimension() {return 3;}


		//
		// CUSTOM FUNCTIONS
		//
	
					// Access data
	ChMatrix33<>* GetRotm() {return &Rot;};
	Vector& GetPos() {return Pos;};
	Vector& GetSize(){return Size;};
	
					// Get the 8 corner points, translated and rotated
	Vector GetP1();
	Vector GetP2();
	Vector GetP3();
	Vector GetP4();
	Vector GetP5();
	Vector GetP6();
	Vector GetP7();
	Vector GetP8();
					/// Get the n-th corner point, with ipoint = 1...8
	Vector GetPn(int ipoint);

					/// Get the volume (assuming no scaling in Rot matrix)
	double GetVolume() {return Size.x*Size.y*Size.z*8.0;};

		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 



		//
		// DATA
		//


			/// Rotation of box
	ChMatrix33<> Rot;
			/// Position of center
	Vector Pos;
			/// Hemi size (extension of box from center to corner)
	Vector Size;
};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
