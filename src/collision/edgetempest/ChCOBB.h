#ifndef CHC_OBB_H
#define CHC_OBB_H

//////////////////////////////////////////////////
//  
//   ChCOBB.h
//
//   Header for the Object oriented Bounding Box
//   class. Useful for BSP collision queries.
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
#include "ChCCompile.h"
#include <vector>

using namespace chrono::geometry;

namespace chrono 
{
namespace collision 
{


///
/// Class for Object aligned Bounding Box (OBB)
/// This class is useful for collision tests based on
/// trees of bounding volumes.
///


class CHOBB
{
public:

			// DATA

  ChMatrix33<> Rot;		///< orientation of RSS & OBB

  Vector To;			///< position of center of obb

  Vector  d;			///< (half) dimensions of obb


  int first_child;      ///< positive value is index of first_child OBB,
                        ///  negative value is -(index + 1) of geometry in geom.list



			// FUNCTIONS		
  CHOBB();
  ~CHOBB();
						// Copy constructor 
  CHOBB(const CHOBB& other) { To=other.To; d=other.d; first_child=other.first_child; Rot.CopyFromMatrix(other.Rot);}

						/// Returns 1 if this is a leaf in BV tree
  inline int IsLeaf()    { return first_child < 0; }
						
						/// Returns the index of the geometry in the geometry vector of the model
						/// Caution: must be used only if IsLeaf()==true
  inline int GetGeometryIndex()	{return (-first_child -1);}

						/// Returns the index of the first child box, in box vector of the model
						/// Caution: must be used only if IsLeaf()==false
  inline int GetFirstChildIndex()	{return first_child;}

 						/// Returns the index of the second child box, in box vector of the model
						/// Caution: must be used only if IsLeaf()==false
  inline int GetSecondChildIndex()	{return first_child+1;}


						/// Returns the size of the OBB (as sphere radius)
  double   GetSize() { return (d.x*d.x + d.y*d.y + d.z*d.z);}
  
  						/// Given a rotation matrix O which tells the direction of the
						/// axis, and a list of geometric object, this function recomputes the
						/// bounding box in order to enclose 'ngeos' geometries, from index 'firstgeo' 
						/// Box may be also 'inflated' by a thinckness='envelope' 
  void     FitToGeometries(ChMatrix33<>& O, std::vector<ChGeometry*> mgeos, int firstgeo, int ngeos, double envelope);

  						/// Find if two box OBB are overlapping, given relative 
						/// rotation matrix B, relative translation T, and half-sizes of the two OBB
  static bool OBB_Overlap(ChMatrix33<>& B, Vector T, Vector a, Vector b);

						/// Find if two box OBB are overlapping, given relative 
						/// rotation matrix B, relative translation T, and the two OBB
  static bool OBB_Overlap(ChMatrix33<>& B, Vector T, CHOBB *b1, CHOBB *b2);

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____




#endif


