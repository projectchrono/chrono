#ifndef CHCAMERA_H
#define CHCAMERA_H

///////////////////////////////////////////////////
//
//   ChCamera.h
//
//   Class for defining a videocamera point of view
//   with basic settings
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChAsset.h"
#include "core/ChVector.h"

namespace chrono
{

/// Class for defining a videocamera point of view
/// with basic settings

class ChApi ChCamera : public ChAsset {

private:
				//
	  			// DATA
				//
	ChVector<> position;
	ChVector<> aimpoint;
	ChVector<> upvector;
	double	angle;
	double	fov;
	double  hvratio;
	bool    isometric;

public:
				//
	  			// CONSTRUCTORS
				//

	ChCamera () 
		{
			position = ChVector<>(0,1,1);
			aimpoint = VNULL;
			upvector = VECT_Y;
			angle = 50;
			fov = 3;
			hvratio = 4./3.;
			isometric = false;
		};

	virtual ~ChCamera () {};

				//
	  			// FUNCTIONS
				//

			/// Sets the position of the observer (eye point).
			/// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
	void SetPosition(ChVector<> mv) {this->position = mv;}
			/// Gets the position of the observer (eye point).
			/// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
	ChVector<> GetPosition() {return this->position;}

			/// Sets the position of the target point (aim point).
			/// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
	void SetAimPoint(ChVector<> mv) {this->aimpoint = mv;}
			/// Gets the position of the target point (aim point).
			/// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
	ChVector<> GetAimPoint() {return this->aimpoint;}

			/// Sets the position of the 'up' direction of the camera (default is vertical, VECT_Y)
			/// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
	void SetUpVector(ChVector<> mv) {this->upvector = mv.GetNormalized();}
			/// Gets the position of the 'up' direction of the camera (default is vertical, VECT_Y)
			/// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
	ChVector<> GetUpVector() {return this->upvector;}

			/// Sets the opening angle of the lenses, in degrees, on horizontal direction. 
			/// Ex. 60=wide angle, 30=tele, etc.
	void SetAngle(double mdeg) {this->angle = mdeg;}
			/// Gets the opening angle of the lenses, in degrees, on horizontal direction
	double GetAngle() {return   this->angle;}

			/// Sets the Field Of View, if the visualization supports focusing.
	void SetFOV(double mf) {this->fov = mf;}
			/// Gets the Field Of View, if the visualization supports focusing.
	double GetFOV() {return this->fov;}

			/// Sets the Horizontal/Vertical size ratio. Default = 4/3.
			/// For instance, 4/3 can be used if rendering to 800x600 images with square pixels.
	void SetHVratio(double mf) {this->hvratio = mf;}
			/// Gets the Horizontal/Vertical size ratio. Default = 4/3.
			/// For instance, 4/3 can be used if rendering to 800x600 images with square pixels.
	double GetHVratio() {return this->hvratio;}

			/// Set to 'true' if you want to disable perspective and get a 'flat' view.
			/// This must be supported by the visualization system. By default is 'false'.
	void SetOrthographic(bool mb)  {this->isometric = mb;}
			/// If 'true' it means that perspective projection is disabled and you get a 'flat' view.
			/// This must be supported by the visualization system. By default is 'false'.
	bool GetOrthographic() {return this->isometric;}
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
