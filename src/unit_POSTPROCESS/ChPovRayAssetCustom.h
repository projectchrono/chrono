#ifndef CHPOVRAYASSETCUSTOM_H
#define CHPOVRAYASSETCUSTOM_H

//////////////////////////////////////////////////
//
//   ChPovRayAssetCustom.h
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <string>
#include "ChPostProcessBase.h"
#include "assets/ChVisualization.h"

namespace chrono{
 namespace postprocess{

		
/// Class for adding custom commands to POVray objects saved with 
/// ChBody items; for example you can specify pigment{...} or
/// texture{...} stuff here.

class ChApiPostProcess ChPovRayAssetCustom : public ChAsset {

protected:
				//
	  			// DATA
				//
	std::string custom_command;
	float foo;
public:
				//
	  			// CONSTRUCTORS
				//

	ChPovRayAssetCustom () { custom_command = ""; };

	virtual ~ChPovRayAssetCustom () {};

				//
	  			// FUNCTIONS
				//

		// Get custom command. This information could be used by visualization postprocessing. 
	std::string GetCommands() const {return custom_command;}

		// Set the custom command for POVray.
		// If you attach this ChPovRayAssetCustom asset to a ChBody, the postprocessing
		// will create a object{...} or union{...} block, it will place the geometries defined
		// by other assets of the body, then it will put the command block of this asset, as it is.
		// For example you can specify pigment{...} or texture{...} stuff here.
	//void SetCommands (const std::string& mcomm) {custom_command = mcomm;}
	void SetCommands (char mcomm[]);

};



 } // end namespace
} // end namespace

#endif