#ifndef CHASSETLEVEL_H
#define CHASSETLEVEL_H

///////////////////////////////////////////////////
//
//   ChAssetLevel.h
//
//   Base class grouping assets
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
#include "core/ChFrame.h"

namespace chrono
{



/// Base class for grouping assets in a level. The
/// level is like a 'subdirectory'. A level can contain 
/// assets; amnog these, also further levels, etc. (but please 
/// avoid circular loops!)
/// A level can have custom rotation and translation respect 
/// its parent level.

class ChApi ChAssetLevel : public ChAsset {

protected:
				//
	  			// DATA
				//
	ChFrame<> levelframe;

	std::vector< ChSharedPtr<ChAsset> > assets;

public:
				//
	  			// CONSTRUCTORS
				//

	ChAssetLevel () : levelframe(chrono::CSYSNORM) { };

	virtual ~ChAssetLevel () {};

				//
	  			// FUNCTIONS
				//

		// Access the coordinate sytem information of the level, for setting/getting its position
		// and rotation respect to its parent.
	ChFrame<>& GetFrame() {return levelframe;}

		/// Access to the list of children assets.
	std::vector< ChSharedPtr<ChAsset> >& GetAssets () { return this->assets;}
	ChSharedPtr<ChAsset> GetAssetN (unsigned int num) { if (num<assets.size()) return assets[num]; else {ChSharedPtr<ChAsset> none; return none;};}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
