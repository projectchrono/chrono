//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHOBJSHAPEFILE_H
#define CHOBJSHAPEFILE_H

///////////////////////////////////////////////////
//
//   ChObjShapeFile.h
//
//   Class for referencing a Wavefront/Alias .obj
//   file containing a shape that can be visualized
//   in some way.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"


namespace chrono
{

/// Class for referencing a Wavefront/Alias .obj
/// file containing a shape that can be visualized
/// in some way.
/// The file is not load into this object: it
/// is simply a reference to the resource on the disk.

class ChApi ChObjShapeFile : public ChVisualization {

protected:
				//
	  			// DATA
				//
	std::string filename;	

public:
				//
	  			// CONSTRUCTORS
				//

	ChObjShapeFile () : filename("") {};

	virtual ~ChObjShapeFile () {};

				//
	  			// FUNCTIONS
				//


	std::string GetFilename() const {return filename;}
	void SetFilename(const std::string ms) {filename = ms;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
