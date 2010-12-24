#ifndef CHFILEUTILS_H
#define CHFILEUTILS_H

//////////////////////////////////////////////////
//
//   ChFileutils.h
//
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChApiCE.h"

namespace chrono
{

///
/// Class with some static functions to manipulate
/// file names and paths. ***TO DO*** use more modern programming style!
///

class ChApi ChFileutils {
public:
			/// Set extension on a file identifier.
			///   - force=1 forces change even if fid already has an extension
			///   - force=0 does not change the extension if there already is one
	static void Change_file_ext (char fid1[], char fid[], char ext[], int force);

			/// Cut off extension on a file identifier.
	static void Cut_file_ext (char fid[]);


			/// Get extension on a file identifier.
	static void Get_file_ext (char fid[], char ext[]);



			/// Get file size.
	static int Get_file_size (char fname[]);

};


} // END_OF_NAMESPACE____


#endif
