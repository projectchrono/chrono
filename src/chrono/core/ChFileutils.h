// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHFILEUTILS_H
#define CHFILEUTILS_H

#include "chrono/core/ChApiCE.h"

namespace chrono {

///
/// Class with some static functions to manipulate
/// file names and paths. ***TO DO*** use more modern programming style!
///

class ChApi ChFileutils {
  public:
    /// Set extension on a file identifier.
    ///   - force=1 forces change even if fid already has an extension
    ///   - force=0 does not change the extension if there already is one
    static void Change_file_ext(char* fid1, const char* fid, const char* ext, int force);

    /// Cut off extension on a file identifier.
    static void Cut_file_ext(char* fid);

    /// Get extension on a file identifier.
    static void Get_file_ext(const char* fid, char* ext);

    /// Get file size.
    static int Get_file_size(const char* fname);

    /// Create a directory
    /// Return 0 if successful, 1 if the directory exists, -1 otherwise
    static int MakeDirectory(const char* dirname);
};

}  // end namespace chrono

#endif
