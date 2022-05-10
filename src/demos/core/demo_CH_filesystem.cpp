// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo for using the third-party library 'filesystem' for file and directory
// operations.
// Note that the version distributed with Chrono includes some  extensions:
//   path::stem
//        extracts the stem (basename) for a file, by stripping extension
//        from the filename.
//   create_subdirectory
//        utility function for creating a hierarchy of nested directories,
//        creating all intermediate subdirectories as needed.
//
// =============================================================================

#include <iostream>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using namespace filesystem;
using namespace chrono;
using std::cout;
using std::endl;

int main(int argc, char** argv) {
#if defined(WIN32)
    path path1("C:\\dir 1\\dir 2\\");
#else
    path path1("/dir 1/dir 2/");
#endif

    path path2("dir 3");

    cout << path1.exists() << endl;
    cout << path1 << endl;
    cout << (path1 / path2) << endl;
    cout << (path1 / path2).parent_path() << endl;
    cout << (path1 / path2).parent_path().parent_path() << endl;
    cout << (path1 / path2).parent_path().parent_path().parent_path() << endl;
    cout << (path1 / path2).parent_path().parent_path().parent_path().parent_path() << endl;
    cout << path().parent_path() << endl;
    cout << endl;

    // Absolute path of current directory
    cout << "Current directory = " << path(".").make_absolute() << endl;
    cout << endl;

    // Create output directory and output file
    std::string out_dir = GetChronoOutputPath() + "DEMO_FILESYSTEM";
    std::string out_file = out_dir + "/foo.txt";

    cout << "Create output directory;  out_dir = " << out_dir << endl;
    cout << "  out_dir exists? " << path(out_dir).exists() << endl;
    if (create_directory(path(out_dir))) {
        cout << "  ...Created output directory" << endl;
    } else {
        cout << "  ...Error creating output directory" << endl;
        return 1;
    }
    cout << "  out_dir exists? " << path(out_dir).exists() << endl;
    cout << "  out_dir is directory? " << path(out_dir).is_directory() << endl;
    cout << "  path of out_dir: " << path(out_dir) << endl;
    cout << "  abs. path of out_dir: " << path(out_dir).make_absolute() << endl;
    cout << endl;

    cout << "Create output file;  out_file = " << out_file << endl;
    cout << "  out_file exists? " << path(out_file).exists() << endl;
    utils::CSV_writer csv(",");
    csv << ChVector<>(1, 2, 3) << ChQuaternion<>(1, 0, 0, 0) << endl;
    csv.write_to_file(out_file);
    cout << "  ...Created output file" << endl;
    cout << "  out_file exists? " << path(out_file).exists() << endl;
    cout << "  out_file is file? " << path(out_file).is_file() << endl;
    cout << "  path of out_file: " << path(out_file) << endl;
    cout << "  file location:    " << path(out_file).parent_path() << endl;
    cout << "  abs. path of out_file: " << path(out_file).make_absolute() << endl;
    cout << endl;

    // Create hierarchy of nested directories
#if defined(WIN32)
    std::string nested = out_dir + "\\child\\grandchild";
#else
    std::string nested = out_dir + "/child/grandchild";
#endif
    cout << "Create nested directories;  nested = " << nested << endl;
    cout << "  nested (as path)   = " << path(nested) << endl;
    cout << "  length of nested   = " << path(nested).length() << endl;
    cout << "  nested exists? " << path(nested).exists() << endl;
    if (create_subdirectory(path(nested))) {
        cout << "  ...Created nested subdirectories" << endl;
    } else {
        cout << "  ...Error creating subdirectories" << endl;
        return 1;
    }
    cout << "  nested exists? " << path(nested).exists() << endl;
    cout << "  nested is directory? " << path(nested).is_directory() << endl;
    cout << "  path of nested: " << path(nested) << endl;
    cout << "  abs. path of nested: " << path(nested).make_absolute() << endl;
    cout << endl;

    // Other tests

    cout << "some/path.ext:operator==() = " << (path("some/path.ext") == path("some/path.ext")) << endl;
    cout << "some/path.ext:operator==() (unequal) = " << (path("some/path.ext") == path("another/path.ext")) << endl;

    cout << "nonexistant:exists = " << path("nonexistant").exists() << endl;
    cout << "nonexistant:is_file = " << path("nonexistant").is_file() << endl;
    cout << "nonexistant:is_directory = " << path("nonexistant").is_directory() << endl;
    cout << "nonexistant:filename = " << path("nonexistant").filename() << endl;
    cout << "nonexistant:extension = " << path("nonexistant").extension() << endl;
    cout << endl;

    cout << "out_file:exists = " << path(out_file).exists() << endl;
    cout << "out_file:is_file = " << path(out_file).is_file() << endl;
    cout << "out_file:is_directory = " << path(out_file).is_directory() << endl;
    cout << "out_file:filename = " << path(out_file).filename() << endl;
    cout << "out_file:stem = " << path(out_file).stem() << endl;
    cout << "out_file:extension = " << path(out_file).extension() << endl;
    cout << "out_file:make_absolute = " << path(out_file).make_absolute() << endl;
    cout << endl;

    cout << "out_dir:exists = " << path(out_dir).exists() << endl;
    cout << "out_dir:is_file = " << path(out_dir).is_file() << endl;
    cout << "out_dir:is_directory = " << path(out_dir).is_directory() << endl;
    cout << "out_dir:filename = " << path(out_dir).filename() << endl;
    cout << "out_dir:stem = " << path(out_dir).stem() << endl;
    cout << "out_dir:extension = " << path(out_dir).extension() << endl;
    cout << "out_dir:make_absolute = " << path(out_dir).make_absolute() << endl;
    cout << endl;

    cout << "resolve(out_file) = " << resolver().resolve(out_file) << endl;
    cout << "resolve(nonexistant) = " << resolver().resolve("nonexistant") << endl;
    return 0;
}
