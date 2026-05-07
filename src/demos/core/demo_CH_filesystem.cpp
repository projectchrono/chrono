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

#include "chrono/core/ChDataPath.h"
#include "chrono/input_output/ChWriterCSV.h"

using namespace std::filesystem;
using namespace chrono;
using std::cout;
using std::endl;

int main(int argc, char** argv) {
#if defined(_WIN32)
    path path1("C:\\dir 1\\dir 2\\");
#else
    path path1("/dir 1/dir 2/");
#endif

    path path2("dir 3");

    cout << exists(path1) << endl;
    cout << path1 << endl;
    cout << (path1 / path2) << endl;
    cout << (path1 / path2).parent_path() << endl;
    cout << (path1 / path2).parent_path().parent_path() << endl;
    cout << (path1 / path2).parent_path().parent_path().parent_path() << endl;
    cout << (path1 / path2).parent_path().parent_path().parent_path().parent_path() << endl;
    cout << path().parent_path() << endl;
    cout << endl;

    // Absolute path of current directory
    cout << "Current directory = " << absolute(path(".")) << endl;
    cout << endl;

    // Create output directory and output file
    std::string out_dir = GetChronoOutputPath() + "DEMO_FILESYSTEM";
    std::string out_file = out_dir + "/foo.txt";

    cout << "Create output directory;  out_dir = " << out_dir << endl;
    cout << "  out_dir exists? " << exists(path(out_dir)) << endl;
    if (create_directory(path(out_dir))) {
        cout << "  ...Created output directory" << endl;
    } else {
        cout << "  ...Error creating output directory" << endl;
        return 1;
    }
    cout << "  out_dir exists? " << exists(path(out_dir)) << endl;
    cout << "  out_dir is directory? " << is_directory(path(out_dir)) << endl;
    cout << "  path of out_dir: " << path(out_dir) << endl;
    cout << "  abs. path of out_dir: " << absolute(path(out_dir)) << endl;
    cout << endl;

    cout << "Create output file;  out_file = " << out_file << endl;
    cout << "  out_file exists? " << exists(path(out_file)) << endl;
    ChWriterCSV csv(",");
    csv << ChVector3d(1, 2, 3) << ChQuaternion<>(1, 0, 0, 0) << endl;
    csv.WriteToFile(out_file);
    cout << "  ...Created output file" << endl;
    cout << "  out_file exists? " << exists(path(out_file)) << endl;
    cout << "  out_file is file? " << is_regular_file(path(out_file)) << endl;
    cout << "  path of out_file: " << path(out_file) << endl;
    cout << "  file location:    " << path(out_file).parent_path() << endl;
    cout << "  abs. path of out_file: " << absolute(path(out_file)) << endl;
    cout << endl;

    // Other tests

    cout << "some/path.ext:operator==() = " << (path("some/path.ext") == path("some/path.ext")) << endl;
    cout << "some/path.ext:operator==() (unequal) = " << (path("some/path.ext") == path("another/path.ext")) << endl;

    cout << "nonexistant:exists = " << exists(path("nonexistant")) << endl;
    cout << "nonexistant:is_file = " << is_regular_file(path("nonexistant")) << endl;
    cout << "nonexistant:is_directory = " << is_directory(path("nonexistant")) << endl;
    cout << "nonexistant:filename = " << path("nonexistant").filename() << endl;
    cout << "nonexistant:extension = " << path("nonexistant").extension().string() << endl;
    cout << endl;

    cout << "out_file:exists = " << exists(path(out_file)) << endl;
    cout << "out_file:is_file = " << is_regular_file(path(out_file)) << endl;
    cout << "out_file:is_directory = " << is_directory(path(out_file)) << endl;
    cout << "out_file:filename = " << path(out_file).filename() << endl;
    cout << "out_file:stem = " << path(out_file).stem() << endl;
    cout << "out_file:extension = " << path(out_file).extension().string() << endl;
    cout << "out_file:make_absolute = " << absolute(path(out_file)) << endl;
    cout << endl;

    cout << "out_dir:exists = " << exists(path(out_dir)) << endl;
    cout << "out_dir:is_file = " << is_regular_file(path(out_dir)) << endl;
    cout << "out_dir:is_directory = " << is_directory(path(out_dir)) << endl;
    cout << "out_dir:filename = " << path(out_dir).filename() << endl;
    cout << "out_dir:stem = " << path(out_dir).stem() << endl;
    cout << "out_dir:extension = " << path(out_dir).extension().string() << endl;
    cout << "out_dir:make_absolute = " << absolute(path(out_dir)) << endl;
    cout << endl;

    return 0;
}
