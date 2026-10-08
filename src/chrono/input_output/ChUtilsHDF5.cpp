// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Utility functions for parsing HDF5 files.
//
// =============================================================================

#include "chrono/input_output/ChUtilsHDF5.h"

namespace chrono {

double ReadDouble(H5::H5File& file, const std::string& data_name) {
    double var;

    H5::DataSet data_set = file.openDataSet(data_name);
    H5::DataType data_type = data_set.getDataType();

    if (H5::PredType::NATIVE_FLOAT == data_type || H5::PredType::NATIVE_DOUBLE == data_type) {
        H5::DataSpace data_space = data_set.getSpace();
        hsize_t dims[2] = {0, 0};
        int rank = data_space.getSimpleExtentDims(dims);
        H5::DataSpace mspace1 = H5::DataSpace(rank, dims);
        data_set.read(&var, H5::PredType::NATIVE_DOUBLE, mspace1, data_space);
    } else {
        throw std::runtime_error("Incorrect data type");
    }

    data_set.close();

    return var;
}

ChVectorDynamic<> ReadVector(H5::H5File& file, const std::string& data_name) {
    ChVectorDynamic<> var;

    // open specific data_set
    H5::DataSet data_set = file.openDataSet(data_name);

    // Get data_space for rank and dimension
    H5::DataSpace data_space = data_set.getSpace();

    // Get number of dimensions in the data_space
    // Get and print the dimension sizes of the file data_space
    hsize_t dims[2] = {0, 0};  // data_set dimensions
    int rank = data_space.getSimpleExtentDims(dims);

    // read file into data_out 2d array
    H5::DataSpace mspace1(rank, dims);
    double* temp = new double[dims[0] * dims[1]];

    // read file info into current_pos
    data_set.read(temp, H5::PredType::NATIVE_DOUBLE, mspace1, data_space);
    var.resize(dims[0] * dims[1]);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(dims[0] * dims[1]); i++) {
        var[i] = temp[i];
    }

    data_set.close();
    delete[] temp;

    return var;
}

ChMatrixDynamic<> ReadMatrix(H5::H5File& file, const std::string& data_name) {
    ChMatrixDynamic<> var;

    // open specific data_set
    H5::DataSet data_set = file.openDataSet(data_name);

    // Get data_space for rank and dimension
    H5::DataSpace data_space = data_set.getSpace();

    hsize_t dims[2] = {0, 0};
    int rank = data_space.getSimpleExtentDims(dims);

    // read file into data_out 2d array
    H5::DataSpace mspace(rank, dims);
    // rirf_dims[0] is number of rows, rirf_dims[1] is number of columns, rirf_dims[2] is number of matrices
    double* temp = new double[dims[0] * dims[1]];

    // read file info into data_out, a 2d array
    data_set.read(temp, H5::PredType::NATIVE_DOUBLE, mspace, data_space);

    // set var here
    var.resize(dims[0], dims[1]);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(dims[0]); i++) {
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(dims[1]); j++) {
            var(i, j) = temp[static_cast<hsize_t>(i) * dims[1] + static_cast<hsize_t>(j)];
        }
    }

    data_set.close();
    delete[] temp;

    return var;
}

}  // namespace chrono
