// Copyright (C) 2022-2023 Adam Lugowski. All rights reserved.
// Use of this source code is governed by the BSD 2-clause license found in the LICENSE.txt file.
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include "../fast_matrix_market.hpp"

#include <armadillo>

namespace fast_matrix_market {

    /**
     * Read a Matrix Market file into an Armadillo dense matrix (and header).
     */
    template <typename VT>
    void read_matrix_market_arma(std::istream &instream,
                                 matrix_market_header& header,
                                 arma::Mat<VT>& m,
                                 const read_options& options = {}) {
        // Armadillo dense matrix is column-major
        // https://arma.sourceforge.net/docs.html#Mat
        read_matrix_market_array(instream, header, m, col_major, options);
        m.reshape(header.nrows, header.ncols);
    }

    /**
     * Read a Matrix Market file into an Armadillo dense matrix.
     */
    template <typename VT>
    void read_matrix_market_arma(std::istream &instream, arma::Mat<VT>& m, const read_options& options = {}) {
        matrix_market_header header;
        read_matrix_market_arma(instream, header, m, options);
    }

    /**
     * Read a Matrix Market file into an Armadillo sparse matrix (and header).
     */
    template <typename VT>
    void read_matrix_market_arma(std::istream &instream,
                                 matrix_market_header& header,
                                 arma::SpMat<VT>& m,
                                 const read_options& options = {}) {
        arma::uvec rows;
        arma::uvec cols;
        arma::Col<VT> vals;

        fast_matrix_market::read_matrix_market_triplet(instream, header, rows, cols, vals, options);

        arma::umat locations = join_rows(rows, cols).t();

        // will call moving operator=(), which will use steal_mem() instead of making a copy
        m = arma::SpMat<VT>(locations, vals, header.nrows, header.ncols);
    }

    /**
     * Read a Matrix Market file into an Armadillo sparse matrix.
     */
    template <typename VT>
    void read_matrix_market_arma(std::istream &instream, arma::SpMat<VT>& m, const read_options& options = {}) {
        matrix_market_header header;
        read_matrix_market_arma<VT>(instream, header, m, options);
    }

    /**
     * Write an Armadillo dense matrix to a Matrix Market file.
     */
    template <typename VT>
    void write_matrix_market_arma(std::ostream &os,
                                  const arma::Mat<VT>& m,
                                  const write_options& options = {},
                                  matrix_market_header header = {}) {
        header.nrows = m.n_rows;
        header.ncols = m.n_cols;
        write_matrix_market_array(os, header, m, col_major, options);
    }

    /**
     * Write an Armadillo sparse matrix to a Matrix Market file.
     */
    template <typename VT>
    void write_matrix_market_arma(std::ostream &os,
                                  const arma::SpMat<VT>& m,
                                  const write_options& options = {},
                                  matrix_market_header header = {}) {
        header.nrows = m.n_rows;
        header.ncols = m.n_cols;
        header.nnz = m.n_nonzero;

        header.object = matrix;
        if (header.field != pattern && options.fill_header_field_type) {
            header.field = get_field_type((const VT *) nullptr);
        }
        header.format = coordinate;

        write_header(os, header, options);

        line_formatter<arma::uword, VT> lf(header, options);
        auto formatter = csc_formatter(lf,
                                       m.col_ptrs, m.col_ptrs + m.n_cols,  // explicitly no +1
                                       m.row_indices, m.row_indices + m.n_nonzero,
                                       m.values, header.field == pattern ? m.values : m.values + m.n_nonzero,
                                       false);
        write_body(os, formatter, options);
    }
}
