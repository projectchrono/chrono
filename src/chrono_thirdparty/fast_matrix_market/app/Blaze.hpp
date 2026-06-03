// Copyright (C) 2022-2023 Adam Lugowski. All rights reserved.
// Use of this source code is governed by the BSD 2-clause license found in the LICENSE.txt file.
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include "../fast_matrix_market.hpp"

#include <blaze/Blaze.h>


namespace fast_matrix_market {
    /**
     * Read Matrix Market file into a sparse Blaze matrix and a header struct.
     */
    template <typename SparseMatrix, typename std::enable_if<blaze::IsSparseMatrix_v<SparseMatrix>, int>::type = 0>
    void read_matrix_market_blaze(std::istream &instream,
                                  matrix_market_header &header,
                                  SparseMatrix& mat,
                                  const read_options& options = {},
                                  typename SparseMatrix::ElementType default_pattern_value = 1) {

        typedef int64_t IT; // Blaze matrices use size_t for indexing, but allowing negative numbers allows more error checking while parsing.
        typedef typename SparseMatrix::ElementType VT;

        read_header(instream, header);
        mat.clear();
        mat.resize(header.nrows, header.ncols);

        // Read into triplets
        std::vector<IT> rows;
        std::vector<IT> cols;
        std::vector<VT> vals;

        read_matrix_market_body_triplet(instream, header, rows, cols, vals, default_pattern_value, options);

        size_t storage_nnz = vals.size();
        mat.reserve(storage_nnz);

        // Set the values into the matrix from the triplets.
        // The matrix needs to be constructed row-by-row (if row-major) or column-by-column (if col-major), in order.
        // So first sort the triplets in the appropriate order (or rather, find a sort permutation),
        // then step through the major dimension and insert values.

        bool is_row_major = blaze::IsRowMajorMatrix_v<SparseMatrix>;
        const std::vector<IT>* major;

        // Find sort permutation
        std::vector<std::size_t> perm(storage_nnz);
        std::iota(perm.begin(), perm.end(), 0);
        if (is_row_major) {
            major = &rows;
            std::sort(perm.begin(), perm.end(),
                      [&](std::size_t i, std::size_t j) {
                          if (rows[i] != rows[j])
                              return rows[i] < rows[j];
                          if (cols[i] != cols[j])
                              return cols[i] < cols[j];

                          return false;
                      });
        } else {
            major = &cols;
            std::sort(perm.begin(), perm.end(),
                      [&](std::size_t i, std::size_t j) {
                          if (cols[i] != cols[j])
                              return cols[i] < cols[j];
                          if (rows[i] != rows[j])
                              return rows[i] < rows[j];

                          return false;
                      });
        }

        // Construct the matrix.
        IT major_end = is_row_major ? header.nrows : header.ncols;
        auto perm_iter = perm.cbegin();

        for (IT major_i = 0; major_i < major_end; ++major_i) {
            while (perm_iter != perm.cend() && (*major)[*perm_iter] == major_i) {
                mat.append(rows[*perm_iter], cols[*perm_iter], vals[*perm_iter]);
                ++perm_iter;
            }

            mat.finalize(major_i);
        }

        if (perm_iter != perm.cend()) {
            throw fmm_error("Did not use all the values that were read!");
        }
    }

    /**
     * Read Matrix Market file into a dense Blaze matrix and a header struct.
     */
    template <typename DenseMatrix, typename std::enable_if<blaze::IsDenseMatrix_v<DenseMatrix>, int>::type = 0>
    void read_matrix_market_blaze(std::istream &instream,
                                  matrix_market_header &header,
                                  DenseMatrix& mat,
                                  const read_options& options = {},
                                  typename DenseMatrix::ElementType default_pattern_value = 1) {
        typedef int64_t IT; // Blaze matrices use size_t for indexing, but allowing negative numbers allows more error checking while parsing.
        typedef typename DenseMatrix::ElementType VT;

        read_header(instream, header);
        mat.resize(header.nrows, header.ncols, false);
        mat.reset();

        auto handler = dense_2d_call_adding_parse_handler<DenseMatrix, IT, VT>(mat);
        read_matrix_market_body(instream, header, handler, default_pattern_value, options);
    }

    /**
     * Read Matrix Market file into a Blaze matrix.
     */
    template <typename BlazeMatrix>
    void read_matrix_market_blaze(std::istream &instream,
                                  BlazeMatrix& mat,
                                  const read_options& options = {},
                                  typename BlazeMatrix::ElementType default_pattern_value = 1) {

        matrix_market_header header;
        read_matrix_market_blaze(instream, header, mat, options, default_pattern_value);
    }

    /**
    * Format Blaze CompressedMatrix Matrix.
    */
    template<typename LF, typename SparseMatrixType>
    class Blaze_CompressedMatrix_formatter {
    public:
        typedef size_t MatIndex;

        explicit Blaze_CompressedMatrix_formatter(LF lf, const SparseMatrixType& mat, MatIndex major_size) : line_formatter(lf), mat(mat), major_size(major_size) {
            nnz_per_major = ((double)mat.nonZeros()) / (double)major_size;
        }

        [[nodiscard]] bool has_next() const {
            return major_iter < major_size;
        }

        class chunk {
        public:
            explicit chunk(LF lf, const SparseMatrixType& mat, MatIndex major_iter, MatIndex major_end) :
                    line_formatter(lf), mat(mat), major_iter(major_iter), major_end(major_end) {}

            std::string operator()() {
                std::string chunk;
                chunk.reserve((major_end - major_iter)*250);

                const bool is_row_major = blaze::IsRowMajorMatrix_v<SparseMatrixType>;

                // iterate over assigned columns (or rows)
                for (; major_iter != major_end; ++major_iter) {
                    for(typename SparseMatrixType::ConstIterator it = mat.cbegin(major_iter); it != mat.cend(major_iter); ++it ) {
                        auto minor_idx = it->index();

                        if (is_row_major) {
                            chunk += line_formatter.coord_matrix(major_iter, minor_idx, it->value());
                        } else {
                            chunk += line_formatter.coord_matrix(minor_idx, major_iter, it->value());
                        }
                    }
                }

                return chunk;
            }

            LF line_formatter;
            const SparseMatrixType& mat;
            MatIndex major_iter, major_end;
        };

        chunk next_chunk(const write_options& options) {
            auto num_columns = (MatIndex)(nnz_per_major * (double)options.chunk_size_values + 1);
            num_columns = std::min(num_columns, major_size - major_iter);

            MatIndex major_end = major_iter + num_columns;
            chunk c(line_formatter, mat, major_iter, major_end);
            major_iter = major_end;

            return c;
        }

    protected:
        LF line_formatter;
        const SparseMatrixType& mat;
        double nnz_per_major;
        MatIndex major_iter = 0;
        MatIndex major_size;
    };

    /**
     * Write a sparse Blaze matrix to MatrixMarket.
     */
    template <typename SparseMatrix, typename std::enable_if<blaze::IsSparseMatrix_v<SparseMatrix>, int>::type = 0>
    void write_matrix_market_blaze(std::ostream &os,
                                   const SparseMatrix& mat,
                                   const write_options& options = {}, matrix_market_header header = {}) {
        typedef size_t IT; // Blaze matrices use size_t for indexing
        typedef typename SparseMatrix::ElementType VT;

        header.nrows = mat.rows();
        header.ncols = mat.columns();
        header.nnz = mat.nonZeros();

        header.object = matrix;
        if (header.field != pattern) {
            header.field = get_field_type((const VT*)nullptr);
        }
        header.format = coordinate;

        write_header(os, header, options);

        const bool is_row_major = blaze::IsRowMajorMatrix_v<SparseMatrix>;

        line_formatter<IT, VT> lf(header, options);
        auto formatter = Blaze_CompressedMatrix_formatter(lf, mat, is_row_major ? header.nrows : header.ncols);
        write_body(os, formatter, options);
    }

    /**
     * Write a dense Blaze matrix to MatrixMarket.
     */
    template <typename DenseMatrix, typename std::enable_if<blaze::IsDenseMatrix_v<DenseMatrix>, int>::type = 0>
    void write_matrix_market_blaze(std::ostream &os,
                                   const DenseMatrix& mat,
                                   const write_options& options = {},
                                   matrix_market_header header = {}) {
        typedef size_t IT; // Blaze matrices use size_t for indexing
        typedef typename DenseMatrix::ElementType VT;

        header.nrows = mat.rows();
        header.ncols = mat.columns();
        header.nnz = mat.rows() * mat.columns();

        header.object = matrix;
        header.field = get_field_type((const VT *) nullptr);
        header.format = array;

        write_header(os, header, options);

        line_formatter<IT, VT> lf(header, options);
        auto formatter = dense_2d_call_formatter(lf, mat, header.nrows, header.ncols);
        write_body(os, formatter, options);
    }


    //////////////////////////////////////////////////////
    // Vector methods

    /**
     * Read Matrix Market file into a sparse Blaze vector and a header struct.
     */
    template <typename SparseVector, typename std::enable_if<blaze::IsSparseVector_v<SparseVector>, int>::type = 0>
    void read_matrix_market_blaze(std::istream &instream,
                                  matrix_market_header &header,
                                  SparseVector& vec,
                                  const read_options& options = {},
                                  typename SparseVector::ElementType default_pattern_value = 1) {

        typedef int64_t IT; // Blaze vectors use size_t for indexing, but allowing negative numbers allows more error checking while parsing.
        typedef typename SparseVector::ElementType VT;

        read_header(instream, header);

        if (std::min(header.nrows, header.ncols) > 1) {
            throw invalid_argument("Cannot load a matrix into a vector structure");
        }

        // Read into doublets
        std::vector<IT> inds(header.vector_length);
        std::vector<VT> vals(header.vector_length);

        auto handler = doublet_parse_handler(inds.begin(), vals.begin());
        read_matrix_market_body(instream, header, handler, default_pattern_value, options);

        // Set the values into the vector from the doublets.
        // The vector needs to be constructed in order, sorted by index.

        // Find sort permutation
        std::vector<std::size_t> perm(header.nnz);
        std::iota(perm.begin(), perm.end(), 0);

        std::sort(perm.begin(), perm.end(),
                  [&](std::size_t i, std::size_t j) {
                      return inds[i] < inds[j];
                  });


        // Construct the vector.
        vec.clear();
        vec.resize(header.vector_length);
        vec.reserve(header.nnz);

        for (auto i : perm) {
            vec.append(inds[i], vals[i]);
        }
    }

    /**
     * Read Matrix Market file into a dense Blaze vector and a header struct.
     */
    template <typename DenseVector, typename std::enable_if<blaze::IsDenseVector_v<DenseVector>, int>::type = 0>
    void read_matrix_market_blaze(std::istream &instream,
                                  matrix_market_header &header,
                                  DenseVector& vec,
                                  const read_options& options = {},
                                  typename DenseVector::ElementType default_pattern_value = 1) {
        read_header(instream, header);

        if (std::min(header.nrows, header.ncols) > 1) {
            throw invalid_argument("Cannot load a matrix into a vector structure");
        }

        vec.resize(header.vector_length);
        vec.reset();

        auto handler = dense_adding_parse_handler(vec.data(), row_major, header.vector_length, 1);
        read_matrix_market_body(instream, header, handler, default_pattern_value, options);
    }

    /**
     * Write a sparse Blaze vector to MatrixMarket.
     */
    template <typename SparseVector, typename std::enable_if<blaze::IsSparseVector_v<SparseVector>, int>::type = 0>
    void write_matrix_market_blaze(std::ostream &os,
                                   const SparseVector& vec,
                                   const write_options& options = {},
                                   matrix_market_header header = {}) {
        typedef size_t IT; // Blaze matrices use size_t for indexing
        typedef typename SparseVector::ElementType VT;

        // Copy into a doublet
        std::vector<IT> indices;
        std::vector<VT> values;
        indices.reserve(vec.nonZeros());
        values.reserve(vec.nonZeros());
        for (auto it = vec.cbegin(); it != vec.cend(); ++it) {
            indices.emplace_back(it->index());
            values.emplace_back(it->value());
        }

        // construct the header
        auto vector_length = (int64_t)vec.size();

        header.vector_length = vector_length;
        header.nnz = (int64_t)indices.size();

        header.object = vector;
        if (header.field != pattern && options.fill_header_field_type) {
            header.field = get_field_type((const VT*)nullptr);
        }
        header.format = coordinate;

        write_header(os, header, options);

        // write the body

        vector_line_formatter<IT, VT> lf(header, options);
        auto formatter = triplet_formatter(lf,
                                           indices.begin(), indices.end(),
                                           indices.begin(), indices.end(),
                                           values.begin(), values.end());
        write_body(os, formatter, options);
    }

    /**
     * Write a dense Blaze vector to MatrixMarket.
     */
    template <typename DenseVector, typename std::enable_if<blaze::IsDenseVector_v<DenseVector>, int>::type = 0>
    void write_matrix_market_blaze(std::ostream &os,
                                   const DenseVector& vec,
                                   const write_options& options = {},
                                   matrix_market_header header = {}) {
        typedef typename DenseVector::ElementType VT;

        auto vector_length = (int64_t)vec.size();

        header.vector_length = vector_length;
        header.nnz = vector_length;

        header.object = vector;
        if (options.fill_header_field_type) {
            header.field = get_field_type((const VT *) nullptr);
        }
        header.format = array;

        write_header(os, header, options);

        line_formatter<int64_t, VT> lf(header, options);
        auto formatter = array_formatter(lf, vec.data(), row_major, vector_length, 1);
        write_body(os, formatter, options);
    }
}
