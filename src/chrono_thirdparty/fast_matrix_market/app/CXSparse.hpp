// Copyright (C) 2022 Adam Lugowski. All rights reserved.
// Use of this source code is governed by the BSD 2-clause license found in the LICENSE.txt file.
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include "../fast_matrix_market.hpp"

namespace fast_matrix_market {
    /**
     * Read Matrix Market file into a CXSparse cs_* triplet structure.
     */
    template <typename CS, typename ALLOC>
    void read_matrix_market_cxsparse(std::istream &instream,
                                    CS **cs,
                                    ALLOC spalloc,
                                    read_options options = {}) {

        matrix_market_header header;
        read_header(instream, header);

        // Sanitize symmetry generalization settings
        if (options.generalize_symmetry && options.generalize_symmetry_app) {
            // cs_compress drops zero elements
            options.generalize_coordinate_diagnonal_values = read_options::ExtraZeroElement;
        }

        // allocate
        *cs = spalloc(header.nrows, header.ncols, get_storage_nnz(header, options),
                      header.field == pattern ? 0 : 1, // pattern field means do not allocate values
                      1);
        if (*cs == nullptr) {
            return;
        }

        auto handler = triplet_parse_handler((*cs)->i, (*cs)->p, (*cs)->x);
        read_matrix_market_body_no_adapters(instream, header, handler, options);

        // nz > 0 indicates a triplet matrix.
        (*cs)->nz = (*cs)->nzmax;
    }

    /**
     * Write a CXSparse cs_* structure to MatrixMarket.
     */
    template <typename CS>
    void write_matrix_market_cxsparse(std::ostream &os,
                                     CS* cs,
                                     const write_options& options = {}) {
        matrix_market_header header;
        header.nrows = cs->m;
        header.ncols = cs->n;
        header.nnz = cs->nz == -1 ? cs->nzmax : cs->nz;

        header.object = matrix;
        if (cs->x == nullptr) {
            header.field = pattern;
        } else if (options.fill_header_field_type) {
            header.field = get_field_type(cs->x);
        }
        header.format = coordinate;

        write_header(os, header, options);

        if (cs->nz == -1) {
            // compressed
            line_formatter<decltype(*cs->i), decltype(*cs->x)> lf(header, options);
            auto formatter = csc_formatter(lf,
                                           cs->p, cs->p + cs->n, // explicitly no +1
                                           cs->i, cs->i + cs->nzmax,
                                           cs->x, header.field == pattern ? cs->x : cs->x + cs->nzmax,
                                           false);
            write_body(os, formatter, options);
        } else {
            // triplet
            line_formatter<decltype(*cs->i), decltype(*cs->x)> lf(header, options);
            auto formatter = triplet_formatter(lf,
                                               cs->i, cs->i + cs->nz,
                                               cs->p, cs->p + cs->nz,
                                               cs->x, header.field == pattern ? cs->x : cs->x + cs->nz);
            write_body(os, formatter, options);
        }
    }
}