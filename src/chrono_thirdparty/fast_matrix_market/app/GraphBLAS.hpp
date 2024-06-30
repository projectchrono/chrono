// Copyright (C) 2022-2023 Adam Lugowski. All rights reserved.
// Use of this source code is governed by the BSD 2-clause license found in the LICENSE.txt file.
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

// for std::iota
#include <numeric>

#include "../fast_matrix_market.hpp"

extern "C" {
#include <GraphBLAS.h>
}

// Define this to disable all extensions.
//#define FMM_NO_GXB

#ifdef FMM_NO_GXB
// Disable all extensions
#define FMM_GXB_COMPLEX 0
#define FMM_GXB_BUILD_SCALAR 0
#define FMM_GXB_ITERATORS 0
#define FMM_GXB_PACK_UNPACK 0
#define FMM_GXB_TYPE_NAME 0
#endif

// Switch to allow using the SuiteSparse:GraphBLAS complex number extension
#ifndef FMM_GXB_COMPLEX
#define FMM_GXB_COMPLEX 1
#endif

// Switch to allow using GxB_Matrix_build_Scalar
#ifndef FMM_GXB_BUILD_SCALAR
#define FMM_GXB_BUILD_SCALAR 1
#endif

// Switch to allow using the SuiteSparse:GraphBLAS iterator extension
#ifndef FMM_GXB_ITERATORS
#define FMM_GXB_ITERATORS 1
#endif

// Switch to allow using the SuiteSparse:GraphBLAS iterators for vectors.
// The author is not fully confident on the correctness of this implementation, even though it does pass
// tests, because the docs and behavior leave some ambiguities. For example, how can pmax (the largest index
// of an element) be larger than nvals (the number of elements)?
// Enable this if you've checked over the implementation and need the zero copy.
// For everyone else, just use the extractTuples version.
#ifndef FMM_GXB_VECTOR_ITERATORS
#define FMM_GXB_VECTOR_ITERATORS 0
#endif

// Switch to allow using pack/unpack methods
#ifndef FMM_GXB_PACK_UNPACK
#define FMM_GXB_PACK_UNPACK 1
#endif

// Switch to allow using GxB_Matrix_type_name methods
#ifndef FMM_GXB_TYPE_NAME
#define FMM_GXB_TYPE_NAME 1
#endif


namespace fast_matrix_market {

    /**
     * Check that a GrB function call was successful and throw an exception if it wasn't.
     */
    void ok(GrB_Info ec) {
        if (ec != GrB_SUCCESS) {
            throw fmm_error("GraphBLAS method returned error code: " + std::to_string(ec));
        }
    }

    /**
     * Return map[key], throwing an exception with a helpful error message if key is not in map.
     */
    template <typename ENUM>
    ENUM parse_key(const std::string& s, std::map<const std::string, ENUM> mp, const std::string& what) {
        auto iter = mp.find(s);
        if (iter != mp.end()) {
            return iter->second;
        }

        std::string acceptable;
        std::string delim;
        for (const auto& [key, value] : mp) {
            acceptable += delim + std::string(key);
            delim = ", ";
        }
        throw invalid_argument(std::string("Invalid ") + what + ". Must be one of: " + acceptable);
    }

    /**
     * Map ctype strings to GraphBLAS GrB_Type.
     */
    inline std::map<const std::string, GrB_Type> str_ctype_to_GrB_Type() {
        return std::map<const std::string, GrB_Type>{
                {"bool", GrB_BOOL},
                {"int8_t", GrB_INT8},
                {"int16_t", GrB_INT16},
                {"int32_t", GrB_INT32},
                {"int64_t", GrB_INT64},
                {"uint8_t", GrB_UINT8},
                {"uint16_t", GrB_UINT16},
                {"uint32_t", GrB_UINT32},
                {"uint64_t", GrB_UINT64},
                {"float", GrB_FP32},
                {"double", GrB_FP64},
#if FMM_GXB_COMPLEX
                {"float complex", GxB_FC32},
                {"double complex", GxB_FC64},
                {"GxB_FC32_t", GxB_FC32},
                {"GxB_FC64_t", GxB_FC64},
#endif
        };
    }

    /**
     * Map GrB_Type to a ctype string. This is the reverse of `str_ctype_to_GrB_Type`.
     */
    inline std::map<GrB_Type, std::string> GrB_Type_to_header_type() {
        std::map<GrB_Type, std::string> ret;
        for (const auto& [str, type] : str_ctype_to_GrB_Type()) {
            ret[type] = str;
        }

#if FMM_GXB_COMPLEX
        // Explicitly define complex values
        ret[GxB_FC32] = "float complex";
        ret[GxB_FC64] = "double complex";
#endif
        return ret;
    }

    /**
     * Parse a type string into a GrB_Type, with error checking.
     */
    GrB_Type parse_type(const std::string& s) {
        return parse_key(s, str_ctype_to_GrB_Type(), "type");
    }

    /**
     * GraphBLAS typed functions are duplicated with their type part of the function name. This makes any call
     * to these functions require a large switch statement. This template specializes each explicit GraphBLAS type
     * so the logic methods can just use regular C++ templates.
     *
     * @tparam T C++ type that has a mapping to GraphBLAS.
     */
    template <typename T>
    struct GraphBLAS_typed {
        static GrB_Type type() { return nullptr; }

        static GrB_Info build_matrix([[maybe_unused]] GrB_Matrix mat, [[maybe_unused]] const GrB_Index* rows, [[maybe_unused]] const GrB_Index* cols, [[maybe_unused]] const T* vals, [[maybe_unused]] GrB_Index nvals) {
            return GrB_PANIC;
        }

        static GrB_Info set_element([[maybe_unused]] GrB_Scalar scalar, [[maybe_unused]] const T& x) {
            return GrB_PANIC;
        }

        static GrB_Info GrB_Matrix_extractTuples([[maybe_unused]] GrB_Index *I, [[maybe_unused]] T *X, [[maybe_unused]] GrB_Index *nvals, [[maybe_unused]] const GrB_Matrix& A) {
            return GrB_PANIC;
        }

#if FMM_GXB_ITERATORS
        static T GxB_Iterator_get([[maybe_unused]] GxB_Iterator iterator) {
            throw fmm_error("FMM Bug: GxB_Iterator_get<T> called.");
        }
#endif
        static GrB_Info build_vector([[maybe_unused]] GrB_Vector vec, [[maybe_unused]] const GrB_Index* indices, [[maybe_unused]] const T* vals, [[maybe_unused]] GrB_Index nvals) {
            return GrB_PANIC;
        }

        static GrB_Info GrB_Vector_extractTuples([[maybe_unused]] GrB_Index *I, [[maybe_unused]] T *X, [[maybe_unused]] GrB_Index *nvals, [[maybe_unused]] const GrB_Vector& A) {
            return GrB_PANIC;
        }
    };
    template <> struct GraphBLAS_typed<bool> {
        static GrB_Type type() { return GrB_BOOL; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const bool* vals, GrB_Index nvals) {
            return GrB_Matrix_build_BOOL(mat, rows, cols, vals, nvals, GrB_PLUS_BOOL);
        }

        static GrB_Info set_element(GrB_Scalar scalar, bool x) {
            return GrB_Scalar_setElement_BOOL(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, bool *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_BOOL(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static bool GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_BOOL(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const bool* vals, GrB_Index nvals) {
            return GrB_Vector_build_BOOL(vec, indices, vals, nvals, GrB_PLUS_BOOL);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, bool *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_BOOL(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<int8_t> {
        static GrB_Type type() { return GrB_INT8; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const int8_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_INT8(mat, rows, cols, vals, nvals, GrB_PLUS_INT8);
        }

        static GrB_Info set_element(GrB_Scalar scalar, int8_t x) {
            return GrB_Scalar_setElement_INT8(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, int8_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_INT8(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static int8_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_INT8(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const int8_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_INT8(vec, indices, vals, nvals, GrB_PLUS_INT8);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, int8_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_INT8(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<int16_t> {
        static GrB_Type type() { return GrB_INT16; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const int16_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_INT16(mat, rows, cols, vals, nvals, GrB_PLUS_INT16);
        }

        static GrB_Info set_element(GrB_Scalar scalar, int16_t x) {
            return GrB_Scalar_setElement_INT16(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, int16_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_INT16(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static int16_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_INT16(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const int16_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_INT16(vec, indices, vals, nvals, GrB_PLUS_INT16);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, int16_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_INT16(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<int32_t> {
        static GrB_Type type() { return GrB_INT32; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const int32_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_INT32(mat, rows, cols, vals, nvals, GrB_PLUS_INT32);
        }

        static GrB_Info set_element(GrB_Scalar scalar, int32_t x) {
            return GrB_Scalar_setElement_INT32(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, int32_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_INT32(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static int32_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_INT32(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const int32_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_INT32(vec, indices, vals, nvals, GrB_PLUS_INT32);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, int32_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_INT32(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<int64_t> {
        static GrB_Type type() { return GrB_INT64; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const int64_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_INT64(mat, rows, cols, vals, nvals, GrB_PLUS_INT64);
        }

        static GrB_Info set_element(GrB_Scalar scalar, int64_t x) {
            return GrB_Scalar_setElement_INT64(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, int64_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_INT64(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static int64_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_INT64(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const int64_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_INT64(vec, indices, vals, nvals, GrB_PLUS_INT64);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, int64_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_INT64(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<uint8_t> {
        static GrB_Type type() { return GrB_UINT8; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const uint8_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_UINT8(mat, rows, cols, vals, nvals, GrB_PLUS_UINT8);
        }

        static GrB_Info set_element(GrB_Scalar scalar, uint8_t x) {
            return GrB_Scalar_setElement_UINT8(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, uint8_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_UINT8(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static uint8_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_UINT8(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const uint8_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_UINT8(vec, indices, vals, nvals, GrB_PLUS_UINT8);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, uint8_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_UINT8(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<uint16_t> {
        static GrB_Type type() { return GrB_UINT16; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const uint16_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_UINT16(mat, rows, cols, vals, nvals, GrB_PLUS_UINT16);
        }

        static GrB_Info set_element(GrB_Scalar scalar, uint16_t x) {
            return GrB_Scalar_setElement_UINT16(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, uint16_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_UINT16(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static uint16_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_UINT16(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const uint16_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_UINT16(vec, indices, vals, nvals, GrB_PLUS_UINT16);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, uint16_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_UINT16(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<uint32_t> {
        static GrB_Type type() { return GrB_UINT32; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const uint32_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_UINT32(mat, rows, cols, vals, nvals, GrB_PLUS_UINT32);
        }

        static GrB_Info set_element(GrB_Scalar scalar, uint32_t x) {
            return GrB_Scalar_setElement_UINT32(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, uint32_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_UINT32(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static uint32_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_UINT32(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const uint32_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_UINT32(vec, indices, vals, nvals, GrB_PLUS_UINT32);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, uint32_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_UINT32(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<uint64_t> {
        static GrB_Type type() { return GrB_UINT64; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const uint64_t* vals, GrB_Index nvals) {
            return GrB_Matrix_build_UINT64(mat, rows, cols, vals, nvals, GrB_PLUS_UINT64);
        }

        static GrB_Info set_element(GrB_Scalar scalar, uint64_t x) {
            return GrB_Scalar_setElement_UINT64(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, uint64_t *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_UINT64(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static uint64_t GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_UINT64(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const uint64_t* vals, GrB_Index nvals) {
            return GrB_Vector_build_UINT64(vec, indices, vals, nvals, GrB_PLUS_UINT64);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, uint64_t *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_UINT64(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<float> {
        static GrB_Type type() { return GrB_FP32; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const float* vals, GrB_Index nvals) {
            return GrB_Matrix_build_FP32(mat, rows, cols, vals, nvals, GrB_PLUS_FP32);
        }

        static GrB_Info set_element(GrB_Scalar scalar, float x) {
            return GrB_Scalar_setElement_FP32(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, float *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_FP32(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static float GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_FP32(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const float* vals, GrB_Index nvals) {
            return GrB_Vector_build_FP32(vec, indices, vals, nvals, GrB_PLUS_FP32);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, float *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_FP32(I, X, nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<double> {
        static GrB_Type type() { return GrB_FP64; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const double* vals, GrB_Index nvals) {
            return GrB_Matrix_build_FP64(mat, rows, cols, vals, nvals, GrB_PLUS_FP64);
        }

        static GrB_Info set_element(GrB_Scalar scalar, double x) {
            return GrB_Scalar_setElement_FP64(scalar, x);
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, double *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GrB_Matrix_extractTuples_FP64(I, J, X, nvals, A);
        }

#if FMM_GXB_ITERATORS
        static double GxB_Iterator_get(GxB_Iterator iterator) {
            return GxB_Iterator_get_FP64(iterator);
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const double* vals, GrB_Index nvals) {
            return GrB_Vector_build_FP64(vec, indices, vals, nvals, GrB_PLUS_FP64);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, double *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GrB_Vector_extractTuples_FP64(I, X, nvals, A);
        }
    };
#if FMM_GXB_COMPLEX
    template <> struct GraphBLAS_typed<std::complex<float>> {
        static GrB_Type type() { return GxB_FC32; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const std::complex<float>* vals, GrB_Index nvals) {
            return GxB_Matrix_build_FC32(mat, rows, cols, reinterpret_cast<const GxB_FC32_t *>(vals), nvals, GxB_PLUS_FC32);
        }

        static GrB_Info set_element(GrB_Scalar scalar, const std::complex<float>& x) {
            return GxB_Scalar_setElement_FC32(scalar, reinterpret_cast<const GxB_FC32_t &>(x));
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, std::complex<float> *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GxB_Matrix_extractTuples_FC32(I, J, reinterpret_cast<GxB_FC32_t *>(X), nvals, A);
        }

#if FMM_GXB_ITERATORS
        static std::complex<float> GxB_Iterator_get(GxB_Iterator iterator) {
            return reinterpret_cast<std::complex<float>&>(GxB_Iterator_get_FC32(iterator));
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const std::complex<float>* vals, GrB_Index nvals) {
            return GxB_Vector_build_FC32(vec, indices, reinterpret_cast<const GxB_FC32_t*>(vals), nvals, GxB_PLUS_FC32);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, std::complex<float> *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GxB_Vector_extractTuples_FC32(I, reinterpret_cast<GxB_FC32_t*>(X), nvals, A);
        }
    };
    template <> struct GraphBLAS_typed<std::complex<double>> {
        static GrB_Type type() { return GxB_FC64; }

        static GrB_Info build_matrix(GrB_Matrix mat, const GrB_Index* rows, const GrB_Index* cols, const std::complex<double>* vals, GrB_Index nvals) {
            return GxB_Matrix_build_FC64(mat, rows, cols, reinterpret_cast<const GxB_FC64_t*>(vals), nvals, GxB_PLUS_FC64);
        }

        static GrB_Info set_element(GrB_Scalar scalar, const std::complex<double>& x) {
            return GxB_Scalar_setElement_FC64(scalar, reinterpret_cast<const GxB_FC64_t&>(x));
        }

        static GrB_Info GrB_Matrix_extractTuples(GrB_Index *I, GrB_Index *J, std::complex<double> *X, GrB_Index *nvals, const GrB_Matrix& A) {
            return GxB_Matrix_extractTuples_FC64(I, J, reinterpret_cast<GxB_FC64_t*>(X), nvals, A);
        }

#if FMM_GXB_ITERATORS
        static std::complex<double> GxB_Iterator_get(GxB_Iterator iterator) {
            return reinterpret_cast<std::complex<double>&>(GxB_Iterator_get_FC64(iterator));
        }
#endif
        static GrB_Info build_vector(GrB_Vector vec, const GrB_Index* indices, const std::complex<double>* vals, GrB_Index nvals) {
            return GxB_Vector_build_FC64(vec, indices, reinterpret_cast<const GxB_FC64_t*>(vals), nvals, GxB_PLUS_FC64);
        }

        static GrB_Info GrB_Vector_extractTuples(GrB_Index *I, std::complex<double> *X, GrB_Index *nvals, const GrB_Vector& A) {
            return GxB_Vector_extractTuples_FC64(I, reinterpret_cast<GxB_FC64_t*>(X), nvals, A);
        }
    };
#endif

    /**
     * Read a Matrix Market coordinate body into a sparse GraphBLAS matrix using triplets.
     */
    template <typename T>
    void read_body_graphblas_coordinate(std::istream &instream,
                                        matrix_market_header &header,
                                        GrB_Matrix mat,
                                        read_options options) {
        size_t storage_nnz = get_storage_nnz(header, options);

        // Read into triplets
        std::vector<GrB_Index> rows(storage_nnz);
        std::vector<GrB_Index> cols(storage_nnz);

        // Sanitize symmetry generalization settings
        if (options.generalize_symmetry && options.generalize_symmetry_app) {
            // Matrix construction drops zero elements
            options.generalize_coordinate_diagnonal_values = read_options::ExtraZeroElement;
        }

#if FMM_GXB_BUILD_SCALAR
        if (header.field == pattern) {
            // read the indices
            auto handler = triplet_pattern_parse_handler(rows.begin(), cols.begin());
            read_matrix_market_body_no_adapters(instream, header, handler, options);

            // create the scalar
            GrB_Scalar one;
            ok(GrB_Scalar_new(&one, GraphBLAS_typed<T>::type()));
            ok(GraphBLAS_typed<T>::set_element(one, pattern_default_value(static_cast<T*>(nullptr))));

            // Build iso matrix
            ok(GxB_Matrix_build_Scalar(mat, rows.data(), cols.data(), one, storage_nnz));

            // clean up
            ok(GrB_Scalar_free(&one));
        } else
#endif
        {
            // Allocate values. Cannot use std::vector due to bool specialization
            auto vals = std::make_unique<T[]>(storage_nnz);

            // read indices and values
            auto handler = triplet_parse_handler(rows.begin(), cols.begin(), vals.get());
            read_matrix_market_body(instream, header, handler, pattern_default_value(static_cast<T*>(nullptr)), options);

            // build matrix from triplets
            ok(GraphBLAS_typed<T>::build_matrix(mat, rows.data(), cols.data(), vals.get(), storage_nnz));
        }
    }


    /**
     * Read a Matrix Market array body into a full GraphBLAS matrix.
     */
    template <typename T>
    void read_body_graphblas_array(std::istream &instream,
                                   matrix_market_header &header,
                                   GrB_Matrix mat,
                                   const read_options& options) {
#if FMM_GXB_PACK_UNPACK
        // Allocate the dense matrix. GraphBLAS takes ownership of this pointer.
        auto arr = (T*)calloc( header.nrows*header.ncols, sizeof(T));

        auto handler = dense_adding_parse_handler(arr, col_major, header.nrows, header.ncols);
        read_matrix_market_body(instream, header, handler, 1, options);
        ok(GxB_Matrix_pack_FullC(mat, (void**)&arr, (header.nrows*header.ncols*sizeof(T)), false, nullptr));
#else
        // No efficient way to read dense arrays in standard GraphBLAS.
        read_body_graphblas_coordinate<T>(instream, header, mat, options);
#endif
    }

    /**
     * Read a Matrix Market body into a GraphBLAS matrix.
     */
    template <typename T>
    void read_body_graphblas(std::istream &instream,
                             matrix_market_header &header,
                             GrB_Matrix mat,
                             const read_options& options) {
        if (header.format == array) {
            read_body_graphblas_array<T>(instream, header, mat, options);
        } else {
            read_body_graphblas_coordinate<T>(instream, header, mat, options);
        }
    }

    /**
     * Parse any '%%GraphBLAS type <ctype>' comments.
     *
     * If not present, use the MatrixMarket header field.
     */
    inline GrB_Type get_type_from_header(const matrix_market_header& header) {
        const std::string kStructuredCommentBanner = "%GraphBLAS"; // The first % was consumed by the comment parser

        GrB_Type desired_type = nullptr;

        std::istringstream comment_iss(header.comment);
        std::string line;
        while (std::getline(comment_iss, line)) {
            if (!fast_matrix_market::starts_with(line, kStructuredCommentBanner)) {
                continue;
            }

            std::istringstream iss(line);
            std::string banner, mode, type_string;

            iss >> banner >> mode;
            if (mode == "type") {
                // The type is the C type. Note that in C the complex types contain whitespace:
                // float complex, double complex
                std::getline(iss, type_string);
            } else {
                // Unknown structured comment. Ignore.
                continue;
            }

            desired_type = parse_type(trim(type_string));
            break;
        }

        if (desired_type == nullptr) {
            // The type is not specified in the file or by the caller. Choose appropriate type based on
            // header field
            switch (header.field) {
                case pattern:
                    desired_type = GrB_BOOL;
                    break;
                case integer:
                    desired_type = GrB_INT64;
                    break;
                case real:
                    desired_type = GrB_FP64;
                    break;
                case complex:
#if FMM_GXB_COMPLEX
                    desired_type = GxB_FC64;
                    break;
#else
                    throw invalid_mm("Matrix is complex but no complex support in GraphBLAS.");
#endif
                default:
                    desired_type = GrB_FP64;
                    break;
            }

        }

        return desired_type;
    }

    /**
     * Read Matrix Market file into a sparse GraphBLAS matrix and a header struct.
     */
    inline void read_matrix_market_graphblas(std::istream &instream,
                                             matrix_market_header &header,
                                             GrB_Matrix* mat,
                                             const read_options& options = {},
                                             GrB_Type desired_type = nullptr) {
        read_header(instream, header);

        // Figure out the matrix type
        if (desired_type == nullptr) {
            desired_type = get_type_from_header(header);
        }

        // sanity check
        if (header.symmetry == skew_symmetric && (desired_type == GrB_BOOL ||
                desired_type == GrB_UINT8 || desired_type == GrB_UINT16 ||
                desired_type == GrB_UINT32 || desired_type == GrB_UINT64)) {
            throw invalid_mm("Skew-symmetric matrix cannot have an unsigned type");
        }

        ok(GrB_Matrix_new(mat, desired_type, header.nrows, header.ncols));

        if (desired_type == GrB_BOOL) {
            read_body_graphblas<bool>(instream, header, *mat, options);
        } else if (desired_type == GrB_INT8) {
            read_body_graphblas<int8_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_INT16) {
            read_body_graphblas<int16_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_INT32) {
            read_body_graphblas<int32_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_INT64) {
            read_body_graphblas<int64_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_UINT8) {
            read_body_graphblas<uint8_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_UINT16) {
            read_body_graphblas<uint16_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_UINT32) {
            read_body_graphblas<uint32_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_UINT64) {
            read_body_graphblas<uint64_t>(instream, header, *mat, options);
        } else if (desired_type == GrB_FP32) {
            read_body_graphblas<float>(instream, header, *mat, options);
        } else if (desired_type == GrB_FP64) {
            read_body_graphblas<double>(instream, header, *mat, options);
#if FMM_GXB_COMPLEX
        } else if (desired_type == GxB_FC32) {
            read_body_graphblas<std::complex<float>>(instream, header, *mat, options);
        } else if (desired_type == GxB_FC64) {
            read_body_graphblas<std::complex<double>>(instream, header, *mat, options);
#endif
        } else {
            throw invalid_mm("Unknown matrix type.");
        }
    }

    /**
     * Read Matrix Market file into a GraphBLAS matrix.
     */
    inline void read_matrix_market_graphblas(std::istream &instream,
                                             GrB_Matrix* mat,
                                             const read_options& options = {},
                                             GrB_Type desired_type = nullptr) {

        matrix_market_header header;
        read_matrix_market_graphblas(instream, header, mat, options, desired_type);
    }

    /**
     * Write a GraphBLAS matrix to a coordinate MatrixMarket body by using GrB_Matrix_extractTuples.
     */
    template <typename T>
    void write_body_graphblas_triplet(std::ostream &os,
                                      matrix_market_header &header,
                                      const GrB_Matrix& mat,
                                      const write_options& options) {
        std::vector<GrB_Index> rows(header.nnz);
        std::vector<GrB_Index> cols(header.nnz);
        auto vals = std::make_unique<T[]>(header.nnz); // Cannot use vector due to bool specialization

        GrB_Index nvals = header.nnz;
        auto ec = GraphBLAS_typed<T>::GrB_Matrix_extractTuples(rows.data(), cols.data(), vals.get(), &nvals, mat);
        if (ec != GrB_SUCCESS) {
            throw fast_matrix_market::invalid_argument("GrB_Matrix_extractTuples returned " + std::to_string(ec));
        }
        line_formatter<GrB_Index, T> lf(header, options);
        auto formatter = triplet_formatter(lf,
                                           rows.cbegin(), rows.cend(),
                                           cols.cbegin(), cols.cend(),
                                           vals.get(), header.field == pattern ? vals.get() : vals.get() + header.nnz);
        write_body(os, formatter, options);
    }

#if FMM_GXB_ITERATORS
    struct gblas_row_iter_impl {
        typedef GrB_Matrix container;

        static std::pair<GrB_Index, GrB_Index> setup(const GrB_Matrix& mat) {
            GrB_Matrix_wait(mat, GrB_MATERIALIZE);

            GrB_Index nvals;
            GrB_Matrix_nvals(&nvals, mat);

            GxB_Iterator iterator = attach(mat);
            GrB_Index kount = GxB_rowIterator_kount(iterator);
            GxB_Iterator_free(&iterator);

            return std::make_pair(kount, nvals);
        }

        static GxB_Iterator attach(const GrB_Matrix& mat) {
            GxB_Iterator iterator;
            ok(GxB_Iterator_new(&iterator));
            GrB_Info info = GxB_rowIterator_attach(iterator, mat, nullptr);
            if (info < 0) {
                throw fmm_error("GxB_rowIterator_attach returned: " + std::to_string(info));
            }
            return iterator;
        }

        static GrB_Info kseek(GxB_Iterator iterator, GrB_Index k) {
            return GxB_rowIterator_kseek(iterator, k);
        }

        static GrB_Index getMajorIndex(GxB_Iterator iterator) {
            return GxB_rowIterator_getRowIndex(iterator);
        }

        static GrB_Index getMinorIndex(GxB_Iterator iterator) {
            return GxB_rowIterator_getColIndex(iterator);
        }

        static GrB_Info nextMinor(GxB_Iterator iterator) {
            return GxB_rowIterator_nextCol(iterator);
        }

        static GrB_Info nextMajor(GxB_Iterator iterator) {
            return GxB_rowIterator_nextRow(iterator);
        }

        static std::pair<GrB_Index, GrB_Index> to_row_col(GrB_Index major, GrB_Index minor) {
            return {major, minor};
        }
    };

    struct gblas_col_iter_impl {
        typedef GrB_Matrix container;

        static std::pair<GrB_Index, GrB_Index> setup(const GrB_Matrix& mat) {
            GrB_Matrix_wait(mat, GrB_MATERIALIZE);

            GrB_Index nvals;
            GrB_Matrix_nvals(&nvals, mat);

            GxB_Iterator iterator = attach(mat);
            GrB_Index kount = GxB_colIterator_kount(iterator);
            GxB_Iterator_free(&iterator);

            return std::make_pair(kount, nvals);
        }

        static GxB_Iterator attach(const GrB_Matrix& mat) {
            GxB_Iterator iterator;
            ok(GxB_Iterator_new(&iterator));
            GrB_Info info = GxB_colIterator_attach(iterator, mat, nullptr);
            if (info < 0) {
                throw fmm_error("GxB_colIterator_attach returned: " + std::to_string(info));
            }
            return iterator;
        }

        static GrB_Info kseek(GxB_Iterator iterator, GrB_Index k) {
            return GxB_colIterator_kseek(iterator, k);
        }

        static GrB_Index getMinorIndex(GxB_Iterator iterator) {
            return GxB_colIterator_getRowIndex(iterator);
        }

        static GrB_Index getMajorIndex(GxB_Iterator iterator) {
            return GxB_colIterator_getColIndex(iterator);
        }

        static GrB_Info nextMinor(GxB_Iterator iterator) {
            return GxB_colIterator_nextRow(iterator);
        }

        static GrB_Info nextMajor(GxB_Iterator iterator) {
            return GxB_colIterator_nextCol(iterator);
        }

        static std::pair<GrB_Index, GrB_Index> to_row_col(GrB_Index major, GrB_Index minor) {
            return {minor, major};
        }
    };

#if FMM_GXB_VECTOR_ITERATORS
    struct gblas_vec_iter_impl {
        typedef GrB_Vector container;

        static std::pair<GrB_Index, GrB_Index> setup(const GrB_Vector& vec) {
            GrB_Vector_wait(vec, GrB_MATERIALIZE);

            GrB_Index nvals;
            GrB_Vector_nvals(&nvals, vec);

            // getpmax() sometimes returns a value > nvals. No idea how that makes sense.
            // reproduced with a dense vector with a zero value.
//            GxB_Iterator iterator = attach(vec);
//            GrB_Index kount = GxB_Vector_Iterator_getpmax(iterator);
//            GxB_Iterator_free(&iterator);

            return std::make_pair(nvals, nvals);
        }
        static GxB_Iterator attach(const GrB_Vector& vec) {
            GxB_Iterator iterator;
            ok(GxB_Iterator_new(&iterator));
            GrB_Info info = GxB_Vector_Iterator_attach(iterator, vec, nullptr);
            if (info < 0) {
                throw fmm_error("GxB_Vector_Iterator_attach returned: " + std::to_string(info));
            }
            return iterator;
        }

        static GrB_Info kseek(GxB_Iterator iterator, GrB_Index k) {
            return GxB_Vector_Iterator_seek(iterator, k);
        }

        static GrB_Index getMinorIndex([[maybe_unused]] GxB_Iterator iterator) {
            return 0; // vectors don't have a minor axis
        }

        static GrB_Index getMajorIndex(GxB_Iterator iterator) {
            return GxB_Vector_Iterator_getIndex(iterator);
        }

        static GrB_Info nextMinor([[maybe_unused]] GxB_Iterator iterator) {
            return GrB_INVALID_VALUE; // vectors don't have a minor axis
        }

        static GrB_Info nextMajor(GxB_Iterator iterator) {
            return GxB_Vector_Iterator_next(iterator);
        }

        static std::pair<GrB_Index, GrB_Index> to_row_col(GrB_Index major, GrB_Index minor) {
            return {major, minor};
        }
    };
#endif

    /**
    * Format GrB_Matrix using iterator.
    */
    template<typename LF, typename T, typename IMPL>
    class GrB_Matrix_Iterator_formatter {
    public:
        explicit GrB_Matrix_Iterator_formatter(LF lf, const typename IMPL::container& mat) : line_formatter(lf), mat(mat) {
            auto pair = IMPL::setup(mat);
            kount = pair.first;
            GrB_Index nvals = pair.second;
            nnz_per_kount = ((double)nvals) / (double)kount;
        }

        [[nodiscard]] bool has_next() const {
            return kount_iter < kount;
        }

        class chunk {
        public:
            explicit chunk(LF lf, const typename IMPL::container& mat, GrB_Index kount_iter, GrB_Index kount_end) :
                    line_formatter(lf), mat(mat), kount_iter(kount_iter), kount_end(kount_end) {}

            std::string operator()() {
                std::string chunk;
                chunk.reserve((kount_end - kount_iter)*250);

                GxB_Iterator iterator = IMPL::attach(mat);

                // seek to first assigned row or column
                GrB_Info info = IMPL::kseek(iterator, kount_iter);
                for (; info != GxB_EXHAUSTED && kount_iter < kount_end; ++kount_iter) {
                    GrB_Index major = IMPL::getMajorIndex (iterator);

                    // iterate over the columns in this row
                    while (info == GrB_SUCCESS) {
                        // get the entry A(i,j)
                        GrB_Index minor = IMPL::getMinorIndex(iterator);
                        auto[row, col] = IMPL::to_row_col(major, minor);

                        const T& value = GraphBLAS_typed<T>::GxB_Iterator_get(iterator);
                        chunk += line_formatter.coord_matrix(row, col, value);

                        // move to the next entry
                        info = IMPL::nextMinor(iterator);
                    }
                    // move to the next row or column
                    info = IMPL::nextMajor(iterator);
                }
                GxB_Iterator_free(&iterator);

                return chunk;
            }

            LF line_formatter;
            const typename IMPL::container& mat;
            GrB_Index kount_iter, kount_end;
        };

        chunk next_chunk(const write_options& options) {
            auto kount_this_chunk = (GrB_Index)(nnz_per_kount * (double)options.chunk_size_values + 1);
            kount_this_chunk = std::min(kount_this_chunk, kount - kount_iter);

            GrB_Index kount_end = kount_iter + kount_this_chunk;
            chunk c(line_formatter, mat, kount_iter, kount_end);
            kount_iter = kount_end;

            return c;
        }

    protected:
        LF line_formatter;
        const typename IMPL::container& mat;
        double nnz_per_kount;
        GrB_Index kount_iter = 0;
        GrB_Index kount;
    };

#if FMM_GXB_ITERATORS
    /**
     * Write a GraphBLAS matrix to an array MatrixMarket body by using a column GrB_Iterator.
     *
     * IMPORTANT! The matrix must be GxB_BY_COL, else GraphBLAS throws GrB_NOT_IMPLEMENTED.
     */
    template <typename T>
    void write_body_graphblas_array_iterator(std::ostream &os,
                                             matrix_market_header &header,
                                             const GrB_Matrix& mat,
                                             const write_options& options) {
        line_formatter<GrB_Index, T> lf(header, options);
        auto formatter = GrB_Matrix_Iterator_formatter<decltype(lf), T, gblas_col_iter_impl>(lf, mat);
        write_body(os, formatter, options);
    }
#endif

    /**
     * Write a GraphBLAS matrix to an array MatrixMarket body by extracting only the values with GrB_Matrix_extractTuples.
     * Since we know the ordering of the values (GxB_BY_ROW) we can write them directly using FMM machinery.
     *
     * This is not as good as direct GraphBLAS iteration because it makes a copy of the values, but it's better than
     * extracting the values *and* the indices. Also iteration "against the grain" is not implemented in GraphBLAS.
     *
     * IMPORTANT! The matrix is assumed to be GxB_BY_ROW, else the written data will be wrong.
     */
    template <typename T>
    void write_body_graphblas_array_row_via_extract(std::ostream &os,
                                                    matrix_market_header &header,
                                                    const GrB_Matrix& mat,
                                                    const write_options& options) {
        std::unique_ptr<T[]> vals = std::make_unique<T[]>(header.nnz); // Cannot use vector due to bool specialization
        GrB_Index nvals = header.nnz;
        ok(GraphBLAS_typed<T>::GrB_Matrix_extractTuples(nullptr, nullptr, vals.get(), &nvals, mat));

        line_formatter<GrB_Index, T> lf(header, options);
        auto formatter = array_formatter(lf, vals.get(), row_major, header.nrows, header.ncols);
        write_body(os, formatter, options);
    }

    /**
     * Write a GraphBLAS matrix to an array MatrixMarket body by using GrB_Matrix_extractTuples.
     *
     * Yes this is very inefficient, tripling storage requirements. But at the moment it appears to be
     * the best possible given available API methods.
     */
    template <typename T>
    void write_body_graphblas_array_via_triplet(std::ostream &os,
                                                matrix_market_header &header,
                                                const GrB_Matrix& mat,
                                                const write_options& options) {
        std::unique_ptr<T[]> sorted_vals; // Cannot use vector due to bool specialization
        {
            auto vals = std::make_unique<T[]>(header.nnz); // Cannot use vector due to bool specialization
            std::vector<std::size_t> perm(header.nnz);
            {
                std::vector<GrB_Index> rows(header.nnz);
                std::vector<GrB_Index> cols(header.nnz);

                GrB_Index nvals = header.nnz;
                ok(GraphBLAS_typed<T>::GrB_Matrix_extractTuples(rows.data(), cols.data(), vals.get(), &nvals, mat));

                // Find sort permutation
                std::iota(perm.begin(), perm.end(), 0);
                std::sort(perm.begin(), perm.end(),
                          [&](std::size_t i, std::size_t j) {
                              if (cols[i] != cols[j])
                                  return cols[i] < cols[j];
                              if (rows[i] != rows[j])
                                  return rows[i] < rows[j];

                              return false;
                          });
            }
            // Apply permutation
            sorted_vals = std::make_unique<T[]>(header.nnz);;
            std::transform(perm.begin(), perm.end(), sorted_vals.get(), [&](auto i) { return vals[i]; });
        }

        line_formatter<GrB_Index, T> lf(header, options);
        auto formatter = array_formatter(lf, sorted_vals.get(), col_major, header.nrows, header.ncols);
        write_body(os, formatter, options);
    }

    /**
     * Write a GraphBLAS matrix to a coordinate MatrixMarket body by using GrB_Iterator.
     * This method is zero-copy, but uses a GraphBLAS extension.
     */
    template <typename T>
    void write_body_graphblas_iterator(std::ostream &os,
                                       matrix_market_header &header,
                                       const GrB_Matrix& mat,
                                       const write_options& options) {
        line_formatter<GrB_Index, T> lf(header, options);

        int32_t format;
        ok(GxB_Matrix_Option_get_INT32(mat, GxB_FORMAT, &format));

        if (format == GxB_BY_ROW) {
            auto formatter = GrB_Matrix_Iterator_formatter<decltype(lf), T, gblas_row_iter_impl>(lf, mat);
            write_body(os, formatter, options);
        } else if (format == GxB_BY_COL) {
            auto formatter = GrB_Matrix_Iterator_formatter<decltype(lf), T, gblas_col_iter_impl>(lf, mat);
            write_body(os, formatter, options);
        } else {
            // shouldn't happen
            write_body_graphblas_triplet<T>(os, header, mat, options);
        }
    }
#endif

    /**
     * Write a GraphBLAS matrix to a MatrixMarket body.
     */
    template <typename T>
    void write_body_graphblas(std::ostream &os,
                              matrix_market_header &header,
                              const GrB_Matrix& mat,
                              const write_options& options) {

        if (header.format == coordinate) {
#if FMM_GXB_ITERATORS
            write_body_graphblas_iterator<T>(os, header, mat, options);
#else
            write_body_graphblas_triplet<T>(os, header, mat, options);
#endif
        } else {
#if FMM_GXB_ITERATORS
            int32_t format;
            ok(GxB_Matrix_Option_get_INT32(mat, GxB_FORMAT, &format));

            if (format == GxB_BY_COL) {
                // MatrixMarket array is column-major, so iterator direction matches
                write_body_graphblas_array_iterator<T>(os, header, mat, options);
            } else if (format == GxB_BY_ROW){
                write_body_graphblas_array_row_via_extract<T>(os, header, mat, options);
            } else {
                // shouldn't happen
                write_body_graphblas_array_via_triplet<T>(os, header, mat, options);
            }
#else
            write_body_graphblas_array_via_triplet<T>(os, header, mat, options);
#endif
        }
    }

    /**
     * Deduce Matrix Market header field type from GrB_Type.
     */
    inline field_type get_field(GrB_Type type) {
        if (type == GrB_FP32 || type == GrB_FP64) {
            return real;
#if FMM_GXB_COMPLEX
        } else if (type == GxB_FC32 || type == GxB_FC64) {
            return complex;
#endif
        } else if (type == GrB_BOOL   || type == GrB_INT8   || type == GrB_INT16  ||
                   type == GrB_INT32  || type == GrB_INT64  || type == GrB_UINT8  ||
                   type == GrB_UINT16 || type == GrB_UINT32 || type == GrB_UINT64) {
            return integer;
        } else {
            throw fast_matrix_market::invalid_argument("Unable to deduce Matrix Market type from GrB_Type type.");
        }
    }

    /**
     * Prepend a '%%GraphBLAS type <ctype>' structured comment to the actual comment
     */
    inline void add_structured_comment(matrix_market_header& header, GrB_Type type, const std::string& str_type) {
        if (type == nullptr) {
            return;
        }

        std::string header_type;
        auto type_map = GrB_Type_to_header_type();
        auto type_map_iter = type_map.find(type);
        if (type_map_iter != type_map.end()) {
            // known type
            header_type = type_map_iter->second;
        } else {
            // unknown type, use the type string
            if (str_type.empty()) {
                return;
            } else {
                header_type = str_type;
            }
        }
        std::string newline = header.comment.empty() ? "" : "\n";
        header.comment = std::string("%GraphBLAS type ") + header_type + newline + header.comment;
    }

    /**
     * Write a GraphBLAS matrix to MatrixMarket.
     */
    inline void write_matrix_market_graphblas(std::ostream &os,
                                              const GrB_Matrix& mat,
                                              const write_options& options = {}, matrix_market_header header = {}) {
        GrB_Type type;
        {
            GrB_Index temp;

            ok(GrB_Matrix_nrows(&temp, mat));
            header.nrows = (int64_t)temp;

            ok(GrB_Matrix_ncols(&temp, mat));
            header.ncols = (int64_t)temp;

            ok(GrB_Matrix_nvals(&temp, mat));
            header.nnz = (int64_t)temp;

            // get the matrix type
#if FMM_GXB_TYPE_NAME
            std::string mat_type(GxB_MAX_NAME_LEN, '\0');
            ok(GxB_Matrix_type_name(mat_type.data(), mat));
            mat_type.resize(mat_type.find('\0')); // truncate to the actual type name length
            type = parse_type(mat_type);

            // Write the '%%GraphBLAS type <ctype>' line
            add_structured_comment(header, type, mat_type);
#else
            type = GrB_FP64; // most compatible type from the standard set.
#endif
        }

        header.object = matrix;
        if (header.field != pattern && options.fill_header_field_type) {
            header.field = get_field(type);
        }
        if (header.field != pattern && header.nnz == header.nrows * header.ncols) {
            // Full matrix.
            header.format = array;
        } else {
            header.format = coordinate;
        }

        write_header(os, header, options);

        if (type == GrB_BOOL) {
            write_body_graphblas<bool>(os, header, mat, options);
        } else if (type == GrB_INT8) {
            write_body_graphblas<int8_t>(os, header, mat, options);
        } else if (type == GrB_INT16) {
            write_body_graphblas<int16_t>(os, header, mat, options);
        } else if (type == GrB_INT32) {
            write_body_graphblas<int32_t>(os, header, mat, options);
        } else if (type == GrB_INT64) {
            write_body_graphblas<int64_t>(os, header, mat, options);
        } else if (type == GrB_UINT8) {
            write_body_graphblas<uint8_t>(os, header, mat, options);
        } else if (type == GrB_UINT16) {
            write_body_graphblas<uint16_t>(os, header, mat, options);
        } else if (type == GrB_UINT32) {
            write_body_graphblas<uint32_t>(os, header, mat, options);
        } else if (type == GrB_UINT64) {
            write_body_graphblas<uint64_t>(os, header, mat, options);
        } else if (type == GrB_FP32) {
            write_body_graphblas<float>(os, header, mat, options);
        } else if (type == GrB_FP64) {
            write_body_graphblas<double>(os, header, mat, options);
#if FMM_GXB_COMPLEX
        } else if (type == GxB_FC32) {
            write_body_graphblas<std::complex<float>>(os, header, mat, options);
        } else if (type == GxB_FC64) {
            write_body_graphblas<std::complex<double>>(os, header, mat, options);
#endif
        } else {
            throw invalid_mm("Unknown matrix type.");
        }
    }


    //////////////////////////////////////////////////////
    // Vector methods


    /**
     * Read a Matrix Market body into a GraphBLAS vector.
     */
    template <typename T>
    void read_body_graphblas(std::istream &instream,
                             matrix_market_header &header,
                             GrB_Vector vec,
                             const read_options& options) {
        // Read into doublets
        std::vector<GrB_Index> inds(header.nnz);
        // Allocate values. Cannot use std::vector due to bool specialization
        auto vals = std::make_unique<T[]>(header.nnz);

        auto handler = doublet_parse_handler(inds.begin(), vals.get());
        read_matrix_market_body(instream, header, handler, pattern_default_value(static_cast<T*>(nullptr)), options);

        // build vector from doublets
        ok(GraphBLAS_typed<T>::build_vector(vec, inds.data(), vals.get(), header.nnz));
    }

    /**
     * Read Matrix Market file into a sparse GraphBLAS vector and a header struct.
     */
    inline void read_matrix_market_graphblas(std::istream &instream,
                                             matrix_market_header &header,
                                             GrB_Vector* vec,
                                             const read_options& options = {},
                                             GrB_Type desired_type = nullptr) {
        read_header(instream, header);

        // Figure out the matrix type
        if (desired_type == nullptr) {
            desired_type = get_type_from_header(header);
        }

        // sanity check
        if (header.symmetry != general) {
            throw invalid_mm("Vectors cannot have non-general symmetry.");
        }
        if (std::min(header.nrows, header.ncols) > 1) {
            throw invalid_argument("Cannot load a matrix into a vector structure");
        }

        ok(GrB_Vector_new(vec, desired_type, header.vector_length));

        if (desired_type == GrB_BOOL) {
            read_body_graphblas<bool>(instream, header, *vec, options);
        } else if (desired_type == GrB_INT8) {
            read_body_graphblas<int8_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_INT16) {
            read_body_graphblas<int16_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_INT32) {
            read_body_graphblas<int32_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_INT64) {
            read_body_graphblas<int64_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_UINT8) {
            read_body_graphblas<uint8_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_UINT16) {
            read_body_graphblas<uint16_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_UINT32) {
            read_body_graphblas<uint32_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_UINT64) {
            read_body_graphblas<uint64_t>(instream, header, *vec, options);
        } else if (desired_type == GrB_FP32) {
            read_body_graphblas<float>(instream, header, *vec, options);
        } else if (desired_type == GrB_FP64) {
            read_body_graphblas<double>(instream, header, *vec, options);
#if FMM_GXB_COMPLEX
        } else if (desired_type == GxB_FC32) {
            read_body_graphblas<std::complex<float>>(instream, header, *vec, options);
        } else if (desired_type == GxB_FC64) {
            read_body_graphblas<std::complex<double>>(instream, header, *vec, options);
#endif
        } else {
            throw invalid_mm("Unknown vector type.");
        }
    }

    /**
     * Read Matrix Market file into a GraphBLAS vector.
     */
    inline void read_matrix_market_graphblas(std::istream &instream,
                                             GrB_Vector* vec,
                                             const read_options& options = {},
                                             GrB_Type desired_type = nullptr) {

        matrix_market_header header;
        read_matrix_market_graphblas(instream, header, vec, options, desired_type);
    }

    /**
     * Write a GraphBLAS vector to a coordinate MatrixMarket body by using GrB_Vector_extractTuples.
     */
    template <typename T>
    void write_body_graphblas_doublet(std::ostream &os,
                                      matrix_market_header &header,
                                      const GrB_Vector& vec,
                                      const write_options& options) {
        std::vector<GrB_Index> indices(header.nnz);
        auto vals = std::make_unique<T[]>(header.nnz); // Cannot use vector due to bool specialization

        GrB_Index nvals = header.nnz;
        ok(GraphBLAS_typed<T>::GrB_Vector_extractTuples(indices.data(), vals.get(), &nvals, vec));

        // write the body
        vector_line_formatter<GrB_Index, T> lf(header, options);
        auto formatter = triplet_formatter(lf,
                                           indices.begin(), indices.end(),
                                           indices.begin(), indices.end(),
                                           vals.get(), vals.get() + header.nnz);
        write_body(os, formatter, options);
    }

#if FMM_GXB_ITERATORS && FMM_GXB_VECTOR_ITERATORS
    /**
     * Write a GraphBLAS vector to a coordinate MatrixMarket body by using GrB_Iterator.
     * This method is zero-copy, but uses a GraphBLAS extension.
     */
    template <typename T>
    void write_body_graphblas_iterator(std::ostream &os,
                                       matrix_market_header &header,
                                       const GrB_Vector& vec,
                                       const write_options& options) {
        vector_line_formatter<GrB_Index, T> lf(header, options);
        auto formatter = GrB_Matrix_Iterator_formatter<decltype(lf), T, gblas_vec_iter_impl>(lf, vec);
        write_body(os, formatter, options);
    }
#endif

    /**
     * Write a GraphBLAS vector to a MatrixMarket body
     */
    template <typename T>
    void write_body_graphblas(std::ostream &os,
                              matrix_market_header &header,
                              const GrB_Vector& vec,
                              const write_options& options) {
#if FMM_GXB_ITERATORS && FMM_GXB_VECTOR_ITERATORS
        write_body_graphblas_iterator<T>(os, header, vec, options);
#else
        write_body_graphblas_doublet<T>(os, header, vec, options);
#endif
    }

    /**
     * Write a GraphBLAS vector to MatrixMarket.
     */
    inline void write_matrix_market_graphblas(std::ostream &os,
                                              const GrB_Vector& vec,
                                              const write_options& options = {}, matrix_market_header header = {}) {
        GrB_Type type;
        {
            GrB_Index temp;

            ok(GrB_Vector_size(&temp, vec));
            header.vector_length = (int64_t)temp;

            ok(GrB_Vector_nvals(&temp, vec));
            header.nnz = (int64_t)temp;

            // get the matrix type
#if FMM_GXB_TYPE_NAME
            std::string mat_type(GxB_MAX_NAME_LEN, '\0');
            ok(GxB_Vector_type_name(mat_type.data(), vec));
            mat_type.resize(mat_type.find('\0')); // truncate to the actual type name length
            type = parse_type(mat_type);

            // Write the '%%GraphBLAS type <ctype>' line
            add_structured_comment(header, type, mat_type);
#else
            type = GrB_FP64; // most compatible type from the standard set.
#endif
        }

        header.object = vector;
        if (header.field != pattern && options.fill_header_field_type) {
            header.field = get_field(type);
        }
        header.format = coordinate;

        write_header(os, header, options);

        if (type == GrB_BOOL) {
            write_body_graphblas<bool>(os, header, vec, options);
        } else if (type == GrB_INT8) {
            write_body_graphblas<int8_t>(os, header, vec, options);
        } else if (type == GrB_INT16) {
            write_body_graphblas<int16_t>(os, header, vec, options);
        } else if (type == GrB_INT32) {
            write_body_graphblas<int32_t>(os, header, vec, options);
        } else if (type == GrB_INT64) {
            write_body_graphblas<int64_t>(os, header, vec, options);
        } else if (type == GrB_UINT8) {
            write_body_graphblas<uint8_t>(os, header, vec, options);
        } else if (type == GrB_UINT16) {
            write_body_graphblas<uint16_t>(os, header, vec, options);
        } else if (type == GrB_UINT32) {
            write_body_graphblas<uint32_t>(os, header, vec, options);
        } else if (type == GrB_UINT64) {
            write_body_graphblas<uint64_t>(os, header, vec, options);
        } else if (type == GrB_FP32) {
            write_body_graphblas<float>(os, header, vec, options);
        } else if (type == GrB_FP64) {
            write_body_graphblas<double>(os, header, vec, options);
#if FMM_GXB_COMPLEX
        } else if (type == GxB_FC32) {
            write_body_graphblas<std::complex<float>>(os, header, vec, options);
        } else if (type == GxB_FC64) {
            write_body_graphblas<std::complex<double>>(os, header, vec, options);
#endif
        } else {
            throw invalid_mm("Unknown vector type.");
        }
    }
}
