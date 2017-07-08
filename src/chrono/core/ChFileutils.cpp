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

#include <cmath>
#include <cstring>
#include <cstdio>

#include "chrono/core/ChFileutils.h"

#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
#include <direct.h>
#include <cerrno>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <cerrno>
#endif

namespace chrono {

void ChFileutils::Change_file_ext(char* fid1, const char* fid, const char* ext, int force) {
    char* p, *q;

    strcpy(fid1, fid);
    size_t l = strlen(fid1);
    p = fid1;
    for (size_t i = 0; i < l; i++)
        if (fid1[i] == '/')
            p = fid1 + i;

    if (!force) {
        q = strchr(p, '.');
        if (q && (q != fid1 + strlen(fid1) - 1))
            return;
    }
    if (!strchr(p, '.'))
        strcat(fid1, ".");
    q = strchr(p, '.');
    if (strlen(ext) > 0)
        q++;
    *q = 0;
    strcat(fid1, ext);
}

/* ----- cut off extension on a file identifier ----- */
void ChFileutils::Cut_file_ext(char* fid) {
    size_t l = strlen(fid);
    for (size_t i = 0; i < l; i++)
        if (fid[i] == '.')
            fid[i] = '\0';
}

/* ----- getext: get extension on a file identifier ----- */
void ChFileutils::Get_file_ext(const char* fid, char* ext) {
    size_t l = strlen(fid);
    size_t k = l - 1;
    for (size_t i = 0; i < l; i++)
        if (fid[i] == '.')
            k = i;

    for (size_t i = k + 1; i < l; i++)
        ext[i - k - 1] = fid[i];
    ext[l - k - 1] = '\0';
}

/* ----- get_file_size ------- */

int ChFileutils::Get_file_size(const char* fname) {
    int m, i;
    FILE* fp;

    if ((fp = fopen(fname, "r")) == NULL) {
        // printf ("Cannot open file to determine size: %s", fname);
        return -1;
    }

    m = 0;
    i = getc(fp);
    while (i != EOF) {
        m++;
        i = getc(fp);
    }
    fclose(fp);
    return m;
}

int ChFileutils::MakeDirectory(const char* dirname) {
#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    int res = _mkdir(dirname);  // Windows
#else
    int res = mkdir(dirname, 0777);  // Linux
#endif

    if (res == 0)
        return 0;

    return (errno == EEXIST) ? 1 : -1;
}

}  // end namespace chrono
