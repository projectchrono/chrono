// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// This is a modification of the code by J.W. Ratcliff (MIT license below)
//
// Copyright (c) 20011 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
//
//
// The MIT license:
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// =============================================================================

#include <cstdio>
#include <map>
#include <unordered_map>

#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTriangleMeshConnected)

#ifdef _WIN32
#define strcasecmp _stricmp
#endif

#pragma warning(disable : 4996)

typedef std::vector<int> IntVector;
typedef std::vector<float> FloatVector;

namespace WAVEFRONT {

/*******************************************************************/
/******************** InParser.h  ********************************/
/*******************************************************************/
class InPlaceParserInterface {
  public:
    virtual ~InPlaceParserInterface() {}
    virtual int ParseLine(
        int lineno,
        int argc,
        const char** argv) = 0;  // return true to continue parsing, return false to abort parsing process
};

enum SeparatorType {
    ST_DATA,  // is data
    ST_HARD,  // is a hard separator
    ST_SOFT,  // is a soft separator
    ST_EOS    // is a comment symbol, and everything past this character should be ignored
};

class InPlaceParser {
  public:
    InPlaceParser(void) { Init(); }

    InPlaceParser(char* data, int len) {
        Init();
        SetSourceData(data, len);
    }

    InPlaceParser(const char* fname) {
        Init();
        SetFile(fname);
    }

    ~InPlaceParser(void);

    void Init(void) {
        mQuoteChar = 34;
        mData = 0;
        mLen = 0;
        mMyAlloc = false;
        for (int i = 0; i < 256; i++) {
            mHard[i] = ST_DATA;
            mHardString[i * 2] = (char)i;
            mHardString[i * 2 + 1] = 0;
        }
        mHard[0] = ST_EOS;
        mHard[32] = ST_SOFT;
        mHard[9] = ST_SOFT;
        mHard[13] = ST_SOFT;
        mHard[10] = ST_SOFT;
    }

    void SetFile(const char* fname);  // use this file as source data to parse.

    void SetSourceData(char* data, int len) {
        mData = data;
        mLen = len;
        mMyAlloc = false;
    };

    int Parse(InPlaceParserInterface*
                  callback);  // returns true if entire file was parsed, false if it aborted for some reason

    int ProcessLine(int lineno, char* line, InPlaceParserInterface* callback);

    const char** GetArglist(char* source,
                            int& count);  // convert source string into an arg list, this is a destructive parse.

    void SetHardSeparator(char c)  // add a hard separator
    {
        mHard[c] = ST_HARD;
    }

    void SetHard(char c)  // add a hard separator
    {
        mHard[c] = ST_HARD;
    }

    void SetCommentSymbol(char c)  // comment character, treated as 'end of string'
    {
        mHard[c] = ST_EOS;
    }

    void ClearHardSeparator(char c) { mHard[c] = ST_DATA; }

    void DefaultSymbols(void);  // set up default symbols for hard seperator and comment symbol of the '#' character.

    bool EOS(char c) {
        if (mHard[c] == ST_EOS) {
            return true;
        }
        return false;
    }

    void SetQuoteChar(char c) { mQuoteChar = c; }

  private:
    inline char* AddHard(int& argc, const char** argv, char* foo);
    inline bool IsHard(char c);
    inline char* SkipSpaces(char* foo);
    inline bool IsWhiteSpace(char c);
    inline bool IsNonSeparator(char c);  // non seperator,neither hard nor soft

    bool mMyAlloc;  // whether or not *I* allocated the buffer and am responsible for deleting it.
    char* mData;    // ascii data to parse.
    int mLen;       // length of data
    SeparatorType mHard[256];
    char mHardString[256 * 2];
    char mQuoteChar;
};

/*******************************************************************/
/******************** InParser.cpp  ********************************/
/*******************************************************************/
void InPlaceParser::SetFile(const char* fname) {
    if (mMyAlloc) {
        free(mData);
    }
    mData = 0;
    mLen = 0;
    mMyAlloc = false;

    FILE* fph = fopen(fname, "rb");
    if (fph) {
        fseek(fph, 0L, SEEK_END);
        mLen = ftell(fph);
        fseek(fph, 0L, SEEK_SET);
        if (mLen) {
            mData = (char*)malloc(sizeof(char) * (mLen + 1));
            size_t ok = fread(mData, mLen, 1, fph);
            if (!ok) {
                free(mData);
                mData = 0;
            } else {
                mData[mLen] = 0;  // zero byte terminate end of file marker.
                mMyAlloc = true;
            }
        }
        fclose(fph);
    }
}

InPlaceParser::~InPlaceParser(void) {
    if (mMyAlloc) {
        free(mData);
    }
}

#define MAXARGS 512

bool InPlaceParser::IsHard(char c) {
    return mHard[c] == ST_HARD;
}

char* InPlaceParser::AddHard(int& argc, const char** argv, char* foo) {
    while (IsHard(*foo)) {
        const char* hard = &mHardString[*foo * 2];
        if (argc < MAXARGS) {
            argv[argc++] = hard;
        }
        foo++;
    }
    return foo;
}

bool InPlaceParser::IsWhiteSpace(char c) {
    return mHard[c] == ST_SOFT;
}

char* InPlaceParser::SkipSpaces(char* foo) {
    while (!EOS(*foo) && IsWhiteSpace(*foo))
        foo++;
    return foo;
}

bool InPlaceParser::IsNonSeparator(char c) {
    if (!IsHard(c) && !IsWhiteSpace(c) && c != 0)
        return true;
    return false;
}

int InPlaceParser::ProcessLine(int lineno, char* line, InPlaceParserInterface* callback) {
    int ret = 0;

    const char* argv[MAXARGS];
    int argc = 0;

    char* foo = line;

    while (!EOS(*foo) && argc < MAXARGS) {
        foo = SkipSpaces(foo);  // skip any leading spaces

        if (EOS(*foo))
            break;

        if (*foo == mQuoteChar)  // if it is an open quote
        {
            foo++;
            if (argc < MAXARGS) {
                argv[argc++] = foo;
            }
            while (!EOS(*foo) && *foo != mQuoteChar)
                foo++;
            if (!EOS(*foo)) {
                *foo = 0;  // replace close quote with zero byte EOS
                foo++;
            }
        } else {
            foo = AddHard(argc, argv, foo);  // add any hard separators, skip any spaces

            if (IsNonSeparator(*foo))  // add non-hard argument.
            {
                bool quote = false;
                if (*foo == mQuoteChar) {
                    foo++;
                    quote = true;
                }

                if (argc < MAXARGS) {
                    argv[argc++] = foo;
                }

                if (quote) {
                    while (*foo && *foo != mQuoteChar)
                        foo++;
                    if (*foo)
                        *foo = 32;
                }

                // continue..until we hit an eos ..
                while (!EOS(*foo))  // until we hit EOS
                {
                    if (IsWhiteSpace(*foo))  // if we hit a space, stomp a zero byte, and exit
                    {
                        *foo = 0;
                        foo++;
                        break;
                    } else if (IsHard(*foo))  // if we hit a hard separator, stomp a zero byte and store the hard
                    // separator argument
                    {
                        const char* hard = &mHardString[*foo * 2];
                        *foo = 0;
                        if (argc < MAXARGS) {
                            argv[argc++] = hard;
                        }
                        foo++;
                        break;
                    }
                    foo++;
                }  // end of while loop...
            }
        }
    }

    if (argc) {
        ret = callback->ParseLine(lineno, argc, argv);
    }

    return ret;
}

int InPlaceParser::Parse(
    InPlaceParserInterface* callback)  // returns true if entire file was parsed, false if it aborted for some reason
{
    assert(callback);
    if (!mData)
        return 0;

    int ret = 0;

    int lineno = 0;

    char* foo = mData;
    char* begin = foo;

    while (*foo) {
        if (*foo == 10 || *foo == 13) {
            lineno++;
            *foo = 0;

            if (*begin)  // if there is any data to parse at all...
            {
                int v = ProcessLine(lineno, begin, callback);
                if (v)
                    ret = v;
            }

            foo++;
            if (*foo == 10)
                foo++;  // skip line feed, if it is in the carraige-return line-feed format...
            begin = foo;
        } else {
            foo++;
        }
    }

    lineno++;  // lasst line.

    int v = ProcessLine(lineno, begin, callback);
    if (v)
        ret = v;
    return ret;
}

void InPlaceParser::DefaultSymbols(void) {
    SetHardSeparator(',');
    SetHardSeparator('(');
    SetHardSeparator(')');
    SetHardSeparator('=');
    SetHardSeparator('[');
    SetHardSeparator(']');
    SetHardSeparator('{');
    SetHardSeparator('}');
    SetCommentSymbol('#');
}

const char** InPlaceParser::GetArglist(
    char* line,
    int& count)  // convert source string into an arg list, this is a destructive parse.
{
    const char** ret = 0;

    static const char* argv[MAXARGS];
    int argc = 0;

    char* foo = line;

    while (!EOS(*foo) && argc < MAXARGS) {
        foo = SkipSpaces(foo);  // skip any leading spaces

        if (EOS(*foo))
            break;

        if (*foo == mQuoteChar)  // if it is an open quote
        {
            foo++;
            if (argc < MAXARGS) {
                argv[argc++] = foo;
            }
            while (!EOS(*foo) && *foo != mQuoteChar)
                foo++;
            if (!EOS(*foo)) {
                *foo = 0;  // replace close quote with zero byte EOS
                foo++;
            }
        } else {
            foo = AddHard(argc, argv, foo);  // add any hard separators, skip any spaces

            if (IsNonSeparator(*foo))  // add non-hard argument.
            {
                bool quote = false;
                if (*foo == mQuoteChar) {
                    foo++;
                    quote = true;
                }

                if (argc < MAXARGS) {
                    argv[argc++] = foo;
                }

                if (quote) {
                    while (*foo && *foo != mQuoteChar)
                        foo++;
                    if (*foo)
                        *foo = 32;
                }

                // continue..until we hit an eos ..
                while (!EOS(*foo))  // until we hit EOS
                {
                    if (IsWhiteSpace(*foo))  // if we hit a space, stomp a zero byte, and exit
                    {
                        *foo = 0;
                        foo++;
                        break;
                    } else if (IsHard(*foo))  // if we hit a hard separator, stomp a zero byte and store the hard
                    // separator argument
                    {
                        const char* hard = &mHardString[*foo * 2];
                        *foo = 0;
                        if (argc < MAXARGS) {
                            argv[argc++] = hard;
                        }
                        foo++;
                        break;
                    }
                    foo++;
                }  // end of while loop...
            }
        }
    }

    count = argc;
    if (argc) {
        ret = argv;
    }

    return ret;
}

/*******************************************************************/
/******************** Geometry.h  ********************************/
/*******************************************************************/

class GeometryVertex {
  public:
    float mPos[3];
    float mNormal[3];
    float mTexel[2];
};

class GeometryInterface {
  public:
    virtual ~GeometryInterface() {}
    virtual void NodeTriangle(const GeometryVertex* /*v1*/,
                              const GeometryVertex* /*v2*/,
                              const GeometryVertex* /*v3*/,
                              bool /*textured*/) {}
};

/*******************************************************************/
/******************** Obj.h  ********************************/
/*******************************************************************/

class OBJ : public InPlaceParserInterface {
  public:
    int LoadMesh(const char* fname, GeometryInterface* callback, bool textured);
    virtual int ParseLine(
        int lineno,
        int argc,
        const char** argv) override;  // return true to continue parsing, return false to abort parsing process
  private:
    void GetVertex(GeometryVertex& v, const char* face) const;

  public:  //***ALEX***
    FloatVector mVerts;
    FloatVector mTexels;
    FloatVector mNormals;

    //***ALEX***
    IntVector mIndexesVerts;
    IntVector mIndexesNormals;
    IntVector mIndexesTexels;

    bool mTextured;

    GeometryInterface* mCallback;
};

/*******************************************************************/
/******************** Obj.cpp  ********************************/
/*******************************************************************/

int OBJ::LoadMesh(const char* fname, GeometryInterface* iface, bool textured) {
    mTextured = textured;
    int ret = 0;

    mVerts.clear();
    mTexels.clear();
    mNormals.clear();

    //***ALEX***
    mIndexesVerts.clear();
    mIndexesNormals.clear();
    mIndexesTexels.clear();

    mCallback = iface;

    InPlaceParser ipp(fname);

    ipp.Parse(this);

    return ret;
}

/***
static const char * GetArg(const char **argv,int i,int argc)
{
const char * ret = 0;
if ( i < argc ) ret = argv[i];
return ret;
}
****/

void OBJ::GetVertex(GeometryVertex& v, const char* face) const {
    v.mPos[0] = 0;
    v.mPos[1] = 0;
    v.mPos[2] = 0;

    v.mTexel[0] = 0;
    v.mTexel[1] = 0;

    v.mNormal[0] = 0;
    v.mNormal[1] = 1;
    v.mNormal[2] = 0;

    int index = atoi(face) - 1;

    const char* texel = strstr(face, "/");

    if (texel) {
        int tindex = atoi(texel + 1) - 1;

        if (tindex >= 0 && tindex < (int)(mTexels.size() / 2)) {
            const float* t = &mTexels[tindex * 2];

            v.mTexel[0] = t[0];
            v.mTexel[1] = t[1];
        }

        const char* normal = strstr(texel + 1, "/");
        if (normal) {
            int nindex = atoi(normal + 1) - 1;

            if (nindex >= 0 && nindex < (int)(mNormals.size() / 3)) {
                const float* n = &mNormals[nindex * 3];

                v.mNormal[0] = n[0];
                v.mNormal[1] = n[1];
                v.mNormal[2] = n[2];
            }
        }
    }

    if (index >= 0 && index < (int)(mVerts.size() / 3)) {
        const float* p = &mVerts[index * 3];

        v.mPos[0] = p[0];
        v.mPos[1] = p[1];
        v.mPos[2] = p[2];
    }
}

int OBJ::ParseLine(int /*lineno*/,
                   int argc,
                   const char** argv)  // return true to continue parsing, return false to abort parsing process
{
    int ret = 0;

    if (argc >= 1) {
        const char* foo = argv[0];
        if (*foo != '#') {
            if (strcasecmp(argv[0], "v") == 0 && argc == 4) {
                float vx = (float)atof(argv[1]);
                float vy = (float)atof(argv[2]);
                float vz = (float)atof(argv[3]);
                mVerts.push_back(vx);
                mVerts.push_back(vy);
                mVerts.push_back(vz);
            } else if (strcasecmp(argv[0], "vt") == 0 && (argc == 3 || argc == 4)) {
                // ignore 3rd component if present
                float tx = (float)atof(argv[1]);
                float ty = (float)atof(argv[2]);
                mTexels.push_back(tx);
                mTexels.push_back(ty);
            } else if (strcasecmp(argv[0], "vn") == 0 && argc == 4) {
                float normalx = (float)atof(argv[1]);
                float normaly = (float)atof(argv[2]);
                float normalz = (float)atof(argv[3]);
                mNormals.push_back(normalx);
                mNormals.push_back(normaly);
                mNormals.push_back(normalz);
            } else if (strcasecmp(argv[0], "f") == 0 && argc >= 4) {
                // ***ALEX*** do not use the BuildMesh stuff
                int vcount = argc - 1;
                const char* argvT[3];
                argvT[0] = argv[1];  // pivot for triangle fans when quad/poly face
                for (int i = 1; i < argc; i++) {
                    if (i >= 3) {
                        argvT[1] = argv[i - 1];
                        argvT[2] = argv[i];

                        // do a fan triangle here..
                        for (int ip = 0; ip < 3; ++ip) {
                            // the index of i-th vertex
                            int index = atoi(argvT[ip]) - 1;
                            this->mIndexesVerts.push_back(index);

                            const char* texel = strstr(argvT[ip], "/");
                            if (texel) {
                                // the index of i-th texel
                                int tindex = atoi(texel + 1) - 1;
                                // If input file only specifies a face w/ verts, normals, this is -1.
                                // Don't push index to array if this happens
                                if (tindex > -1) {
                                    mIndexesTexels.push_back(tindex);
                                }

                                const char* normal = strstr(texel + 1, "/");
                                if (normal) {
                                    // the index of i-th normal
                                    int nindex = atoi(normal + 1) - 1;
                                    this->mIndexesNormals.push_back(nindex);
                                }
                            }
                        }
                    }
                }

                /* ***ALEX***
               GeometryVertex v[32];

               int vcount = argc-1;

               for (int i=1; i<argc; i++)
               {
               GetVertex(v[i-1],argv[i] );
               }


               mCallback->NodeTriangle(&v[0],&v[1],&v[2], mTextured);

               if ( vcount >=3 ) // do the fan
               {
               for (int i=2; i<(vcount-1); i++)
               {
               mCallback->NodeTriangle(&v[0],&v[i],&v[i+1], mTextured);
               }
               }
               */
            }
        }
    }

    return ret;
}

class BuildMesh : public GeometryInterface {
  public:
    int GetIndex(const float* p, const float* texCoord) {
        int vcount = (int)mVertices.size() / 3;

        if (vcount > 0) {
            // New MS STL library checks indices in debug build, so zero causes an assert if it is empty.
            const float* v = &mVertices[0];
            const float* t = texCoord != NULL ? &mTexCoords[0] : NULL;

            for (int i = 0; i < vcount; i++) {
                if (v[0] == p[0] && v[1] == p[1] && v[2] == p[2]) {
                    if (texCoord == NULL || (t[0] == texCoord[0] && t[1] == texCoord[1])) {
                        return i;
                    }
                }
                v += 3;
                if (t != NULL)
                    t += 2;
            }
        }

        mVertices.push_back(p[0]);
        mVertices.push_back(p[1]);
        mVertices.push_back(p[2]);

        if (texCoord != NULL) {
            mTexCoords.push_back(texCoord[0]);
            mTexCoords.push_back(texCoord[1]);
        }

        return vcount;
    }

    virtual void NodeTriangle(const GeometryVertex* v1,
                              const GeometryVertex* v2,
                              const GeometryVertex* v3,
                              bool textured) override {
        mIndices.push_back(GetIndex(v1->mPos, textured ? v1->mTexel : NULL));
        mIndices.push_back(GetIndex(v2->mPos, textured ? v2->mTexel : NULL));
        mIndices.push_back(GetIndex(v3->mPos, textured ? v3->mTexel : NULL));
    }

    const FloatVector& GetVertices(void) const { return mVertices; };
    const FloatVector& GetTexCoords(void) const { return mTexCoords; };
    const IntVector& GetIndices(void) const { return mIndices; };

  private:
    FloatVector mVertices;
    FloatVector mTexCoords;
    IntVector mIndices;
};

}  // end namespace WAVEFRONT


// -----------------------------------------------------------------------------

ChTriangleMeshConnected::ChTriangleMeshConnected(const ChTriangleMeshConnected& source) {
    m_vertices = source.m_vertices;
    m_normals = source.m_normals;
    m_UV = source.m_UV;
    m_colors = source.m_colors;

    m_face_v_indices = source.m_face_v_indices;
    m_face_n_indices = source.m_face_n_indices;
    m_face_uv_indices = source.m_face_uv_indices;
    m_face_col_indices = source.m_face_col_indices;
}

// Following function is a modified version of:
//
// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)
//
// void ComputeMassProperties (const Vector3<Real>* vertices, int numTriangles,
//                             const int* indices, bool bodyCoords, Real& mass,
//                             Vector3<Real>& center, Matrix3<Real>& inertia)
//
void ChTriangleMeshConnected::ComputeMassProperties(bool bodyCoords,
                                                    double& mass,
                                                    ChVector<>& center,
                                                    ChMatrix33<>& inertia) {
    const double oneDiv6 = (double)(1.0 / 6.0);
    const double oneDiv24 = (double)(1.0 / 24.0);
    const double oneDiv60 = (double)(1.0 / 60.0);
    const double oneDiv120 = (double)(1.0 / 120.0);

    // order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
    double integral[10] = {(double)0.0, (double)0.0, (double)0.0, (double)0.0, (double)0.0,
                           (double)0.0, (double)0.0, (double)0.0, (double)0.0, (double)0.0};

    for (int i = 0; i < this->getNumTriangles(); i++) {
        // Get vertices of triangle i.
        ChVector<double> v0 = this->m_vertices[m_face_v_indices[i].x()];
        ChVector<double> v1 = this->m_vertices[m_face_v_indices[i].y()];
        ChVector<double> v2 = this->m_vertices[m_face_v_indices[i].z()];

        // Get cross product of edges and normal vector.
        ChVector<double> V1mV0 = v1 - v0;
        ChVector<double> V2mV0 = v2 - v0;
        ChVector<double> N = Vcross(V1mV0, V2mV0);

        // Compute integral terms.
        double tmp0, tmp1, tmp2;
        double f1x, f2x, f3x, g0x, g1x, g2x;
        tmp0 = v0.x() + v1.x();
        f1x = tmp0 + v2.x();
        tmp1 = v0.x() * v0.x();
        tmp2 = tmp1 + v1.x() * tmp0;
        f2x = tmp2 + v2.x() * f1x;
        f3x = v0.x() * tmp1 + v1.x() * tmp2 + v2.x() * f2x;
        g0x = f2x + v0.x() * (f1x + v0.x());
        g1x = f2x + v1.x() * (f1x + v1.x());
        g2x = f2x + v2.x() * (f1x + v2.x());

        double f1y, f2y, f3y, g0y, g1y, g2y;
        tmp0 = v0.y() + v1.y();
        f1y = tmp0 + v2.y();
        tmp1 = v0.y() * v0.y();
        tmp2 = tmp1 + v1.y() * tmp0;
        f2y = tmp2 + v2.y() * f1y;
        f3y = v0.y() * tmp1 + v1.y() * tmp2 + v2.y() * f2y;
        g0y = f2y + v0.y() * (f1y + v0.y());
        g1y = f2y + v1.y() * (f1y + v1.y());
        g2y = f2y + v2.y() * (f1y + v2.y());

        double f1z, f2z, f3z, g0z, g1z, g2z;
        tmp0 = v0.z() + v1.z();
        f1z = tmp0 + v2.z();
        tmp1 = v0.z() * v0.z();
        tmp2 = tmp1 + v1.z() * tmp0;
        f2z = tmp2 + v2.z() * f1z;
        f3z = v0.z() * tmp1 + v1.z() * tmp2 + v2.z() * f2z;
        g0z = f2z + v0.z() * (f1z + v0.z());
        g1z = f2z + v1.z() * (f1z + v1.z());
        g2z = f2z + v2.z() * (f1z + v2.z());

        // Update integrals.
        integral[0] += N.x() * f1x;
        integral[1] += N.x() * f2x;
        integral[2] += N.y() * f2y;
        integral[3] += N.z() * f2z;
        integral[4] += N.x() * f3x;
        integral[5] += N.y() * f3y;
        integral[6] += N.z() * f3z;
        integral[7] += N.x() * (v0.y() * g0x + v1.y() * g1x + v2.y() * g2x);
        integral[8] += N.y() * (v0.z() * g0y + v1.z() * g1y + v2.z() * g2y);
        integral[9] += N.z() * (v0.x() * g0z + v1.x() * g1z + v2.x() * g2z);
    }

    integral[0] *= oneDiv6;
    integral[1] *= oneDiv24;
    integral[2] *= oneDiv24;
    integral[3] *= oneDiv24;
    integral[4] *= oneDiv60;
    integral[5] *= oneDiv60;
    integral[6] *= oneDiv60;
    integral[7] *= oneDiv120;
    integral[8] *= oneDiv120;
    integral[9] *= oneDiv120;

    // mass
    mass = integral[0];

    // center of mass
    center = ChVector<double>(integral[1], integral[2], integral[3]) / mass;

    // inertia relative to world origin
    inertia[0][0] = integral[5] + integral[6];
    inertia[0][1] = -integral[7];
    inertia[0][2] = -integral[9];
    inertia[1][0] = inertia[0][1];
    inertia[1][1] = integral[4] + integral[6];
    inertia[1][2] = -integral[8];
    inertia[2][0] = inertia[0][2];
    inertia[2][1] = inertia[1][2];
    inertia[2][2] = integral[4] + integral[5];

    // inertia relative to center of mass
    if (bodyCoords) {
        inertia[0][0] -= mass * (center.y() * center.y() + center.z() * center.z());
        inertia[0][1] += mass * center.x() * center.y();
        inertia[0][2] += mass * center.z() * center.x();
        inertia[1][0] = inertia[0][1];
        inertia[1][1] -= mass * (center.z() * center.z() + center.x() * center.x());
        inertia[1][2] += mass * center.y() * center.z();
        inertia[2][0] = inertia[0][2];
        inertia[2][1] = inertia[1][2];
        inertia[2][2] -= mass * (center.x() * center.x() + center.y() * center.y());
    }
}

using namespace WAVEFRONT;

void ChTriangleMeshConnected::LoadWavefrontMesh(std::string filename, bool load_normals, bool load_uv) {
    this->m_vertices.clear();
    this->m_normals.clear();
    this->m_UV.clear();
    this->m_face_v_indices.clear();
    this->m_face_n_indices.clear();
    this->m_face_uv_indices.clear();

    GeometryInterface emptybm;  // BuildMesh bm;

    m_filename = filename;

    OBJ obj;

    obj.LoadMesh(filename.c_str(), &emptybm, true);

    for (unsigned int iv = 0; iv < obj.mVerts.size(); iv += 3) {
        this->m_vertices.push_back(ChVector<double>(obj.mVerts[iv], obj.mVerts[iv + 1], obj.mVerts[iv + 2]));
    }
    for (unsigned int in = 0; in < obj.mNormals.size(); in += 3) {
        this->m_normals.push_back(ChVector<double>(obj.mNormals[in], obj.mNormals[in + 1], obj.mNormals[in + 2]));
    }
    for (unsigned int it = 0; it < obj.mTexels.size(); it += 2)  // +2 because only u,v each texel
    {
        this->m_UV.push_back(ChVector<double>(obj.mTexels[it], obj.mTexels[it + 1], 0));
    }
    for (unsigned int iiv = 0; iiv < obj.mIndexesVerts.size(); iiv += 3) {
        this->m_face_v_indices.push_back(
            ChVector<int>(obj.mIndexesVerts[iiv], obj.mIndexesVerts[iiv + 1], obj.mIndexesVerts[iiv + 2]));
    }
    for (unsigned int iin = 0; iin < obj.mIndexesNormals.size(); iin += 3) {
        this->m_face_n_indices.push_back(
            ChVector<int>(obj.mIndexesNormals[iin], obj.mIndexesNormals[iin + 1], obj.mIndexesNormals[iin + 2]));
    }
    for (unsigned int iit = 0; iit < obj.mIndexesTexels.size(); iit += 3) {
        this->m_face_uv_indices.push_back(
            ChVector<int>(obj.mIndexesTexels[iit], obj.mIndexesTexels[iit + 1], obj.mIndexesTexels[iit + 2]));
    }

    if (!load_normals) {
        this->m_normals.clear();
        this->m_face_n_indices.clear();
    }
    if (!load_uv) {
        this->m_UV.clear();
        this->m_face_uv_indices.clear();
    }
}

/*
using namespace WAVEFRONT;


WavefrontObj::WavefrontObj(void)
{
    mVertexCount = 0;
    mTriCount    = 0;
    mIndices     = 0;
    mVertices    = NULL;
    mTexCoords   = NULL;
}

WavefrontObj::~WavefrontObj(void)
{
    delete mIndices;
    delete mVertices;
}

unsigned int WavefrontObj::loadObj(const char *fname, bool textured) // load a wavefront obj returns number of triangles
that were loaded.  Data is persists until the class is destructed.
{

    unsigned int ret = 0;

    delete mVertices;
    mVertices = 0;
    delete mIndices;
    mIndices = 0;
    mVertexCount = 0;
    mTriCount = 0;


  BuildMesh bm;

  OBJ obj;

  obj.LoadMesh(fname,&bm, textured);


    const FloatVector &vlist = bm.GetVertices();
    const IntVector &indices = bm.GetIndices();
    if ( vlist.size() )
    {
        mVertexCount = (int)vlist.size()/3;
        mVertices = new float[mVertexCount*3];
        memcpy( mVertices, &vlist[0], sizeof(float)*mVertexCount*3 );

        if (textured)
        {
            mTexCoords = new float[mVertexCount * 2];
            const FloatVector& tList = bm.GetTexCoords();
            memcpy( mTexCoords, &tList[0], sizeof(float) * mVertexCount * 2);
        }

        mTriCount = (int)indices.size()/3;
        mIndices = new int[mTriCount*3*sizeof(int)];
        memcpy(mIndices, &indices[0], sizeof(int)*mTriCount*3);
        ret = mTriCount;
    }


    return ret;
}


bool WavefrontObj::saveObj(const char *fname,int vcount,const float *vertices,int tcount,const int *indices)
{
  bool ret = false;

  FILE *fph = fopen(fname,"wb");
  if ( fph )
  {
    for (int i=0; i<vcount; i++)
    {
      fprintf(fph,"v %0.9f %0.9f %0.9f\r\n", vertices[0], vertices[1], vertices[2] );
      vertices+=3;
    }
    for (int i=0; i<tcount; i++)
    {
      fprintf(fph,"f %d %d %d\r\n", indices[0]+1, indices[1]+1, indices[2]+1 );
      indices+=3;
    }
    fclose(fph);
    ret = true;
  }
  return ret;
}

*/

void ChTriangleMeshConnected::Transform(const ChVector<> displ, const ChMatrix33<> rotscale) {
    for (int i = 0; i < m_vertices.size(); ++i) {
        m_vertices[i] = rotscale * m_vertices[i];
        m_vertices[i] += displ;
    }
    for (int i = 0; i < m_normals.size(); ++i) {
        m_normals[i] = rotscale * m_normals[i];
        m_normals[i].Normalize();
    }
}

bool ChTriangleMeshConnected::ComputeNeighbouringTriangleMap(std::vector<std::array<int, 4>>& tri_map) const {
    bool pathological_edges = false;

    std::multimap<std::pair<int, int>, int> edge_map;

    for (int it = 0; it < this->m_face_v_indices.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<int, int> medgeA(this->m_face_v_indices[it].x(), this->m_face_v_indices[it].y());
        std::pair<int, int> medgeB(this->m_face_v_indices[it].y(), this->m_face_v_indices[it].z());
        std::pair<int, int> medgeC(this->m_face_v_indices[it].z(), this->m_face_v_indices[it].x());
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
        edge_map.insert({medgeA, it});
        edge_map.insert({medgeB, it});
        edge_map.insert({medgeC, it});
    }

    // Create a map of neighbouring triangles, vector of:
    // [Ti TieA TieB TieC]
    tri_map.resize(this->m_face_v_indices.size());
    for (int it = 0; it < this->m_face_v_indices.size(); ++it) {
        tri_map[it][0] = it;
        tri_map[it][1] = -1;  // default no neighbour
        tri_map[it][2] = -1;  // default no neighbour
        tri_map[it][3] = -1;  // default no neighbour
        // edges = pairs of vertexes indexes
        std::pair<int, int> medgeA(this->m_face_v_indices[it].x(), this->m_face_v_indices[it].y());
        std::pair<int, int> medgeB(this->m_face_v_indices[it].y(), this->m_face_v_indices[it].z());
        std::pair<int, int> medgeC(this->m_face_v_indices[it].z(), this->m_face_v_indices[it].x());
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
        if (edge_map.count(medgeA) > 2 || edge_map.count(medgeB) > 2 || edge_map.count(medgeC) > 2) {
            pathological_edges = true;
            // GetLog() << "Warning, edge shared with more than two triangles! \n";
        }
        auto retA = edge_map.equal_range(medgeA);
        for (auto fedge = retA.first; fedge != retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][1] = fedge->second;
                break;
            }
        }
        auto retB = edge_map.equal_range(medgeB);
        for (auto fedge = retB.first; fedge != retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][2] = fedge->second;
                break;
            }
        }
        auto retC = edge_map.equal_range(medgeC);
        for (auto fedge = retC.first; fedge != retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][3] = fedge->second;
                break;
            }
        }
    }
    return pathological_edges;
}

bool ChTriangleMeshConnected::ComputeWingedEdges(std::map<std::pair<int, int>, std::pair<int, int>>& winged_edges,
                                                 bool allow_single_wing) const {
    bool pathological_edges = false;

    std::multimap<std::pair<int, int>, int> edge_map;

    for (int it = 0; it < this->m_face_v_indices.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<int, int> medgeA(this->m_face_v_indices[it].x(), this->m_face_v_indices[it].y());
        std::pair<int, int> medgeB(this->m_face_v_indices[it].y(), this->m_face_v_indices[it].z());
        std::pair<int, int> medgeC(this->m_face_v_indices[it].z(), this->m_face_v_indices[it].x());
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
        edge_map.insert({medgeA, it});
        edge_map.insert({medgeB, it});
        edge_map.insert({medgeC, it});
    }

    for (auto aedge = edge_map.begin(); aedge != edge_map.end(); ++aedge) {
        auto ret = edge_map.equal_range(aedge->first);
        int nt = 0;
        std::pair<int, int> wingedge;
        std::pair<int, int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge = ret.first; fedge != ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second = fedge->first.second;
            if (nt == 0)
                wingtri.first = fedge->second;
            if (nt == 1)
                wingtri.second = fedge->second;
            ++nt;
            if (nt == 2)
                break;
        }
        if ((nt == 2) || ((nt == 1) && allow_single_wing)) {
            winged_edges.insert(
                std::pair<std::pair<int, int>, std::pair<int, int>>(wingedge, wingtri));  // ok found winged edge!
            aedge->second = -1;  // deactivate this way otherwise found again by sister
        }
        if (nt == 3) {
            pathological_edges = true;
            // GetLog() << "Warning: winged edge between "<< wing[0] << " and " << wing[1]  << " shared with more than
            // two triangles.\n";
        }
    }
    return pathological_edges;
}

int ChTriangleMeshConnected::RepairDuplicateVertexes(const double tolerance) {
    int nmerged = 0;
    std::vector<ChVector<>> processed_verts;
    std::vector<int> new_indexes(m_vertices.size());

    // merge vertexes
    for (int i = 0; i < this->m_vertices.size(); ++i) {
        bool tomerge = false;
        for (int j = 0; j < processed_verts.size(); ++j) {
            if ((m_vertices[i] - processed_verts[j]).Length2() < tolerance) {
                tomerge = true;
                ++nmerged;
                new_indexes[i] = j;
                break;
            }
        }
        if (!tomerge) {
            processed_verts.push_back(m_vertices[i]);
            new_indexes[i] = (int)processed_verts.size() - 1;
        }
    }

    this->m_vertices = processed_verts;

    // update the merged vertexes also in face indexes to vertexes
    for (int i = 0; i < this->m_face_v_indices.size(); ++i) {
        m_face_v_indices[i].x() = new_indexes[m_face_v_indices[i].x()];
        m_face_v_indices[i].y() = new_indexes[m_face_v_indices[i].y()];
        m_face_v_indices[i].z() = new_indexes[m_face_v_indices[i].z()];
    }
    for (int i = 0; i < this->m_face_n_indices.size(); ++i) {
        m_face_n_indices[i].x() = new_indexes[m_face_n_indices[i].x()];
        m_face_n_indices[i].y() = new_indexes[m_face_n_indices[i].y()];
        m_face_n_indices[i].z() = new_indexes[m_face_n_indices[i].z()];
    }
    for (int i = 0; i < this->m_face_uv_indices.size(); ++i) {
        m_face_uv_indices[i].x() = new_indexes[m_face_uv_indices[i].x()];
        m_face_uv_indices[i].y() = new_indexes[m_face_uv_indices[i].y()];
        m_face_uv_indices[i].z() = new_indexes[m_face_uv_indices[i].z()];
    }
    for (int i = 0; i < this->m_face_col_indices.size(); ++i) {
        m_face_col_indices[i].x() = new_indexes[m_face_col_indices[i].x()];
        m_face_col_indices[i].y() = new_indexes[m_face_col_indices[i].y()];
        m_face_col_indices[i].z() = new_indexes[m_face_col_indices[i].z()];
    }

    return nmerged;
}

// Offset algorithm based on:
// " A 3D surface offset method for STL-format models"
//   Xiuzhi Qu and Brent Stucker

bool ChTriangleMeshConnected::MakeOffset(const double moffset) {
    std::map<int, std::vector<int>> map_vertex_triangles;
    std::vector<ChVector<>> voffsets(this->m_vertices.size());

    // build the topological info for triangles connected to vertex
    for (int i = 0; i < this->m_face_v_indices.size(); ++i) {
        map_vertex_triangles[m_face_v_indices[i].x()].push_back(i);
        map_vertex_triangles[m_face_v_indices[i].y()].push_back(i);
        map_vertex_triangles[m_face_v_indices[i].z()].push_back(i);
    }

    // scan through vertexes and offset them
    for (int i = 0; i < this->m_vertices.size(); ++i) {
        auto mpair = map_vertex_triangles.find(i);
        if (mpair != map_vertex_triangles.end()) {
            std::vector<int>& mverttriangles = mpair->second;
            int ntri = (int)mverttriangles.size();
            ChMatrixDynamic<> A(ntri, ntri);
            ChMatrixDynamic<> b(ntri, 1);
            ChMatrixDynamic<> x(ntri, 1);
            for (int j = 0; j < ntri; ++j) {
                b(j, 0) = 1;
                for (int k = 0; k < ntri; ++k) {
                    A(j, k) = Vdot(this->getTriangle(mverttriangles[j]).GetNormal(),
                                   this->getTriangle(mverttriangles[k]).GetNormal());
                }
            }
            ChLinearAlgebra::Solve_LinSys(A, &b, &x);

            // weighted sum as offset vector
            voffsets[i] = VNULL;
            for (int j = 0; j < ntri; ++j) {
                voffsets[i] += this->getTriangle(mverttriangles[j]).GetNormal() * x(j);
            }
        }
    }

    // apply offset vectors to itself:
    for (int i = 0; i < this->m_vertices.size(); ++i) {
        m_vertices[i] += voffsets[i] * moffset;
    }

    return true;
}


/// Return the indexes of the two vertexes of the i-th edge of the triangle
std::pair<int, int> ChTriangleMeshConnected::GetTriangleEdgeIndexes (
        const std::vector<ChVector<int>>& indexes, ///< indexes, xyz per each face, ex. getIndicesVertexes()
        int it,      ///< triangle id
        int nedge,   ///< number of edge: 0,1,2 
        bool unique  ///< if true, swaps the pair so that 1st is always < 2nd id, so can test sharing wiht other triangle
        ) {
        std::pair<int, int> medge;
        if (nedge ==0)
            medge = {indexes[it].x(),  indexes[it].y()};
        if (nedge ==1)
            medge = {indexes[it].y(),  indexes[it].z()};
        if (nedge ==2)
            medge = {indexes[it].z(),  indexes[it].x()};
        if (unique) {
            // unique? sort always in increasing order to avoid ambiguous duplicated edges
            if (medge.first > medge.second)
                medge = std::pair<int, int>(medge.second, medge.first);
        }
        return medge;
}



bool InterpolateAndInsert(ChTriangleMeshConnected& mesh, int ibuffer, int i1, int i2, int& created_index ) {
    switch (ibuffer) {
        case 0: {
            if (mesh.m_vertices.empty()) 
                return false;
            ChVector<> Vnew = (mesh.m_vertices[i1] + mesh.m_vertices[i2]) * 0.5;
            mesh.m_vertices.push_back(Vnew);
            created_index = (int)mesh.m_vertices.size()-1;
            return true;
        }
        case 1: {
            if (mesh.m_normals.empty()) 
                return false;
            ChVector<> Vnew = (mesh.m_normals[i1] + mesh.m_normals[i2]) * 0.5;
            Vnew.Normalize();
            mesh.m_normals.push_back(Vnew);
            created_index = (int)mesh.m_normals.size()-1;
            return true;
        }
        case 2: {
            if (mesh.m_UV.empty()) 
                return false;
            ChVector<> Vnew = (mesh.m_UV[i1] + mesh.m_UV[i2]) * 0.5;
            mesh.m_UV.push_back(Vnew);
            created_index = (int)mesh.m_UV.size()-1;
            return true;
        }
        case 3: {
            if (mesh.m_colors.empty()) 
                return false;
            ChVector<float> Vnew = (mesh.m_colors[i1] + mesh.m_colors[i2]) * 0.5;
            mesh.m_colors.push_back(Vnew);
            created_index = (int)mesh.m_colors.size()-1;
            return true;
        }
        default:
            return false;
    }
}


// Split an edge by inserting a point in the middle
// It also updates uv buffer, normals buffer, etc. and recomputes neighbouring map.
bool ChTriangleMeshConnected::SplitEdge (
        int itA,      ///< triangle index,
        int itB,      ///< triangle index, -1 if not existing (means free edge on A)
        int neA,      ///< n.edge on tri A: 0,1,2
        int neB,      ///< n.edge on tri B: 0,1,2
        int& itA_1,   ///< returns the index of split triangle A, part1
        int& itA_2,   ///< returns the index of split triangle A, part2
        int& itB_1,   ///< returns the index of split triangle B, part1
        int& itB_2,   ///< returns the index of split triangle B, part2
        std::vector<std::array<int, 4>>& tri_map, ///< triangle neighbouring map
        std::vector<std::vector<double>*>& aux_data_double, ///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<int>*>& aux_data_int,       ///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<bool>*>& aux_data_bool,      ///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<ChVector<>>*>& aux_data_vect///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        ) {
    
    std::array<std::vector<ChVector<int>>*, 4> face_indexes;
    face_indexes[0]=&m_face_v_indices;
    face_indexes[1]=&m_face_n_indices; 
    face_indexes[2]=&m_face_uv_indices; 
    face_indexes[3]=&m_face_col_indices;

    int iea, ieb;

    for (int ibuffer = 0; ibuffer < 4; ++ibuffer) {

        if (face_indexes[ibuffer]->size()) {

            // case where one used normals, uv, color buffers, with custom face indexes:
            std::pair<int, int> eA; 
            eA = ChTriangleMeshConnected::GetTriangleEdgeIndexes(*face_indexes[ibuffer], itA,neA,false);
            std::pair<int, int> eAB =eA;
            std::pair<int, int> eB;
            if (itB != -1)
                eB = ChTriangleMeshConnected::GetTriangleEdgeIndexes(*face_indexes[ibuffer], itB,neB,false);
            bool swapA = false;
            if (eA.first > eA.second) {
                swapA = true;
                eAB = {eA.second, eA.first};
            }
            bool swapB = false;
            if (eB.first > eB.second)
                swapB = true;
    
            // average new vertex/property and add it
            int iVnew = -1;
            InterpolateAndInsert(*this, ibuffer, eAB.first, eAB.second, iVnew);

            if (ibuffer==0) {
                iea = eAB.first;
                ieb = eAB.second;
            }

            itA_1 = -1;
            itA_2 = -1;
            itB_1 = -1;
            itB_2 = -1;

            // Split triangle A in two (reuse existing, and allocate one new)
            ChVector<int> tA_1 = face_indexes[ibuffer]->at(itA);
            if (tA_1.x() == eAB.first) tA_1.x() = iVnew;
            if (tA_1.y() == eAB.first) tA_1.y() = iVnew;
            if (tA_1.z() == eAB.first) tA_1.z() = iVnew;
            ChVector<int> tA_2 = face_indexes[ibuffer]->at(itA);
            if (tA_2.x() == eAB.second) tA_2.x() = iVnew;
            if (tA_2.y() == eAB.second) tA_2.y() = iVnew;
            if (tA_2.z() == eAB.second) tA_2.z() = iVnew;
            face_indexes[ibuffer]->at(itA) = tA_1; // reuse face
            itA_1 = itA;
            face_indexes[ibuffer]->push_back(tA_2); // allocate new face
            itA_2 = (int)face_indexes[ibuffer]->size() -1;

            // Split triangle B in two (reuse existing, and allocate one new)
            if (itB != -1) {
                ChVector<int> tB_1 = face_indexes[ibuffer]->at(itB);
                if (tB_1.x() == eAB.first) tB_1.x() = iVnew;
                if (tB_1.y() == eAB.first) tB_1.y() = iVnew;
                if (tB_1.z() == eAB.first) tB_1.z() = iVnew;
                ChVector<int> tB_2 = face_indexes[ibuffer]->at(itB);
                if (tB_2.x() == eAB.second) tB_2.x() = iVnew;
                if (tB_2.y() == eAB.second) tB_2.y() = iVnew;
                if (tB_2.z() == eAB.second) tB_2.z() = iVnew;
                face_indexes[ibuffer]->at(itB) = tB_1; // reuse face 
                itB_1 = itB;
                face_indexes[ibuffer]->push_back(tB_2); // allocate new face
                itB_2 = (int)face_indexes[ibuffer]->size() -1;
            }

            // for m_face_v_indices buffer (vertex indexes) only:
            if (ibuffer ==0)  {
                
                // Update triangle neighbouring map

                std::array<int, 4> topo_A_1 = tri_map[itA];
                std::array<int, 4> topo_A_2 = tri_map[itA];
                topo_A_1[1+neA] = itB_1;
                topo_A_2[1+neA] = itB_2;
                int is1 = 1+((neA+2)%3);
                int is2 = 1+((neA+1)%3);
                if (swapA) std::swap(is1,is2);
                topo_A_1[is1] = itA_2;
                topo_A_2[is2] = itA_1;
                int itD = topo_A_1[is2];
                int itC = topo_A_2[is1];
                for (int in = 1; in<4; ++in)
                    if (tri_map[itD][in] == itA)
                        tri_map[itD][in] = itA_1; // not needed?
                for (int in = 1; in<4; ++in)
                    if (tri_map[itC][in] == itA)
                        tri_map[itC][in] = itA_2;
                tri_map[itA] = topo_A_1; // reuse  
                tri_map.push_back(topo_A_2); // allocate
                topo_A_2[0] = (int)tri_map.size() -1;

                if (itB != -1) {
                    std::array<int, 4> topo_B_1 = tri_map[itB];
                    std::array<int, 4> topo_B_2 = tri_map[itB];
                    topo_B_1[1+neB] = itA_1;
                    topo_B_2[1+neB] = itA_2;
                    int is1 = 1+((neB+2)%3);
                    int is2 = 1+((neB+1)%3);
                    if (swapB) std::swap(is1,is2);
                    topo_B_1[is1] = itB_2;
                    topo_B_2[is2] = itB_1;
                    int itF = topo_B_1[is2];
                    int itE = topo_B_2[is1];
                    for (int in = 1; in<4; ++in)
                        if (tri_map[itF][in] == itB)
                            tri_map[itF][in] = itB_1; // not needed?
                    for (int in = 1; in<4; ++in)
                        if (tri_map[itE][in] == itB)
                            tri_map[itE][in] = itB_2;
                    tri_map[itB] = topo_B_1; // reuse  
                    tri_map.push_back(topo_B_2); // allocate
                    topo_B_2[0] = (int)tri_map.size() -1;
                }
            }
        } else {
            // case of n or uv or color buffers without indexes, because assumed matching the vertex pos buffer:
            int iVnew = -1;
            InterpolateAndInsert(*this, ibuffer, iea, ieb, iVnew);
        }

    } // end loop on buffers

    // just in case the user populated the vector of external auxiliary data buffers,
    // interpolate and store the created value. Assume those have same size of m_vertices
    for (auto data_buffer : aux_data_double) {
        double data = (data_buffer->at(iea) + data_buffer->at(ieb)) * 0.5;
        data_buffer->push_back(data);
    }
    for (auto data_buffer : aux_data_vect) {
        ChVector<> data = (data_buffer->at(iea) + data_buffer->at(ieb)) * 0.5;
        data_buffer->push_back(data);
    }
    for (auto data_buffer : aux_data_int) {
        int data = std::max(data_buffer->at(iea), data_buffer->at(ieb));
        data_buffer->push_back(data);
    }
    for (auto data_buffer : aux_data_bool) {
        bool data = data_buffer->at(iea) || data_buffer->at(ieb);
        data_buffer->push_back(data);
    }
    
    return true;
}


// Performs mesh refinement using Rivara LEPP long-edge bisection algorithm.
// Based on "Multithread parallelization of Lepp-bisection algorithms"
//    M.-C. Rivara et al., Applied Numerical Mathematics 62 (2012) 473–488

void ChTriangleMeshConnected::RefineMeshEdges(
        std::vector<int>& marked_tris,  ///< triangles to refine (aso surrounding triangles might be affected by refinements)
        double edge_maxlen,             ///< maximum length of edge (small values give higher resolution)
        ChRefineEdgeCriterion* criterion, ///< criterion for computing lenght (or other merit function) of edge, if =0 uses default (euclidean length)
        std::vector<std::array<int, 4>>* atri_map, ///< triangle connectivity map: use and modify it. Optional. If =0, creates a temporary one just for life span of function.
        std::vector<std::vector<double>*>& aux_data_double, ///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<int>*>& aux_data_int,       ///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<bool>*>& aux_data_bool,      ///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<ChVector<>>*>& aux_data_vect///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        ) {

    // initialize the list of triangles to refine, copying from marked triangles:
    std::list<int> S(marked_tris.begin(), marked_tris.end());
    
    
    // compute the connectivity map between triangles:
    std::vector<std::array<int, 4>> tmp_tri_map;
    std::vector<std::array<int, 4>>& tri_map = tmp_tri_map;
    if (atri_map) {
        tri_map = *atri_map;
    } else {
        this->ComputeNeighbouringTriangleMap(tri_map);
    }

    std::vector<bool> marked_tris_flagged;
    marked_tris_flagged.resize(this->m_face_v_indices.size());
    for (auto i : marked_tris)
        marked_tris_flagged[i] = true;

    std::vector<int> new_marked_tris;

    for (auto t_0 : S) {
        
        // Insert-lepp-points

        std::list<int> mlist;
        mlist.push_back(t_0);

        while(!mlist.empty()) {

            //  find last triangle in ordered list
            auto t_N = mlist.back();

            //  find longest-edge neighbour
            int t_N1 = 0; 
            double L_max = 0;
            int edge_N =0;
            double L_e;
            for (int ie = 0; ie<3; ++ie) {
                std::pair<int, int> ie_verts = this->GetTriangleEdgeIndexes(m_face_v_indices, t_N, ie, true);
                if (criterion)
                    L_e = criterion->ComputeLength(ie_verts.first, ie_verts.second, this);
                else
                    L_e = (this->m_vertices[ie_verts.first] - this->m_vertices[ie_verts.second]).Length(); 
                if (L_e > L_max) {
                    L_max = L_e;
                    edge_N = ie;
                    t_N1 = tri_map[t_N][1+ie]; // neighbour triangle
                }
            }

            if(L_max < edge_maxlen) {
                //  GetLog() << "  already small triangle - pop it and break while " << "\n";
                mlist.pop_back();
                break;
            }

            // add longest-edge neighbour to the list
            mlist.push_back(t_N1);

            if (mlist.size() > 1000) throw ChException("overflow in ChTriangleMeshConnected::RefineMeshEdges");

            // if boundary edge: always terminal edge
            if (t_N1 == -1) {
  
                // split triangle edge 
                if (L_max > edge_maxlen) {
                    // do edge split
                    int itA_1, itA_2, itB_1, itB_2;
                    SplitEdge(t_N, -1, edge_N, 0, 
                        itA_1, itA_2, itB_1, itB_2,
                        tri_map,
                        aux_data_double, aux_data_int, aux_data_bool, aux_data_vect);

                    // prepare list of original triangles after split, for the next iteration of bisection
                    if (t_N  < marked_tris_flagged.size() && marked_tris_flagged[t_N] == true) {
                        new_marked_tris.push_back(itA_1);
                        new_marked_tris.push_back(itA_2);
                    }

                }

                // remove from list
                mlist.pop_back();
                mlist.pop_back();

            } else {
                //  find longest-edge in neighbouring triangle
                double T1_L_max = 0;
                int edge_N1 =0;
                int t_shared = 0; 
                for (int ie = 0; ie<3; ++ie) {
                    std::pair<int, int> T1_ie_verts = this->GetTriangleEdgeIndexes(m_face_v_indices, t_N1, ie, true);
                    if (criterion)
                        L_e = criterion->ComputeLength(T1_ie_verts.first, T1_ie_verts.second, this);
                    else
                        L_e = (this->m_vertices[T1_ie_verts.first] - this->m_vertices[T1_ie_verts.second]).Length(); 
                    if (L_e > T1_L_max) {
                        T1_L_max = L_e;
                        edge_N1 = ie;
                        t_shared = tri_map[t_N1][1+ie]; // neighbour triangle
                    }
                }
                // shared terminal edge (it is the longest edge in both triangles)
                if (t_shared == t_N) {

                    // split triangle edge 
                    if (L_max > edge_maxlen) {
                        // do edge split
                        int itA_1, itA_2, itB_1, itB_2;
                        SplitEdge(t_N, t_N1, edge_N, edge_N1, 
                            itA_1, itA_2, itB_1, itB_2,
                            tri_map,
                            aux_data_double, aux_data_int, aux_data_bool, aux_data_vect);

                        // prepare list of original triangles after split, for the next iteration of bisection
                        if (t_N  < marked_tris_flagged.size() && marked_tris_flagged[t_N] == true) {
                            new_marked_tris.push_back(itA_1);
                            new_marked_tris.push_back(itA_2);
                        }
                        if (t_N1 < marked_tris_flagged.size() && marked_tris_flagged[t_N1] == true) {
                            new_marked_tris.push_back(itB_1);
                            new_marked_tris.push_back(itB_2);
                        }
                    }

                    // remove from list
                    mlist.pop_back();
                    mlist.pop_back();
                }

            }

        }

    }
    marked_tris = new_marked_tris;
}



}  // end namespace geometry
}  // end namespace chrono
