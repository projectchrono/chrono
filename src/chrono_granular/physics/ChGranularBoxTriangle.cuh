// Creative Commons Legal Code
// 
// CC0 1.0 Universal
// 
//     CREATIVE COMMONS CORPORATION IS NOT A LAW FIRM AND DOES NOT PROVIDE
//     LEGAL SERVICES. DISTRIBUTION OF THIS DOCUMENT DOES NOT CREATE AN
//     ATTORNEY-CLIENT RELATIONSHIP. CREATIVE COMMONS PROVIDES THIS
//     INFORMATION ON AN "AS-IS" BASIS. CREATIVE COMMONS MAKES NO WARRANTIES
//     REGARDING THE USE OF THIS DOCUMENT OR THE INFORMATION OR WORKS
//     PROVIDED HEREUNDER, AND DISCLAIMS LIABILITY FOR DAMAGES RESULTING FROM
//     THE USE OF THIS DOCUMENT OR THE INFORMATION OR WORKS PROVIDED
//     HEREUNDER.
// 
// Statement of Purpose
// 
// The laws of most jurisdictions throughout the world automatically confer
// exclusive Copyright and Related Rights (defined below) upon the creator
// and subsequent owner(s) (each and all, an "owner") of an original work of
// authorship and/or a database (each, a "Work").
// 
// Certain owners wish to permanently relinquish those rights to a Work for
// the purpose of contributing to a commons of creative, cultural and
// scientific works ("Commons") that the public can reliably and without fear
// of later claims of infringement build upon, modify, incorporate in other
// works, reuse and redistribute as freely as possible in any form whatsoever
// and for any purposes, including without limitation commercial purposes.
// These owners may contribute to the Commons to promote the ideal of a free
// culture and the further production of creative, cultural and scientific
// works, or to gain reputation or greater distribution for their Work in
// part through the use and efforts of others.
// 
// For these and/or other purposes and motivations, and without any
// expectation of additional consideration or compensation, the person
// associating CC0 with a Work (the "Affirmer"), to the extent that he or she
// is an owner of Copyright and Related Rights in the Work, voluntarily
// elects to apply CC0 to the Work and publicly distribute the Work under its
// terms, with knowledge of his or her Copyright and Related Rights in the
// Work and the meaning and intended legal effect of CC0 on those rights.
// 
// 1. Copyright and Related Rights. A Work made available under CC0 may be
// protected by copyright and related or neighboring rights ("Copyright and
// Related Rights"). Copyright and Related Rights include, but are not
// limited to, the following:
// 
//   i. the right to reproduce, adapt, distribute, perform, display,
//      communicate, and translate a Work;
//  ii. moral rights retained by the original author(s) and/or performer(s);
// iii. publicity and privacy rights pertaining to a person's image or
//      likeness depicted in a Work;
//  iv. rights protecting against unfair competition in regards to a Work,
//      subject to the limitations in paragraph 4(a), below;
//   v. rights protecting the extraction, dissemination, use and reuse of data
//      in a Work;
//  vi. database rights (such as those arising under Directive 96/9/EC of the
//      European Parliament and of the Council of 11 March 1996 on the legal
//      protection of databases, and under any national implementation
//      thereof, including any amended or successor version of such
//      directive); and
// vii. other similar, equivalent or corresponding rights throughout the
//      world based on applicable law or treaty, and any national
//      implementations thereof.
// 
// 2. Waiver. To the greatest extent permitted by, but not in contravention
// of, applicable law, Affirmer hereby overtly, fully, permanently,
// irrevocably and unconditionally waives, abandons, and surrenders all of
// Affirmer's Copyright and Related Rights and associated claims and causes
// of action, whether now known or unknown (including existing as well as
// future claims and causes of action), in the Work (i) in all territories
// worldwide, (ii) for the maximum duration provided by applicable law or
// treaty (including future time extensions), (iii) in any current or future
// medium and for any number of copies, and (iv) for any purpose whatsoever,
// including without limitation commercial, advertising or promotional
// purposes (the "Waiver"). Affirmer makes the Waiver for the benefit of each
// member of the public at large and to the detriment of Affirmer's heirs and
// successors, fully intending that such Waiver shall not be subject to
// revocation, rescission, cancellation, termination, or any other legal or
// equitable action to disrupt the quiet enjoyment of the Work by the public
// as contemplated by Affirmer's express Statement of Purpose.
// 
// 3. Public License Fallback. Should any part of the Waiver for any reason
// be judged legally invalid or ineffective under applicable law, then the
// Waiver shall be preserved to the maximum extent permitted taking into
// account Affirmer's express Statement of Purpose. In addition, to the
// extent the Waiver is so judged Affirmer hereby grants to each affected
// person a royalty-free, non transferable, non sublicensable, non exclusive,
// irrevocable and unconditional license to exercise Affirmer's Copyright and
// Related Rights in the Work (i) in all territories worldwide, (ii) for the
// maximum duration provided by applicable law or treaty (including future
// time extensions), (iii) in any current or future medium and for any number
// of copies, and (iv) for any purpose whatsoever, including without
// limitation commercial, advertising or promotional purposes (the
// "License"). The License shall be deemed effective as of the date CC0 was
// applied by Affirmer to the Work. Should any part of the License for any
// reason be judged legally invalid or ineffective under applicable law, such
// partial invalidity or ineffectiveness shall not invalidate the remainder
// of the License, and in such case Affirmer hereby affirms that he or she
// will not (i) exercise any of his or her remaining Copyright and Related
// Rights in the Work or (ii) assert any associated claims and causes of
// action with respect to the Work, in either case contrary to Affirmer's
// express Statement of Purpose.
// 
// 4. Limitations and Disclaimers.
// 
//  a. No trademark or patent rights held by Affirmer are waived, abandoned,
//     surrendered, licensed or otherwise affected by this document.
//  b. Affirmer offers the Work as-is and makes no representations or
//     warranties of any kind concerning the Work, express, implied,
//     statutory or otherwise, including without limitation warranties of
//     title, merchantability, fitness for a particular purpose, non
//     infringement, or the absence of latent or other defects, accuracy, or
//     the present or absence of errors, whether or not discoverable, all to
//     the greatest extent permissible under applicable law.
//  c. Affirmer disclaims responsibility for clearing rights of other persons
//     that may apply to the Work or any use thereof, including without
//     limitation any person's Copyright and Related Rights in the Work.
//     Further, Affirmer disclaims responsibility for obtaining any necessary
//     consents, permissions or other rights required for any use of the
//     Work.
//  d. Affirmer understands and acknowledges that Creative Commons is not a
//     party to this document and has no duty or obligation with respect to
//     this CC0 or use of the Work.
                                                          
/********************************************************/
/* AABB-triangle overlap test code                      */
/* originally by Tomas Akenine-Möller                   */
/* modified by Conlain Kelly for ProjectChrono          */
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/

#pragma once

#include <math.h>
#include <stdio.h>

#define X 0
#define Y 1
#define Z 2

#define CROSS(dest, v1, v2)                  \
    dest[0] = v1[1] * v2[2] - v1[2] * v2[1]; \
    dest[1] = v1[2] * v2[0] - v1[0] * v2[2]; \
    dest[2] = v1[0] * v2[1] - v1[1] * v2[0];

#define DOT(v1, v2) (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2])

#define SUB(dest, v1, v2)    \
    dest[0] = v1[0] - v2[0]; \
    dest[1] = v1[1] - v2[1]; \
    dest[2] = v1[2] - v2[2];

#define SUBTRACT(dest, v1, v2) \
    dest[0] = v1.x - v2[0];    \
    dest[1] = v1.y - v2[1];    \
    dest[2] = v1.z - v2[2];

#define FINDMINMAX(x0, x1, x2, min, max) \
    min = max = x0;                      \
    if (x1 < min)                        \
        min = x1;                        \
    if (x1 > max)                        \
        max = x1;                        \
    if (x2 < min)                        \
        min = x2;                        \
    if (x2 > max)                        \
        max = x2;

inline __device__ bool planeBoxOverlap(float normal[3], float vert[3], float maxbox[3]) {
    int q;
    float vmin[3], vmax[3], v;
    for (q = X; q <= Z; q++) {
        v = vert[q];
        if (normal[q] > 0.0f) {
            vmin[q] = -maxbox[q] - v;
            vmax[q] = maxbox[q] - v;
        } else {
            vmin[q] = maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }
    if (DOT(normal, vmin) > 0.0f)
        return false;
    if (DOT(normal, vmax) >= 0.0f)
        return true;

    return false;
}

/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)                   \
    p0 = a * v0[Y] - b * v0[Z];                      \
    p2 = a * v2[Y] - b * v2[Z];                      \
    if (p0 < p2) {                                   \
        min = p0;                                    \
        max = p2;                                    \
    } else {                                         \
        min = p2;                                    \
        max = p0;                                    \
    }                                                \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z]; \
    if (min > rad || max < -rad)                     \
        return false;

#define AXISTEST_X2(a, b, fa, fb)                    \
    p0 = a * v0[Y] - b * v0[Z];                      \
    p1 = a * v1[Y] - b * v1[Z];                      \
    if (p0 < p1) {                                   \
        min = p0;                                    \
        max = p1;                                    \
    } else {                                         \
        min = p1;                                    \
        max = p0;                                    \
    }                                                \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z]; \
    if (min > rad || max < -rad)                     \
        return false;

/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)                   \
    p0 = -a * v0[X] + b * v0[Z];                     \
    p2 = -a * v2[X] + b * v2[Z];                     \
    if (p0 < p2) {                                   \
        min = p0;                                    \
        max = p2;                                    \
    } else {                                         \
        min = p2;                                    \
        max = p0;                                    \
    }                                                \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z]; \
    if (min > rad || max < -rad)                     \
        return false;

#define AXISTEST_Y1(a, b, fa, fb)                    \
    p0 = -a * v0[X] + b * v0[Z];                     \
    p1 = -a * v1[X] + b * v1[Z];                     \
    if (p0 < p1) {                                   \
        min = p0;                                    \
        max = p1;                                    \
    } else {                                         \
        min = p1;                                    \
        max = p0;                                    \
    }                                                \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z]; \
    if (min > rad || max < -rad)                     \
        return false;

/*======================== Z-tests ========================*/

#define AXISTEST_Z12(a, b, fa, fb)                   \
    p1 = a * v1[X] - b * v1[Y];                      \
    p2 = a * v2[X] - b * v2[Y];                      \
    if (p2 < p1) {                                   \
        min = p2;                                    \
        max = p1;                                    \
    } else {                                         \
        min = p1;                                    \
        max = p2;                                    \
    }                                                \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y]; \
    if (min > rad || max < -rad)                     \
        return false;

#define AXISTEST_Z0(a, b, fa, fb)                    \
    p0 = a * v0[X] - b * v0[Y];                      \
    p1 = a * v1[X] - b * v1[Y];                      \
    if (p0 < p1) {                                   \
        min = p0;                                    \
        max = p1;                                    \
    } else {                                         \
        min = p1;                                    \
        max = p0;                                    \
    }                                                \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y]; \
    if (min > rad || max < -rad)                     \
        return false;

/**
* Figure out whether a triangle touches an SD. Function gets hit a lot, very desirable to be fast.
Input:
- boxcenter: defines center of the box
- boxhalfsize: half the size of the box in an axis aligned context
- vA, vB, vC: the three vertices of the triangle
Output:
- "true" if there is overlap; "false" otherwise
NOTE: This function works with "float" - precision is not paramount.
*/
inline __device__ bool check_TriangleBoxOverlap(float boxcenter[3],
                                                float boxhalfsize[3],
                                                const float3& vA,
                                                const float3& vB,
                                                const float3& vC) {
    /**    Use the separating axis theorem to test overlap between triangle and box.
    We test for overlap in these directions:
    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle we do not even need to test these)
    2) normal of the triangle
    3) crossproduct(edge from tri, {x,y,z}-directin) this gives 3x3=9 more tests */
    float v0[3], v1[3], v2[3];
    float min, max, p0, p1, p2, rad, fex, fey, fez;
    float normal[3], e0[3], e1[3], e2[3];

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    SUBTRACT(v0, vA, boxcenter);
    SUBTRACT(v1, vB, boxcenter);
    SUBTRACT(v2, vC, boxcenter);

    /* compute triangle edges */
    SUB(e0, v1, v0); /* tri edge 0 */
    SUB(e1, v2, v1); /* tri edge 1 */
    SUB(e2, v0, v2); /* tri edge 2 */

    /* Case 3)  */
    /*  test the 9 tests first (this was faster) */
    fex = fabsf(e0[X]);
    fey = fabsf(e0[Y]);
    fez = fabsf(e0[Z]);
    AXISTEST_X01(e0[Z], e0[Y], fez, fey);
    AXISTEST_Y02(e0[Z], e0[X], fez, fex);
    AXISTEST_Z12(e0[Y], e0[X], fey, fex);

    fex = fabsf(e1[X]);
    fey = fabsf(e1[Y]);
    fez = fabsf(e1[Z]);
    AXISTEST_X01(e1[Z], e1[Y], fez, fey);
    AXISTEST_Y02(e1[Z], e1[X], fez, fex);
    AXISTEST_Z0(e1[Y], e1[X], fey, fex);

    fex = fabsf(e2[X]);
    fey = fabsf(e2[Y]);
    fez = fabsf(e2[Z]);
    AXISTEST_X2(e2[Z], e2[Y], fez, fey);
    AXISTEST_Y1(e2[Z], e2[X], fez, fex);
    AXISTEST_Z12(e2[Y], e2[X], fey, fex);

    /* Case 1) */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in X-direction */
    FINDMINMAX(v0[X], v1[X], v2[X], min, max);
    if (min > boxhalfsize[X] || max < -boxhalfsize[X])
        return false;

    /* test in Y-direction */
    FINDMINMAX(v0[Y], v1[Y], v2[Y], min, max);
    if (min > boxhalfsize[Y] || max < -boxhalfsize[Y])
        return false;

    /* test in Z-direction */
    FINDMINMAX(v0[Z], v1[Z], v2[Z], min, max);
    if (min > boxhalfsize[Z] || max < -boxhalfsize[Z])
        return false;

    /* Case 2) */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    CROSS(normal, e0, e1);
    if (!planeBoxOverlap(normal, v0, boxhalfsize))
        return false;

    return true; /* box and triangle overlaps */
}
