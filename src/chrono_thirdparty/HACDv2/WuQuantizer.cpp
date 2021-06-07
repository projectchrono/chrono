#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/*!
**
** Copyright (c) 20011 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
**
**
** The MIT license:
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is furnished
** to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.

** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
** WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "WuQuantizer.h"
#include "SparseArray.h"

///////////////////////////////////////////////////////////////////////
//	    C Implementation of Wu's Color Quantizer (v. 2)
//	    (see Graphics Gems vol. II, pp. 126-133)
//
// Author:	Xiaolin Wu
// Dept. of Computer Science
// Univ. of Western Ontario
// London, Ontario N6A 5B7
// wu@csd.uwo.ca
// 
// Algorithm: Greedy orthogonal bipartition of RGB space for variance
// 	   minimization aided by inclusion-exclusion tricks.
// 	   For speed no nearest neighbor search is done. Slightly
// 	   better performance can be expected by more sophisticated
// 	   but more expensive versions.
// 
// The author thanks Tom Lane at Tom_Lane@G.GP.CS.CMU.EDU for much of
// additional documentation and a cure to a previous bug.
// 
// Free to distribute, comments and suggestions are appreciated.
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
// History
// -------
// July 2000:  C++ Implementation of Wu's Color Quantizer
//             and adaptation for the FreeImage 2 Library
//             Author: Hervé Drolon (drolon@infonie.fr)
// March 2004: Adaptation for the FreeImage 3 library (port to big endian processors)
//             Author: Hervé Drolon (drolon@infonie.fr)
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////



#pragma warning(disable:4100)

using namespace hacd;

namespace HACD
{

#define FI_RGBA_RED				2
#define FI_RGBA_GREEN			1
#define FI_RGBA_BLUE			0
#define FI_RGBA_ALPHA			3
#define FI_RGBA_RED_MASK		0x00FF0000
#define FI_RGBA_GREEN_MASK		0x0000FF00
#define FI_RGBA_BLUE_MASK		0x000000FF
#define FI_RGBA_ALPHA_MASK		0xFF000000
#define FI_RGBA_RED_SHIFT		16
#define FI_RGBA_GREEN_SHIFT		8
#define FI_RGBA_BLUE_SHIFT		0
#define FI_RGBA_ALPHA_SHIFT		24

////////////////////////////////////////////////////////////////
class Vec3
{
public:

	Vec3(HaF32 _x,HaF32 _y,HaF32 _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Vec3(void)
	{
	}

	Vec3(const hacd::HaF32 *p)
	{
		x = p[0];
		y = p[1];
		z = p[2];
	}

	HaF32	x;
	HaF32	y;
	HaF32	z;
};

	typedef hacd::vector< Vec3 > Vec3Vector;


/**
  Xiaolin Wu color quantization algorithm
*/
class WuColorQuantizer
{
public:
	// Constructor - Input parameter: DIB 24-bit to be quantized
	WuColorQuantizer(void);
	// Destructor
	~WuColorQuantizer();
	// Quantizer - Return value: quantized 8-bit (color palette) DIB
	void Quantize(HaI32 PaletteSize,Vec3Vector &outputVertices);

	void addColor(HaF32 x,HaF32 y,HaF32 z);


typedef struct tagBox 
{
    HaI32 r0;			 // min value, exclusive
    HaI32 r1;			 // max value, inclusive
    HaI32 g0;  
    HaI32 g1;  
    HaI32 b0;  
    HaI32 b1;
    HaI32 vol;
} Box;

private:
	HaI32 table[256];
    HaF32 *mSumSquared;
	HaI32 *mWeight;
	HaI32 *mSumX;
	HaI32 *mSumY;
	HaI32 *mSumZ;

	void M3D(HaI32 *vwt, HaI32 *vmr, HaI32 *vmg, HaI32 *vmb, HaF32 *m2);
	HaI32 Vol(Box *cube, HaI32 *mmt);
	HaI32 Bottom(Box *cube, HaU8 dir, HaI32 *mmt);
	HaI32 Top(Box *cube, HaU8 dir, HaI32 pos, HaI32 *mmt);
	HaF32 Var(Box *cube);
	HaF32 Maximize(Box *cube, HaU8 dir, HaI32 first, HaI32 last , HaI32 *cut,
				   HaI32 whole_r, HaI32 whole_g, HaI32 whole_b, HaI32 whole_w);
	bool Cut(Box *set1, Box *set2);
	void Mark(Box *cube, HaI32 label, HaU8 *tag);

};



// Size of a 3D array : 33 x 33 x 33
#define SIZE_3D	35937

// 3D array indexation
#define INDEX(r, g, b)	((r << 10) + (r << 6) + r + (g << 5) + g + b)

#define MAXCOLOR	1024

// Constructor / Destructor

WuColorQuantizer::WuColorQuantizer(void)
{
	// Allocate 3D arrays
	mSumSquared = (HaF32*)malloc(SIZE_3D * sizeof(HaF32));
	mWeight = (HaI32*)malloc(SIZE_3D * sizeof(HaI32));
	mSumX = (HaI32*)malloc(SIZE_3D * sizeof(HaI32));
	mSumY = (HaI32*)malloc(SIZE_3D * sizeof(HaI32));
	mSumZ = (HaI32*)malloc(SIZE_3D * sizeof(HaI32));

	memset(mSumSquared, 0, SIZE_3D * sizeof(HaF32));
	memset(mWeight, 0, SIZE_3D * sizeof(HaI32));
	memset(mSumX, 0, SIZE_3D * sizeof(HaI32));
	memset(mSumY, 0, SIZE_3D * sizeof(HaI32));
	memset(mSumZ, 0, SIZE_3D * sizeof(HaI32));

	for(HaI32 i = 0; i < 256; i++)
		table[i] = i * i;

}

WuColorQuantizer::~WuColorQuantizer() 
{
	if(mSumSquared)	free(mSumSquared);
	if(mWeight)	free(mWeight);
	if(mSumX)	free(mSumX);
	if(mSumY)	free(mSumY);
	if(mSumZ)	free(mSumZ);
}


// Histogram is in elements 1..HISTSIZE along each axis,
// element 0 is for base or marginal value
// NB: these must start out 0!

// Build 3-D color histogram of counts, r/g/b, c^2

void WuColorQuantizer::addColor(HaF32 x,HaF32 y,HaF32 z)
{
	HaU32 red = (HaU32)(x*128+128);
	HaU32 green = (HaU32)(y*128+128);
	HaU32 blue = (HaU32)(z*128+128);
	HACD_ASSERT( red < 256 );
	HACD_ASSERT( green < 256 );
	HACD_ASSERT( blue < 256 );

	HaU32 inr = (red >> 3) + 1;
	HaU32 ing = (green >> 3) + 1;
	HaU32 inb = (blue >> 3) + 1;
	HaU32 ind = INDEX(inr, ing, inb);
 
	mWeight[ind]++;
	mSumX[ind] += red;
	mSumY[ind] += green;
	mSumZ[ind] += blue;
	mSumSquared[ind] += table[red] + table[green] + table[blue];
}


// At conclusion of the histogram step, we can interpret
// mWeight[r][g][b] = sum over voxel of P(c)
// mSumX[r][g][b] = sum over voxel of r*P(c)  ,  similarly for mSumY, mSumZ
// m2[r][g][b] = sum over voxel of c^2*P(c)
// Actually each of these should be divided by 'ImageSize' to give the usual
// interpretation of P() as ranging from 0 to 1, but we needn't do that here.


// We now convert histogram into moments so that we can rapidly calculate
// the sums of the above quantities over any desired box.

// Compute cumulative moments
void WuColorQuantizer::M3D(HaI32 *vwt, HaI32 *vmr, HaI32 *vmg, HaI32 *vmb, HaF32 *m2) 
{
	HaU32 ind1, ind2;
	HaU8 i, r, g, b;
	HaI32 line, line_r, line_g, line_b;
	HaI32 area[33], area_r[33], area_g[33], area_b[33];
	HaF32 line2, area2[33];

	for(r = 1; r <= 32; r++) 
	{
		for(i = 0; i <= 32; i++) 
		{
			area2[i] = 0;
			area[i] = area_r[i] = area_g[i] = area_b[i] = 0;
		}
		for(g = 1; g <= 32; g++) 
		{
			line2 = 0;
			line = line_r = line_g = line_b = 0;
			for(b = 1; b <= 32; b++) 
			{			 
				ind1 = INDEX(r, g, b); // [r][g][b]
				line += vwt[ind1];
				line_r += vmr[ind1]; 
				line_g += vmg[ind1]; 
				line_b += vmb[ind1];
				line2 += m2[ind1];
				area[b] += line;
				area_r[b] += line_r;
				area_g[b] += line_g;
				area_b[b] += line_b;
				area2[b] += line2;
				ind2 = ind1 - 1089; // [r-1][g][b]
				vwt[ind1] = vwt[ind2] + area[b];
				vmr[ind1] = vmr[ind2] + area_r[b];
				vmg[ind1] = vmg[ind2] + area_g[b];
				vmb[ind1] = vmb[ind2] + area_b[b];
				m2[ind1] = m2[ind2] + area2[b];
			}
		}
	}
}

	// Compute sum over a box of any given statistic
	HaI32 
		WuColorQuantizer::Vol( Box *cube, HaI32 *mmt ) {
			return( mmt[INDEX(cube->r1, cube->g1, cube->b1)] 
			- mmt[INDEX(cube->r1, cube->g1, cube->b0)]
			- mmt[INDEX(cube->r1, cube->g0, cube->b1)]
			+ mmt[INDEX(cube->r1, cube->g0, cube->b0)]
			- mmt[INDEX(cube->r0, cube->g1, cube->b1)]
			+ mmt[INDEX(cube->r0, cube->g1, cube->b0)]
			+ mmt[INDEX(cube->r0, cube->g0, cube->b1)]
			- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
	}

	// The next two routines allow a slightly more efficient calculation
	// of Vol() for a proposed subbox of a given box.  The sum of Top()
	// and Bottom() is the Vol() of a subbox split in the given direction
	// and with the specified new upper bound.


	// Compute part of Vol(cube, mmt) that doesn't depend on r1, g1, or b1
	// (depending on dir)

	HaI32 
		WuColorQuantizer::Bottom(Box *cube, HaU8 dir, HaI32 *mmt) {
			switch(dir)
			{
			case FI_RGBA_RED:
				return( - mmt[INDEX(cube->r0, cube->g1, cube->b1)]
				+ mmt[INDEX(cube->r0, cube->g1, cube->b0)]
				+ mmt[INDEX(cube->r0, cube->g0, cube->b1)]
				- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
				break;
			case FI_RGBA_GREEN:
				return( - mmt[INDEX(cube->r1, cube->g0, cube->b1)]
				+ mmt[INDEX(cube->r1, cube->g0, cube->b0)]
				+ mmt[INDEX(cube->r0, cube->g0, cube->b1)]
				- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
				break;
			case FI_RGBA_BLUE:
				return( - mmt[INDEX(cube->r1, cube->g1, cube->b0)]
				+ mmt[INDEX(cube->r1, cube->g0, cube->b0)]
				+ mmt[INDEX(cube->r0, cube->g1, cube->b0)]
				- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
				break;
			}

			return 0;
	}


	// Compute remainder of Vol(cube, mmt), substituting pos for
	// r1, g1, or b1 (depending on dir)

	HaI32 	WuColorQuantizer::Top(Box *cube, HaU8 dir, HaI32 pos, HaI32 *mmt) 
	{
			switch(dir)
			{
			case FI_RGBA_RED:
				return( mmt[INDEX(pos, cube->g1, cube->b1)] 
				-mmt[INDEX(pos, cube->g1, cube->b0)]
				-mmt[INDEX(pos, cube->g0, cube->b1)]
				+mmt[INDEX(pos, cube->g0, cube->b0)] );
				break;
			case FI_RGBA_GREEN:
				return( mmt[INDEX(cube->r1, pos, cube->b1)] 
				-mmt[INDEX(cube->r1, pos, cube->b0)]
				-mmt[INDEX(cube->r0, pos, cube->b1)]
				+mmt[INDEX(cube->r0, pos, cube->b0)] );
				break;
			case FI_RGBA_BLUE:
				return( mmt[INDEX(cube->r1, cube->g1, pos)]
				-mmt[INDEX(cube->r1, cube->g0, pos)]
				-mmt[INDEX(cube->r0, cube->g1, pos)]
				+mmt[INDEX(cube->r0, cube->g0, pos)] );
				break;
			}

			return 0;
	}

	// Compute the weighted variance of a box 
	// NB: as with the raw statistics, this is really the variance * ImageSize 

	HaF32	WuColorQuantizer::Var(Box *cube) 
	{
		HaF32 dr = (HaF32) Vol(cube, mSumX); 
		HaF32 dg = (HaF32) Vol(cube, mSumY); 
		HaF32 db = (HaF32) Vol(cube, mSumZ);
		HaF32 xx =  mSumSquared[INDEX(cube->r1, cube->g1, cube->b1)] 
			-mSumSquared[INDEX(cube->r1, cube->g1, cube->b0)]
			-mSumSquared[INDEX(cube->r1, cube->g0, cube->b1)]
			+mSumSquared[INDEX(cube->r1, cube->g0, cube->b0)]
			-mSumSquared[INDEX(cube->r0, cube->g1, cube->b1)]
			+mSumSquared[INDEX(cube->r0, cube->g1, cube->b0)]
			+mSumSquared[INDEX(cube->r0, cube->g0, cube->b1)]
			-mSumSquared[INDEX(cube->r0, cube->g0, cube->b0)];

		return (xx - (dr*dr+dg*dg+db*db)/(HaF32)Vol(cube,mWeight));    
	}

	// We want to minimize the sum of the variances of two subboxes.
	// The sum(c^2) terms can be ignored since their sum over both subboxes
	// is the same (the sum for the whole box) no matter where we split.
	// The remaining terms have a minus sign in the variance formula,
	// so we drop the minus sign and MAXIMIZE the sum of the two terms.

	HaF32	WuColorQuantizer::Maximize(Box *cube, HaU8 dir, HaI32 first, HaI32 last , HaI32 *cut, HaI32 whole_r, HaI32 whole_g, HaI32 whole_b, HaI32 whole_w) 
	{
			HaI32 half_r, half_g, half_b, half_w;
			HaI32 i;
			HaF32 temp;

			HaI32 base_r = Bottom(cube, dir, mSumX);
			HaI32 base_g = Bottom(cube, dir, mSumY);
			HaI32 base_b = Bottom(cube, dir, mSumZ);
			HaI32 base_w = Bottom(cube, dir, mWeight);

			HaF32 max = 0.0;

			*cut = -1;

			for (i = first; i < last; i++) {
				half_r = base_r + Top(cube, dir, i, mSumX);
				half_g = base_g + Top(cube, dir, i, mSumY);
				half_b = base_b + Top(cube, dir, i, mSumZ);
				half_w = base_w + Top(cube, dir, i, mWeight);

				// now half_x is sum over lower half of box, if split at i

				if (half_w == 0) {		// subbox could be empty of pixels!
					continue;			// never split into an empty box
				} else {
					temp = ((HaF32)half_r*half_r + (HaF32)half_g*half_g + (HaF32)half_b*half_b)/half_w;
				}

				half_r = whole_r - half_r;
				half_g = whole_g - half_g;
				half_b = whole_b - half_b;
				half_w = whole_w - half_w;

				if (half_w == 0) {		// subbox could be empty of pixels!
					continue;			// never split into an empty box
				} else {
					temp += ((HaF32)half_r*half_r + (HaF32)half_g*half_g + (HaF32)half_b*half_b)/half_w;
				}

				if (temp > max) {
					max=temp;
					*cut=i;
				}
			}

			return max;
	}

	bool
		WuColorQuantizer::Cut(Box *set1, Box *set2) {
			HaU8 dir;
			HaI32 cutr, cutg, cutb;

			HaI32 whole_r = Vol(set1, mSumX);
			HaI32 whole_g = Vol(set1, mSumY);
			HaI32 whole_b = Vol(set1, mSumZ);
			HaI32 whole_w = Vol(set1, mWeight);

			HaF32 maxr = Maximize(set1, FI_RGBA_RED, set1->r0+1, set1->r1, &cutr, whole_r, whole_g, whole_b, whole_w);    
			HaF32 maxg = Maximize(set1, FI_RGBA_GREEN, set1->g0+1, set1->g1, &cutg, whole_r, whole_g, whole_b, whole_w);    
			HaF32 maxb = Maximize(set1, FI_RGBA_BLUE, set1->b0+1, set1->b1, &cutb, whole_r, whole_g, whole_b, whole_w);

			if ((maxr >= maxg) && (maxr >= maxb)) {
				dir = FI_RGBA_RED;

				if (cutr < 0) {
					return false; // can't split the box
				}
			} else if ((maxg >= maxr) && (maxg>=maxb)) {
				dir = FI_RGBA_GREEN;
			} else {
				dir = FI_RGBA_BLUE;
			}

			set2->r1 = set1->r1;
			set2->g1 = set1->g1;
			set2->b1 = set1->b1;

			switch (dir) {
		case FI_RGBA_RED:
			set2->r0 = set1->r1 = cutr;
			set2->g0 = set1->g0;
			set2->b0 = set1->b0;
			break;

		case FI_RGBA_GREEN:
			set2->g0 = set1->g1 = cutg;
			set2->r0 = set1->r0;
			set2->b0 = set1->b0;
			break;

		case FI_RGBA_BLUE:
			set2->b0 = set1->b1 = cutb;
			set2->r0 = set1->r0;
			set2->g0 = set1->g0;
			break;
			}

			set1->vol = (set1->r1-set1->r0)*(set1->g1-set1->g0)*(set1->b1-set1->b0);
			set2->vol = (set2->r1-set2->r0)*(set2->g1-set2->g0)*(set2->b1-set2->b0);

			return true;
	}


	void
		WuColorQuantizer::Mark(Box *cube, HaI32 label, HaU8 *tag) {
			for (HaI32 r = cube->r0 + 1; r <= cube->r1; r++) {
				for (HaI32 g = cube->g0 + 1; g <= cube->g1; g++) {
					for (HaI32 b = cube->b0 + 1; b <= cube->b1; b++) {
						tag[INDEX(r, g, b)] = (HaU8)label;
					}
				}
			}
	}

// Wu Quantization algorithm
void WuColorQuantizer::Quantize(HaI32 PaletteSize,Vec3Vector &outputVertices) 
{
	HaU8 *tag = NULL;

	if ( PaletteSize > MAXCOLOR )
	{
		PaletteSize = MAXCOLOR;
	}
	Box	cube[MAXCOLOR];
	HaI32	next;
	HaI32 i, weight;
	HaI32 k;
	HaF32 vv[MAXCOLOR], temp;

	// Compute moments
	M3D(mWeight, mSumX, mSumY, mSumZ, mSumSquared);

	cube[0].r0 = cube[0].g0 = cube[0].b0 = 0;
	cube[0].r1 = cube[0].g1 = cube[0].b1 = 32;
	next = 0;

	for (i = 1; i < PaletteSize; i++) 
	{
		if(Cut(&cube[next], &cube[i])) 
		{
			// volume test ensures we won't try to cut one-cell box
			vv[next] = (cube[next].vol > 1) ? Var(&cube[next]) : 0;
			vv[i] = (cube[i].vol > 1) ? Var(&cube[i]) : 0;
		} 
		else 
		{
			vv[next] = 0.0;   // don't try to split this box again
			i--;              // didn't create box i
		}

		next = 0; temp = vv[0];

		for (k = 1; k <= i; k++) 
		{
			if (vv[k] > temp) 
			{
				temp = vv[k]; next = k;
			}
		}

		if (temp <= 0.0) 
		{
			PaletteSize = i + 1;
			// Error: "Only got 'PaletteSize' boxes"
			break;
		}
	}

	// Partition done
	// the space for array mSumSquared can be freed now
	free(mSumSquared);
	mSumSquared = NULL;

	// create an optimized palette
	tag = (HaU8*) malloc(SIZE_3D * sizeof(HaU8));
	memset(tag, 0, SIZE_3D * sizeof(HaU8));

	for (k = 0; k < PaletteSize ; k++) 
	{
		Mark(&cube[k], k, tag);
		weight = Vol(&cube[k], mWeight);

		if (weight) 
		{
			HaI32 red	= (HaI32)(((HaF32)Vol(&cube[k], mSumX) / (HaF32)weight) + 0.5f);
			HaI32 green = (HaI32)(((HaF32)Vol(&cube[k], mSumY) / (HaF32)weight) + 0.5f);
			HaI32 blue	= (HaI32)(((HaF32)Vol(&cube[k], mSumZ) / (HaF32)weight) + 0.5f);
			HACD_ASSERT( red >= 0 && red < 256 );
			HACD_ASSERT( green >= 0 && green < 256 );
			HACD_ASSERT( blue >= 0 && blue < 256 );
			Vec3 v;
			v.x = (red-128.0f)/128.0f;
			v.y = (green-128.0f)/128.0f;
			v.z = (blue-128.0f)/128.0f;
			outputVertices.push_back(v);
		} 
		else 
		{
		}
	}
}


hacd::HaU32	kmeans_cluster3d(const hacd::HaF32 *input,				// an array of input 3d data points.
		hacd::HaU32 inputSize,				// the number of input data points.
		hacd::HaU32 clumpCount,				// the number of clumps you wish to product.
		hacd::HaF32	*outputClusters,		// The output array of clumps 3d vectors, should be at least 'clumpCount' in size.
		hacd::HaU32	*outputIndices,			// A set of indices which remaps the input vertices to clumps; should be at least 'inputSize'
		hacd::HaF32 errorThreshold=0.01f,	// The error threshold to converge towards before giving up.
		hacd::HaF32 collapseDistance=0.01f); // distance so small it is not worth bothering to create a new clump.

template <class Type> struct Vec3d
{
	inline Type distanceSquared(const Vec3d &a) const
	{
		Type dx = x-a.x;
		Type dy = y-a.y;
		Type dz = z-a.z;
		return dx*dx+dy*dy+dz*dz;
	}
	inline void operator+=(const Vec3d &v)
	{
		x+=v.x;
		y+=v.y;
		z+=v.z;
	}
	inline void operator*=(const Type v)
	{
		x*=v;
		y*=v;
		z*=v;
	}
	inline void zero(void) { x = 0; y = 0; z = 0; };

	Type x;
	Type y;
	Type z;
};

template <class Vec,class Type >
hacd::HaU32	kmeans_cluster(const Vec *input,
						   hacd::HaU32 inputCount,
						   hacd::HaU32 clumpCount,
						   Vec *clusters,
						   hacd::HaU32 *outputIndices,
						   Type threshold, // controls how long it works to converge towards a least errors solution.
						   Type collapseDistance) // distance between clumps to consider them to be essentially equal.
{
	hacd::HaU32 convergeCount = 64; // maximum number of iterations attempting to converge to a solution..
	hacd::HaU32 *counts = (hacd::HaU32 *)HACD_ALLOC(sizeof(hacd::HaU32)*clumpCount);
	Type error=0;
	if ( inputCount <= clumpCount ) // if the number of input points is less than our clumping size, just return the input points.
	{
		clumpCount = inputCount;
		for (hacd::HaU32 i=0; i<inputCount; i++)
		{
			if ( outputIndices )
			{
				outputIndices[i] = i;
			}
			clusters[i] = input[i];
			counts[i] = 1;
		}
	}
	else
	{
		Vec *centroids = (Vec *)HACD_ALLOC(sizeof(Vec)*clumpCount);

		// Take a sampling of the input points as initial centroid estimates.
		for (hacd::HaU32 i=0; i<clumpCount; i++)
		{
			hacd::HaU32 index = (i*inputCount)/clumpCount;
			assert( index >= 0 && index < inputCount );
			clusters[i] = input[index];
		}

		// Here is the main convergence loop
		Type old_error = FLT_MAX;	// old and initial error estimates are max Type
		error = FLT_MAX;
		do
		{
			old_error = error;	// preserve the old error
			// reset the counts and centroids to current cluster location
			for (hacd::HaU32 i=0; i<clumpCount; i++)
			{
				counts[i] = 0;
				centroids[i].zero();
			}
			error = 0;
			// For each input data point, figure out which cluster it is closest too and add it to that cluster.
			for (hacd::HaU32 i=0; i<inputCount; i++)
			{
				Type min_distance = FLT_MAX;
				// find the nearest clump to this point.
				for (hacd::HaU32 j=0; j<clumpCount; j++)
				{
					Type distance = input[i].distanceSquared( clusters[j] );
					if ( distance < min_distance )
					{
						min_distance = distance;
						outputIndices[i] = j; // save which clump this point indexes
					}
				}
				hacd::HaU32 index = outputIndices[i]; // which clump was nearest to this point.
				centroids[index]+=input[i];
				counts[index]++;	// increment the counter indicating how many points are in this clump.
				error+=min_distance; // save the error accumulation
			}
			// Now, for each clump, compute the mean and store the result.
			for (hacd::HaU32 i=0; i<clumpCount; i++)
			{
				if ( counts[i] ) // if this clump got any points added to it...
				{
					Type recip = 1.0f / (Type)counts[i];	// compute the average (center of those points)
					centroids[i]*=recip;	// compute the average center of the points in this clump.
					clusters[i] = centroids[i]; // store it as the new cluster.
				}
			}
			// decrement the convergence counter and bail if it is taking too long to converge to a solution.
			convergeCount--;
			if (convergeCount == 0 )
			{
				break;
			}
			if ( error < threshold ) // early exit if our first guess is already good enough (if all input points are the same)
				break;
		} while ( fabs(error - old_error) > threshold ); // keep going until the error is reduced by this threshold amount.

		HACD_FREE(centroids);
	}

	// ok..now we prune the clumps if necessary.
	// The rules are; first, if a clump has no 'counts' then we prune it as it's unused.
	// The second, is if the centroid of this clump is essentially  the same (based on the distance tolerance)
	// as an existing clump, then it is pruned and all indices which used to point to it, now point to the one
	// it is closest too.
	hacd::HaU32 outCount = 0; // number of clumps output after pruning performed.
	Type d2 = collapseDistance*collapseDistance; // squared collapse distance.
	for (hacd::HaU32 i=0; i<clumpCount; i++)
	{
		if ( counts[i] == 0 ) // if no points ended up in this clump, eliminate it.
			continue;
		// see if this clump is too close to any already accepted clump.
		bool add = true;
		hacd::HaU32 remapIndex = outCount; // by default this clump will be remapped to its current index.
		for (hacd::HaU32 j=0; j<outCount; j++)
		{
			Type distance = clusters[i].distanceSquared(clusters[j]);
			if ( distance < d2 )
			{
				remapIndex = j;
				add = false; // we do not add this clump
				break;
			}
		}
		// If we have fewer output clumps than input clumps so far, then we need to remap the old indices to the new ones.
		if ( outputIndices )
		{
			if ( outCount != i || !add ) // we need to remap indices!  everything that was index 'i' now needs to be remapped to 'outCount'
			{
				for (hacd::HaU32 j=0; j<inputCount; j++)
				{
					if ( outputIndices[j] == i )
					{
						outputIndices[j] = remapIndex; //
					}
				}
			}
		}
		if ( add )
		{
			clusters[outCount] = clusters[i];
			outCount++;
		}
	}
	HACD_FREE(counts);
	clumpCount = outCount;
	return clumpCount;
}

hacd::HaU32	kmeans_cluster3d(const hacd::HaF32 *input,				// an array of input 3d data points.
							 hacd::HaU32 inputSize,				// the number of input data points.
							 hacd::HaU32 clumpCount,				// the number of clumps you wish to produce
							 hacd::HaF32	*outputClusters,		// The output array of clumps 3d vectors, should be at least 'clumpCount' in size.
							 hacd::HaU32	*outputIndices,			// A set of indices which remaps the input vertices to clumps; should be at least 'inputSize'
							 hacd::HaF32 errorThreshold,	// The error threshold to converge towards before giving up.
							 hacd::HaF32 collapseDistance) // distance so small it is not worth bothering to create a new clump.
{
	const Vec3d< hacd::HaF32 > *_input = (const Vec3d<hacd::HaF32> *)input;
	Vec3d<hacd::HaF32> *_output = (Vec3d<hacd::HaF32> *)outputClusters;
	return kmeans_cluster< Vec3d<hacd::HaF32>, hacd::HaF32 >(_input,inputSize,clumpCount,_output,outputIndices,errorThreshold,collapseDistance);
}

class MyWuQuantizer : public WuQuantizer, public UANS::UserAllocated
{
public:
	MyWuQuantizer(void)
	{
		mScale = Vec3(1,1,1);
		mCenter = Vec3(0,0,0);
	}

	// use the Wu quantizer with 10 bits of resolution on each axis.  Precision down to 0.0009765625
	// All input data is normalized to a unit cube.

	virtual const hacd::HaF32 * wuQuantize3D(hacd::HaU32 vcount,
		const hacd::HaF32 *vertices,
		bool denormalizeResults,
		hacd::HaU32 maxVertices,
		hacd::HaU32 &outputCount)
	{
		const hacd::HaF32 *ret = NULL;
		outputCount = 0;

		normalizeInput(vcount,vertices);

		WuColorQuantizer wcq;

		for (HaU32 i=0; i<vcount; i++)
		{
			const Vec3 &v = mNormalizedInput[i];
			wcq.addColor(v.x,v.y,v.z);
		}

		wcq.Quantize(maxVertices,mQuantizedOutput);

		outputCount = (HaU32)mQuantizedOutput.size();

		if ( outputCount > 0 )
		{
			if ( denormalizeResults )
			{
				for (HaU32 i=0; i<outputCount; i++)
				{
					Vec3 &v = mQuantizedOutput[i];
					v.x = v.x*mScale.x + mCenter.x;
					v.y = v.x*mScale.y + mCenter.y;
					v.z = v.x*mScale.z + mCenter.z;
					mQuantizedOutput.push_back(v);
				}
			}
			ret = &mQuantizedOutput[0].x;
		}


		return ret;
	}

	// Use the kemans quantizer, similar results, but much slower.
	virtual const hacd::HaF32 * kmeansQuantize3D(hacd::HaU32 vcount,
		const hacd::HaF32 *vertices,
		bool denormalizeResults,
		hacd::HaU32 maxVertices,
		hacd::HaU32 &outputCount)
	{
		const hacd::HaF32 *ret = NULL;
		outputCount = 0;
		mNormalizedInput.clear();
		mQuantizedOutput.clear();

		if ( vcount > 0 )
		{
			normalizeInput(vcount,vertices);
			HaF32 *quantizedOutput = (HaF32 *)HACD_ALLOC( sizeof(HaF32)*3*vcount);
			HaU32 *quantizedIndices = (HaU32 *)HACD_ALLOC( sizeof(HaU32)*vcount );
			outputCount = kmeans_cluster3d(&mNormalizedInput[0].x, vcount, maxVertices, quantizedOutput, quantizedIndices, 0.01f, 0.0001f );
			if ( outputCount > 0 )
			{
				if ( denormalizeResults )
				{
					for (HaU32 i=0; i<outputCount; i++)
					{
						Vec3 v( &quantizedOutput[i*3] );
						v.x = v.x*mScale.x + mCenter.x;
						v.y = v.x*mScale.y + mCenter.y;
						v.z = v.x*mScale.z + mCenter.z;
						mQuantizedOutput.push_back(v);
					}
				}
				else
				{
					for (HaU32 i=0; i<outputCount; i++)
					{
						Vec3 v( &quantizedOutput[i*3] );
						mQuantizedOutput.push_back(v);
					}
				}
				ret = &mQuantizedOutput[0].x;
			}
			HACD_FREE(quantizedOutput);
			HACD_FREE(quantizedIndices);
		}

		return ret;
	}

	virtual void release(void)
	{
		delete this;
	}

	virtual const hacd::HaF32 * getDenormalizeScale(void) const 
	{
		return &mScale.x;
	}

	virtual const hacd::HaF32 * getDenormalizeCenter(void) const
	{
		return &mCenter.x;
	}



private:

	void normalizeInput(HaU32 vcount,const HaF32 *vertices)
	{
		mNormalizedInput.clear();
		mQuantizedOutput.clear();
		Vec3 bmin(vertices);
		Vec3 bmax(vertices);
		for (HaU32 i=1; i<vcount; i++)
		{
			Vec3 v(&vertices[i*3]);

			if ( v.x < bmin.x ) 
			{
				bmin.x = v.x;
			}
			else if ( v.x > bmax.x )
			{
				bmax.x = v.x;
			}

			if ( v.y < bmin.y ) 
			{
				bmin.y = v.y;
			}
			else if ( v.y > bmax.y )
			{
				bmax.y = v.y;
			}

			if ( v.z < bmin.z ) 
			{
				bmin.z = v.z;
			}
			else if ( v.z > bmax.z )
			{
				bmax.z = v.z;
			}
		}

		mCenter.x = (bmin.x+bmax.x)*0.5f;
		mCenter.y = (bmin.y+bmax.y)*0.5f;
		mCenter.z = (bmin.z+bmax.z)*0.5f;

		HaF32 dx = bmax.x-bmin.x;
		HaF32 dy = bmax.y-bmin.y;
		HaF32 dz = bmax.z-bmin.z;

		if ( dx == 0 )
		{
			mScale.x = 1;
		}
		else
		{
			dx = dx*1.001f;
			mScale.x = dx*0.5f;
		}
		if ( dy == 0 )
		{
			mScale.y = 1;
		}
		else
		{
			dy = dy*1.001f;
			mScale.y = dy*0.5f;
		}
		if ( dz == 0 )
		{
			mScale.z = 1;
		}
		else
		{
			dz = dz*1.001f;
			mScale.z = dz*0.5f;
		}

		Vec3 recip;
		recip.x = 1.0f / mScale.x;
		recip.y = 1.0f / mScale.y;
		recip.z = 1.0f / mScale.z;

		for (HaU32 i=0; i<vcount; i++)
		{
			Vec3 v(&vertices[i*3]);

			v.x = (v.x-mCenter.x)*recip.x;
			v.y = (v.y-mCenter.y)*recip.y;
			v.z = (v.z-mCenter.z)*recip.z;

			HACD_ASSERT( v.x >= -1 && v.x <= 1 );
			HACD_ASSERT( v.y >= -1 && v.y <= 1 );
			HACD_ASSERT( v.z >= -1 && v.z <= 1 );
			mNormalizedInput.push_back(v);
		}
	}

	virtual ~MyWuQuantizer(void)
	{

	}

	Vec3		mScale;
	Vec3		mCenter;
	Vec3Vector	mNormalizedInput;
	Vec3Vector	mQuantizedOutput;
};

WuQuantizer * createWuQuantizer(void)
{
	MyWuQuantizer *m = HACD_NEW(MyWuQuantizer);
	return static_cast< WuQuantizer *>(m);
}


} // end of HACD namespace
