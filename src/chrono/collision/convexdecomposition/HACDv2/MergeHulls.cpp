#include "MergeHulls.h"
#include "ConvexHull.h"
#include "SparseArray.h"

#include <string.h>
#include <math.h>

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

#pragma warning(disable:4100 4189 4996)

using namespace hacd;

namespace HACD
{

typedef SparseArray< HaF32 > TestedMap;

static HaF32 fm_computeBestFitAABB(HaU32 vcount,const HaF32 *points,HaU32 pstride,HaF32 *bmin,HaF32 *bmax) // returns the diagonal distance
{

	const HaU8 *source = (const HaU8 *) points;

	bmin[0] = points[0];
	bmin[1] = points[1];
	bmin[2] = points[2];

	bmax[0] = points[0];
	bmax[1] = points[1];
	bmax[2] = points[2];


	for (HaU32 i=1; i<vcount; i++)
	{
		source+=pstride;
		const HaF32 *p = (const HaF32 *) source;

		if ( p[0] < bmin[0] ) bmin[0] = p[0];
		if ( p[1] < bmin[1] ) bmin[1] = p[1];
		if ( p[2] < bmin[2] ) bmin[2] = p[2];

		if ( p[0] > bmax[0] ) bmax[0] = p[0];
		if ( p[1] > bmax[1] ) bmax[1] = p[1];
		if ( p[2] > bmax[2] ) bmax[2] = p[2];

	}

	HaF32 dx = bmax[0] - bmin[0];
	HaF32 dy = bmax[1] - bmin[1];
	HaF32 dz = bmax[2] - bmin[2];

	return (HaF32) ::sqrtf( dx*dx + dy*dy + dz*dz );

}




static bool fm_intersectAABB(const HaF32 *bmin1,const HaF32 *bmax1,const HaF32 *bmin2,const HaF32 *bmax2)
{
	if ((bmin1[0] > bmax2[0]) || (bmin2[0] > bmax1[0])) return false;
	if ((bmin1[1] > bmax2[1]) || (bmin2[1] > bmax1[1])) return false;
	if ((bmin1[2] > bmax2[2]) || (bmin2[2] > bmax1[2])) return false;
	return true;
}


static HACD_INLINE HaF32 det(const HaF32 *p1,const HaF32 *p2,const HaF32 *p3)
{
	return  p1[0]*p2[1]*p3[2] + p2[0]*p3[1]*p1[2] + p3[0]*p1[1]*p2[2] -p1[0]*p3[1]*p2[2] - p2[0]*p1[1]*p3[2] - p3[0]*p2[1]*p1[2];
}


static HaF32  fm_computeMeshVolume(const HaF32 *vertices,HaU32 tcount,const HaU32 *indices)
{
	HaF32 volume = 0;
	for (HaU32 i=0; i<tcount; i++,indices+=3)
	{
		const HaF32 *p1 = &vertices[ indices[0]*3 ];
		const HaF32 *p2 = &vertices[ indices[1]*3 ];
		const HaF32 *p3 = &vertices[ indices[2]*3 ];
		volume+=det(p1,p2,p3); // compute the volume of the tetrahedran relative to the origin.
	}

	volume*=(1.0f/6.0f);
	if ( volume < 0 )
		volume*=-1;
	return volume;
}



class ChUll : public UANS::UserAllocated
	{
	public:
		ChUll(HaU32 vcount,const HaF32 *vertices,HaU32 tcount,const HaU32 *indices,HaU32 guid)
		{
			mGuid = guid;
			mVertexCount = vcount;
			mTriangleCount = tcount;
			mVertices = (HaF32 *)HACD_ALLOC(sizeof(HaF32)*3*vcount);
			memcpy(mVertices,vertices,sizeof(HaF32)*3*vcount);
			mIndices = (HaU32 *)HACD_ALLOC(sizeof(HaU32)*3*tcount);
			memcpy(mIndices,indices,sizeof(HaU32)*3*tcount);
			mVolume = fm_computeMeshVolume( mVertices, mTriangleCount, mIndices);
			mDiagonal = fm_computeBestFitAABB( mVertexCount, mVertices, sizeof(hacd::HaF32)*3, mMin, mMax );
			hacd::HaF32 dx = mMax[0] - mMin[0];
			hacd::HaF32 dy = mMax[1] - mMin[1];
			hacd::HaF32 dz = mMax[2] - mMin[2];
			dx*=0.1f; // inflate 1/10th on each edge
			dy*=0.1f; // inflate 1/10th on each edge
			dz*=0.1f; // inflate 1/10th on each edge
			mMin[0]-=dx;
			mMin[1]-=dy;
			mMin[2]-=dz;
			mMax[0]+=dx;
			mMax[1]+=dy;
			mMax[2]+=dz;
		}

		~ChUll(void)
		{
			HACD_FREE(mVertices);
			HACD_FREE(mIndices);
		}

		bool overlap(const ChUll &h) const
		{
			return fm_intersectAABB(mMin,mMax, h.mMin, h.mMax );
		}

		HaU32			mGuid;
		hacd::HaF32		mMin[3];
		hacd::HaF32		mMax[3];
		hacd::HaF32		mVolume;
		hacd::HaF32		mDiagonal; // long edge..
		HaU32			mVertexCount;
		HaU32			mTriangleCount;
		HaF32			*mVertices;
		HaU32			*mIndices;
	};

	// Usage: std::sort( list.begin(), list.end(), StringSortRef() );
	class ChUllSort
	{
	public:

		bool operator()(const ChUll *a,const ChUll *b) const
		{
			return a->mVolume < b->mVolume;
		}
	};



typedef hacd::vector< ChUll * > ChUllVector;

class MyMergeHullsInterface : public MergeHullsInterface, public UANS::UserAllocated
{
public:
	MyMergeHullsInterface(void)
	{
		mHasBeenTested = NULL;
	}

	virtual ~MyMergeHullsInterface(void)
	{

	}

	// Merge these input hulls.
	virtual hacd::HaU32 mergeHulls(const MergeHullVector &inputHulls,
		MergeHullVector &outputHulls,
		hacd::HaU32 mergeHullCount,
		hacd::HaF32 smallClusterThreshold,
		hacd::HaU32 maxHullVertices)
	{
		mGuid = 0;

		HaU32 count = (HaU32)inputHulls.size();
		mHasBeenTested = HACD_NEW(TestedMap)(count*count);
		mSmallClusterThreshold = smallClusterThreshold;
		mMaxHullVertices = maxHullVertices;
		mMergeNumHulls = mergeHullCount;

		mTotalVolume = 0;
		for (HaU32 i=0; i<inputHulls.size(); i++)
		{
			const MergeHull &h = inputHulls[i];
			ChUll *ch = HACD_NEW(ChUll)(h.mVertexCount,h.mVertices,h.mTriangleCount,h.mIndices,mGuid++);
			mChulls.push_back(ch);
			mTotalVolume+=ch->mVolume;
		}

		for(;;)
		{
			bool combined = combineHulls(); // mege smallest hulls first, up to the max merge count.
			if ( !combined ) break;
		} 

		// return results..
		for (HaU32 i=0; i<mChulls.size(); i++)
		{
			ChUll *ch = mChulls[i];
			MergeHull mh;
			mh.mVertexCount = ch->mVertexCount;
			mh.mTriangleCount = ch->mTriangleCount;
			mh.mIndices = ch->mIndices;
			mh.mVertices = ch->mVertices;
			outputHulls.push_back(mh);
		}
		delete mHasBeenTested;
		return (HaU32)outputHulls.size();
	}

	virtual void ConvexDecompResult(hacd::HaU32 hvcount,const hacd::HaF32 *hvertices,hacd::HaU32 htcount,const hacd::HaU32 *hindices)
	{
		ChUll *ch = HACD_NEW(ChUll)(hvcount,hvertices,htcount,hindices,mGuid++);
		if ( ch->mVolume > 0.00001f )
		{
			mChulls.push_back(ch);
		}
		else
		{
			delete ch;
		}
	}


	virtual void release(void) 
	{
		delete this;
	}

	HaF32 canMerge(ChUll *a,ChUll *b)
	{
		if ( !a->overlap(*b) ) return 0; // if their AABB's (with a little slop) don't overlap, then return.

		// ok..we are going to combine both meshes into a single mesh
		// and then we are going to compute the concavity...
		HaF32 ret = 0;

		HaU32 combinedVertexCount = a->mVertexCount + b->mVertexCount;
		HaF32 *combinedVertices = (HaF32 *)HACD_ALLOC(combinedVertexCount*sizeof(HaF32)*3);
		HaF32 *dest = combinedVertices;
		memcpy(dest,a->mVertices, sizeof(HaF32)*3*a->mVertexCount);
		dest+=a->mVertexCount*3;
		memcpy(dest,b->mVertices,sizeof(HaF32)*3*b->mVertexCount);

		HullResult hresult;
		HullLibrary hl;
		HullDesc   desc;
		desc.mVcount       = combinedVertexCount;
		desc.mVertices     = combinedVertices;
		desc.mVertexStride = sizeof(hacd::HaF32)*3;
		desc.mUseWuQuantizer = true;
		HullError hret = hl.CreateConvexHull(desc,hresult);
		HACD_ASSERT( hret == QE_OK );
		if ( hret == QE_OK )
		{
			ret  = fm_computeMeshVolume( hresult.mOutputVertices, hresult.mNumTriangles, hresult.mIndices );
		}
		HACD_FREE(combinedVertices);
		hl.ReleaseResult(hresult);
		return ret;
	}


	ChUll * doMerge(ChUll *a,ChUll *b)
	{
		ChUll *ret = 0;
		HaU32 combinedVertexCount = a->mVertexCount + b->mVertexCount;
		HaF32 *combinedVertices = (HaF32 *)HACD_ALLOC(combinedVertexCount*sizeof(HaF32)*3);
		HaF32 *dest = combinedVertices;
		memcpy(dest,a->mVertices, sizeof(HaF32)*3*a->mVertexCount);
		dest+=a->mVertexCount*3;
		memcpy(dest,b->mVertices,sizeof(HaF32)*3*b->mVertexCount);
		HullResult hresult;
		HullLibrary hl;
		HullDesc   desc;
		desc.mVcount       = combinedVertexCount;
		desc.mVertices     = combinedVertices;
		desc.mVertexStride = sizeof(hacd::HaF32)*3;
		desc.mMaxVertices = mMaxHullVertices;
		desc.mUseWuQuantizer = true;
		HullError hret = hl.CreateConvexHull(desc,hresult);
		HACD_ASSERT( hret == QE_OK );
		if ( hret == QE_OK )
		{
			ret = HACD_NEW(ChUll)(hresult.mNumOutputVertices, hresult.mOutputVertices, hresult.mNumTriangles, hresult.mIndices,mGuid++);
		}
		HACD_FREE(combinedVertices);
		hl.ReleaseResult(hresult);
		return ret;
	}

	bool combineHulls(void)
	{
		bool combine = false;
		// each new convex hull is given a unique guid.
		// A hash map is used to make sure that no hulls are tested twice.
		ChUllVector output;
		HaU32 count = (HaU32)mChulls.size();

		ChUll *mergeA = NULL;
		ChUll *mergeB = NULL;

		// Early out to save walking all the hulls. Hulls are combined based on 
		// a target number or on a number of generated hulls.
		bool mergeTargetMet = (HaU32)mChulls.size() <= mMergeNumHulls;
		if (mergeTargetMet && (mSmallClusterThreshold == 0.0f))
			return false;
		
		HaF32 bestVolume = mTotalVolume;
		{
			for (HaU32 i=0; i<count; i++)
			{
				ChUll *cr = mChulls[i];
				for (HaU32 j=i+1; j<count; j++)
				{
					ChUll *match = mChulls[j];
					HaU32 hashIndex;
					if ( match->mGuid < cr->mGuid )
					{
						hashIndex = (match->mGuid << 16) | cr->mGuid;
					}
					else
					{
						hashIndex = (cr->mGuid << 16 ) | match->mGuid;
					}
					HaF32 combinedVolume;
					HaF32 *v = mHasBeenTested->find(hashIndex);
					if ( v == NULL )
					{
						combinedVolume = canMerge(cr,match);
						(*mHasBeenTested)[hashIndex] = combinedVolume;
					}
					else
					{
						combinedVolume = *v;
					}
					if ( combinedVolume != 0 )
					{
						if ( combinedVolume < bestVolume )
						{
							bestVolume = combinedVolume;
							mergeA = cr;
							mergeB = match;
						}
					}
				}
			}
		}

		// If we found a merge pair, and we are below the merge threshold or we haven't reduced to the target
		// do the merge.
		bool thresholdBelow = ((bestVolume / mTotalVolume) * 100.0f) < mSmallClusterThreshold;
		if ( mergeA && (thresholdBelow || !mergeTargetMet))
		{
			ChUll *merge = doMerge(mergeA,mergeB);

			HaF32 volumeA = mergeA->mVolume;
			HaF32 volumeB = mergeB->mVolume;
			if ( merge )
			{
				combine = true;
				output.push_back(merge);
				for (ChUllVector::iterator j=mChulls.begin(); j!=mChulls.end(); ++j)
				{
					ChUll *h = (*j);
					if ( h !=mergeA && h != mergeB )
					{
						output.push_back(h);
					}
				}
				delete mergeA;
				delete mergeB;

				// Remove the old volumes and add the new one.
				mTotalVolume -= (volumeA + volumeB);
				mTotalVolume += merge->mVolume;
			}
			mChulls = output;
		}

		return combine;
	}

private:
	TestedMap			*mHasBeenTested;
	HaU32				mGuid;
	HaF32				mTotalVolume;
	HaF32				mSmallClusterThreshold;
	HaU32				mMergeNumHulls;
	HaU32				mMaxHullVertices;
	ChUllVector			mChulls;
};

MergeHullsInterface * createMergeHullsInterface(void)
{
	MyMergeHullsInterface *m = HACD_NEW(MyMergeHullsInterface);
	return static_cast< MergeHullsInterface *>(m);
}


}
