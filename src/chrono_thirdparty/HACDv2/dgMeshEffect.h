/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgMeshEffect_H__
#define __dgMeshEffect_H__

#include "dgTypes.h"
#include "dgRefCounter.h"
#include "dgPolyhedra.h"
#include "dgVector.h"
#include "dgPlane.h"
#include "dgMatrix.h"

class dgMeshEffect;
class dgMeshEffectSolidTree;
class dgMeshTreeCSGEdgePool;
class dgConvexHull3d;


#define DG_MESH_EFFECT_INITIAL_VERTEX_SIZE	8
#define DG_MESH_EFFECT_BOLLEAN_STACK		512
#define DG_MESH_EFFECT_POINT_SPLITED		512
#define DG_MESH_EFFECT_POLYGON_SPLITED		256
#define DG_MESH_EFFECT_FLAT_CUT_BORDER_EDGE	0x01
#define DG_VERTEXLIST_INDEXLIST_TOL			(hacd::HaF64 (0.0f))


#define DG_MESH_EFFECT_PRECISION_BITS		30
#define DG_MESH_EFFECT_PRECISION_SCALE		hacd::HaF64(1<<DG_MESH_EFFECT_PRECISION_BITS)
#define DG_MESH_EFFECT_PRECISION_SCALE_INV	(hacd::HaF64 (1.0f) / DG_MESH_EFFECT_PRECISION_SCALE)


#define DG_MESG_EFFECT_BOOLEAN_INIT()					\
	dgMeshEffect* result = NULL;						\
	dgMeshEffect* sourceCoplanar = NULL;				\
	dgMeshEffect* leftMeshSource = NULL;				\
	dgMeshEffect* rightMeshSource = NULL;				\
	dgMeshEffect* clipperCoplanar = NULL;				\
	dgMeshEffect* leftMeshClipper = NULL;				\
	dgMeshEffect* rightMeshClipper = NULL;

#define DG_MESG_EFFECT_BOOLEAN_FINISH()					\
	if (sourceCoplanar) {								\
		sourceCoplanar->Release();						\
	}													\
	if (clipperCoplanar) {								\
		clipperCoplanar->Release();						\
	}													\
	if (leftMeshClipper) {								\
		leftMeshClipper->Release();						\
	}													\
	if (rightMeshClipper) {								\
		rightMeshClipper->Release();					\
	}													\
	if (leftMeshSource) {								\
		leftMeshSource->Release();						\
	}													\
	if (rightMeshSource) {								\
		rightMeshSource->Release();						\
	}													\
	if (result) {										\
		result->ConvertToPolygons();					\
		dgStack<hacd::HaI32> map(result->m_pointCount + 1);	\
		result->RemoveUnusedVertices(&map[0]);			\
	}													



class dgMeshEffect: public dgPolyhedra, public dgRefCounter, public UANS::UserAllocated
{
	public:

	class dgVertexAtribute 
	{
		public:
		dgBigVector m_vertex;
		hacd::HaF64 m_normal_x;
		hacd::HaF64 m_normal_y;
		hacd::HaF64 m_normal_z;
		hacd::HaF64 m_u0;
		hacd::HaF64 m_v0;
		hacd::HaF64 m_u1;
		hacd::HaF64 m_v1;
		hacd::HaF64 m_material;
	};

	class dgIndexArray 
	{
		public:
		hacd::HaI32 m_materialCount;
		hacd::HaI32 m_indexCount;
		hacd::HaI32 m_materials[256];
		hacd::HaI32 m_materialsIndexCount[256];
		hacd::HaI32* m_indexList;
	};


	dgMeshEffect(bool preAllocaBuffers);
	dgMeshEffect(const dgMeshEffect& source);
	dgMeshEffect(dgPolyhedra& mesh, const dgMeshEffect& source);

	// Create a convex hull Mesh form point cloud
	dgMeshEffect (const hacd::HaF64* const vertexCloud, hacd::HaI32 count, hacd::HaI32 strideInByte, hacd::HaF64 distTol);

	// create a convex approximation
	dgMeshEffect (const dgMeshEffect& source, hacd::HaF32 maxConcavity, hacd::HaI32 maxCount = 32, hacd::ICallback* callback = NULL);

	// create a planar Mesh
	dgMeshEffect(const dgMatrix& planeMatrix, hacd::HaF32 witdth, hacd::HaF32 breadth, hacd::HaI32 material, const dgMatrix& textureMatrix0, const dgMatrix& textureMatrix1);
	virtual ~dgMeshEffect(void);

	dgMatrix CalculateOOBB (dgBigVector& size) const;
	void CalculateAABB (dgBigVector& min, dgBigVector& max) const;

	void CalculateNormals (hacd::HaF64 angleInRadians);
	void SphericalMapping (hacd::HaI32 material);
	void BoxMapping (hacd::HaI32 front, hacd::HaI32 side, hacd::HaI32 top);
	void UniformBoxMapping (hacd::HaI32 material, const dgMatrix& textruMatrix);
	void CylindricalMapping (hacd::HaI32 cylinderMaterial, hacd::HaI32 capMaterial);

	dgEdge* InsertEdgeVertex (dgEdge* const edge, hacd::HaF64 param);

	dgMeshEffect* GetFirstLayer ();
	dgMeshEffect* GetNextLayer (dgMeshEffect* const layer);

	void Triangulate ();
	void ConvertToPolygons ();

	void RemoveUnusedVertices(hacd::HaI32* const vertexRemapTable);
	
	void BeginPolygon ();
	void AddPolygon (hacd::HaI32 count, const hacd::HaF32* const vertexList, hacd::HaI32 stride, hacd::HaI32 material);
	void AddPolygon (hacd::HaI32 count, const hacd::HaF64* const vertexList, hacd::HaI32 stride, hacd::HaI32 material);
	void EndPolygon (hacd::HaF64 tol);

	void PackVertexArrays ();

	void BuildFromVertexListIndexList(hacd::HaI32 faceCount, const hacd::HaI32 * const faceIndexCount, const hacd::HaI32 * const faceMaterialIndex, 
		const hacd::HaF32* const vertex, hacd::HaI32  vertexStrideInBytes, const hacd::HaI32 * const vertexIndex,
		const hacd::HaF32* const normal, hacd::HaI32  normalStrideInBytes, const hacd::HaI32 * const normalIndex,
		const hacd::HaF32* const uv0, hacd::HaI32  uv0StrideInBytes, const hacd::HaI32 * const uv0Index,
		const hacd::HaF32* const uv1, hacd::HaI32  uv1StrideInBytes, const hacd::HaI32 * const uv1Index);


	hacd::HaI32 GetVertexCount() const;
	hacd::HaI32 GetVertexStrideInByte() const;
	hacd::HaF64* GetVertexPool () const;

	hacd::HaI32 GetPropertiesCount() const;
	hacd::HaI32 GetPropertiesStrideInByte() const;
	hacd::HaF64* GetAttributePool() const;
	hacd::HaF64* GetNormalPool() const;
	hacd::HaF64* GetUV0Pool() const;
	hacd::HaF64* GetUV1Pool() const;

	dgEdge* ConectVertex (dgEdge* const e0, dgEdge* const e1);

	hacd::HaI32 GetTotalFaceCount() const;
	hacd::HaI32 GetTotalIndexCount() const;
	void GetFaces (hacd::HaI32* const faceCount, hacd::HaI32* const materials, void** const faceNodeList) const;

	void RepairTJoints (bool triangulate);
	bool SeparateDuplicateLoops (dgEdge* const edge);
	bool HasOpenEdges () const;

	void GetVertexStreams (hacd::HaI32 vetexStrideInByte, hacd::HaF32* const vertex, 
						   hacd::HaI32 normalStrideInByte, hacd::HaF32* const normal, 
						   hacd::HaI32 uvStrideInByte0, hacd::HaF32* const uv0, 
						   hacd::HaI32 uvStrideInByte1, hacd::HaF32* const uv1);

	void GetIndirectVertexStreams(hacd::HaI32 vetexStrideInByte, hacd::HaF64* const vertex, hacd::HaI32* const vertexIndices, hacd::HaI32* const vertexCount,
								  hacd::HaI32 normalStrideInByte, hacd::HaF64* const normal, hacd::HaI32* const normalIndices, hacd::HaI32* const normalCount,
								  hacd::HaI32 uvStrideInByte0, hacd::HaF64* const uv0, hacd::HaI32* const uvIndices0, hacd::HaI32* const uvCount0,
								  hacd::HaI32 uvStrideInByte1, hacd::HaF64* const uv1, hacd::HaI32* const uvIndices1, hacd::HaI32* const uvCount1);

	

	dgIndexArray* MaterialGeometryBegin();
	void MaterialGeomteryEnd(dgIndexArray* const handle);
	hacd::HaI32 GetFirstMaterial (dgIndexArray* const handle);
	hacd::HaI32 GetNextMaterial (dgIndexArray* const handle, hacd::HaI32 materialHandle);
	hacd::HaI32 GetMaterialID (dgIndexArray* const handle, hacd::HaI32 materialHandle);
	hacd::HaI32 GetMaterialIndexCount (dgIndexArray* const handle, hacd::HaI32 materialHandle);
	void GetMaterialGetIndexStream (dgIndexArray* const handle, hacd::HaI32 materialHandle, hacd::HaI32* const index);
	void GetMaterialGetIndexStreamShort (dgIndexArray* const handle, hacd::HaI32 materialHandle, hacd::HaI16* const index);
	
	dgConvexHull3d * CreateConvexHull(hacd::HaF64 tolerance,hacd::HaI32 maxVertexCount) const;

	dgMeshEffect* CreateConvexApproximation (hacd::HaF32 maxConcavity, hacd::HaI32 maxCount = 32, hacd::ICallback* callback = NULL) const;

	dgVertexAtribute& GetAttribute (hacd::HaI32 index) const;
	void TransformMesh (const dgMatrix& matrix);


	void* GetFirstVertex ();
	void* GetNextVertex (const void* const vertex);
	int GetVertexIndex (const void* const vertex) const;

	void* GetFirstPoint ();
	void* GetNextPoint (const void* const point);
	int GetPointIndex (const void* const point) const;
	int GetVertexIndexFromPoint (const void* const point) const;


	void* GetFirstEdge ();
	void* GetNextEdge (const void* const edge);
	void GetEdgeIndex (const void* const edge, hacd::HaI32& v0, hacd::HaI32& v1) const;
//	void GetEdgeAttributeIndex (const void* edge, hacd::HaI32& v0, hacd::HaI32& v1) const;

	void* GetFirstFace ();
	void* GetNextFace (const void* const face);
	int IsFaceOpen (const void* const face) const;
	int GetFaceMaterial (const void* const face) const;
	int GetFaceIndexCount (const void* const face) const;
	void GetFaceIndex (const void* const face, int* const indices) const;
	void GetFaceAttributeIndex (const void* const face, int* const indices) const;

	bool Sanity () const;

	protected:

	void Init (bool preAllocaBuffers);
	dgBigVector GetOrigin ()const;
	hacd::HaI32 CalculateMaxAttributes () const;
	hacd::HaF64 QuantizeCordinade(hacd::HaF64 val) const;
	void EnumerateAttributeArray (dgVertexAtribute* const attib);
	void ApplyAttributeArray (dgVertexAtribute* const attib);
	void AddVertex(const dgBigVector& vertex);
	void AddAtribute (const dgVertexAtribute& attib);
	void AddPoint(const hacd::HaF64* vertexList, hacd::HaI32 material);
	void FixCylindricalMapping (dgVertexAtribute* const attib) const;

	void MergeFaces (const dgMeshEffect* const source);
	void ReverseMergeFaces (dgMeshEffect* const source);
	dgVertexAtribute InterpolateEdge (dgEdge* const edge, hacd::HaF64 param) const;
	dgVertexAtribute InterpolateVertex (const dgBigVector& point, dgEdge* const face) const;

	dgMeshEffect* GetNextLayer (hacd::HaI32 mark);

	void FilterCoplanarFaces (const dgMeshEffect* const otherCap, hacd::HaF32 sign);

	bool CheckSingleMesh() const;


	hacd::HaI32 m_pointCount;
	hacd::HaI32 m_maxPointCount;

	hacd::HaI32 m_atribCount;
	hacd::HaI32 m_maxAtribCount;

	dgBigVector* m_points;
	dgVertexAtribute* m_attib;

	
	friend class dgConvexHull3d;
	friend class dgConvexHull4d;
	friend class dgMeshTreeCSGFace;
	friend class dgMeshEffectSolidTree;
};



inline hacd::HaI32 dgMeshEffect::GetVertexCount() const
{
	return m_pointCount;
}

inline hacd::HaI32 dgMeshEffect::GetPropertiesCount() const
{
	return m_atribCount;
}

inline hacd::HaI32 dgMeshEffect::GetMaterialID (dgIndexArray* const handle, hacd::HaI32 materialHandle)
{
	return handle->m_materials[materialHandle];
}

inline hacd::HaI32 dgMeshEffect::GetMaterialIndexCount (dgIndexArray* const handle, hacd::HaI32 materialHandle)
{
	return handle->m_materialsIndexCount[materialHandle];
}

inline dgMeshEffect::dgVertexAtribute& dgMeshEffect::GetAttribute (hacd::HaI32 index) const 
{
	return m_attib[index];
}

inline hacd::HaI32 dgMeshEffect::GetPropertiesStrideInByte() const 
{
	return sizeof (dgVertexAtribute);
}

inline hacd::HaF64* dgMeshEffect::GetAttributePool() const 
{
	return &m_attib->m_vertex.m_x;
}

inline hacd::HaF64* dgMeshEffect::GetNormalPool() const 
{
	return &m_attib->m_normal_x;
}

inline hacd::HaF64* dgMeshEffect::GetUV0Pool() const 
{
	return &m_attib->m_u0;
}

inline hacd::HaF64* dgMeshEffect::GetUV1Pool() const 
{
	return &m_attib->m_u1;
}

inline hacd::HaI32 dgMeshEffect::GetVertexStrideInByte() const 
{
	return sizeof (dgBigVector);
}

inline hacd::HaF64* dgMeshEffect::GetVertexPool () const 
{
	return &m_points[0].m_x;
}


inline dgMeshEffect* dgMeshEffect::GetFirstLayer ()
{
	return GetNextLayer (IncLRU());
}

inline dgMeshEffect* dgMeshEffect::GetNextLayer (dgMeshEffect* const layerSegment)
{
	if (!layerSegment) {
		return NULL;
	}
	return GetNextLayer (layerSegment->IncLRU() - 1);
}


inline hacd::HaF64 dgMeshEffect::QuantizeCordinade(hacd::HaF64 x) const
{
	hacd::HaI32 exp;
	hacd::HaF64 mantissa = frexp(x, &exp);
	mantissa = DG_MESH_EFFECT_PRECISION_SCALE_INV * floor (mantissa * DG_MESH_EFFECT_PRECISION_SCALE);

	hacd::HaF64 x1 = ldexp(mantissa, exp);
	return x1;
}

#endif
