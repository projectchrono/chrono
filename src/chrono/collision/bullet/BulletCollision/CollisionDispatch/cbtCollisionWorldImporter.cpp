/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "cbtCollisionWorldImporter.h"
#include "cbtBulletCollisionCommon.h"
#include "LinearMath/cbtSerializer.h"  //for cbtBulletSerializedArrays definition

#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
#include "BulletCollision/Gimpact/cbtGImpactShape.h"
#endif  //SUPPORT_GIMPACT_SHAPE_IMPORT

cbtCollisionWorldImporter::cbtCollisionWorldImporter(cbtCollisionWorld* world)
	: m_collisionWorld(world),
	  m_verboseMode(0)
{
}

cbtCollisionWorldImporter::~cbtCollisionWorldImporter()
{
}

bool cbtCollisionWorldImporter::convertAllObjects(cbtBulletSerializedArrays* arrays)
{
	m_shapeMap.clear();
	m_bodyMap.clear();

	int i;

	for (i = 0; i < arrays->m_bvhsDouble.size(); i++)
	{
		cbtOptimizedBvh* bvh = createOptimizedBvh();
		cbtQuantizedBvhDoubleData* bvhData = arrays->m_bvhsDouble[i];
		bvh->deSerializeDouble(*bvhData);
		m_bvhMap.insert(arrays->m_bvhsDouble[i], bvh);
	}
	for (i = 0; i < arrays->m_bvhsFloat.size(); i++)
	{
		cbtOptimizedBvh* bvh = createOptimizedBvh();
		cbtQuantizedBvhFloatData* bvhData = arrays->m_bvhsFloat[i];
		bvh->deSerializeFloat(*bvhData);
		m_bvhMap.insert(arrays->m_bvhsFloat[i], bvh);
	}

	for (i = 0; i < arrays->m_colShapeData.size(); i++)
	{
		cbtCollisionShapeData* shapeData = arrays->m_colShapeData[i];
		cbtCollisionShape* shape = convertCollisionShape(shapeData);
		if (shape)
		{
			//		printf("shapeMap.insert(%x,%x)\n",shapeData,shape);
			m_shapeMap.insert(shapeData, shape);
		}

		if (shape && shapeData->m_name)
		{
			char* newname = duplicateName(shapeData->m_name);
			m_objectNameMap.insert(shape, newname);
			m_nameShapeMap.insert(newname, shape);
		}
	}

	for (i = 0; i < arrays->m_collisionObjectDataDouble.size(); i++)
	{
		cbtCollisionObjectDoubleData* colObjData = arrays->m_collisionObjectDataDouble[i];
		cbtCollisionShape** shapePtr = m_shapeMap.find(colObjData->m_collisionShape);
		if (shapePtr && *shapePtr)
		{
			cbtTransform startTransform;
			colObjData->m_worldTransform.m_origin.m_floats[3] = 0.f;
			startTransform.deSerializeDouble(colObjData->m_worldTransform);

			cbtCollisionShape* shape = (cbtCollisionShape*)*shapePtr;
			cbtCollisionObject* body = createCollisionObject(startTransform, shape, colObjData->m_name);
			body->setFriction(cbtScalar(colObjData->m_friction));
			body->setRestitution(cbtScalar(colObjData->m_restitution));

#ifdef USE_INTERNAL_EDGE_UTILITY
			if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				cbtBvhTriangleMeshShape* trimesh = (cbtBvhTriangleMeshShape*)shape;
				if (trimesh->getTriangleInfoMap())
				{
					body->setCollisionFlags(body->getCollisionFlags() | cbtCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
				}
			}
#endif  //USE_INTERNAL_EDGE_UTILITY
			m_bodyMap.insert(colObjData, body);
		}
		else
		{
			printf("error: no shape found\n");
		}
	}
	for (i = 0; i < arrays->m_collisionObjectDataFloat.size(); i++)
	{
		cbtCollisionObjectFloatData* colObjData = arrays->m_collisionObjectDataFloat[i];
		cbtCollisionShape** shapePtr = m_shapeMap.find(colObjData->m_collisionShape);
		if (shapePtr && *shapePtr)
		{
			cbtTransform startTransform;
			colObjData->m_worldTransform.m_origin.m_floats[3] = 0.f;
			startTransform.deSerializeFloat(colObjData->m_worldTransform);

			cbtCollisionShape* shape = (cbtCollisionShape*)*shapePtr;
			cbtCollisionObject* body = createCollisionObject(startTransform, shape, colObjData->m_name);

#ifdef USE_INTERNAL_EDGE_UTILITY
			if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				cbtBvhTriangleMeshShape* trimesh = (cbtBvhTriangleMeshShape*)shape;
				if (trimesh->getTriangleInfoMap())
				{
					body->setCollisionFlags(body->getCollisionFlags() | cbtCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
				}
			}
#endif  //USE_INTERNAL_EDGE_UTILITY
			m_bodyMap.insert(colObjData, body);
		}
		else
		{
			printf("error: no shape found\n");
		}
	}

	return true;
}

void cbtCollisionWorldImporter::deleteAllData()
{
	int i;

	for (i = 0; i < m_allocatedCollisionObjects.size(); i++)
	{
		if (m_collisionWorld)
			m_collisionWorld->removeCollisionObject(m_allocatedCollisionObjects[i]);
		delete m_allocatedCollisionObjects[i];
	}

	m_allocatedCollisionObjects.clear();

	for (i = 0; i < m_allocatedCollisionShapes.size(); i++)
	{
		delete m_allocatedCollisionShapes[i];
	}
	m_allocatedCollisionShapes.clear();

	for (i = 0; i < m_allocatedBvhs.size(); i++)
	{
		delete m_allocatedBvhs[i];
	}
	m_allocatedBvhs.clear();

	for (i = 0; i < m_allocatedTriangleInfoMaps.size(); i++)
	{
		delete m_allocatedTriangleInfoMaps[i];
	}
	m_allocatedTriangleInfoMaps.clear();
	for (i = 0; i < m_allocatedTriangleIndexArrays.size(); i++)
	{
		delete m_allocatedTriangleIndexArrays[i];
	}
	m_allocatedTriangleIndexArrays.clear();
	for (i = 0; i < m_allocatedNames.size(); i++)
	{
		delete[] m_allocatedNames[i];
	}
	m_allocatedNames.clear();

	for (i = 0; i < m_allocatedcbtStridingMeshInterfaceDatas.size(); i++)
	{
		cbtStridingMeshInterfaceData* curData = m_allocatedcbtStridingMeshInterfaceDatas[i];

		for (int a = 0; a < curData->m_numMeshParts; a++)
		{
			cbtMeshPartData* curPart = &curData->m_meshPartsPtr[a];
			if (curPart->m_vertices3f)
				delete[] curPart->m_vertices3f;

			if (curPart->m_vertices3d)
				delete[] curPart->m_vertices3d;

			if (curPart->m_indices32)
				delete[] curPart->m_indices32;

			if (curPart->m_3indices16)
				delete[] curPart->m_3indices16;

			if (curPart->m_indices16)
				delete[] curPart->m_indices16;

			if (curPart->m_3indices8)
				delete[] curPart->m_3indices8;
		}
		delete[] curData->m_meshPartsPtr;
		delete curData;
	}
	m_allocatedcbtStridingMeshInterfaceDatas.clear();

	for (i = 0; i < m_indexArrays.size(); i++)
	{
		cbtAlignedFree(m_indexArrays[i]);
	}
	m_indexArrays.clear();

	for (i = 0; i < m_shortIndexArrays.size(); i++)
	{
		cbtAlignedFree(m_shortIndexArrays[i]);
	}
	m_shortIndexArrays.clear();

	for (i = 0; i < m_charIndexArrays.size(); i++)
	{
		cbtAlignedFree(m_charIndexArrays[i]);
	}
	m_charIndexArrays.clear();

	for (i = 0; i < m_floatVertexArrays.size(); i++)
	{
		cbtAlignedFree(m_floatVertexArrays[i]);
	}
	m_floatVertexArrays.clear();

	for (i = 0; i < m_doubleVertexArrays.size(); i++)
	{
		cbtAlignedFree(m_doubleVertexArrays[i]);
	}
	m_doubleVertexArrays.clear();
}

cbtCollisionShape* cbtCollisionWorldImporter::convertCollisionShape(cbtCollisionShapeData* shapeData)
{
	cbtCollisionShape* shape = 0;

	switch (shapeData->m_shapeType)
	{
		case STATIC_PLANE_PROXYTYPE:
		{
			cbtStaticPlaneShapeData* planeData = (cbtStaticPlaneShapeData*)shapeData;
			cbtVector3 planeNormal, localScaling;
			planeNormal.deSerializeFloat(planeData->m_planeNormal);
			localScaling.deSerializeFloat(planeData->m_localScaling);
			shape = createPlaneShape(planeNormal, planeData->m_planeConstant);
			shape->setLocalScaling(localScaling);

			break;
		}
		case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			cbtScaledTriangleMeshShapeData* scaledMesh = (cbtScaledTriangleMeshShapeData*)shapeData;
			cbtCollisionShapeData* colShapeData = (cbtCollisionShapeData*)&scaledMesh->m_trimeshShapeData;
			colShapeData->m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
			cbtCollisionShape* childShape = convertCollisionShape(colShapeData);
			cbtBvhTriangleMeshShape* meshShape = (cbtBvhTriangleMeshShape*)childShape;
			cbtVector3 localScaling;
			localScaling.deSerializeFloat(scaledMesh->m_localScaling);

			shape = createScaledTrangleMeshShape(meshShape, localScaling);
			break;
		}
#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
		case GIMPACT_SHAPE_PROXYTYPE:
		{
			cbtGImpactMeshShapeData* gimpactData = (cbtGImpactMeshShapeData*)shapeData;
			if (gimpactData->m_gimpactSubType == CONST_GIMPACT_TRIMESH_SHAPE)
			{
				cbtStridingMeshInterfaceData* interfaceData = createStridingMeshInterfaceData(&gimpactData->m_meshInterface);
				cbtTriangleIndexVertexArray* meshInterface = createMeshInterface(*interfaceData);

				cbtGImpactMeshShape* gimpactShape = createGimpactShape(meshInterface);
				cbtVector3 localScaling;
				localScaling.deSerializeFloat(gimpactData->m_localScaling);
				gimpactShape->setLocalScaling(localScaling);
				gimpactShape->setMargin(cbtScalar(gimpactData->m_collisionMargin));
				gimpactShape->updateBound();
				shape = gimpactShape;
			}
			else
			{
				printf("unsupported gimpact sub type\n");
			}
			break;
		}
#endif  //SUPPORT_GIMPACT_SHAPE_IMPORT                                                                        \
		//The cbtCapsuleShape* API has issue passing the margin/scaling/halfextents unmodified through the API \
		//so deal with this
		case CAPSULE_SHAPE_PROXYTYPE:
		{
			cbtCapsuleShapeData* capData = (cbtCapsuleShapeData*)shapeData;

			switch (capData->m_upAxis)
			{
				case 0:
				{
					shape = createCapsuleShapeX(1, 1);
					break;
				}
				case 1:
				{
					shape = createCapsuleShapeY(1, 1);
					break;
				}
				case 2:
				{
					shape = createCapsuleShapeZ(1, 1);
					break;
				}
				default:
				{
					printf("error: wrong up axis for cbtCapsuleShape\n");
				}
			};
			if (shape)
			{
				cbtCapsuleShape* cap = (cbtCapsuleShape*)shape;
				cap->deSerializeFloat(capData);
			}
			break;
		}
		case CYLINDER_SHAPE_PROXYTYPE:
        case CYLSHELL_SHAPE_PROXYTYPE:
		case CONE_SHAPE_PROXYTYPE:
		case BOX_SHAPE_PROXYTYPE:
		case SPHERE_SHAPE_PROXYTYPE:
		case MULTI_SPHERE_SHAPE_PROXYTYPE:
		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			cbtConvexInternalShapeData* bsd = (cbtConvexInternalShapeData*)shapeData;
			cbtVector3 implicitShapeDimensions;
			implicitShapeDimensions.deSerializeFloat(bsd->m_implicitShapeDimensions);
			cbtVector3 localScaling;
			localScaling.deSerializeFloat(bsd->m_localScaling);
			cbtVector3 margin(bsd->m_collisionMargin, bsd->m_collisionMargin, bsd->m_collisionMargin);
			switch (shapeData->m_shapeType)
			{
				case BOX_SHAPE_PROXYTYPE:
				{
					cbtBoxShape* box = (cbtBoxShape*)createBoxShape(implicitShapeDimensions / localScaling + margin);
					//box->initializePolyhedralFeatures();
					shape = box;

					break;
				}
				case SPHERE_SHAPE_PROXYTYPE:
				{
					shape = createSphereShape(implicitShapeDimensions.getX());
					break;
				}

				case CYLINDER_SHAPE_PROXYTYPE:
				{
					cbtCylinderShapeData* cylData = (cbtCylinderShapeData*)shapeData;
					cbtVector3 halfExtents = implicitShapeDimensions + margin;
					switch (cylData->m_upAxis)
					{
						case 0:
						{
							shape = createCylinderShapeX(halfExtents.getY(), halfExtents.getX());
							break;
						}
						case 1:
						{
							shape = createCylinderShapeY(halfExtents.getX(), halfExtents.getY());
							break;
						}
						case 2:
						{
							shape = createCylinderShapeZ(halfExtents.getX(), halfExtents.getZ());
							break;
						}
						default:
						{
							printf("unknown Cylinder up axis\n");
						}
					};

					break;
				}

                case CYLSHELL_SHAPE_PROXYTYPE: { /* ***CHRONO*** */
                    ////cbtCylindricalShellShapeData* cylData = (cbtCylindricalShellShapeData*)shapeData;
                    cbtVector3 halfExtents = implicitShapeDimensions + margin;
                    cbtScalar radius = halfExtents.getX();
                    cbtScalar hlen = halfExtents.getY();
					shape = createCylindricalShellShape(radius, hlen);

                    break;
                }

				case CONE_SHAPE_PROXYTYPE:
				{
					cbtConeShapeData* conData = (cbtConeShapeData*)shapeData;
					cbtVector3 halfExtents = implicitShapeDimensions;  //+margin;
					switch (conData->m_upIndex)
					{
						case 0:
						{
							shape = createConeShapeX(halfExtents.getY(), halfExtents.getX());
							break;
						}
						case 1:
						{
							shape = createConeShapeY(halfExtents.getX(), halfExtents.getY());
							break;
						}
						case 2:
						{
							shape = createConeShapeZ(halfExtents.getX(), halfExtents.getZ());
							break;
						}
						default:
						{
							printf("unknown Cone up axis\n");
						}
					};

					break;
				}
				case MULTI_SPHERE_SHAPE_PROXYTYPE:
				{
					cbtMultiSphereShapeData* mss = (cbtMultiSphereShapeData*)bsd;
					int numSpheres = mss->m_localPositionArraySize;

					cbtAlignedObjectArray<cbtVector3> tmpPos;
					cbtAlignedObjectArray<cbtScalar> radii;
					radii.resize(numSpheres);
					tmpPos.resize(numSpheres);
					int i;
					for (i = 0; i < numSpheres; i++)
					{
						tmpPos[i].deSerializeFloat(mss->m_localPositionArrayPtr[i].m_pos);
						radii[i] = mss->m_localPositionArrayPtr[i].m_radius;
					}
					shape = createMultiSphereShape(&tmpPos[0], &radii[0], numSpheres);
					break;
				}
				case CONVEX_HULL_SHAPE_PROXYTYPE:
				{
					//	int sz = sizeof(cbtConvexHullShapeData);
					//	int sz2 = sizeof(cbtConvexInternalShapeData);
					//	int sz3 = sizeof(cbtCollisionShapeData);
					cbtConvexHullShapeData* convexData = (cbtConvexHullShapeData*)bsd;
					int numPoints = convexData->m_numUnscaledPoints;

					cbtAlignedObjectArray<cbtVector3> tmpPoints;
					tmpPoints.resize(numPoints);
					int i;
					for (i = 0; i < numPoints; i++)
					{
#ifdef BT_USE_DOUBLE_PRECISION
						if (convexData->m_unscaledPointsDoublePtr)
							tmpPoints[i].deSerialize(convexData->m_unscaledPointsDoublePtr[i]);
						if (convexData->m_unscaledPointsFloatPtr)
							tmpPoints[i].deSerializeFloat(convexData->m_unscaledPointsFloatPtr[i]);
#else
						if (convexData->m_unscaledPointsFloatPtr)
							tmpPoints[i].deSerialize(convexData->m_unscaledPointsFloatPtr[i]);
						if (convexData->m_unscaledPointsDoublePtr)
							tmpPoints[i].deSerializeDouble(convexData->m_unscaledPointsDoublePtr[i]);
#endif  //BT_USE_DOUBLE_PRECISION
					}
					cbtConvexHullShape* hullShape = createConvexHullShape();
					for (i = 0; i < numPoints; i++)
					{
						hullShape->addPoint(tmpPoints[i]);
					}
					hullShape->setMargin(bsd->m_collisionMargin);
					//hullShape->initializePolyhedralFeatures();
					shape = hullShape;
					break;
				}
				default:
				{
					printf("error: cannot create shape type (%d)\n", shapeData->m_shapeType);
				}
			}

			if (shape)
			{
				shape->setMargin(bsd->m_collisionMargin);

				cbtVector3 localScaling;
				localScaling.deSerializeFloat(bsd->m_localScaling);
				shape->setLocalScaling(localScaling);
			}
			break;
		}
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			cbtTriangleMeshShapeData* trimesh = (cbtTriangleMeshShapeData*)shapeData;
			cbtStridingMeshInterfaceData* interfaceData = createStridingMeshInterfaceData(&trimesh->m_meshInterface);
			cbtTriangleIndexVertexArray* meshInterface = createMeshInterface(*interfaceData);
			if (!meshInterface->getNumSubParts())
			{
				return 0;
			}

			cbtVector3 scaling;
			scaling.deSerializeFloat(trimesh->m_meshInterface.m_scaling);
			meshInterface->setScaling(scaling);

			cbtOptimizedBvh* bvh = 0;
#if 1
			if (trimesh->m_quantizedFloatBvh)
			{
				cbtOptimizedBvh** bvhPtr = m_bvhMap.find(trimesh->m_quantizedFloatBvh);
				if (bvhPtr && *bvhPtr)
				{
					bvh = *bvhPtr;
				}
				else
				{
					bvh = createOptimizedBvh();
					bvh->deSerializeFloat(*trimesh->m_quantizedFloatBvh);
				}
			}
			if (trimesh->m_quantizedDoubleBvh)
			{
				cbtOptimizedBvh** bvhPtr = m_bvhMap.find(trimesh->m_quantizedDoubleBvh);
				if (bvhPtr && *bvhPtr)
				{
					bvh = *bvhPtr;
				}
				else
				{
					bvh = createOptimizedBvh();
					bvh->deSerializeDouble(*trimesh->m_quantizedDoubleBvh);
				}
			}
#endif

			cbtBvhTriangleMeshShape* trimeshShape = createBvhTriangleMeshShape(meshInterface, bvh);
			trimeshShape->setMargin(trimesh->m_collisionMargin);
			shape = trimeshShape;

			if (trimesh->m_triangleInfoMap)
			{
				cbtTriangleInfoMap* map = createTriangleInfoMap();
				map->deSerialize(*trimesh->m_triangleInfoMap);
				trimeshShape->setTriangleInfoMap(map);

#ifdef USE_INTERNAL_EDGE_UTILITY
				gContactAddedCallback = cbtAdjustInternalEdgeContactsCallback;
#endif  //USE_INTERNAL_EDGE_UTILITY
			}

			//printf("trimesh->m_collisionMargin=%f\n",trimesh->m_collisionMargin);
			break;
		}
		case COMPOUND_SHAPE_PROXYTYPE:
		{
			cbtCompoundShapeData* compoundData = (cbtCompoundShapeData*)shapeData;
			cbtCompoundShape* compoundShape = createCompoundShape();

			//cbtCompoundShapeChildData* childShapeDataArray = &compoundData->m_childShapePtr[0];

			cbtAlignedObjectArray<cbtCollisionShape*> childShapes;
			for (int i = 0; i < compoundData->m_numChildShapes; i++)
			{
				//cbtCompoundShapeChildData* ptr = &compoundData->m_childShapePtr[i];

				cbtCollisionShapeData* cd = compoundData->m_childShapePtr[i].m_childShape;

				cbtCollisionShape* childShape = convertCollisionShape(cd);
				if (childShape)
				{
					cbtTransform localTransform;
					localTransform.deSerializeFloat(compoundData->m_childShapePtr[i].m_transform);
					compoundShape->addChildShape(localTransform, childShape);
				}
				else
				{
#ifdef _DEBUG
					printf("error: couldn't create childShape for compoundShape\n");
#endif
				}
			}
			shape = compoundShape;

			break;
		}
		case SOFTBODY_SHAPE_PROXYTYPE:
		{
			return 0;
		}
		default:
		{
#ifdef _DEBUG
			printf("unsupported shape type (%d)\n", shapeData->m_shapeType);
#endif
		}
	}

	return shape;
}

char* cbtCollisionWorldImporter::duplicateName(const char* name)
{
	if (name)
	{
		int l = (int)strlen(name);
		char* newName = new char[l + 1];
		memcpy(newName, name, l);
		newName[l] = 0;
		m_allocatedNames.push_back(newName);
		return newName;
	}
	return 0;
}

cbtTriangleIndexVertexArray* cbtCollisionWorldImporter::createMeshInterface(cbtStridingMeshInterfaceData& meshData)
{
	cbtTriangleIndexVertexArray* meshInterface = createTriangleMeshContainer();

	for (int i = 0; i < meshData.m_numMeshParts; i++)
	{
		cbtIndexedMesh meshPart;
		meshPart.m_numTriangles = meshData.m_meshPartsPtr[i].m_numTriangles;
		meshPart.m_numVertices = meshData.m_meshPartsPtr[i].m_numVertices;

		if (meshData.m_meshPartsPtr[i].m_indices32)
		{
			meshPart.m_indexType = PHY_INTEGER;
			meshPart.m_triangleIndexStride = 3 * sizeof(int);
			int* indexArray = (int*)cbtAlignedAlloc(sizeof(int) * 3 * meshPart.m_numTriangles, 16);
			m_indexArrays.push_back(indexArray);
			for (int j = 0; j < 3 * meshPart.m_numTriangles; j++)
			{
				indexArray[j] = meshData.m_meshPartsPtr[i].m_indices32[j].m_value;
			}
			meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
		}
		else
		{
			if (meshData.m_meshPartsPtr[i].m_3indices16)
			{
				meshPart.m_indexType = PHY_SHORT;
				meshPart.m_triangleIndexStride = sizeof(short int) * 3;  //sizeof(cbtShortIntIndexTripletData);

				short int* indexArray = (short int*)cbtAlignedAlloc(sizeof(short int) * 3 * meshPart.m_numTriangles, 16);
				m_shortIndexArrays.push_back(indexArray);

				for (int j = 0; j < meshPart.m_numTriangles; j++)
				{
					indexArray[3 * j] = meshData.m_meshPartsPtr[i].m_3indices16[j].m_values[0];
					indexArray[3 * j + 1] = meshData.m_meshPartsPtr[i].m_3indices16[j].m_values[1];
					indexArray[3 * j + 2] = meshData.m_meshPartsPtr[i].m_3indices16[j].m_values[2];
				}

				meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
			}
			if (meshData.m_meshPartsPtr[i].m_indices16)
			{
				meshPart.m_indexType = PHY_SHORT;
				meshPart.m_triangleIndexStride = 3 * sizeof(short int);
				short int* indexArray = (short int*)cbtAlignedAlloc(sizeof(short int) * 3 * meshPart.m_numTriangles, 16);
				m_shortIndexArrays.push_back(indexArray);
				for (int j = 0; j < 3 * meshPart.m_numTriangles; j++)
				{
					indexArray[j] = meshData.m_meshPartsPtr[i].m_indices16[j].m_value;
				}

				meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
			}

			if (meshData.m_meshPartsPtr[i].m_3indices8)
			{
				meshPart.m_indexType = PHY_UCHAR;
				meshPart.m_triangleIndexStride = sizeof(unsigned char) * 3;

				unsigned char* indexArray = (unsigned char*)cbtAlignedAlloc(sizeof(unsigned char) * 3 * meshPart.m_numTriangles, 16);
				m_charIndexArrays.push_back(indexArray);

				for (int j = 0; j < meshPart.m_numTriangles; j++)
				{
					indexArray[3 * j] = meshData.m_meshPartsPtr[i].m_3indices8[j].m_values[0];
					indexArray[3 * j + 1] = meshData.m_meshPartsPtr[i].m_3indices8[j].m_values[1];
					indexArray[3 * j + 2] = meshData.m_meshPartsPtr[i].m_3indices8[j].m_values[2];
				}

				meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
			}
		}

		if (meshData.m_meshPartsPtr[i].m_vertices3f)
		{
			meshPart.m_vertexType = PHY_FLOAT;
			meshPart.m_vertexStride = sizeof(cbtVector3FloatData);
			cbtVector3FloatData* vertices = (cbtVector3FloatData*)cbtAlignedAlloc(sizeof(cbtVector3FloatData) * meshPart.m_numVertices, 16);
			m_floatVertexArrays.push_back(vertices);

			for (int j = 0; j < meshPart.m_numVertices; j++)
			{
				vertices[j].m_floats[0] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[0];
				vertices[j].m_floats[1] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[1];
				vertices[j].m_floats[2] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[2];
				vertices[j].m_floats[3] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[3];
			}
			meshPart.m_vertexBase = (const unsigned char*)vertices;
		}
		else
		{
			meshPart.m_vertexType = PHY_DOUBLE;
			meshPart.m_vertexStride = sizeof(cbtVector3DoubleData);

			cbtVector3DoubleData* vertices = (cbtVector3DoubleData*)cbtAlignedAlloc(sizeof(cbtVector3DoubleData) * meshPart.m_numVertices, 16);
			m_doubleVertexArrays.push_back(vertices);

			for (int j = 0; j < meshPart.m_numVertices; j++)
			{
				vertices[j].m_floats[0] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[0];
				vertices[j].m_floats[1] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[1];
				vertices[j].m_floats[2] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[2];
				vertices[j].m_floats[3] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[3];
			}
			meshPart.m_vertexBase = (const unsigned char*)vertices;
		}

		if (meshPart.m_triangleIndexBase && meshPart.m_vertexBase)
		{
			meshInterface->addIndexedMesh(meshPart, meshPart.m_indexType);
		}
	}

	return meshInterface;
}

cbtStridingMeshInterfaceData* cbtCollisionWorldImporter::createStridingMeshInterfaceData(cbtStridingMeshInterfaceData* interfaceData)
{
	//create a new cbtStridingMeshInterfaceData that is an exact copy of shapedata and store it in the WorldImporter
	cbtStridingMeshInterfaceData* newData = new cbtStridingMeshInterfaceData;

	newData->m_scaling = interfaceData->m_scaling;
	newData->m_numMeshParts = interfaceData->m_numMeshParts;
	newData->m_meshPartsPtr = new cbtMeshPartData[newData->m_numMeshParts];

	for (int i = 0; i < newData->m_numMeshParts; i++)
	{
		cbtMeshPartData* curPart = &interfaceData->m_meshPartsPtr[i];
		cbtMeshPartData* curNewPart = &newData->m_meshPartsPtr[i];

		curNewPart->m_numTriangles = curPart->m_numTriangles;
		curNewPart->m_numVertices = curPart->m_numVertices;

		if (curPart->m_vertices3f)
		{
			curNewPart->m_vertices3f = new cbtVector3FloatData[curNewPart->m_numVertices];
			memcpy(curNewPart->m_vertices3f, curPart->m_vertices3f, sizeof(cbtVector3FloatData) * curNewPart->m_numVertices);
		}
		else
			curNewPart->m_vertices3f = NULL;

		if (curPart->m_vertices3d)
		{
			curNewPart->m_vertices3d = new cbtVector3DoubleData[curNewPart->m_numVertices];
			memcpy(curNewPart->m_vertices3d, curPart->m_vertices3d, sizeof(cbtVector3DoubleData) * curNewPart->m_numVertices);
		}
		else
			curNewPart->m_vertices3d = NULL;

		int numIndices = curNewPart->m_numTriangles * 3;
		///the m_3indices8 was not initialized in some Bullet versions, this can cause crashes at loading time
		///we catch it by only dealing with m_3indices8 if none of the other indices are initialized
		bool uninitialized3indices8Workaround = false;

		if (curPart->m_indices32)
		{
			uninitialized3indices8Workaround = true;
			curNewPart->m_indices32 = new cbtIntIndexData[numIndices];
			memcpy(curNewPart->m_indices32, curPart->m_indices32, sizeof(cbtIntIndexData) * numIndices);
		}
		else
			curNewPart->m_indices32 = NULL;

		if (curPart->m_3indices16)
		{
			uninitialized3indices8Workaround = true;
			curNewPart->m_3indices16 = new cbtShortIntIndexTripletData[curNewPart->m_numTriangles];
			memcpy(curNewPart->m_3indices16, curPart->m_3indices16, sizeof(cbtShortIntIndexTripletData) * curNewPart->m_numTriangles);
		}
		else
			curNewPart->m_3indices16 = NULL;

		if (curPart->m_indices16)
		{
			uninitialized3indices8Workaround = true;
			curNewPart->m_indices16 = new cbtShortIntIndexData[numIndices];
			memcpy(curNewPart->m_indices16, curPart->m_indices16, sizeof(cbtShortIntIndexData) * numIndices);
		}
		else
			curNewPart->m_indices16 = NULL;

		if (!uninitialized3indices8Workaround && curPart->m_3indices8)
		{
			curNewPart->m_3indices8 = new cbtCharIndexTripletData[curNewPart->m_numTriangles];
			memcpy(curNewPart->m_3indices8, curPart->m_3indices8, sizeof(cbtCharIndexTripletData) * curNewPart->m_numTriangles);
		}
		else
			curNewPart->m_3indices8 = NULL;
	}

	m_allocatedcbtStridingMeshInterfaceDatas.push_back(newData);

	return (newData);
}

#ifdef USE_INTERNAL_EDGE_UTILITY
extern ContactAddedCallback gContactAddedCallback;

static bool cbtAdjustInternalEdgeContactsCallback(cbtManifoldPoint& cp, const cbtCollisionObject* colObj0, int partId0, int index0, const cbtCollisionObject* colObj1, int partId1, int index1)
{
	cbtAdjustInternalEdgeContacts(cp, colObj1, colObj0, partId1, index1);
	//cbtAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
	//cbtAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	return true;
}
#endif  //USE_INTERNAL_EDGE_UTILITY

/*
cbtRigidBody*  cbtWorldImporter::createRigidBody(bool isDynamic, cbtScalar mass, const cbtTransform& startTransform,cbtCollisionShape* shape,const char* bodyName)
{
	cbtVector3 localInertia;
	localInertia.setZero();

	if (mass)
		shape->calculateLocalInertia(mass,localInertia);

	cbtRigidBody* body = new cbtRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);

	if (m_dynamicsWorld)
		m_dynamicsWorld->addRigidBody(body);

	if (bodyName)
	{
		char* newname = duplicateName(bodyName);
		m_objectNameMap.insert(body,newname);
		m_nameBodyMap.insert(newname,body);
	}
	m_allocatedRigidBodies.push_back(body);
	return body;

}
*/

cbtCollisionObject* cbtCollisionWorldImporter::getCollisionObjectByName(const char* name)
{
	cbtCollisionObject** bodyPtr = m_nameColObjMap.find(name);
	if (bodyPtr && *bodyPtr)
	{
		return *bodyPtr;
	}
	return 0;
}

cbtCollisionObject* cbtCollisionWorldImporter::createCollisionObject(const cbtTransform& startTransform, cbtCollisionShape* shape, const char* bodyName)
{
	cbtCollisionObject* colObj = new cbtCollisionObject();
	colObj->setWorldTransform(startTransform);
	colObj->setCollisionShape(shape);
	m_collisionWorld->addCollisionObject(colObj);  //todo: flags etc

	if (bodyName)
	{
		char* newname = duplicateName(bodyName);
		m_objectNameMap.insert(colObj, newname);
		m_nameColObjMap.insert(newname, colObj);
	}
	m_allocatedCollisionObjects.push_back(colObj);

	return colObj;
}

cbtCollisionShape* cbtCollisionWorldImporter::createPlaneShape(const cbtVector3& planeNormal, cbtScalar planeConstant)
{
	cbtStaticPlaneShape* shape = new cbtStaticPlaneShape(planeNormal, planeConstant);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
cbtCollisionShape* cbtCollisionWorldImporter::createBoxShape(const cbtVector3& halfExtents)
{
	cbtBoxShape* shape = new cbtBoxShape(halfExtents);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
cbtCollisionShape* cbtCollisionWorldImporter::createSphereShape(cbtScalar radius)
{
	cbtSphereShape* shape = new cbtSphereShape(radius);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createCapsuleShapeX(cbtScalar radius, cbtScalar height)
{
	cbtCapsuleShapeX* shape = new cbtCapsuleShapeX(radius, height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createCapsuleShapeY(cbtScalar radius, cbtScalar height)
{
	cbtCapsuleShape* shape = new cbtCapsuleShape(radius, height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createCapsuleShapeZ(cbtScalar radius, cbtScalar height)
{
	cbtCapsuleShapeZ* shape = new cbtCapsuleShapeZ(radius, height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createCylinderShapeX(cbtScalar radius, cbtScalar height)
{
	cbtCylinderShapeX* shape = new cbtCylinderShapeX(cbtVector3(height, radius, radius));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createCylinderShapeY(cbtScalar radius, cbtScalar height)
{
	cbtCylinderShape* shape = new cbtCylinderShape(cbtVector3(radius, height, radius));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createCylinderShapeZ(cbtScalar radius, cbtScalar height)
{
	cbtCylinderShapeZ* shape = new cbtCylinderShapeZ(cbtVector3(radius, radius, height));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

/* ***CHRONO*** */
cbtCollisionShape* cbtCollisionWorldImporter::createCylindricalShellShape(cbtScalar radius,
                                                                        cbtScalar height) {
    cbtCylindricalShellShape* shape = new cbtCylindricalShellShape(radius, height);
    m_allocatedCollisionShapes.push_back(shape);
    return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createConeShapeX(cbtScalar radius, cbtScalar height)
{
	cbtConeShapeX* shape = new cbtConeShapeX(radius, height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createConeShapeY(cbtScalar radius, cbtScalar height)
{
	cbtConeShape* shape = new cbtConeShape(radius, height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCollisionShape* cbtCollisionWorldImporter::createConeShapeZ(cbtScalar radius, cbtScalar height)
{
	cbtConeShapeZ* shape = new cbtConeShapeZ(radius, height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtTriangleIndexVertexArray* cbtCollisionWorldImporter::createTriangleMeshContainer()
{
	cbtTriangleIndexVertexArray* in = new cbtTriangleIndexVertexArray();
	m_allocatedTriangleIndexArrays.push_back(in);
	return in;
}

cbtOptimizedBvh* cbtCollisionWorldImporter::createOptimizedBvh()
{
	cbtOptimizedBvh* bvh = new cbtOptimizedBvh();
	m_allocatedBvhs.push_back(bvh);
	return bvh;
}

cbtTriangleInfoMap* cbtCollisionWorldImporter::createTriangleInfoMap()
{
	cbtTriangleInfoMap* tim = new cbtTriangleInfoMap();
	m_allocatedTriangleInfoMaps.push_back(tim);
	return tim;
}

cbtBvhTriangleMeshShape* cbtCollisionWorldImporter::createBvhTriangleMeshShape(cbtStridingMeshInterface* trimesh, cbtOptimizedBvh* bvh)
{
	if (bvh)
	{
		cbtBvhTriangleMeshShape* bvhTriMesh = new cbtBvhTriangleMeshShape(trimesh, bvh->isQuantized(), false);
		bvhTriMesh->setOptimizedBvh(bvh);
		m_allocatedCollisionShapes.push_back(bvhTriMesh);
		return bvhTriMesh;
	}

	cbtBvhTriangleMeshShape* ts = new cbtBvhTriangleMeshShape(trimesh, true);
	m_allocatedCollisionShapes.push_back(ts);
	return ts;
}
cbtCollisionShape* cbtCollisionWorldImporter::createConvexTriangleMeshShape(cbtStridingMeshInterface* trimesh)
{
	return 0;
}
#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
cbtGImpactMeshShape* cbtCollisionWorldImporter::createGimpactShape(cbtStridingMeshInterface* trimesh)
{
	cbtGImpactMeshShape* shape = new cbtGImpactMeshShape(trimesh);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
#endif  //SUPPORT_GIMPACT_SHAPE_IMPORT

cbtConvexHullShape* cbtCollisionWorldImporter::createConvexHullShape()
{
	cbtConvexHullShape* shape = new cbtConvexHullShape();
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtCompoundShape* cbtCollisionWorldImporter::createCompoundShape()
{
	cbtCompoundShape* shape = new cbtCompoundShape();
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtScaledBvhTriangleMeshShape* cbtCollisionWorldImporter::createScaledTrangleMeshShape(cbtBvhTriangleMeshShape* meshShape, const cbtVector3& localScaling)
{
	cbtScaledBvhTriangleMeshShape* shape = new cbtScaledBvhTriangleMeshShape(meshShape, localScaling);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

cbtMultiSphereShape* cbtCollisionWorldImporter::createMultiSphereShape(const cbtVector3* positions, const cbtScalar* radi, int numSpheres)
{
	cbtMultiSphereShape* shape = new cbtMultiSphereShape(positions, radi, numSpheres);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

// query for data
int cbtCollisionWorldImporter::getNumCollisionShapes() const
{
	return m_allocatedCollisionShapes.size();
}

cbtCollisionShape* cbtCollisionWorldImporter::getCollisionShapeByIndex(int index)
{
	return m_allocatedCollisionShapes[index];
}

cbtCollisionShape* cbtCollisionWorldImporter::getCollisionShapeByName(const char* name)
{
	cbtCollisionShape** shapePtr = m_nameShapeMap.find(name);
	if (shapePtr && *shapePtr)
	{
		return *shapePtr;
	}
	return 0;
}

const char* cbtCollisionWorldImporter::getNameForPointer(const void* ptr) const
{
	const char* const* namePtr = m_objectNameMap.find(ptr);
	if (namePtr && *namePtr)
		return *namePtr;
	return 0;
}

int cbtCollisionWorldImporter::getNumRigidBodies() const
{
	return m_allocatedRigidBodies.size();
}

cbtCollisionObject* cbtCollisionWorldImporter::getRigidBodyByIndex(int index) const
{
	return m_allocatedRigidBodies[index];
}

int cbtCollisionWorldImporter::getNumBvhs() const
{
	return m_allocatedBvhs.size();
}
cbtOptimizedBvh* cbtCollisionWorldImporter::getBvhByIndex(int index) const
{
	return m_allocatedBvhs[index];
}

int cbtCollisionWorldImporter::getNumTriangleInfoMaps() const
{
	return m_allocatedTriangleInfoMaps.size();
}

cbtTriangleInfoMap* cbtCollisionWorldImporter::getTriangleInfoMapByIndex(int index) const
{
	return m_allocatedTriangleInfoMaps[index];
}
