/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// This file was created by Alex Silverman

#ifndef BT_MATERIAL_H
#define BT_MATERIAL_H

// Material class to be used by cbtMultimaterialTriangleMeshShape to store triangle properties
class cbtMaterial
{
	// public members so that materials can change due to world events
public:
	cbtScalar m_friction;
	cbtScalar m_restitution;
	int pad[2];

	cbtMaterial() {}
	cbtMaterial(cbtScalar fric, cbtScalar rest)
	{
		m_friction = fric;
		m_restitution = rest;
	}
};

#endif  // BT_MATERIAL_H
