#ifndef BT_DEFAULT_MOTION_STATE_H
#define BT_DEFAULT_MOTION_STATE_H

#include "cbtMotionState.h"

///The cbtDefaultMotionState provides a common implementation to synchronize world transforms with offsets.
ATTRIBUTE_ALIGNED16(struct)
cbtDefaultMotionState : public cbtMotionState
{
	cbtTransform m_graphicsWorldTrans;
	cbtTransform m_centerOfMassOffset;
	cbtTransform m_startWorldTrans;
	void* m_userPointer;

	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtDefaultMotionState(const cbtTransform& startTrans = cbtTransform::getIdentity(), const cbtTransform& centerOfMassOffset = cbtTransform::getIdentity())
		: m_graphicsWorldTrans(startTrans),
		  m_centerOfMassOffset(centerOfMassOffset),
		  m_startWorldTrans(startTrans),
		  m_userPointer(0)

	{
	}

	///synchronizes world transform from user to physics
	virtual void getWorldTransform(cbtTransform & centerOfMassWorldTrans) const
	{
		centerOfMassWorldTrans = m_graphicsWorldTrans * m_centerOfMassOffset.inverse();
	}

	///synchronizes world transform from physics to user
	///Bullet only calls the update of worldtransform for active objects
	virtual void setWorldTransform(const cbtTransform& centerOfMassWorldTrans)
	{
		m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset;
	}
};

#endif  //BT_DEFAULT_MOTION_STATE_H
