/*************************************************************************
 *                                                                       *
 * Tokamak Physics Engine, Copyright (C) 2002-2007 David Lam.            *
 * All rights reserved.  Email: david@tokamakphysics.com                 *
 *                       Web: www.tokamakphysics.com                     *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

#pragma inline_recursion( on )
#pragma inline_depth( 250 )


/*
#ifdef _WIN32
#include <windows.h>
#endif
*/

#include "tokamak.h"
#include "collision.h"
#include "constraint.h"
#include "rigidbody.h"
#include "scenery.h"
#include "simulator.h"

/****************************************************************************
*
*	neCollisionBody_::UpdateAABB
*
****************************************************************************/

void neCollisionBody::UpdateAABB() {
    if (col.convexCount == 0 && !isCustomCD)
        return;
/*
	neM3 c;
		
	c[0] = col.obb.as.box.boxSize[0] * col.obb.c2p.rot[0];
	c[1] = col.obb.as.box.boxSize[1] * col.obb.c2p.rot[1];
	c[2] = col.obb.as.box.boxSize[2] * col.obb.c2p.rot[2];
*/
    neT3 c2w = b2w * obb;

    neV3 & pos = c2w.pos;

    int i;

    for (i = 0; i < 3; i++) {
        neReal a = neAbs(c2w.rot[0][i]) + neAbs(c2w.rot[1][i]) + neAbs(c2w.rot[2][i]);

        minBound[i] = pos[i] - a;
        maxBound[i] = pos[i] + a;

        if (minCoord[i])
            minCoord[i]->value = pos[i] - a;// - col.boundingRadius;

        if (maxCoord[i])
            maxCoord[i]->value = pos[i] + a;// + col.boundingRadius;
    }
};

void neCollisionBody::Free() {
    neRigidBodyBase::Free();

    RemoveConstraintHeader();
}

/****************************************************************************
*
*	neCollisionBody::RemoveGeometry
*
****************************************************************************/

bool neCollisionBody::RemoveGeometry(neGeometry *g) {
    if (!col.convex)
        return false;

    TConvexItem *gi = (TConvexItem *) col.convex;

    while (gi) {
        TConvex *convex = reinterpret_cast<TConvex *>(gi);

        gi = gi->next;

        if (convex == reinterpret_cast<TConvex *>(g)) {
            if (col.convex == convex) {
                col.convex = (TConvex *) gi;
            }

            sim->geometryHeap.Dealloc(convex, 1);

            col.convexCount--;

            if (col.convexCount == 0) {
                col.convex = nullptr;

                if (IsInRegion() && !isCustomCD)
                    sim->region.RemoveBody(this);
            }

            return true;
        }
    }
    return false;
}

/****************************************************************************
*
*	neCollisionBody::BreakGeometry
*
****************************************************************************/

neRigidBody *neCollisionBody::BreakGeometry(neGeometry *g) {
    neRigidBody *newBody = sim->CreateRigidBodyFromConvex((TConvex *) g, this);
    return (neRigidBody *) newBody;
}

/****************************************************************************
*
*	neCollisionBody::UseCustomCollisionDetection
*
****************************************************************************/

void neCollisionBody::UseCustomCollisionDetection(bool yes, const neT3 *obb, neReal boundingRadius) {
    if (yes) {
        this->obb = *obb;

        col.boundingRadius = boundingRadius;

        isCustomCD = yes;

        if (isActive && !IsInRegion()) {
            sim->region.AddBody(this, nullptr);
        }
    } else {
        isCustomCD = yes;

        this->UpdateBoundingInfo();

        if (IsInRegion() && GetGeometryCount() == 0) {
            sim->region.RemoveBody(this);
        }
    }
}


/****************************************************************************
*
*	neCollisionBody::RemoveSensor
*
****************************************************************************/

bool neCollisionBody::RemoveSensor(neSensor *s) {
    if (!sensors)
        return false;

    neSensorItem *si = (neSensorItem *) sensors;

    while (si) {
        neSensor_ *sensor = (neSensor_ *) si;

        si = si->next;

        if (sensor == reinterpret_cast<neSensor_ *>(s)) {
            //reinterpret_cast<neSensorItem *>(s)->Remove();

            sim->sensorHeap.Dealloc(sensor, 1);

            return true;
        }
    }
    return false;
}

bool neCollisionBody::Moved() const {
    return(lastStepWhenMoved == sim->stepSoFar);
}

void neCollisionBody::SetMoved() {
    lastStepWhenMoved = sim->stepSoFar;
}