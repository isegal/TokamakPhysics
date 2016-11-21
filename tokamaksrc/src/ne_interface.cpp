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

#include "math/ne_type.h"
#include "math/ne_debug.h"
#include "tokamak.h"
#include "containers.h"
#include "collision.h"
#include "constraint.h"
#include "rigidbody.h"
#include "scenery.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include "simulator.h"

#define CAST_THIS(a, b) a& b = reinterpret_cast<a&>(*this);

#ifdef TOKAMAK_COMPILE_DLL

BOOL APIENTRY DllMain( HANDLE hModule,
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
        case DLL_PROCESS_ATTACH:
        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}

#endif

/****************************************************************************
*
*	neGeometry::SetBoxSize
*
****************************************************************************/

void neGeometry::SetBoxSize(neReal width, neReal height, neReal depth) {
    CAST_THIS(TConvex, con);

    con.SetBoxSize(width, height, depth);
}

/****************************************************************************
*
*	neGeometry::SetBoxSize
*
****************************************************************************/

void neGeometry::SetBoxSize(const neV3 &boxSize) {
    CAST_THIS(TConvex, con);

    con.SetBoxSize(boxSize[0], boxSize[1], boxSize[2]);
}

/****************************************************************************
*
*	neGeometry::SetCylinder
*
****************************************************************************/

void neGeometry::SetCylinder(neReal diameter, neReal height) {
    CAST_THIS(TConvex, con);

    con.type = TConvex::CYLINDER;

    con.as.cylinder.radius = diameter * 0.5f;

    con.as.cylinder.radiusSq = con.as.cylinder.radius * con.as.cylinder.radius;

    con.as.cylinder.halfHeight = height * 0.5f;
}

/****************************************************************************
*
*	neGeometry::GetCylinder
*
****************************************************************************/

bool neGeometry::GetCylinder(neReal &diameter, neReal &height) // return false if geometry is not a cylinder
{
    CAST_THIS(TConvex, con);

    if (con.type != TConvex::CYLINDER)
        return false;

    diameter = con.CylinderRadius() * 2.0f;

    height = con.CylinderHalfHeight() * 2.0f;

    return true;
}

/****************************************************************************
*
*	neGeometry::SetConvexMesh
*
****************************************************************************/

void neGeometry::SetConvexMesh(neByte *convexData) {
    CAST_THIS(TConvex, con);

    con.SetConvexMesh(convexData);
}

/****************************************************************************
*
*	neGeometry::GetConvexMesh
*
****************************************************************************/

bool neGeometry::GetConvexMesh(neByte *&convexData) {
    CAST_THIS(TConvex, con);

    if (con.type != TConvex::CONVEXDCD)
        return false;

    convexData = con.as.convexDCD.convexData;

    return true;
}

/****************************************************************************
*
*	neGeometry::SetTransform
*
****************************************************************************/

void neGeometry::SetTransform(neT3 &t) {
    CAST_THIS(TConvex, con);

    con.SetTransform(t);
}

/****************************************************************************
*
*	neGeometry::SetMaterialIndex
*
****************************************************************************/

void neGeometry::SetMaterialIndex(s32 index) {
    CAST_THIS(TConvex, con);

    con.SetMaterialId(index);
}

/****************************************************************************
*
*	neGeometry::GetMaterialIndex
*
****************************************************************************/

s32 neGeometry::GetMaterialIndex() {
    CAST_THIS(TConvex, con);

    return con.matIndex;
}

/****************************************************************************
*
*	neGeometry::GetTransform
*
****************************************************************************/

neT3 neGeometry::GetTransform() {
    CAST_THIS(TConvex, con);

    return con.c2p;
}

/****************************************************************************
*
*	neGeometry::SetUserData
*
****************************************************************************/

void neGeometry::SetUserData(void * userData) {
    CAST_THIS(TConvex, con);

    con.userData = userData;
}

/****************************************************************************
*
*	neGeometry::GetUserData
*
****************************************************************************/

void * neGeometry::GetUserData() {
    CAST_THIS(TConvex, con);

    return con.userData;
}

/****************************************************************************
*
*	neGeometry::GetBoxSize
*
****************************************************************************/

bool neGeometry::GetBoxSize(neV3 &boxSize) // return false if geometry is not a box
{
    CAST_THIS(TConvex, con);

    if (con.type != TConvex::BOX)
        return false;

    boxSize = con.as.box.boxSize * 2.0f;

    return true;
}

/****************************************************************************
*
*	neGeometry::SetSphereDiameter
*
****************************************************************************/

void neGeometry::SetSphereDiameter(neReal diameter) {
    CAST_THIS(TConvex, con);

    con.type = TConvex::SPHERE;

    con.as.sphere.radius = diameter * 0.5f;

    con.as.sphere.radiusSq = con.as.sphere.radius * con.as.sphere.radius;
}

/****************************************************************************
*
*	neGeometry::GetSphereDiameter
*
****************************************************************************/

bool neGeometry::GetSphereDiameter(neReal &diameter) // return false if geometry is not a sphere
{
    CAST_THIS(TConvex, con);

    if (con.type != TConvex::SPHERE)
        return false;

    diameter = con.Radius() * 2.0f;

    return true;
}

/****************************************************************************
*
*	neGeometry::SetBreakFlag
*
****************************************************************************/

void neGeometry::SetBreakageFlag(neBreakFlag flag) {
    CAST_THIS(TConvex, con);

    con.breakInfo.flag = flag;
}

/****************************************************************************
*
*	neGeometry::GetBreakFlag
*
****************************************************************************/

neGeometry::neBreakFlag neGeometry::GetBreakageFlag() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.flag;
}

/****************************************************************************
*
*	neGeometry::SetBreakageMass
*
****************************************************************************/

void neGeometry::SetBreakageMass(neReal mass) {
    CAST_THIS(TConvex, con);

    con.breakInfo.mass = mass;
}

/****************************************************************************
*
*	neGeometry::GetBreakageMass
*
****************************************************************************/

neReal neGeometry::GetBreakageMass() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.mass;
}

/****************************************************************************
*
*	neGeometry::SetBreakageInertiaTensor
*
****************************************************************************/

void neGeometry::SetBreakageInertiaTensor(const neV3 &tensor) {
    CAST_THIS(TConvex, con);

    con.breakInfo.inertiaTensor = tensor;
}

/****************************************************************************
*
*	neGeometry::GetBreakageInertiaTensor
*
****************************************************************************/

neV3 neGeometry::GetBreakageInertiaTensor() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.inertiaTensor;
}

/****************************************************************************
*
*	neGeometry::SetBreakageMagnitude
*
****************************************************************************/

void neGeometry::SetBreakageMagnitude(neReal mag) {
    CAST_THIS(TConvex, con);

    con.breakInfo.breakMagnitude = mag;
}

/****************************************************************************
*
*	neGeometry::GetBreakageMagnitude
*
****************************************************************************/

neReal neGeometry::GetBreakageMagnitude() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.breakMagnitude;
}

/****************************************************************************
*
*	neGeometry::SetBreakageAbsorption
*
****************************************************************************/

void neGeometry::SetBreakageAbsorption(neReal absorb) {
    CAST_THIS(TConvex, con);

    con.breakInfo.breakAbsorb = absorb;
}

void neGeometry::SetBreakagePlane(const neV3 &planeNormal) {
    CAST_THIS(TConvex, con);

    con.breakInfo.breakPlane = planeNormal;
}

neV3 neGeometry::GetBreakagePlane() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.breakPlane;
}

/****************************************************************************
*
*	neGeometry::GetBreakageAbsorption
*
****************************************************************************/

neReal neGeometry::GetBreakageAbsorption() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.breakAbsorb;
}

/****************************************************************************
*
*	neGeometry::SetBreakNeighbourRadius
*
****************************************************************************/

void neGeometry::SetBreakageNeighbourRadius(neReal radius) {
    CAST_THIS(TConvex, con);

    con.breakInfo.neighbourRadius = radius;
}

/****************************************************************************
*
*	neGeometry::GetBreakNeighbourRadius
*
****************************************************************************/

neReal neGeometry::GetBreakageNeighbourRadius() {
    CAST_THIS(TConvex, con);

    return con.breakInfo.neighbourRadius;
}

/****************************************************************************
*
*	neAnimatedBody::GetPos
*
****************************************************************************/

neV3 neAnimatedBody::GetPos() {
    CAST_THIS(neCollisionBody_, cb);

    return cb.b2w.pos;
}

/****************************************************************************
*
*	neAnimatedBody::SetPos
*
****************************************************************************/

void neAnimatedBody::SetPos(const neV3 &p) {
    CAST_THIS(neCollisionBody_, cb);

    cb.b2w.pos = p;

    cb.UpdateAABB();

    cb.moved = true;
}

/****************************************************************************
*
*	neAnimatedBody::GetRotationM3
*
****************************************************************************/

neM3 neAnimatedBody::GetRotationM3() {
    CAST_THIS(neCollisionBody_, cb);

    return cb.b2w.rot;
}

/****************************************************************************
*
*	neAnimatedBody::GetRotationQ
*
****************************************************************************/

neQ neAnimatedBody::GetRotationQ() {
    CAST_THIS(neCollisionBody_, cb);

    neQ q;

    q.SetupFromMatrix3(cb.b2w.rot);

    return q;
}

/****************************************************************************
*
*	neAnimatedBody::SetRotation
*
****************************************************************************/

void neAnimatedBody::SetRotation(const neM3 &m) {
    CAST_THIS(neCollisionBody_, cb);

    cb.b2w.rot = m;

    cb.moved = true;
}

/****************************************************************************
*
*	neAnimatedBody::SetRotation
*
****************************************************************************/

void neAnimatedBody::SetRotation(const neQ &q) {
    CAST_THIS(neCollisionBody_, cb);

    cb.b2w.rot = q.BuildMatrix3();

    cb.moved = true;
}

/****************************************************************************
*
*	neAnimatedBody::GetTransform
*
****************************************************************************/

neT3 neAnimatedBody::GetTransform() {
    CAST_THIS(neCollisionBody_, cb);

    return cb.b2w;
}

/****************************************************************************
*
*	neAnimatedBody::SetCollisionID
*
****************************************************************************/

void neAnimatedBody::SetCollisionID(s32 cid) {
    CAST_THIS(neCollisionBody_, cb);

    cb.cid = cid;
}

/****************************************************************************
*
*	neAnimatedBody::GetCollisionID
*
****************************************************************************/

s32 neAnimatedBody::GetCollisionID() {
    CAST_THIS(neCollisionBody_, cb);

    return cb.cid;
}

/****************************************************************************
*
*	neAnimatedBody::SetUserData
*
****************************************************************************/

void neAnimatedBody::SetUserData(void * cookies) {
    CAST_THIS(neCollisionBody_, cb);

    cb.cookies = cookies;
}

/****************************************************************************
*
*	neAnimatedBody::GetUserData
*
****************************************************************************/

void * neAnimatedBody::GetUserData() {
    CAST_THIS(neCollisionBody_, cb);

    return cb.cookies;
}

/****************************************************************************
*
*	neAnimatedBody::GetGeometryCount
*
****************************************************************************/

s32 neAnimatedBody::GetGeometryCount() {
    CAST_THIS(neCollisionBody_, cb);

    return cb.col.convexCount;
}

/****************************************************************************
*
*	neAnimatedBody::GetGeometry
*
****************************************************************************/
/*
neGeometry * neAnimatedBody::GetGeometry(s32 index)
{
	CAST_THIS(neCollisionBody_, cb);

	return reinterpret_cast<neGeometry*>(cb.GetConvex(index));
}
*/
/****************************************************************************
*
*	neAnimatedBody::SetGeometry
*
****************************************************************************/
/*
void neAnimatedBody::SetGeometry(s32 geometryCount, neGeometry * geometryArray)
{
	CAST_THIS(neCollisionBody_, cb);

	//todo
}
*/
/****************************************************************************
*
*	neAnimatedBody::UpdateBoundingInfo
*
****************************************************************************/

void neAnimatedBody::UpdateBoundingInfo() {
    CAST_THIS(neRigidBodyBase, rb);

    rb.RecalcBB();
}

/****************************************************************************
*
*	neAnimatedBody::CollideConnected
*
****************************************************************************/

void neAnimatedBody::CollideConnected(bool yes) {
    CAST_THIS(neRigidBodyBase, rb);

    rb.CollideConnected(yes);
}

/****************************************************************************
*
*	neAnimatedBody::IsCollideConnected
*
****************************************************************************/

bool neAnimatedBody::CollideConnected() {
    CAST_THIS(neRigidBodyBase, rb);

    return rb.CollideConnected();
}

/****************************************************************************
*
*	neAnimatedBody::CollideDirectlyConnected
*
****************************************************************************/

void neAnimatedBody::CollideDirectlyConnected(bool yes) {
    CAST_THIS(neRigidBodyBase, rb);

    rb.isCollideDirectlyConnected = yes;
}

/****************************************************************************
*
*	neAnimatedBody::CollideDirectlyConnected
*
****************************************************************************/

bool neAnimatedBody::CollideDirectlyConnected() {
    CAST_THIS(neRigidBodyBase, rb);

    return rb.isCollideDirectlyConnected;
}

/****************************************************************************
*
*	neAnimatedBody::AddGeometry
*
****************************************************************************/

neGeometry *neAnimatedBody::AddGeometry() {
    CAST_THIS(neCollisionBody_, ab);

    TConvex *g = ab.AddGeometry();

    return reinterpret_cast<neGeometry *>(g);
}

/****************************************************************************
*
*	neAnimatedBody::RemoveGeometry
*
****************************************************************************/

bool neAnimatedBody::RemoveGeometry(neGeometry *g) {
    CAST_THIS(neCollisionBody_, ab);

    if (!ab.col.convex)
        return false;

    TConvexItem *gi = (TConvexItem *) ab.col.convex;

    while (gi) {
        TConvex *convex = reinterpret_cast<TConvex *>(gi);

        gi = gi->next;

        if (convex == reinterpret_cast<TConvex *>(g)) {
            if (ab.col.convex == convex) {
                ab.col.convex = (TConvex *) gi;
            }

            ab.sim->geometryHeap.Dealloc(convex, 1);

            ab.col.convexCount--;

            if (ab.col.convexCount == 0) {
                ab.col.convex = nullptr;

                if (ab.IsInRegion() && !ab.isCustomCD)
                    ab.sim->region.RemoveBody(&ab);
            }

            return true;
        }
    }
    return false;
}

/****************************************************************************
*
*	neAnimatedBody::BeginIterateGeometry
*
****************************************************************************/

void neAnimatedBody::BeginIterateGeometry() {
    CAST_THIS(neCollisionBody_, ab);

    ab.BeginIterateGeometry();
}

/****************************************************************************
*
*	neAnimatedBody::GetNextGeometry
*
****************************************************************************/

neGeometry *neAnimatedBody::GetNextGeometry() {
    CAST_THIS(neCollisionBody_, ab);

    return reinterpret_cast<neGeometry *>(ab.GetNextGeometry());
}

/****************************************************************************
*
*	neAnimatedBody::BreakGeometry
*
****************************************************************************/

neRigidBody *neAnimatedBody::BreakGeometry(neGeometry *g) {
    CAST_THIS(neCollisionBody_, ab);

    neRigidBody_ *newBody = ab.sim->CreateRigidBodyFromConvex((TConvex *) g, &ab);

    return (neRigidBody *) newBody;
}

/****************************************************************************
*
*	neAnimatedBody::UseCustomCollisionDetection
*
****************************************************************************/

void neAnimatedBody::UseCustomCollisionDetection(bool yes, const neT3 *obb, neReal boundingRadius) {
    CAST_THIS(neCollisionBody_, ab);

    if (yes) {
        ab.obb = *obb;

        ab.col.boundingRadius = boundingRadius;

        ab.isCustomCD = yes;

        if (ab.isActive && !ab.IsInRegion()) {
            ab.sim->region.AddBody(&ab, nullptr);
        }
    } else {
        ab.isCustomCD = yes;

        this->UpdateBoundingInfo();

        if (ab.IsInRegion() && GetGeometryCount() == 0) {
            ab.sim->region.RemoveBody(&ab);
        }
    }
}

/****************************************************************************
*
*	neAnimatedBody::UseCustomCollisionDetection
*
****************************************************************************/

bool neAnimatedBody::UseCustomCollisionDetection() {
    CAST_THIS(neCollisionBody_, ab);

    return ab.isCustomCD;
}

/****************************************************************************
*
*	neAnimatedBody::AddSensor
*
****************************************************************************/

neSensor *neAnimatedBody::AddSensor() {
    CAST_THIS(neRigidBodyBase, ab);

    neSensor_ *s = ab.AddSensor();

    return reinterpret_cast<neSensor *>(s);
}

/****************************************************************************
*
*	neAnimatedBody::RemoveSensor
*
****************************************************************************/

bool neAnimatedBody::RemoveSensor(neSensor *s) {
    CAST_THIS(neRigidBodyBase, ab);

    if (!ab.sensors)
        return false;

    neSensorItem *si = (neSensorItem *) ab.sensors;

    while (si) {
        neSensor_ *sensor = (neSensor_ *) si;

        si = si->next;

        if (sensor == reinterpret_cast<neSensor_ *>(s)) {
            //reinterpret_cast<neSensorItem *>(s)->Remove();

            ab.sim->sensorHeap.Dealloc(sensor, 1);

            return true;
        }
    }
    return false;
}

/****************************************************************************
*
*	neAnimatedBody::BeginIterateSensor
*
****************************************************************************/

void neAnimatedBody::BeginIterateSensor() {
    CAST_THIS(neRigidBodyBase, ab);

    ab.BeginIterateSensor();
}

/****************************************************************************
*
*	neAnimatedBody::GetNextSensor
*
****************************************************************************/

neSensor *neAnimatedBody::GetNextSensor() {
    CAST_THIS(neRigidBodyBase, ab);

    return reinterpret_cast<neSensor *>(ab.GetNextSensor());
}

/****************************************************************************
*
*	neAnimatedBody::Active
*
****************************************************************************/

void neAnimatedBody::Active(bool yes, neRigidBody *hint) {
    CAST_THIS(neRigidBodyBase, ab);

    ab.Active(yes, (neRigidBodyBase *) hint);
}

/****************************************************************************
*
*	neAnimatedBody::Active
*
****************************************************************************/

void neAnimatedBody::Active(bool yes, neAnimatedBody *hint) {
    CAST_THIS(neRigidBodyBase, ab);

    ab.Active(yes, (neRigidBodyBase *) hint);
}

/****************************************************************************
*
*	neAnimatedBody::IsActive
*
****************************************************************************/

bool neAnimatedBody::Active() {
    CAST_THIS(neRigidBodyBase, ab);

    return ab.isActive;
}

/****************************************************************************
*
*	neRigidBody::GetMass
*
****************************************************************************/

//neReal neRigidBody::GetMass() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.mass;
//}

/****************************************************************************
*
*	neRigidBody::SetMass
*
****************************************************************************/

//void neRigidBody::SetMass(neReal mass) {
//    CAST_THIS(neRigidBody_, rb);
//
//    ASSERT(neIsFinite(mass));
//
//    rb.mass = mass;
//
//    rb.oneOnMass = 1.0f / mass;
//}

/****************************************************************************
*
*	neRigidBody::SetInertiaTensor
*
****************************************************************************/

//void neRigidBody::SetInertiaTensor(const neM3 &tensor) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.Ibody = tensor;
//
//    rb.IbodyInv.SetInvert(tensor);
//
//    //ASSERT(tensor.Invert(rb.IbodyInv));
//}

/****************************************************************************
*
*	neRigidBody::SetInertiaTensor
*
****************************************************************************/

//void neRigidBody::SetInertiaTensor(const neV3 &tensor) {
//    CAST_THIS(neRigidBody_, rb);
//
//    neM3 i;
//
//    i.SetIdentity();
//
//    i[0][0] = tensor[0];
//    i[1][1] = tensor[1];
//    i[2][2] = tensor[2];
//
//    rb.Ibody = i;
//
//    rb.IbodyInv.SetInvert(rb.Ibody);
//}

/****************************************************************************
*
*	neRigidBody::SetCollisionID
*
****************************************************************************/

//void neRigidBody::SetCollisionID(s32 cid) {
//    CAST_THIS(neRigidBodyBase, rb);
//
//    rb.cid = cid;
//}

/****************************************************************************
*
*	neRigidBody::GetCollisionID
*
****************************************************************************/

//s32 neRigidBody::GetCollisionID() {
//    CAST_THIS(neRigidBodyBase, rb);
//
//    return rb.cid;
//}

/****************************************************************************
*
*	neRigidBody::SetUserData
*
****************************************************************************/

//void neRigidBody::SetUserData(void * cookies) {
//    CAST_THIS(neRigidBodyBase, rb);
//
//    rb.cookies = cookies;
//}

/****************************************************************************
*
*	neRigidBody::GetUserData
*
****************************************************************************/

//void * neRigidBody::GetUserData() {
//    CAST_THIS(neRigidBodyBase, rb);
//
//    return rb.cookies;
//}

/****************************************************************************
*
*	neRigidBody::GetGeometryCount
*
****************************************************************************/

//s32 neRigidBody::GetGeometryCount() {
//    CAST_THIS(neRigidBodyBase, rb);
//
//    return rb.col.convexCount;
//}

//void neRigidBody::SetLinearDamping(neReal damp) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.linearDamp = neAbs(damp);
//}

//neReal neRigidBody::GetLinearDamping() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.linearDamp;
//}
//
//void neRigidBody::SetAngularDamping(neReal damp) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.angularDamp = neAbs(damp);
//}
//
//neReal neRigidBody::GetAngularDamping() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.angularDamp;
//}
//
//void neRigidBody::SetSleepingParameter(neReal sleepingParam) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.sleepingParam = sleepingParam;
//}
//
//neReal neRigidBody::GetSleepingParameter() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.sleepingParam;
//}


/****************************************************************************
*
*	neRigidBody::GetGeometry
*
****************************************************************************/
/*
neGeometry * neRigidBody::GetGeometry(s32 index)
{
	CAST_THIS(neRigidBodyBase, rb);

	return reinterpret_cast<neGeometry*>(rb.GetConvex(index));
}
*/
/****************************************************************************
*
*	neRigidBody::SetGeometry
*
****************************************************************************/
/*
void neRigidBody::SetGeometry(s32 geometryCount, neGeometry * geometryArray)
{
	//todo
}
*/
/****************************************************************************
*
*	neRigidBody::GetPos
*
****************************************************************************/

//neV3 neRigidBody::GetPos() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.GetPos();
//}

/****************************************************************************
*
*	neRigidBody::SetPos
*
****************************************************************************/

//void neRigidBody::SetPos(const neV3 &p) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.SetPos(p);
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::GetRotationM3
*
****************************************************************************/

//neM3 neRigidBody::GetRotationM3() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.State().rot();
//}

/****************************************************************************
*
*	neRigidBody::GetRotationQ
*
****************************************************************************/

//neQ neRigidBody::GetRotationQ() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.State().q;
//}

/****************************************************************************
*
*	neRigidBody::SetRotation
*
****************************************************************************/

//void neRigidBody::SetRotation(const neM3 &m) {
//    ASSERT(m.IsOrthogonalNormal());
//
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.State().rot() = m;
//
//    rb.State().q.SetupFromMatrix3(m);
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::SetRotation
*
****************************************************************************/

//void neRigidBody::SetRotation(const neQ &q) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.State().q = q;
//
//    rb.State().rot() = q.BuildMatrix3();
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::GetTransform
*
****************************************************************************/

//neT3 neRigidBody::GetTransform() {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.State().b2w.rot[0].v[3] = 0.0f;
//    rb.State().b2w.rot[1].v[3] = 0.0f;
//    rb.State().b2w.rot[2].v[3] = 0.0f;
//    rb.State().b2w.pos.v[3] = 1.0f;
//    return rb.State().b2w;
//}

/****************************************************************************
*
*	neRigidBody::GetVelocity
*
****************************************************************************/

//neV3 neRigidBody::GetVelocity() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.Derive().linearVel;
//}

/****************************************************************************
*
*	neRigidBody::SetVelocity
*
****************************************************************************/

//void neRigidBody::SetVelocity(const neV3 &v) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.Derive().linearVel = v;
//
//    rb.WakeUpAllJoint();
//}

/****************************************************************************
*
*	neRigidBody::GetAngularVelocity
*
****************************************************************************/

//neV3 neRigidBody::GetAngularVelocity() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.Derive().angularVel;
//}

/****************************************************************************
*
*	neRigidBody::GetAngularMomentum
*
****************************************************************************/


//neV3 neRigidBody::GetAngularMomentum() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.State().angularMom;
//}

/****************************************************************************
*
*	neRigidBody::SetAngularMomemtum
*
****************************************************************************/

//void neRigidBody::SetAngularMomentum(const neV3 &am) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.SetAngMom(am);
//
//    rb.WakeUpAllJoint();
//}

/****************************************************************************
*
*	neRigidBody::GetVelocityAtPoint
*
****************************************************************************/

//neV3 neRigidBody::GetVelocityAtPoint(const neV3 &pt) {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.VelocityAtPoint(pt);
//}

/****************************************************************************
*
*	neRigidBody::UpdateBoundingInfo
*
****************************************************************************/

//void neRigidBody::UpdateBoundingInfo() {
//    CAST_THIS(neRigidBodyBase, rb);
//
//    rb.RecalcBB();
//}

/****************************************************************************
*
*	neRigidBody::UpdateInertiaTensor
*
****************************************************************************/

//void neRigidBody::UpdateInertiaTensor() {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.RecalcInertiaTensor();
//}

/****************************************************************************
*
*	neRigidBody::SetForce
*
****************************************************************************/

//void neRigidBody::SetForce(const neV3 &force, const neV3 &pos) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (force.IsConsiderZero()) {
//        rb.force = force;
//
//        rb.torque = ((pos - rb.GetPos()).Cross(force));
//
//        return;
//    }
//
//    rb.force = force;
//
//    rb.torque = ((pos - rb.GetPos()).Cross(force));
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::SetForce
*
****************************************************************************/

//void neRigidBody::SetTorque(const neV3 &torque) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (torque.IsConsiderZero()) {
//        rb.torque = torque;
//
//        return;
//    }
//    rb.torque = torque;
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::ApplyForceCOG
*
****************************************************************************/

//void neRigidBody::SetForce(const neV3 &force) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (force.IsConsiderZero()) {
//        rb.force = force;
//
//        return;
//    }
//    rb.force = force;
//
//    rb.WakeUp();
//}

//neV3 neRigidBody::GetForce() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.force;
//}
//
//neV3 neRigidBody::GetTorque() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.torque;
//}

/****************************************************************************
*
*	neRigidBody::AddImpulse
*
****************************************************************************/

//void neRigidBody::ApplyImpulse(const neV3 &impulse) {
//    CAST_THIS(neRigidBody_, rb);
//
//    neV3 dv = impulse * rb.oneOnMass;
//
//    rb.Derive().linearVel += dv;
//
//    //rb.WakeUp();
//    rb.WakeUpAllJoint();
//}

/****************************************************************************
*
*	neRigidBody::AddImpulseWithTwist
*
****************************************************************************/

//void neRigidBody::ApplyImpulse(const neV3 &impulse, const neV3 &pos) {
//    CAST_THIS(neRigidBody_, rb);
//
//    neV3 dv = impulse * rb.oneOnMass;
//
//    neV3 da = (pos - rb.GetPos()).Cross(impulse);
//
//    rb.Derive().linearVel += dv;
//
//    neV3 newAM = rb.State().angularMom + da;
//
//    rb.SetAngMom(newAM);
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::ApplyTwist
*
****************************************************************************/

//void neRigidBody::ApplyTwist(const neV3 &twist) {
//    CAST_THIS(neRigidBody_, rb);
//
//    neV3 newAM = twist;
//
//    rb.SetAngMom(newAM);
//
//    rb.WakeUp();
//}

/****************************************************************************
*
*	neRigidBody::AddController
*
****************************************************************************/

//neRigidBodyController *neRigidBody::AddController(neRigidBodyControllerCallback *controller, s32 period) {
//    CAST_THIS(neRigidBody_, rb);
//
//    return (neRigidBodyController *) rb.AddController(controller, period);
//}

/****************************************************************************
*
*	neRigidBody::RemoveController
*
****************************************************************************/

//bool neRigidBody::RemoveController(neRigidBodyController *rbController) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (!rb.controllers)
//        return false;
//
//    neControllerItem *ci = (neControllerItem *) rb.controllers;
//
//    while (ci) {
//        neController *con = reinterpret_cast<neController *>(ci);
//
//        ci = ci->next;
//
//        if (con == reinterpret_cast<neController *>(rbController)) {
//            //reinterpret_cast<neControllerItem *>(con)->Remove();
//
//            rb.sim->controllerHeap.Dealloc(con, 1);
//
//            return true;
//        }
//    }
//    return false;
//}

/****************************************************************************
*
*	neRigidBody::BeginIterateController
*
****************************************************************************/

//void neRigidBody::BeginIterateController() {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.BeginIterateController();
//}

/****************************************************************************
*
*	neRigidBody::GetNextController
*
****************************************************************************/

//neRigidBodyController *neRigidBody::GetNextController() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return (neRigidBodyController *) rb.GetNextController();
//}

/****************************************************************************
*
*	neRigidBody::GravityEnable
*
****************************************************************************/

//void neRigidBody::GravityEnable(bool yes) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.GravityEnable(yes);
//}

/****************************************************************************
*
*	neRigidBody::GravityEnable
*
****************************************************************************/

//bool neRigidBody::GravityEnable() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.gravityOn;
//}

/****************************************************************************
*
*	neRigidBody::CollideConnected
*
****************************************************************************/

//void neRigidBody::CollideConnected(bool yes) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.CollideConnected(yes);
//}

/****************************************************************************
*
*	neRigidBody::CollideConnected
*
****************************************************************************/

//bool neRigidBody::CollideConnected() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.CollideConnected();
//}

/****************************************************************************
*
*	neRigidBody::CollideDirectlyConnected
*
****************************************************************************/

//void neRigidBody::CollideDirectlyConnected(bool yes) {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.isCollideDirectlyConnected = yes;
//}

/****************************************************************************
*
*	neRigidBody::CollideConnected
*
****************************************************************************/

//bool neRigidBody::CollideDirectlyConnected() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.isCollideDirectlyConnected;
//}

/****************************************************************************
*
*	neRigidBody::AddGeometry
*
****************************************************************************/

//neGeometry *neRigidBody::AddGeometry() {
//    CAST_THIS(neRigidBody_, rb);
//
//    TConvex *g = rb.AddGeometry();
//
//    return reinterpret_cast<neGeometry *>(g);
//}

/****************************************************************************
*
*	neRigidBody::RemoveGeometry
*
****************************************************************************/

//bool neRigidBody::RemoveGeometry(neGeometry *g) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (!rb.col.convex)
//        return false;
//
//    TConvexItem *gi = (TConvexItem *) rb.col.convex;
//
//    while (gi) {
//        TConvex *convex = reinterpret_cast<TConvex *>(gi);
//
//        gi = gi->next;
//
//        if (convex == reinterpret_cast<TConvex *>(g)) {
//            if (rb.col.convex == convex) {
//                rb.col.convex = (TConvex *) gi;
//            }
//            rb.sim->geometryHeap.Dealloc(convex, 1);
//
//            rb.col.convexCount--;
//
//            if (rb.col.convexCount == 0) {
//                rb.col.convex = nullptr;
//
//                if (rb.IsInRegion() && !rb.isCustomCD)
//                    rb.sim->region.RemoveBody(&rb);
//            }
//            return true;
//        }
//    }
//    return false;
//}

/****************************************************************************
*
*	neRigidBody::BeginIterateGeometry
*
****************************************************************************/

//void neRigidBody::BeginIterateGeometry() {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.BeginIterateGeometry();
//}

/****************************************************************************
*
*	neRigidBody::GetNextGeometry
*
****************************************************************************/

//neGeometry *neRigidBody::GetNextGeometry() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return reinterpret_cast<neGeometry *>(rb.GetNextGeometry());
//}

/****************************************************************************
*
*	neRigidBody::BreakGeometry
*
****************************************************************************/

//neRigidBody *neRigidBody::BreakGeometry(neGeometry *g) {
//    CAST_THIS(neRigidBody_, rb);
//
//    neRigidBody_ *newBody = rb.sim->CreateRigidBodyFromConvex((TConvex *) g, &rb);
//
//    return (neRigidBody *) newBody;
//}

/****************************************************************************
*
*	neRigidBody::UseCustomCollisionDetection
*
****************************************************************************/

//void neRigidBody::UseCustomCollisionDetection(bool yes, const neT3 *obb, neReal boundingRadius) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (yes) {
//        rb.obb = *obb;
//
//        rb.col.boundingRadius = boundingRadius;
//
//        rb.isCustomCD = yes;
//
//        if (rb.isActive && !rb.IsInRegion()) {
//            rb.sim->region.AddBody(&rb, nullptr);
//        }
//    } else {
//        rb.isCustomCD = yes;
//
//        this->UpdateBoundingInfo();
//
//        if (rb.IsInRegion() && GetGeometryCount() == 0) {
//            rb.sim->region.RemoveBody(&rb);
//        }
//    }
//}

/****************************************************************************
*
*	neRigidBody::UseCustomCollisionDetection
*
****************************************************************************/

//bool neRigidBody::UseCustomCollisionDetection() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return rb.isCustomCD;
//}

/****************************************************************************
*
*	neRigidBody::AddSensor
*
****************************************************************************/

//neSensor *neRigidBody::AddSensor() {
//    CAST_THIS(neRigidBody_, rb);
//
//    neSensor_ *s = rb.AddSensor();
//
//    return reinterpret_cast<neSensor *>(s);
//}

/****************************************************************************
*
*	neRigidBody::RemoveSensor
*
****************************************************************************/

//bool neRigidBody::RemoveSensor(neSensor *s) {
//    CAST_THIS(neRigidBody_, rb);
//
//    if (!rb.sensors)
//        return false;
//
//    neSensorItem *si = (neSensorItem *) rb.sensors;
//
//    while (si) {
//        neSensor_ *sensor = reinterpret_cast<neSensor_ *>(si);
//
//        si = si->next;
//
//        if (sensor == reinterpret_cast<neSensor_ *>(s)) {
//            //reinterpret_cast<neSensorItem *>(s)->Remove();
//
//            rb.sim->sensorHeap.Dealloc(sensor, 1);
//
//            return true;
//        }
//    }
//    return false;
//}

/****************************************************************************
*
*	neRigidBody::BeginIterateSensor
*
****************************************************************************/

//void neRigidBody::BeginIterateSensor() {
//    CAST_THIS(neRigidBody_, rb);
//
//    rb.BeginIterateSensor();
//}

/****************************************************************************
*
*	neRigidBody::GetNextSensor
*
****************************************************************************/

//neSensor *neRigidBody::GetNextSensor() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return reinterpret_cast<neSensor *>(rb.GetNextSensor());
//}

/****************************************************************************
*
*	neRigidBody::Active
*
****************************************************************************/

//void neRigidBody::Active(bool yes, neRigidBody *hint) {
//    CAST_THIS(neRigidBodyBase, ab);
//
//    ab.Active(yes, (neRigidBodyBase *) hint);
//}

/****************************************************************************
*
*	neRigidBody::Active
*
****************************************************************************/

//void neRigidBody::Active(bool yes, neAnimatedBody *hint) {
//    CAST_THIS(neRigidBodyBase, ab);
//
//    ab.Active(yes, (neRigidBodyBase *) hint);
//}

/****************************************************************************
*
*	neAnimatedBody::IsActive
*
****************************************************************************/

//bool neRigidBody::Active() {
//    CAST_THIS(neRigidBodyBase, ab);
//
//    return ab.isActive;
//}
//
//bool neRigidBody::IsIdle() {
//    CAST_THIS(neRigidBody_, rb);
//
//    return (rb.status == neRigidBody_::NE_RBSTATUS_IDLE);
//}

/****************************************************************************
*
*	neSimulator::Gravity
*
****************************************************************************/

//neV3 neSimulator::Gravity() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.gravity;
//}

/****************************************************************************
*
*	neSimulator::Gravity
*
****************************************************************************/

//void neSimulator::Gravity(const neV3 &g) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.SetGravity(g);
///*
//	sim.gravity = g;
//
//	sim.gravityVector = g;
//
//	sim.gravityVector.Normalize();
//*/
//}

/****************************************************************************
*
*	neSimulator::CreateRigidBody
*
****************************************************************************/

//neRigidBody *neSimulator::CreateRigidBody() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    neRigidBody_ *ret = sim.CreateRigidBody();
//
//    return reinterpret_cast<neRigidBody *>(ret);
//}

/****************************************************************************
*
*	neSimulator::CreateRigidParticle
*
****************************************************************************/

//neRigidBody *neSimulator::CreateRigidParticle() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    neRigidBody_ *ret = sim.CreateRigidBody(true);
//
//    return reinterpret_cast<neRigidBody *>(ret);
//}

/****************************************************************************
*
*	neSimulator::CreateCollisionBody()
*
****************************************************************************/

//neAnimatedBody *neSimulator::CreateAnimatedBody() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    neCollisionBody_ *ret = sim.CreateCollisionBody();
//
//    return reinterpret_cast<neAnimatedBody *>(ret);
//}

/****************************************************************************
*
*	neSimulator::FreeBody
*
****************************************************************************/

//void neSimulator::FreeRigidBody(neRigidBody *body) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.Free(reinterpret_cast<neRigidBody_ *>(body));
//}

/****************************************************************************
*
*	neSimulator::FreeCollisionBody
*
****************************************************************************/

//void neSimulator::FreeAnimatedBody(neAnimatedBody *body) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.Free(reinterpret_cast<neRigidBody_ *>(body));
//}

/****************************************************************************
*
*	neSimulator::GetCollisionTable
*
****************************************************************************/

//neCollisionTable *neSimulator::GetCollisionTable() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return (neCollisionTable *) (&sim.colTable);
//}

/****************************************************************************
*
*	neSimulator::GetMaterial
*
****************************************************************************/

//bool neSimulator::SetMaterial(s32 index, neReal friction, neReal restitution) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.SetMaterial(index, friction, restitution, 0.0f);
//}

/****************************************************************************
*
*	neSimulator::GetMaterial
*
****************************************************************************/

//bool neSimulator::GetMaterial(s32 index, neReal &friction, neReal &restitution) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    neReal density;
//
//    return sim.GetMaterial(index, friction, restitution, density);
//}

/****************************************************************************
*
*	neSimulator::Advance
*
****************************************************************************/

//void neSimulator::Advance(neReal sec, s32 step, nePerformanceReport *perfReport) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.Advance(sec, step, perfReport);
//}
//
//void neSimulator::Advance(neReal sec, neReal minTimeStep, neReal maxTimeStep, nePerformanceReport *perfReport) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.Advance(sec, minTimeStep, maxTimeStep, perfReport);
//}
//
//void neSimulator::ForeachActiveRB(std::function<bool(neRigidBody&)> rbCallback) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//    sim.ForeachActiveRB(std::move(rbCallback));
//}

/****************************************************************************
*
*	neSimulator::SetTerrainMesh
*
****************************************************************************/
//
//void neSimulator::SetTerrainMesh(neTriangleMesh *tris) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.SetTerrainMesh(tris);
//}
//
//void neSimulator::FreeTerrainMesh() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.FreeTerrainMesh();
//
//}

/****************************************************************************
*
*	 neSimulator::CreateJoint
*
****************************************************************************/

//neJoint *neSimulator::CreateJoint(neRigidBody *bodyA) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    if (!bodyA)
//        return nullptr;
//
//    _neConstraint *constr = sim.constraintHeap.Alloc(1); // 1 means make it solo
//
//    if (!constr) {
//        sprintf(sim.logBuffer, MSG_CONSTRAINT_FULL);
//
//        sim.LogOutput(neSimulator::LOG_OUTPUT_LEVEL_ONE);
//
//        return nullptr;
//    }
//
//    constr->Reset();
//
//    constr->sim = &sim;
//
//    constr->bodyA = (neRigidBody_ *) bodyA;
//
//    neRigidBody_ *ba = (neRigidBody_ *) bodyA;
//
//    ba->constraintCollection.Add(&constr->bodyAHandle);
//
//    return reinterpret_cast<neJoint *>(constr);
//}

/****************************************************************************
*
*	 neSimulator::CreateJoint
*
****************************************************************************/

//neJoint *neSimulator::CreateJoint(neRigidBody *bodyA, neRigidBody *bodyB) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    if (!bodyA)
//        return nullptr;
//
//    if (!bodyB)
//        return nullptr;
//
//    _neConstraint *constr = sim.constraintHeap.Alloc(1); // 1 means make it solo
//
//    if (!constr) {
//        sprintf(sim.logBuffer, MSG_CONSTRAINT_FULL);
//
//        sim.LogOutput(neSimulator::LOG_OUTPUT_LEVEL_ONE);
//
//        return nullptr;
//    }
//
//    constr->Reset();
//
//    constr->sim = &sim;
//
//    constr->bodyA = (neRigidBody_ *) bodyA;
//
//    neRigidBody_ *ba = (neRigidBody_ *) bodyA;
//
//    ba->constraintCollection.Add(&constr->bodyAHandle);
//
//    constr->bodyB = (neRigidBodyBase *) bodyB;
//
//    neRigidBody_ *bb = (neRigidBody_ *) bodyB;
//
//    bb->constraintCollection.Add(&constr->bodyBHandle);
//
//    return reinterpret_cast<neJoint *>(constr);
//}

/****************************************************************************
*
*	 neSimulator::CreateJoint
*
****************************************************************************/

//neJoint *neSimulator::CreateJoint(neRigidBody *bodyA, neAnimatedBody *bodyB) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    if (!bodyA)
//        return nullptr;
//
//    if (!bodyB)
//        return nullptr;
//
//    _neConstraint *constr = sim.constraintHeap.Alloc(1); // 1 means make it solo
//
//    if (!constr) {
//        sprintf(sim.logBuffer, MSG_CONSTRAINT_FULL);
//
//        sim.LogOutput(neSimulator::LOG_OUTPUT_LEVEL_ONE);
//
//        return nullptr;
//    }
//
//    constr->Reset();
//
//    constr->sim = &sim;
//
//    constr->bodyA = (neRigidBody_ *) bodyA;
//
//    neRigidBody_ *ba = (neRigidBody_ *) bodyA;
//
//    ba->constraintCollection.Add(&constr->bodyAHandle);
//
//    constr->bodyB = (neRigidBodyBase *) bodyB;
//
//    neRigidBodyBase *bb = (neRigidBodyBase *) bodyB;
//
//    bb->constraintCollection.Add(&constr->bodyBHandle);
//
//    return reinterpret_cast<neJoint *>(constr);
//}

/****************************************************************************
*
*	neSimulator::FreeJoint
*
****************************************************************************/

//void neSimulator::FreeJoint(neJoint *constraint) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    _neConstraint *c = (_neConstraint *) constraint;
//
//    ASSERT(sim.constraintHeap.CheckBelongAndInUse(c));
//
//    if (c->bodyA) {
//        c->bodyA->constraintCollection.Remove(&c->bodyAHandle);
//
//        if (c->bodyB)
//            c->bodyB->constraintCollection.Remove(&c->bodyBHandle);
//
//        neConstraintHeader *h = c->bodyA->GetConstraintHeader();
//
//        if (h) {
//            h->Remove(c);
//
//            h->flag = neConstraintHeader::FLAG_NEED_REORG;
//        }
//        sim.constraintHeap.Dealloc(c, 1);
//
//        if (c->bodyA->constraintCollection.count == 0)
//            c->bodyA->RemoveConstraintHeader();
//
//        if (c->bodyB &&
//            c->bodyB->constraintCollection.count == 0)
//            c->bodyB->RemoveConstraintHeader();
//    } else {
//        sim.constraintHeap.Dealloc(c, 1);
//    }
//}

/****************************************************************************
*
*	neSimulator::SetCollisionCallback
*
****************************************************************************/

//void neSimulator::SetCollisionCallback(neCollisionCallback *fn) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.SetCollisionCallback(fn);
//}

/****************************************************************************
*
*	neSimulator::GetCollisionCallback
*
****************************************************************************/

//neCollisionCallback *neSimulator::GetCollisionCallback() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.collisionCallback;
//}


/****************************************************************************
*
*	neSimulator::SetBreakageCallback
*
****************************************************************************/

//void neSimulator::SetBreakageCallback(neBreakageCallback *cb) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.breakageCallback = cb;
//}

/****************************************************************************
*
*	neSimulator::GetBreakageCallback
*
****************************************************************************/

//neBreakageCallback *neSimulator::GetBreakageCallback() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.breakageCallback;
//}

/****************************************************************************
*
*	neSimulator::SetTerrainTriangleQueryCallback
*
****************************************************************************/

//void neSimulator::SetTerrainTriangleQueryCallback(neTerrainTriangleQueryCallback *cb) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.terrainQueryCallback = cb;
//}

/****************************************************************************
*
*	neSimulator::GetTerrainTriangleQueryCallback
*
****************************************************************************/

//neTerrainTriangleQueryCallback *neSimulator::GetTerrainTriangleQueryCallback() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.terrainQueryCallback;
//}

/****************************************************************************
*
*	neSimulator::SetCustomCDRB2RBCallback
*
****************************************************************************/

//void neSimulator::SetCustomCDRB2RBCallback(neCustomCDRB2RBCallback *cb) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.customCDRB2RBCallback = cb;
//}

/****************************************************************************
*
*	neSimulator::GetCustomCDRB2RBCallback
*
****************************************************************************/

//neCustomCDRB2RBCallback *neSimulator::GetCustomCDRB2RBCallback() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.customCDRB2RBCallback;
//}

/****************************************************************************
*
*	neSimulator::SetCustomCDRB2ABCallback
*
****************************************************************************/

//void neSimulator::SetCustomCDRB2ABCallback(neCustomCDRB2ABCallback *cb) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.customCDRB2ABCallback = cb;
//}

/****************************************************************************
*
*	neSimulator::GetCustomCDRB2ABCallback
*
****************************************************************************/

//neCustomCDRB2ABCallback *neSimulator::GetCustomCDRB2ABCallback() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.customCDRB2ABCallback;
//}

/****************************************************************************
*
*	neSimulator::SetLogOutputCallback
*
****************************************************************************/

//void neSimulator::SetLogOutputCallback(neLogOutputCallback *fn) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.SetLogOutputCallback(fn);
//}
/*
neReal neSimulator::GetMagicNumber()
{
	CAST_THIS(neSimulator, sim);

	return sim.magicNumber;
}
*/
/****************************************************************************
*
*	neSimulator::GetLogOutputCallback
*
****************************************************************************/

//neLogOutputCallback *neSimulator::GetLogOutputCallback() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.logCallback;
//}

/****************************************************************************
*
*	neSimulator::SetLogOutputLevel
*
****************************************************************************/

//void neSimulator::SetLogOutputLevel(LOG_OUTPUT_LEVEL lvl) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.SetLogOutputLevel(lvl);
//}

/****************************************************************************
*
*	neSimulator::GetCurrentSizeInfo
*
****************************************************************************/

//neSimulatorSizeInfo neSimulator::GetCurrentSizeInfo() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    neSimulatorSizeInfo ret;
//
//    ret.rigidBodiesCount = sim.rigidBodyHeap.GetUsedCount();
//    ret.animatedBodiesCount = sim.collisionBodyHeap.GetUsedCount();
//    ret.rigidParticleCount = sim.rigidParticleHeap.GetUsedCount();
//    ret.controllersCount = sim.controllerHeap.GetUsedCount();
//    ret.overlappedPairsCount = sim.region.overlappedPairs.GetUsedCount();
//    ret.geometriesCount = sim.geometryHeap.GetUsedCount();
//
//    ret.constraintsCount = sim.constraintHeap.GetUsedCount();
//    ret.constraintSetsCount = sim.constraintHeaders.GetUsedCount();
////	ret.constraintBufferSize = sim.miniConstraintHeap.GetUsedCount();
//    ret.sensorsCount = sim.sensorHeap.GetUsedCount();
//
//    ret.terrainNodesStartCount = sim.region.terrainTree.nodes.GetUsedCount();
//    ret.terrainNodesGrowByCount = sim.sizeInfo.terrainNodesGrowByCount;
//
//    return ret;
//}

/****************************************************************************
*
*	neSimulator::GetStartSizeInfo
*
****************************************************************************/

//neSimulatorSizeInfo neSimulator::GetStartSizeInfo() {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    return sim.sizeInfo;
//}

/****************************************************************************
*
*	neSimulator::GetMemoryUsage
*
****************************************************************************/

//void neSimulator::GetMemoryAllocated(s32 &memoryAllocated) {
//    CAST_THIS(neFixedTimeStepSimulator, sim);
//
//    sim.GetMemoryAllocated(memoryAllocated);
//}

/****************************************************************************
*
*	neJoint::SetType
*
****************************************************************************/

void neJoint::SetType(ConstraintType t) {
    CAST_THIS(_neConstraint, c);

    c.SetType(t);
}

/****************************************************************************
*
*	neJoint::GetType
*
****************************************************************************/

neJoint::ConstraintType neJoint::GetType() {
    CAST_THIS(_neConstraint, c);

    return c.type;
}

/****************************************************************************
*
*	neJoint::GetRigidBodyA
*
****************************************************************************/

neRigidBody *neJoint::GetRigidBodyA() {
    CAST_THIS(_neConstraint, c);

    return reinterpret_cast<neRigidBody *>(c.bodyA);
}

/****************************************************************************
*
*	neJoint::GetRigidBodyB
*
****************************************************************************/

neRigidBody *neJoint::GetRigidBodyB() {
    CAST_THIS(_neConstraint, c);

    if (!c.bodyB)
        return nullptr;

    if (c.bodyB->AsCollisionBody())
        return nullptr;

    return reinterpret_cast<neRigidBody *>(c.bodyB);
}

/****************************************************************************
*
*	neJoint::GetAnimatedBodyB
*
****************************************************************************/

neAnimatedBody *neJoint::GetAnimatedBodyB() {
    CAST_THIS(_neConstraint, c);

    if (!c.bodyB)
        return nullptr;

    if (c.bodyB->AsRigidBody())
        return nullptr;

    return reinterpret_cast<neAnimatedBody *>(c.bodyB);
}

/****************************************************************************
*
*	neJoint::SetJointFrameA
*
****************************************************************************/

void neJoint::SetJointFrameA(const neT3 &frameA) {
    CAST_THIS(_neConstraint, c);

    c.frameA = frameA;
}

/****************************************************************************
*
*	neJoint::SetJointFrameB
*
****************************************************************************/

void neJoint::SetJointFrameB(const neT3 &frameB) {
    CAST_THIS(_neConstraint, c);

    c.frameB = frameB;
}

void neJoint::SetJointFrameWorld(const neT3 &frame) {
    CAST_THIS(_neConstraint, c);

    neT3 w2b;

    w2b = c.bodyA->GetB2W().FastInverse();

    c.frameA = w2b * frame;

    if (!c.bodyB) {
        c.frameB = frame;

        return;
    }
    w2b = c.bodyB->GetB2W().FastInverse();

    c.frameB = w2b * frame;
}

/****************************************************************************
*
*	neJoint::GetJointFrameA
*
****************************************************************************/

neT3 neJoint::GetJointFrameA() {
    CAST_THIS(_neConstraint, c);

    if (!c.bodyA) {
        return c.frameA;
    }
    neT3 ret;

    ret = c.bodyA->State().b2w * c.frameA;

    return ret;
}

/****************************************************************************
*
*	neJoint::GetJointFrameB
*
****************************************************************************/

neT3 neJoint::GetJointFrameB() {
    CAST_THIS(_neConstraint, c);

    if (!c.bodyB) {
        return c.frameB;
    }
    neT3 ret;

    neCollisionBody_ *cb = c.bodyB->AsCollisionBody();

    if (cb) {
        ret = cb->b2w * c.frameB;
    } else {
        neRigidBody_ *rb = c.bodyB->AsRigidBody();

        ret = rb->State().b2w * c.frameB;
    }
    return ret;
}

/****************************************************************************
*
*	neJoint::SetJointLength
*
****************************************************************************/

void neJoint::SetJointLength(neReal length) {
    CAST_THIS(_neConstraint, c);

    c.jointLength = length;
}

/****************************************************************************
*
*	neJoint::GetJointLength
*
****************************************************************************/

neReal neJoint::GetJointLength() {
    CAST_THIS(_neConstraint, c);

    return c.jointLength;
}

/****************************************************************************
*
*	neJoint::Enable
*
****************************************************************************/

void neJoint::Enable(bool yes) {
    CAST_THIS(_neConstraint, c);

    c.Enable(yes);
}

/****************************************************************************
*
*	neJoint::IsEnable
*
****************************************************************************/

bool neJoint::Enable() {
    CAST_THIS(_neConstraint, c);

    return c.enable;
}

/****************************************************************************
*
*	neJoint::InfiniteMassB
*
****************************************************************************/
/*
void neJoint::InfiniteMassB(bool yes)
{
	CAST_THIS(_neConstraint, c);

	c.InfiniteMassB(yes);
}
*/
/****************************************************************************
*
*	neJoint::SetDampingFactor
*
****************************************************************************/

void neJoint::SetDampingFactor(neReal damp) {
    CAST_THIS(_neConstraint, c);

    c.jointDampingFactor = damp;
}

/****************************************************************************
*
*	neJoint::GetDampingFactor
*
****************************************************************************/

neReal neJoint::GetDampingFactor() {
    CAST_THIS(_neConstraint, c);

    return c.jointDampingFactor;
}

/****************************************************************************
*
*	neJoint::SetEpsilon
*
****************************************************************************/

void neJoint::SetEpsilon(neReal t) {
    CAST_THIS(_neConstraint, c);

    c.accuracy = t;
}

/****************************************************************************
*
*	neJoint::GetEpsilon
*
****************************************************************************/

neReal neJoint::GetEpsilon() {
    CAST_THIS(_neConstraint, c);

    if (c.accuracy <= 0.0f)
        return DEFAULT_CONSTRAINT_EPSILON;

    return c.accuracy;
}

/****************************************************************************
*
*	neJoint::SetIteration2
*
****************************************************************************/

void neJoint::SetIteration(s32 i) {
    CAST_THIS(_neConstraint, c);

    c.iteration = i;
}

/****************************************************************************
*
*	neJoint::GetIteration2
*
****************************************************************************/

s32 neJoint::GetIteration() {
    CAST_THIS(_neConstraint, c);

    return c.iteration;
}

/****************************************************************************
*
*	neJoint::GetUpperLimit
*
****************************************************************************/

neReal neJoint::GetUpperLimit() {
    CAST_THIS(_neConstraint, c);

    return c.limitStates[0].upperLimit;
}

/****************************************************************************
*
*	neJoint::SetUpperLimit
*
****************************************************************************/

void neJoint::SetUpperLimit(neReal upperLimit) {
    CAST_THIS(_neConstraint, c);

    c.limitStates[0].upperLimit = upperLimit;
}

/****************************************************************************
*
*	neJoint::GetLowerLimit
*
****************************************************************************/

neReal neJoint::GetLowerLimit() {
    CAST_THIS(_neConstraint, c);

    return c.limitStates[0].lowerLimit;
}

/****************************************************************************
*
*	neJoint::SetLowerLimit
*
****************************************************************************/

void neJoint::SetLowerLimit(neReal lowerLimit) {
    CAST_THIS(_neConstraint, c);

    c.limitStates[0].lowerLimit = lowerLimit;
}

/****************************************************************************
*
*	neJoint::IsEnableLimit
*
****************************************************************************/

bool neJoint::EnableLimit() {
    CAST_THIS(_neConstraint, c);

    return c.limitStates[0].enableLimit;
}

/****************************************************************************
*
*	neJoint::EnableLimite
*
****************************************************************************/

void neJoint::EnableLimit(bool yes) {
    CAST_THIS(_neConstraint, c);

    c.limitStates[0].enableLimit = yes;
}

/****************************************************************************
*
*	neJoint::GetUpperLimit2
*
****************************************************************************/

neReal neJoint::GetUpperLimit2() {
    CAST_THIS(_neConstraint, c);

    return c.limitStates[1].upperLimit;
}

/****************************************************************************
*
*	neJoint::SetUpperLimit2
*
****************************************************************************/

void neJoint::SetUpperLimit2(neReal upperLimit) {
    CAST_THIS(_neConstraint, c);

    c.limitStates[1].upperLimit = upperLimit;
}

/****************************************************************************
*
*	neJoint::GetLowerLimit2
*
****************************************************************************/

neReal neJoint::GetLowerLimit2() {
    CAST_THIS(_neConstraint, c);

    return c.limitStates[1].lowerLimit;
}

/****************************************************************************
*
*	neJoint::SetLowerLimit2
*
****************************************************************************/

void neJoint::SetLowerLimit2(neReal lowerLimit) {
    CAST_THIS(_neConstraint, c);

    c.limitStates[1].lowerLimit = lowerLimit;
}

/****************************************************************************
*
*	neJoint::IsEnableLimit2
*
****************************************************************************/

bool neJoint::EnableLimit2() {
    CAST_THIS(_neConstraint, c);

    return c.limitStates[1].enableLimit;
}

/****************************************************************************
*
*	neJoint::EnableMotor
*
****************************************************************************/

bool neJoint::EnableMotor() {
    CAST_THIS(_neConstraint, c);

    return c.motors[0].enable;
}

/****************************************************************************
*
*	neJoint::EnableMotor
*
****************************************************************************/

void neJoint::EnableMotor(bool yes) {
    CAST_THIS(_neConstraint, c);

    c.motors[0].enable = yes;
}

/****************************************************************************
*
*	neJoint::SetMotor
*
****************************************************************************/

void neJoint::SetMotor(MotorType motorType, neReal desireValue, neReal maxForce) {
    CAST_THIS(_neConstraint, c);

    c.motors[0].motorType = motorType;

    c.motors[0].desireVelocity = desireValue;

    c.motors[0].maxForce = neAbs(maxForce);
}

/****************************************************************************
*
*	neJoint::GetMotor
*
****************************************************************************/

void neJoint::GetMotor(MotorType &motorType, neReal &desireValue, neReal &maxForce) {
    CAST_THIS(_neConstraint, c);

    motorType = c.motors[0].motorType;

    desireValue = c.motors[0].desireVelocity;

    maxForce = c.motors[0].maxForce;
}

/****************************************************************************
*
*	neJoint::EnableMotor2
*
****************************************************************************/

bool neJoint::EnableMotor2() {
    CAST_THIS(_neConstraint, c);

    return c.motors[1].enable;
}

/****************************************************************************
*
*	neJoint::EnableMotor2
*
****************************************************************************/

void neJoint::EnableMotor2(bool yes) {
    CAST_THIS(_neConstraint, c);

    c.motors[1].enable = yes;
}

/****************************************************************************
*
*	neJoint::SetMotor2
*
****************************************************************************/

void neJoint::SetMotor2(MotorType motorType, neReal desireValue, neReal maxForce) {
    CAST_THIS(_neConstraint, c);

    c.motors[1].motorType = motorType;

    c.motors[1].desireVelocity = desireValue;

    c.motors[1].maxForce = neAbs(maxForce);
}

/****************************************************************************
*
*	neJoint::GetMotor2
*
****************************************************************************/

void neJoint::GetMotor2(MotorType &motorType, neReal &desireValue, neReal &maxForce) {
    CAST_THIS(_neConstraint, c);

    motorType = c.motors[1].motorType;

    desireValue = c.motors[1].desireVelocity;

    maxForce = c.motors[1].maxForce;
}

/****************************************************************************
*
*	neJoint::EnableLimite
*
****************************************************************************/

void neJoint::EnableLimit2(bool yes) {
    CAST_THIS(_neConstraint, c);

    c.limitStates[1].enableLimit = yes;
}


/****************************************************************************
*
*	neJoint::AddController
*
****************************************************************************/

neJointController *neJoint::AddController(neJointControllerCallback *controller, s32 period) {
    CAST_THIS(_neConstraint, c);

    return (neJointController *) c.AddController(controller, period);
}

/****************************************************************************
*
*	neJoint::RemoveController
*
****************************************************************************/

bool neJoint::RemoveController(neJointController *jController) {
    CAST_THIS(_neConstraint, c);

    if (!c.controllers)
        return false;

    neControllerItem *ci = (neControllerItem *) c.controllers;

    while (ci) {
        neController *con = reinterpret_cast<neController *>(ci);

        ci = ci->next;

        if (con == reinterpret_cast<neController *>(jController)) {
            //reinterpret_cast<neControllerItem *>(con)->Remove();

            c.sim->controllerHeap.Dealloc(con, 1);

            return true;
        }
    }
    return false;
}

/****************************************************************************
*
*	neJoint::BeginIterateController
*
****************************************************************************/

void neJoint::BeginIterateController() {
    CAST_THIS(_neConstraint, c);

    c.BeginIterateController();
}

/****************************************************************************
*
*	neJoint::GetNextController
*
****************************************************************************/

neJointController *neJoint::GetNextController() {
    CAST_THIS(_neConstraint, c);

    return (neJointController *) c.GetNextController();
}

/****************************************************************************
*
*	neJoint::SetBSPoints
*
****************************************************************************/
/*
bool neJoint::SetBSPoints(const neV3 & pointA, const neV3 & pointB)
{
	CAST_THIS(_neConstraint, c);

	if (c.type != NE_Constraint_BALLSOCKET)
		return false;

	c.cpointsA[0].PtBody() = pointA;

	c.cpointsB[0].PtBody() = pointB;

	return true;
}
*/
/****************************************************************************
*
*	neJoint::SetHingePoints
*
****************************************************************************/
/*
bool neJoint::SetHingePoints(const neV3 & pointA1, const neV3 & pointA2,
						const neV3 & pointB1, const neV3 & pointB2)
{
	CAST_THIS(_neConstraint, c);

	if (c.type != NE_Constraint_HINGE)
		return false;

	c.cpointsA[0].PtBody() = pointA1;

	c.cpointsA[1].PtBody() = pointA2;

	c.cpointsB[0].PtBody() = pointB1;

	c.cpointsB[1].PtBody() = pointB2;

	return true;
}
*/
/****************************************************************************
*
*	neSensor::SetLineSensor
*
****************************************************************************/

void neSensor::SetLineSensor(const neV3 &pos, const neV3 &lineVector) {
    CAST_THIS(neSensor_, sensor);

    sensor.pos = pos;

    sensor.dir = lineVector;

    sensor.dirNormal = lineVector;

    sensor.dirNormal.Normalize();

    sensor.length = lineVector.Length();
}

/****************************************************************************
*
*	neSensor::SetUserData
*
****************************************************************************/

void neSensor::SetUserData(void * cookies) {
    CAST_THIS(neSensor_, sensor);

    sensor.cookies = cookies;
}

/****************************************************************************
*
*	neSensor::GetUserData
*
****************************************************************************/

void * neSensor::GetUserData() {
    CAST_THIS(neSensor_, sensor);

    return sensor.cookies;
}

/****************************************************************************
*
*	neSensor::GetLineNormal
*
****************************************************************************/

neV3 neSensor::GetLineVector() {
    CAST_THIS(neSensor_, sensor);

    return sensor.dir;
}

/****************************************************************************
*
*	neSensor::GetLineNormal
*
****************************************************************************/

neV3 neSensor::GetLineUnitVector() {
    CAST_THIS(neSensor_, sensor);

    return sensor.dirNormal;
}

/****************************************************************************
*
*	neSensor::GetLinePos
*
****************************************************************************/

neV3 neSensor::GetLinePos() {
    CAST_THIS(neSensor_, sensor);

    return sensor.pos;
}

/****************************************************************************
*
*	neSensor::GetDetectDepth
*
****************************************************************************/

neReal neSensor::GetDetectDepth() {
    CAST_THIS(neSensor_, sensor);

    return sensor.depth;
}

/****************************************************************************
*
*	neSensor::GetDetectNormal
*
****************************************************************************/

neV3 neSensor::GetDetectNormal() {
    CAST_THIS(neSensor_, sensor);

    return sensor.normal;
}

/****************************************************************************
*
*	neSensor::GetDetectContactPoint
*
****************************************************************************/

neV3 neSensor::GetDetectContactPoint() {
    CAST_THIS(neSensor_, sensor);

    return sensor.contactPoint;
}

/****************************************************************************
*
*	neSensor::GetDetectRigidBody
*
****************************************************************************/

neRigidBody *neSensor::GetDetectRigidBody() {
    CAST_THIS(neSensor_, sensor);

    if (!sensor.body)
        return nullptr;

    if (sensor.body->AsCollisionBody())
        return nullptr;

    return (neRigidBody *) sensor.body;
}

/****************************************************************************
*
*	neSensor::GetDetectAnimatedBody
*
****************************************************************************/

neAnimatedBody *neSensor::GetDetectAnimatedBody() {
    CAST_THIS(neSensor_, sensor);

    if (!sensor.body)
        return nullptr;

    if (sensor.body->AsRigidBody())
        return nullptr;

    return (neAnimatedBody *) sensor.body;
}

/****************************************************************************
*
*	neSensor::
*
****************************************************************************/

s32 neSensor::GetDetectMaterial() {
    CAST_THIS(neSensor_, sensor);

    return sensor.materialID;
}

/****************************************************************************
*
*	neRigidBodyController::
*
****************************************************************************/

neRigidBody *neRigidBodyController::GetRigidBody() {
    CAST_THIS(neController, c);

    return (neRigidBody *) c.rb;
}

/****************************************************************************
*
*	neRigidBodyController::
*
****************************************************************************/

neV3 neRigidBodyController::GetControllerForce() {
    CAST_THIS(neController, c);

    return c.forceA;
}

/****************************************************************************
*
*	neRigidBodyController::
*
****************************************************************************/

neV3 neRigidBodyController::GetControllerTorque() {
    CAST_THIS(neController, c);

    return c.torqueA;
}

/****************************************************************************
*
*	neRigidBodyController::
*
****************************************************************************/

void neRigidBodyController::SetControllerForce(const neV3 &force) {
    CAST_THIS(neController, c);

    c.forceA = force;
}

/****************************************************************************
*
*	neRigidBodyController::
*
****************************************************************************/

void neRigidBodyController::SetControllerForceWithTorque(const neV3 &force, const neV3 &pos) {
    CAST_THIS(neController, c);

    c.forceA = force;

    c.torqueA = ((pos - c.rb->GetPos()).Cross(force));
}

/****************************************************************************
*
*	neRigidBodyController::
*
****************************************************************************/

void neRigidBodyController::SetControllerTorque(const neV3 &torque) {
    CAST_THIS(neController, c);

    c.torqueA = torque;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

neJoint *neJointController::GetJoint() {
    CAST_THIS(neController, c);

    return (neJoint *) c.constraint;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

neV3 neJointController::GetControllerForceBodyA() {
    CAST_THIS(neController, c);

    return c.forceA;
}


/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

neV3 neJointController::GetControllerForceBodyB() {
    CAST_THIS(neController, c);

    return c.forceB;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

neV3 neJointController::GetControllerTorqueBodyA() {
    CAST_THIS(neController, c);

    return c.torqueA;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

neV3 neJointController::GetControllerTorqueBodyB() {
    CAST_THIS(neController, c);

    return c.torqueB;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

void neJointController::SetControllerForceBodyA(const neV3 &force) {
    CAST_THIS(neController, c);

    c.forceA = force;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

void neJointController::SetControllerForceWithTorqueBodyA(const neV3 &force, const neV3 &pos) {
    CAST_THIS(neController, c);

    c.forceA = force;

    c.torqueA = ((pos - c.constraint->bodyA->GetPos()).Cross(force));
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

void neJointController::SetControllerForceWithTorqueBodyB(const neV3 &force, const neV3 &pos) {
    CAST_THIS(neController, c);

    c.forceB = force;

    if (c.constraint->bodyB &&
        !c.constraint->bodyB->AsCollisionBody()) {
        neRigidBody_ *rb = (neRigidBody_ *) c.constraint->bodyB;

        c.torqueB = ((pos - rb->GetPos()).Cross(force));
    }
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

void neJointController::SetControllerForceBodyB(const neV3 &force) {
    CAST_THIS(neController, c);

    c.forceB = force;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

void neJointController::SetControllerTorqueBodyA(const neV3 &torque) {
    CAST_THIS(neController, c);

    c.torqueA = torque;
}

/****************************************************************************
*
*	neJointController::
*
****************************************************************************/

void neJointController::SetControllerTorqueBodyB(const neV3 &torque) {
    CAST_THIS(neController, c);

    c.torqueB = torque;
}

/****************************************************************************
*
*	neCollisionTable::Set
*
****************************************************************************/

void neCollisionTable::Set(s32 collisionID1, s32 collisionID2, neReponseBitFlag response) {
    CAST_THIS(neCollisionTable_, ct);

    ct.Set(collisionID1, collisionID2, response);
}

/****************************************************************************
*
*	neCollisionTable::Get
*
****************************************************************************/

neCollisionTable::neReponseBitFlag neCollisionTable::Get(s32 collisionID1, s32 collisionID2) {
    CAST_THIS(neCollisionTable_, ct);

    ASSERT(collisionID1 < NE_COLLISION_TABLE_MAX);

    ASSERT(collisionID2 < NE_COLLISION_TABLE_MAX);

    if (collisionID1 < NE_COLLISION_TABLE_MAX && collisionID2 < NE_COLLISION_TABLE_MAX) {
        return ct.table[collisionID1][collisionID2];
    } else {
        return RESPONSE_IGNORE;
    }

}

/****************************************************************************
*
*	neCollisionTable::GetMaxCollisionID
*
****************************************************************************/

s32 neCollisionTable::GetMaxCollisionID() {
    return NE_COLLISION_TABLE_MAX;
}

/****************************************************************************
*
*	Helper functions
*
****************************************************************************/

/****************************************************************************
*
*	BoxInertiaTensor
*
****************************************************************************/

neV3 neBoxInertiaTensor(const neV3 &boxSize, neReal mass) {
    return neBoxInertiaTensor(boxSize[0], boxSize[1], boxSize[2], mass);
}

neV3 neBoxInertiaTensor(neReal width, neReal height, neReal depth, neReal mass) {
    neV3 ret;

    neReal maxdim = width;

    if (height > maxdim)
        maxdim = height;

    if (depth > maxdim)
        maxdim = depth;
#if 0
    neReal	xsq = width;
    neReal	ysq = height;
    neReal	zsq = depth;
#else
    neReal xsq = maxdim;
    neReal ysq = maxdim;
    neReal zsq = maxdim;
#endif

    xsq *= xsq;
    ysq *= ysq;
    zsq *= zsq;

    ret[0] = (ysq + zsq) * mass / 3.0f;
    ret[1] = (xsq + zsq) * mass / 3.0f;
    ret[2] = (xsq + ysq) * mass / 3.0f;

    return ret;
}

neV3 neSphereInertiaTensor(neReal diameter, neReal mass) {
    neReal radius = diameter * 0.5f;

    neReal value = 2.0f / 5.0f * mass * radius * radius;

    neV3 ret;

    ret.Set(value);

    return ret;
}

neV3 neCylinderInertiaTensor(neReal diameter, neReal height, neReal mass) {
//	if (height > diameter)
//	{
//		diameter = height;
//	}
    neReal radius = diameter * 0.5f;

    neReal radiusSq = radius * radius;

    neReal Ixz = 1.0f / 12.0f * mass * height * height + 0.25f * mass * radiusSq;

    neReal Iyy = 0.5f * mass * radiusSq;

    neV3 ret;

    ret.Set(Ixz, Iyy, Ixz);

    return ret;
}

/*
	bool IsEnableLimit();

	void EnableLimit(bool yes);

	neReal GetUpperLimit();

	void SetUpperLimit(neReal upperLimit);

	neReal GetLowerLimit();

	void SetLowerLimit(neReal lowerLimit);
*/