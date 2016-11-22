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

#include <cstdio>

/*
#ifdef _WIN32
#include <windows.h>
#endif
*/

#include "tokamak.h"
#include "containers.h"
#include "collision.h"
#include "constraint.h"
#include "rigidbody.h"
#include "scenery.h"
#include "simulator.h"
#include "message.h"

#define ne_Default_Mass 1.0f

//extern void DrawLine(const neV3 & colour, neV3 * startpoint, s32 count);


/****************************************************************************
*
*	neRigidBodyState::neRigidBodyState
*
****************************************************************************/

neRigidBodyState::neRigidBodyState() {
    q.Identity();

    b2w.SetIdentity();

//	b2w.pos.SetZero(); // x

    angularMom.SetZero(); // L

//	rot().SetIdentity();

    linearMom.SetZero();
}

/****************************************************************************
*
*	neRigidBody_::~neRigidBody_
*
****************************************************************************/

neRigidBody::~neRigidBody() {

}

/****************************************************************************
*
*	neRigidBody_::neRigidBody_
*
****************************************************************************/

neRigidBody::neRigidBody() {
    btype = NE_OBJECT_RIGID;

    rbExtra = &eggs;

    curState = 0;

    mass = ne_Default_Mass;

    oneOnMass = 1.0f / ne_Default_Mass;

    IbodyInv.SetZero();

    IbodyInv.M[0][0] = oneOnMass;
    IbodyInv.M[1][1] = oneOnMass;
    IbodyInv.M[2][2] = oneOnMass;

    Ibody.SetZero();

    IbodyInv.M[0][0] = ne_Default_Mass;
    IbodyInv.M[1][1] = ne_Default_Mass;
    IbodyInv.M[2][2] = ne_Default_Mass;

    cookies = nullptr;

    force.SetZero();
    torque.SetZero();

    gravityOn = true;

    status = NE_RBSTATUS_NORMAL;

    SetConstraintHeader(nullptr);

    s32 i;

    for (i = 0; i < NE_RB_MAX_RESTON_RECORDS; i++) {
        GetRestRecord(i).Init();
    }
    for (i = 0; i < NE_RB_MAX_PAST_RECORDS; i++) {
        GetVelRecord(i).SetZero();
        GetAngVelRecord(i).SetZero();
        dvRecord[i].SetZero();
        davRecord[i].SetZero();
    }

    stackInfo = nullptr;

    lowEnergyCounter = 0;

    GetRestHull().htype = neRestHull::NONE;

    isShifted = isShifted2 = false;

    controllers = nullptr;

    controllerCursor = nullptr;

    gforce.SetZero();

    cforce.SetZero();

    ctorque.SetZero();

    subType = NE_RIGID_NORMAL;

    acc.SetZero();

    angularDamp = 0.0f;

    linearDamp = 0.0f;

    sleepingParam = 0.2f;

    SyncOldState();
}

/****************************************************************************
*
*	neRigidBody_::Free
*
****************************************************************************/

void neRigidBody::Free() {
    neRigidBodyBase::Free();

    //free controller

    neFreeListItem<neController> *ci = (neFreeListItem<neController> *) controllers;

    while (ci) {
        neFreeListItem<neController> *next = ci->next;

        ci->Remove();

        sim->controllerHeap.Dealloc((neController *) ci, 1);

        ci = next;
    }
    controllers = nullptr;

    //free constraint header

    RemoveConstraintHeader();

    //free stack

    if (stackInfo) {
        ASSERT(stackInfo->stackHeader);

        stackInfo->stackHeader->Remove(stackInfo);

        //sim->stackInfoHeap.Dealloc(stackInfo, 1);
        FreeStackInfo();
    }
    for (s32 i = 0; i < NE_MAX_REST_ON; i++) {
        GetRestRecord(i).SetInvalid();
    }
}

/****************************************************************************
*
*	neRigidBody_::RecalcInertiaTensor
*
****************************************************************************/

void neRigidBody::RecalcInertiaTensor() {
    if (col.convexCount == 0)
        return;

    neM3 tensor;
    neV3 cogShift;
    neReal _mass = 0.0f;

    neReal density = 0.0f;
    neReal friction = 0.0f;
    neReal restitution = 0.0f;

    cogShift.SetZero();

    tensor.SetZero();

    if (col.convexCount == 1) {
        sim->GetMaterial(col.obb.GetMaterialId(), friction, restitution, density);

        tensor = col.obb.CalcInertiaTensor(density, _mass);

        IbodyInv.SetInvert(tensor);

        mass = _mass;

        cogShift = col.obb.c2p.pos * _mass;
    } else {
        s32 i;
        neReal m;
        neM3 _tensor;

        for (i = 0; i < col.convexCount; i++) {
            sim->GetMaterial(col.convex[i].GetMaterialId(), friction, restitution, density);

            _tensor = col.convex[i].CalcInertiaTensor(density, m);

            tensor += _tensor;

            _mass += m;

            cogShift += col.convex[i].c2p.pos * _mass;
        }

        mass = _mass;
    }
    oneOnMass = 1.0f / mass;

    cogShift = cogShift * (1.0f / mass);

    TranslateCOM(tensor, cogShift, mass, -1.0f);

    Ibody = tensor;

    IbodyInv.SetInvert(tensor);
}

/****************************************************************************
*
*	neRigidBody_::AddController
*
****************************************************************************/

neController *neRigidBody::AddController(neRigidBodyControllerCallback *rbc, s32 period) {
    neController *c = sim->controllerHeap.Alloc(1);

    if (!c) {
        sprintf(sim->logBuffer, MSG_CONTROLLER_FULL);

        sim->LogOutput(neSimulator::LOG_OUTPUT_LEVEL_ONE);

        return nullptr;
    }
    if (!controllers) {
        controllers = c;
    } else {
        //((neControllerItem *)controllers)->Append((neControllerItem *)c);

        neControllerItem *citem = (neControllerItem *) controllers;

        while (citem->next) {
            citem = citem->next;
        }
        citem->Append((neControllerItem *) c);
    }
    c->rb = this;

    c->constraint = nullptr;

    c->rbc = rbc;

    c->jc = nullptr;

    c->period = period;

    c->count = period;

    c->forceA.SetZero();

    c->torqueA.SetZero();

    c->forceB.SetZero();

    c->torqueB.SetZero();

    return c;
}

/****************************************************************************
*
*	neRigidBody_::BeginIterateController
*
****************************************************************************/

void neRigidBody::BeginIterateController() {
    controllerCursor = (neControllerItem *) controllers;
}

/****************************************************************************
*
*	neRigidBody_::GetNextController
*
****************************************************************************/

neController *neRigidBody::GetNextController() {
    if (!controllerCursor)
        return nullptr;

    neController *ret = (neController *) controllerCursor;

    controllerCursor = controllerCursor->next;

    return ret;
}

/****************************************************************************
*
*	neRigidBody_::GravityEnable
*
****************************************************************************/

void neRigidBody::GravityEnable(bool yes)
{
    gravityOn = yes;
}

/****************************************************************************
*
*	neRigidBody_::UpdateDerive
*
****************************************************************************/
void neRigidBody::SetAngMom(const neV3 &am) {
    neQ w;

    State().angularMom = am;

    Derive().angularVel = Derive().Iinv * State().angularMom;

    w.Set(Derive().angularVel, 0.0f);

    Derive().qDot = w * State().q;

    Derive().qDot *= 0.5f;
}

void neRigidBody::UpdateDerive() {
    neRigidBodyState & state = State();

    state.q = state.q.Normalize();

    state.rot() = state.q.BuildMatrix3();

    // Iinv = R * IbodyInv Rtrans
    //derive.Iinv = state.rot() * IbodyInv * neM3_BuildTranspose(state.rot());
    derive.Iinv = state.rot() * IbodyInv * neM3().SetTranspose(state.rot());

    // w = Iinv * L
    derive.angularVel = derive.Iinv * state.angularMom;

    neQ w;
    w.W = 0.0f;
    w.X = derive.angularVel[0];
    w.Y = derive.angularVel[1];
    w.Z = derive.angularVel[2];

    derive.qDot = w * state.q;

    derive.qDot = derive.qDot * 0.5f;

    derive.speed = derive.linearVel.Length();
}


/****************************************************************************
*
*	neRigidBody_::Advance
*
****************************************************************************/

void neRigidBody::AdvanceDynamic(neReal tStep) {
    oldCounter++;

    totalDV.SetZero();

    totalDA.SetZero();

    impulseCount = 0;

    twistCount = 0;

    isAddedToSolver = false;

    if (status == neRigidBody::NE_RBSTATUS_IDLE) {
        if (!cforce.IsConsiderZero())
            WakeUp();

        if (!ctorque.IsConsiderZero())
            WakeUp();
    }

    if (status == neRigidBody::NE_RBSTATUS_IDLE) {
        if (CheckStillIdle()) {
            return;
        }
    }

    if (status == neRigidBody::NE_RBSTATUS_ANIMATED) {
        return;
    }

    dvRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS].SetZero();
    davRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS].SetZero();

    neRigidBodyState & state = State();

    int newStateId = (curState + 1) % MAX_RB_STATES;

    neRigidBodyState & newState = stateBuffer[newStateId];

    totalForce += (force + gforce + cforce);

    dvRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS] = totalForce * oneOnMass * tStep;

    acc = totalForce * oneOnMass;

    derive.linearVel += acc * tStep;

    derive.linearVel *= (1.0f - linearDamp);

    totalTorque += (torque + ctorque);

    //L = L0 + torque * t

    MidPointIntegration(totalTorque, tStep);
}

void neRigidBody::MidPointIntegration(const neV3 &totalTorque, neReal tStep) {
    State().angularMom *= (1.0f - angularDamp);

    neV3 newAngularMom = State().angularMom + totalTorque * tStep;

    neQ tmpQ;

    tmpQ = State().q;

    neReal midStep = tStep * 0.5f;

    tmpQ = State().q + Derive().qDot * midStep;

    neV3 tmpL = State().angularMom + totalTorque * midStep;

    tmpQ = tmpQ.Normalize();

    neM3 rot = tmpQ.BuildMatrix3();

    neM3 tmpIinv = rot * IbodyInv * neM3().SetTranspose(rot);

    neV3 tmpAngVel = tmpIinv * tmpL;

    neQ tmpW;

    tmpW.Set(tmpAngVel, 0.0f);

    Derive().qDot = tmpW * tmpQ * 0.5f;

    curState = (curState + 1) % MAX_RB_STATES;

    State().angularMom = newAngularMom;

    Derive().angularVel = tmpAngVel;

    //SetAngMom(am);
}

void neRigidBody::ImprovedEulerIntegration(const neV3 &totalTorque, neReal tStep) {
/*	neV3 newAngularMom = State().angularMom + totalTorque * tStep;

	neQ tmpQ = State().q + Derive().qDot * tStep;

	tmpQ.Normalize();

	neM3 rot = tmpQ.BuildMatrix3();

	neV3 Iinv = rot * IbodyInv * neM3().SetTranspose(rot);

	neV3 angVel = Iinv * newAngularMom;



	curState = (curState + 1) % MAX_RB_STATES;

	State().angularMom = newAngularMom;
*/
}

void neRigidBody::RungeKutta4Integration(const neV3 &totalTorque, neReal tStep) {
/*	neV3 newAngularMom = State().angularMom + totalTorque * tStep;

	neQ q1, q2, q3, q4;

	neV3 L1, L2, L3, L4;

	neReal midStep = tStep * 0.5f;

	q1 = State().q + Derive().qDot * tStep;

	L1 = 

	curState = (curState + 1) % MAX_RB_STATES;

	State().angularMom = newAngularMom;
*/
}

void neRigidBody::AdvancePosition(neReal tStep) {
//	needSolveContactDynamic = true;

//	totalForce.SetZero();

//	totalTorque.SetZero();

//	if (status == neRigidBody::NE_RBSTATUS_IDLE)
//		return;

    //derive.linearVel *= 0.99;

    neV3 newPos;

    newPos = GetPos() + derive.linearVel * tStep + 0.5f * acc * tStep * tStep;

    neV3 bv = newPos - GetPos();

    backupVector = 0.7f * backupVector + 0.3f * bv;

    SetPos(newPos);

//	derive.linearVel += acc * tStep;
/*	
	neQ tmpQ;

	tmpQ = State().q;

	neReal midStep = tStep * 0.5f;

	tmpQ = State().q + derive.qDot * midStep;

	neV3 tmpL = State().angularMom + totalTorque  * midStep;

	tmpQ = tmpQ.Normalize();

	neM3 rot = tmpQ.BuildMatrix3();

	neM3 tmpIinv = rot * IbodyInv * neM3().SetTranspose(rot);

	neV3 tmpAngVel = tmpIinv * tmpL;

	neQ tmpW; 
	
	tmpW.Set(tmpAngVel, 0.0f);

	neQ tmpQDot = tmpW * tmpQ * 0.5f;
*/
    //q = q0 + qdot * t

    neV3 am = State().angularMom;// * 0.95f;

    SetAngMom(am);

    State().q = State().q + Derive().qDot * tStep;

    UpdateDerive();

    UpdateController();

    if (sensors)
        ClearSensor();

    GetVelRecord(sim->stepSoFar % NE_RB_MAX_PAST_RECORDS) = Derive().linearVel;

    GetAngVelRecord(sim->stepSoFar % NE_RB_MAX_PAST_RECORDS) = Derive().angularVel;

    //CheckForIdle();
}

/****************************************************************************
*
*	neRigidBody_::UpdateController
*
****************************************************************************/

void neRigidBody::UpdateController() {
    if (gravityOn && status != neRigidBody::NE_RBSTATUS_IDLE) {
        gforce = sim->gravity * mass;
    } else {
        gforce.SetZero();
    }

    cforce.SetZero();

    ctorque.SetZero();

    neControllerItem *ci = (neControllerItem *) controllers;

    while (ci) {
        neController *con = (neController *) ci;

        ci = ci->next;

        if (con->count == 0) {
            ASSERT(con->rbc);

            con->rbc->RigidBodyControllerCallback((neRigidBodyController *) con, sim->_currentTimeStep);

            con->count = con->period;
        } else {
            con->count--;
        }
        con->rb->cforce += con->forceA;

        con->rb->ctorque += con->torqueA;
    }
}

/****************************************************************************
*
*	neRigidBody_::UpdateAABB
*
****************************************************************************/

void neRigidBody::UpdateAABB() {
    if (col.convexCount == 0 && !isCustomCD)
        return;

//	for (s32 i = 0; i < 3; i++)
    {
#if 0
        if (minCoord[0])
            minCoord[0]->value = p[0] /*- neAbs(c2w[0][0]) - neAbs(c2w[0][1]) - neAbs(c2w[0][2])*/ - col.boundingRadius;
        if (maxCoord[0])
            maxCoord[0]->value = p[0] /*+ neAbs(c2w[0][0]) + neAbs(c2w[0][1]) + neAbs(c2w[0][2]);//*/ + col.boundingRadius;

        if (minCoord[1])
            minCoord[1]->value = p[1] /*- neAbs(c2w[1][0]) - neAbs(c2w[1][1]) - neAbs(c2w[1][2])*/ - col.boundingRadius;
        if (maxCoord[1])
            maxCoord[1]->value = p[1] /*+ neAbs(c2w[1][0]) + neAbs(c2w[1][1]) + neAbs(c2w[1][2]);*/ + col.boundingRadius;

        if (minCoord[2])
            minCoord[2]->value = p[2] /*- neAbs(c2w[2][0]) - neAbs(c2w[2][1]) - neAbs(c2w[2][2])*/ - col.boundingRadius;
        if (maxCoord[2])
            maxCoord[2]->value = p[2] /*+ neAbs(c2w[2][0]) + neAbs(c2w[2][1]) + neAbs(c2w[2][2]);*/ + col.boundingRadius;
#else
        //neM3 & rot = State().rot();

        neT3 c2w = State().b2w * obb;
/*
		neM3 c2w2 = rot * dobb;

		for (s32 i = 0; i < 3; i++)
		{
			neReal a = neAbs(c2w2[0][i]) + neAbs(c2w2[1][i]) + neAbs(c2w2[2][i]);

			debugMinBound[i] = p[i] - a;
		}
*/
/*
	c2w.M[0][0] = obb.M[0][0] * rot.M[0][0] + obb.M[0][1] * rot.M[1][0] + obb.M[0][2] * rot.M[2][0];
	c2w.M[0][1] = obb.M[0][0] * rot.M[0][1] + obb.M[0][1] * rot.M[1][1] + obb.M[0][2] * rot.M[2][1];
	c2w.M[0][2] = obb.M[0][0] * rot.M[0][2] + obb.M[0][1] * rot.M[1][2] + obb.M[0][2] * rot.M[2][2];
	c2w.M[1][0] = obb.M[1][0] * rot.M[0][0] + obb.M[1][1] * rot.M[1][0] + obb.M[1][2] * rot.M[2][0];
	c2w.M[1][1] = obb.M[1][0] * rot.M[0][1] + obb.M[1][1] * rot.M[1][1] + obb.M[1][2] * rot.M[2][1];
	c2w.M[1][2] = obb.M[1][0] * rot.M[0][2] + obb.M[1][1] * rot.M[1][2] + obb.M[1][2] * rot.M[2][2];
	c2w.M[2][0] = obb.M[2][0] * rot.M[0][0] + obb.M[2][1] * rot.M[1][0] + obb.M[2][2] * rot.M[2][0];
	c2w.M[2][1] = obb.M[2][0] * rot.M[0][1] + obb.M[2][1] * rot.M[1][1] + obb.M[2][2] * rot.M[2][1];
	c2w.M[2][2] = obb.M[2][0] * rot.M[0][2] + obb.M[2][1] * rot.M[1][2] + obb.M[2][2] * rot.M[2][2];
*/

        neV3 & p = c2w.pos;

        neReal a = neAbs(c2w.rot[0][0]) + neAbs(c2w.rot[1][0]) + neAbs(c2w.rot[2][0]);

        minBound[0] = p[0] - a;
        maxBound[0] = p[0] + a;

        if (minCoord[0])
            minCoord[0]->value = p[0] - a;
        if (maxCoord[0])
            maxCoord[0]->value = p[0] + a;

        a = neAbs(c2w.rot[0][1]) + neAbs(c2w.rot[1][1]) + neAbs(c2w.rot[2][1]);

        minBound[1] = p[1] - a;
        maxBound[1] = p[1] + a;

        if (minCoord[1])
            minCoord[1]->value = p[1] - a;
        if (maxCoord[1])
            maxCoord[1]->value = p[1] + a;

        a = neAbs(c2w.rot[0][2]) + neAbs(c2w.rot[1][2]) + neAbs(c2w.rot[2][2]);

        minBound[2] = p[2] - a;
        maxBound[2] = p[2] + a;

        if (minCoord[2])
            minCoord[2]->value = p[2] - a;
        if (maxCoord[2])
            maxCoord[2]->value = p[2] + a;
#endif
    }
}

void neRigidBody::WakeUp() {
    if(status == neRigidBody::NE_RBSTATUS_NORMAL) {
        return;
    }
    status = neRigidBody::NE_RBSTATUS_NORMAL;

    lowEnergyCounter = 0;

    SyncOldState();
}

void neRigidBody::SyncOldState() {
    oldPosition = State().b2w.pos;

    oldRotation = State().q;

    oldVelocity = Derive().linearVel;

    oldAngularVelocity = Derive().angularVel;

    oldCounter = 0;
}

void neRigidBody::BecomeIdle() {
    //return;

    status = neRigidBody::NE_RBSTATUS_IDLE;

    ZeroMotion();
}

void neRigidBody::ZeroMotion() {
    Derive().angularVel.SetZero();
    Derive().linearVel.SetZero();
    Derive().qDot.Zero();
    Derive().speed = 0.0f;
    State().angularMom.SetZero();

    for (s32 i = 0; i < NE_RB_MAX_PAST_RECORDS; i++) {
        GetVelRecord(i).SetZero();
        GetAngVelRecord(i).SetZero();
        dvRecord[i].SetZero();
    }
}

/****************************************************************************
*
*	neRigidBody_::ApplyCollisionImpulse
*
****************************************************************************/

bool neRigidBody::ApplyCollisionImpulse(const neV3 &impulse, const neV3 &contactPoint, neImpulseType itype) {
    neV3 dv, da, newAM;

    dv = impulse * oneOnMass;

    da = contactPoint.Cross(impulse);

    bool immediate = true;

    if (itype == IMPULSE_CONTACT)// && sim->solverStage == 1) // only if contact and resolve penetration stage
    {
        immediate = false;
    }

/*	if (itype != IMPULSE_NORMAL && sim->solverStage != 2)
	{
		immediate = false;
	}
*/

    immediate = true;

    if (immediate) {
        Derive().linearVel += dv;

        dvRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS] += dv;

        neV3 dav = Derive().Iinv * da;

        davRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS] += dav;

        newAM = State().angularMom + da;

        SetAngMom(newAM);

        neReal l = dv.Length();
/*
		if (id == 1 && l > sim->magicNumber)
		{
			sim->magicNumber = l;
		}
*/
        ASSERT(newAM.IsFinite());
    } else {
        totalDV += dv;
        totalDA += da;
        impulseCount++;
        twistCount++;
    }
    return true;
}

void neRigidBody::AddRestContact(neRestRecord & rc) {
    //search existing matching records

    s32 oldest = -1;

    s32 freeOne = -1;

    neReal shallowest = 0.0f;

    s32 i;

    for (i = 0; i < NE_RB_MAX_RESTON_RECORDS; i++) {
        if (!GetRestRecord(i).IsValid()) {
            freeOne = i;

            continue;
        }
        if (shallowest == 0.0f) {
            shallowest = GetRestRecord(i).depth;
            oldest = i;
        } else if (GetRestRecord(i).depth < shallowest) {
            shallowest = GetRestRecord(i).depth;
            oldest = i;
        }
        neV3 diff = rc.bodyPoint - GetRestRecord(i).bodyPoint;

        neReal dot = diff.Dot(diff);

        if (dot < 0.0025f) //difference of 0.05 meters, or 5 cm
        {
            //found
            break;
        }
    }
/*	if (i < NE_RB_MAX_RESTON_RECORDS)
	{
		GetRestRecord(i).SetInvalid();
	}
	else
	{
		if (freeOne != -1)
		{
			i = freeOne;
		}
		else
		{
			//use the olderest one
			ASSERT(oldest != -1);
			
			i = oldest;
		}
	}
*/
    if (i == NE_RB_MAX_RESTON_RECORDS) {    //not found

        //find a free one
        if (freeOne != -1) {
            i = freeOne;
        } else {
            //use the olderest one
            ASSERT(oldest != -1);

            i = oldest;

            GetRestRecord(i).SetInvalid();
        }
    } else {
        GetRestRecord(i).SetInvalid();
    }
    if (i >= NE_RB_MAX_RESTON_RECORDS || i < 0)
        return;

    GetRestRecord(i).Set(this, rc);
}

bool neRigidBody::IsConstraintNeighbour(neRigidBodyBase *otherBody) {
    neConstraintHandle *chandle = constraintCollection.GetHead();

    while (chandle) {
        _neConstraint *c = chandle->thing;

        chandle = constraintCollection.GetNext(chandle);

        neConstraintHandle *chandleB = otherBody->constraintCollection.GetHead();

        while (chandleB) {
            _neConstraint *cB = chandleB->thing;

            if (c == cB)
                return true;

            chandleB = otherBody->constraintCollection.GetNext(chandleB);
        }
    };
    return false;
}

void neRigidBody::SetAngMomComponent(const neV3 &angMom, const neV3 &dir) {
    neV3 newMom = State().angularMom;

    newMom.RemoveComponent(dir);

    newMom += angMom;

    SetAngMom(newMom);

    needRecalc = true;
}

void neRigidBody::WakeUpAllJoint() {
    if (!GetConstraintHeader()) {
        WakeUp();
        return;
    }
    neBodyHandle *bodyHandle = GetConstraintHeader()->bodies.GetHead();

    while (bodyHandle) {
        if (!bodyHandle->thing->AsRigidBody()) {
            bodyHandle = bodyHandle->next;

            continue;
        }

        bodyHandle->thing->AsRigidBody()->WakeUp();

        bodyHandle = bodyHandle->next;
    }
}

void neRigidBody::ApplyLinearConstraint() {
    neV3 dv = totalDV / (neReal) impulseCount;

//	cacheImpulse = totalDV;

    Derive().linearVel += dv;

    totalDV.SetZero();

    //dvRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS] += dv;

    impulseCount = 0;
}

void neRigidBody::ApplyAngularConstraint() {
    neV3 da = totalDA / (neReal) twistCount;

//	cacheTwist += da;

    neV3 newAM = State().angularMom + da;

    SetAngMom(newAM);

    totalDA.SetZero();

    twistCount = 0;
}

/*
void neRigidBody::ConstraintDoSleepCheck()
{
	neReal len = dvRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS].Length();


	if (len > 0.3f)
	{
		//WakeUpAllJoint();
		//WakeUp();
	}
	else 
	{
		len = davRecord[sim->stepSoFar % NE_RB_MAX_PAST_RECORDS].Length();

		if (len > 0.5f)
		{
			//WakeUp();//AllJoint();
		}
		else
		{
			BecomeIdle();
		}
	}
}
*/

bool neRestRecord::CanConsiderOtherBodyIdle() {
    if (rtype == REST_ON_WORLD)
        return true;

    ASSERT(otherBody != NULL);

    neRigidBody *rb = otherBody->AsRigidBody();

    if (rb != nullptr) {
        return (rb->status == neRigidBody::NE_RBSTATUS_IDLE);
    }
    neCollisionBody_ *cb = otherBody->AsCollisionBody();

    ASSERT(cb);

    return (!cb->moved);
}

bool neRestRecord::CheckOtherBody(neSimulator *sim) {
    ASSERT(otherBody);

    if (otherBody != sim->GetTerrainBody() &&
        (!otherBody->IsValid() || !otherBody->isActive)) {
        return false;
    }
    return true;
}

neV3 neRestRecord::GetOtherBodyPoint() {
    ASSERT(otherBody);

    neCollisionBody_ *cb = otherBody->AsCollisionBody();

    neV3 ret;

    if (cb) {
        normalWorld = cb->b2w.rot * normalBody;

        ret = cb->b2w * otherBodyPoint;
    } else {
        normalWorld = ((neRigidBody *) otherBody)->State().b2w.rot * normalBody;

        ret = ((neRigidBody *) otherBody)->State().b2w * otherBodyPoint;
    }
    return ret;
}

void neRestRecord::Set(neRigidBody *thisBody, const neRestRecord &rc) {
    bodyPoint = rc.bodyPoint;
    otherBodyPoint = rc.otherBodyPoint;
    otherBody = rc.otherBody;
    counter = thisBody->sim->stepSoFar;
    normalBody = rc.normalBody;
    depth = rc.depth;
    body = thisBody;
    material = rc.material;
    otherMaterial = rc.otherMaterial;

    if (rc.otherBody->AsCollisionBody())
        rtype = neRestRecord::REST_ON_COLLISION_BODY;
    else
        rtype = neRestRecord::REST_ON_RIGID_BODY;

    otherBody->rbRestingOnMe.Add(&restOnHandle);
}

void
neRestRecord::SetTmp(neRigidBodyBase *_otherBody, const neV3 &contactA, const neV3 &contactB, const neV3 &_normalBody,
                     s32 matA, s32 matB) {
    otherBody = _otherBody;
    bodyPoint = contactA;
    otherBodyPoint = contactB;
    material = matA;
    otherMaterial = matB;
    normalBody = _normalBody;
}

void neRestRecord::SetInvalid() {
    //ASSERT(rtype != REST_ON_NOT_VALID);

    rtype = REST_ON_NOT_VALID;

    counter = 0;

//	body = NULL;

    if (otherBody) {
        otherBody->rbRestingOnMe.Remove(&restOnHandle);
    }
    otherBody = nullptr;
}

/****************************************************************************
*
*	neRigidBody::SetInertiaTensor
*
****************************************************************************/

void neRigidBody::SetInertiaTensor(const neM3 &tensor) {
    Ibody = tensor;
    IbodyInv.SetInvert(tensor);

    //ASSERT(tensor.Invert(rb.IbodyInv));
}

/****************************************************************************
*
*	neRigidBody::SetInertiaTensor
*
****************************************************************************/

void neRigidBody::SetInertiaTensor(const neV3 &tensor) {
    neM3 i;

    i.SetIdentity();

    i[0][0] = tensor[0];
    i[1][1] = tensor[1];
    i[2][2] = tensor[2];

    Ibody = i;

    IbodyInv.SetInvert(Ibody);
}


/*
neGeometry * neRigidBody::GetGeometry(s32 index)
{
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


void neRigidBody::SetForce(const neV3 &force, const neV3 &pos) {

    if (force.IsConsiderZero()) {
        this->force = force;

        torque = ((pos - GetPos()).Cross(force));

        return;
    }

    this->force = force;

    torque = ((pos - GetPos()).Cross(force));

    WakeUp();
}

void neRigidBody::SetTorque(const neV3 &torque) {

    if (torque.IsConsiderZero()) {
        this->torque = torque;

        return;
    }
    this->torque = torque;

    WakeUp();
}

void neRigidBody::SetForce(const neV3 &force) {

    if (force.IsConsiderZero()) {
        this->force = force;
        return;
    }
    this->force = force;

    WakeUp();
}

void neRigidBody::ApplyImpulse(const neV3 &impulse) {
    neV3 dv = impulse * oneOnMass;
    Derive().linearVel += dv;
    WakeUpAllJoint();
}

/****************************************************************************
*
*	neRigidBody::AddImpulseWithTwist
*
****************************************************************************/

void neRigidBody::ApplyImpulse(const neV3 &impulse, const neV3 &pos) {


    neV3 dv = impulse * oneOnMass;

    neV3 da = (pos - GetPos()).Cross(impulse);

    Derive().linearVel += dv;

    neV3 newAM = State().angularMom + da;

    SetAngMom(newAM);

    WakeUp();
}

/****************************************************************************
*
*	neRigidBody::ApplyTwist
*
****************************************************************************/

void neRigidBody::ApplyTwist(const neV3 &twist) {

    neV3 newAM = twist;

    SetAngMom(newAM);

    WakeUp();
}

/****************************************************************************
*
*	neRigidBody::AddController
*
****************************************************************************/

//neRigidBodyController *neRigidBody::AddController(neRigidBodyControllerCallback *controller, s32 period) {
//
//
//    return (neRigidBodyController *) AddController(controller, period);
//}

/****************************************************************************
*
*	neRigidBody::RemoveController
*
****************************************************************************/

bool neRigidBody::RemoveController(neRigidBodyController *rbController) {


    if (!controllers)
        return false;

    neControllerItem *ci = (neControllerItem *) controllers;

    while (ci) {
        neController *con = reinterpret_cast<neController *>(ci);

        ci = ci->next;

        if (con == reinterpret_cast<neController *>(rbController)) {
            //reinterpret_cast<neControllerItem *>(con)->Remove();

            sim->controllerHeap.Dealloc(con, 1);

            return true;
        }
    }
    return false;
}

neGeometry *neRigidBody::AddGeometry() {

    TConvex *g = neRigidBodyBase::AddGeometry();

    return reinterpret_cast<neGeometry *>(g);
}

bool neRigidBody::RemoveGeometry(neGeometry *g) {


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
*	neRigidBody::BreakGeometry
*
****************************************************************************/
neRigidBody *neRigidBody::BreakGeometry(neGeometry *g) {
    neRigidBody *newBody = sim->CreateRigidBodyFromConvex((TConvex *) g, this);
    return (neRigidBody *) newBody;
}

/****************************************************************************
*
*	neRigidBody::UseCustomCollisionDetection
*
****************************************************************************/

void neRigidBody::UseCustomCollisionDetection(bool yes, const neT3 *obb, neReal boundingRadius) {


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

neSensor *neRigidBody::AddSensor() {
    neSensor_ *s = neRigidBodyBase::AddSensor();
    return reinterpret_cast<neSensor *>(s);
}

/****************************************************************************
*
*	neRigidBody::RemoveSensor
*
****************************************************************************/

bool neRigidBody::RemoveSensor(neSensor *s) {


    if (!sensors)
        return false;

    neSensorItem *si = (neSensorItem *) sensors;

    while (si) {
        neSensor_ *sensor = reinterpret_cast<neSensor_ *>(si);

        si = si->next;

        if (sensor == reinterpret_cast<neSensor_ *>(s)) {
            //reinterpret_cast<neSensorItem *>(s)->Remove();

            sim->sensorHeap.Dealloc(sensor, 1);

            return true;
        }
    }
    return false;
}

/****************************************************************************
*
*	neRigidBody::GetNextSensor
*
****************************************************************************/

neSensor *neRigidBody::GetNextSensor() {
    return reinterpret_cast<neSensor *>(neRigidBodyBase::GetNextSensor());
}

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

