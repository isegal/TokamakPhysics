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

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#pragma inline_depth( 8 )

#define MAX_RB_STATES 1
#define MAX_RB_IMPULSE_RECORDS 4
#define MAX_RB_CONTROLLERS 5

class neRigidBodyBase;

class neRigidBody_;

class neSimulator;

class neConstraintSolver;

class neRestRecord;

///////////////////////////////////////////////////////////////////
//
//	neControllerCallback
//
///////////////////////////////////////////////////////////////////
class neController {
PLACEMENT_MAGIC

public:
    s32 period;
    s32 count;
    neRigidBodyControllerCallback *rbc;
    neJointControllerCallback *jc;
    _neConstraint *constraint;
    neRigidBody_ *rb;
    neV3 forceA;
    neV3 torqueA;
    neV3 forceB;
    neV3 torqueB;

public:
    neController() {
        period = 0;
        count = 0;
        rbc = nullptr;
        jc = nullptr;
        constraint = nullptr;
        rb = nullptr;
    }
};

//typedef neFreeListItem<neController> neControllerItem;

//coordinate list

class CCoordListEntry {
PLACEMENT_MAGIC

public:
    enum {
        LowEnd = 0,
        HighEnd = 1,
    };
    neByte flag;
    neReal value;
    neRigidBodyBase *bb;
};

using PCCoordListEntry = CCoordListEntry *;

using CCoordListEntryItem = neDLinkList<CCoordListEntry>::listItem;

///////////////////////////////////////////////////////////////////
//
//	neRigidBodyBase
//
///////////////////////////////////////////////////////////////////
class _neConstraint;

using neSensorItem = neFreeListItem<neSensor_>;

class neCollisionBody_;

using neBodyHandle = neCollection<neRigidBodyBase>::itemType;

//neCollection<_neConstraint>::itemType neConstraintItem;

typedef enum {
    NE_OBJECT_COLISION,
    NE_OBJECT_RIGID,
} neObjectType;

typedef enum {
    NE_RIGID_NORMAL,
    NE_RIGID_PARTICLE,
} neRigidType;

class neRigidBodyBase {
public:
    neRigidBodyBase() {
        //isAnimated = true;
        btype = NE_OBJECT_COLISION;
        col.convex = nullptr;
        col.convexCount = 0;
        col.obb.Initialise();
        sim = nullptr;
        cid = 0;
        isCollideConnected = false;
        isCollideDirectlyConnected = false;
        sensors = nullptr;

        geometryCursor = nullptr;
        sensorCursor = nullptr;

        constraintHeaderItem.thing = this;
        isActive = true;
        regionHandle = nullptr;

        _constraintHeader = nullptr;

        isCustomCD = false;

        for (s32 i = 0; i < 3; i++) {
            maxCoord[i] = nullptr;
            minCoord[i] = nullptr;
        }

        backupVector.SetZero();

        pendingAddToRegion = 0;
    }

    ~neRigidBodyBase() {
        //if (col.convex)
        //	delete [] col.convex;
    };

    void RecalcBB();

    //TConvex * GetConvex(s32 index);

    void Free();

    NEINLINE neByte IsAABOverlapped(neRigidBodyBase *b) {
        neByte ret = 0;

        for (s32 i = 0; i < 3; i++) {
            if (minCoord[i]) {
                if (!(minCoord[i]->value >= b->maxCoord[i]->value ||
                      maxCoord[i]->value <= b->minCoord[i]->value))
                    ret |= (1 << i);
            }
        }

        return ret;
    }

    bool IsValid();

    neV3 VelocityAtPoint(const neV3 &pt);

    NEINLINE neV3 GetLinearVelocity();

    NEINLINE neV3 GetAngularVelocity();

    void CollideConnected(bool yes);

    bool CollideConnected();

    neSensor_ *AddSensor();

    void BeginIterateSensor();

    neSensor_ *GetNextSensor();

    void ClearSensor();

    TConvex *AddGeometry();

    void BeginIterateGeometry();

    TConvex *GetNextGeometry();

    void Active(bool yes, neRigidBodyBase *hint);

    NEINLINE neCollisionBody_ *AsCollisionBody() {
        if (btype != NE_OBJECT_COLISION)
            return nullptr;

        return (neCollisionBody_ *) this;
    }

    NEINLINE neRigidBody_ *AsRigidBody() {
        if (btype != NE_OBJECT_RIGID)
            return nullptr;

        return (neRigidBody_ *) this;
    }

    neT3 &GetB2W();

    NEINLINE void SetConstraintHeader(neConstraintHeader *cheader) {
        _constraintHeader = cheader;
    }

    NEINLINE neConstraintHeader *GetConstraintHeader() {
        return _constraintHeader;
    }

    void RemoveConstraintHeader();

    NEINLINE bool IsInRegion() {
        return (regionHandle != nullptr);
    }

    neBodyHandle constraintHeaderItem;

    //neBodyHandle activeHandle;

    neCollection<_neConstraint> constraintCollection;

    TConvexItem *geometryCursor;

    neSensorItem *sensorCursor;

    void * cookies;

    neCollision col;

    uint32_t id;

    uint32_t cid;

    neConstraintHeader *_constraintHeader;

    //bool isAnimated;
    neObjectType btype;

    bool isActive;

    bool isCollideConnected;

    bool isCollideDirectlyConnected;

    neSensor_ *sensors;

    neFreeListItem<neRigidBodyBase *> *regionHandle;

    neSimulator *sim;

    neT3 obb;

    bool isCustomCD;

    neV3 minBound;
    neV3 maxBound;
    CCoordListEntry *maxCoord[3];
    CCoordListEntry *minCoord[3];

    neV3 backupVector;

    neCollection<neRestRecord> rbRestingOnMe;

    s32 pendingAddToRegion;

//	neV3 debugMinBound;
//	neM3 dobb;
};

///////////////////////////////////////////////////////////////////
//
//	neRigidBody_
//
///////////////////////////////////////////////////////////////////

class neRigidBodyState {
public:
    neRigidBodyState();

    neQ q;
    neV3 linearMom; // P
    neV3 angularMom; // L

//	NEINLINE neV3 &pos() const{ 
//		return b2w.pos;}

//	NEINLINE void SetPos(const neV3 & pos){ 
//		b2w.pos = pos;}


    NEINLINE neM3 &rot() {
        return b2w.rot;
    }

//private:
    neT3 b2w;
};

class neRigidBodyDerive {
public:
    neRigidBodyDerive() {
        linearVel.SetZero();
        angularVel.SetZero();
        qDot.Zero();
        Iinv.SetZero();
    }

    neV3 linearVel; // v
    neV3 angularVel; // w
    neQ qDot;
    neM3 Iinv; // R * IbodyInv * Rtrans
    neReal speed;
};

struct neImpulseRecord {
    neV3 point;
    uint32_t stepCount;
};

///////////////////////////////////////////////////////////////////
//
//	neCollisionBody_
//
///////////////////////////////////////////////////////////////////

class neCollisionBody_ : public neRigidBodyBase {
public:
PLACEMENT_MAGIC

    neCollisionBody_() {
        moved = false;
    }

    void UpdateAABB();

    void Free();

public:
    neT3 b2w;

    bool moved;
};

class neStackInfo;

class neStackHeader;

/////////////////////////////////////////////////////////////////

using neRestRecordHandle = neCollection<neRestRecord>::itemType;

class neRestRecord {
public:
    typedef enum {
        REST_ON_NOT_VALID,
        REST_ON_RIGID_BODY,
        REST_ON_COLLISION_BODY,
        REST_ON_WORLD,

    } RestOnType;

    neV3 bodyPoint;
    neV3 otherBodyPoint;
    neV3 worldThisBody;
    neV3 worldOtherBody;
    neV3 worldDiff;
    neV3 normalBody; //normal define in the body space of otherBody
    neV3 normalWorld; //normal define in the world space
    neReal depth;
    neReal normalDiff;
    neReal tangentialDiffSq;
    s32 material;
    s32 otherMaterial;

private:
    RestOnType rtype;
    neRigidBodyBase *otherBody;
    neRestRecordHandle restOnHandle;
    neRigidBody_ *body;
    s32 counter;

public:
    neRestRecord() {
        restOnHandle.thing = this;
    }

    void Init() {
        rtype = REST_ON_NOT_VALID;
        counter = 0;
        otherBody = nullptr;
        body = nullptr;
        restOnHandle.Remove();
    }

    bool IsValid() {
        return rtype != REST_ON_NOT_VALID;
    }

    void Update();

    neRigidBodyBase *GetOtherBody() const {
        return otherBody;
    }

    neRigidBody_ *GetOtherRigidBody() const {
        if (!otherBody)
            return nullptr;

        return otherBody->AsRigidBody();
    }

    neCollisionBody_ *GetOtherCollisionBody() const {
        if (!otherBody)
            return nullptr;

        return otherBody->AsCollisionBody();
    }

    bool CanConsiderOtherBodyIdle();

    bool CheckOtherBody(neSimulator *sim);

    void SetInvalid();

    neV3 GetOtherBodyPoint();

    void Set(neRigidBody_ *thisBody, const neRestRecord &rc);

    void SetTmp(neRigidBodyBase *otherb, const neV3 &contactA, const neV3 &contactB, const neV3 &normalBody, s32 matA,
                s32 matB);
};

class neRestHull {
public:
    typedef enum {
        NONE,
        POINT,
        LINE,
        TRIANGLE,
        QUAD,
    } neRestHullType;

    s32 htype;
    s32 indices[4];
    neV3 normal;
};

enum {
    NE_RB_MAX_PAST_RECORDS = 10,
    NE_RB_MAX_RESTON_RECORDS = 3,
};

class neRBExtra {
public:
    neRestRecord restOnRecord[NE_RB_MAX_RESTON_RECORDS];

    neV3 velRecords[NE_RB_MAX_PAST_RECORDS];

    neV3 angVelRecords[NE_RB_MAX_PAST_RECORDS];

    neRestHull restHull;
};

class neMotionCorrectionInfo {
public:
    neQ lastQuat;
    neV3 lastPos;
    neV3 lastAM;
    neV3 lastW;
    neM3 lastIinv;
    neV3 lastVel;
    neV3 lastAVel;
};

class neRigidBody_ : public neRigidBodyBase {
PLACEMENT_MAGIC

    friend class neSimulator;

public:

    enum {
        NE_RBSTATUS_NORMAL = 0,
        NE_RBSTATUS_IDLE,
        NE_RBSTATUS_ANIMATED,
    };
    neReal mass;
    neReal oneOnMass;
    neM3 IbodyInv;
    neM3 Ibody;
    neV3 force;
    neV3 torque;
    s32 status;
    bool gravityOn;
    neV3 gforce;
    neV3 cforce;
    neV3 ctorque;
    neV3 totalTorque;
    neV3 totalForce;
    neV3 acc;

    neReal linearDamp;
    neReal angularDamp;

    uint32_t curState;

    neRigidBodyState stateBuffer[MAX_RB_STATES];

    neRigidBodyDerive derive;

    s32 lowEnergyCounter;

    // constraints
    neStackInfo *stackInfo;

    bool isShifted;

    bool isShifted2;

    neController *controllers;

    neControllerItem *controllerCursor;

    bool needRecalc;

    bool isAddedToSolver;

    neRBExtra *rbExtra;

    neRBExtra eggs;

    neRigidType subType;

    neQ totalRot;

    neV3 totalTrans;

    s32 rotCount;

    s32 transCount;

    neQ totalLastRot;

    s32 lastRotCount;

    neV3 totalLastTrans;

    s32 lastTransCount;

    bool needSolveContactDynamic;

    neV3 totalDV;

    neV3 totalDA;

    s32 impulseCount;

    s32 twistCount;

    neV3 dvRecord[NE_RB_MAX_PAST_RECORDS];
    neV3 davRecord[NE_RB_MAX_PAST_RECORDS];

    neCollisionResult *maxErrCResult;

    neReal sleepingParam;

    neV3 oldPosition;

    neQ oldRotation;

    neV3 oldVelocity;

    neV3 oldAngularVelocity;

    s32 oldCounter;

    //////////////////////////////////////////////////

    NEINLINE bool IsParticle() {
        return (subType == NE_RIGID_PARTICLE);
    }

    NEINLINE neRestRecord &GetRestRecord(s32 index) {
        ASSERT(rbExtra);
        return rbExtra->restOnRecord[index];
    }

    NEINLINE neRestHull &GetRestHull() {
        ASSERT(rbExtra);
        return rbExtra->restHull;
    }

    NEINLINE neV3 &GetVelRecord(s32 index) {
        ASSERT(rbExtra);
        return rbExtra->velRecords[index];
    }

    NEINLINE neV3 &GetAngVelRecord(s32 index) {
        ASSERT(rbExtra);
        return rbExtra->angVelRecords[index];
    }

    NEINLINE void SetMass(neReal newMass) {
        mass = newMass;
        oneOnMass = 1.0f / newMass;
    }

    NEINLINE neReal GetMass() const {
        return mass;
    }

    NEINLINE void SetVelocity(const neV3 &v) {
        Derive().linearVel = v;
        WakeUpAllJoint();
    }

    NEINLINE neV3 GetVelocity() {
        return Derive().linearVel;
    }

    NEINLINE bool Active() const {
        return isActive;
    }

public:
    neRigidBody_();

    ~neRigidBody_();

    NEINLINE neRigidBodyState &State() {
        return stateBuffer[curState];
    };

    NEINLINE void SetPos(const neV3 &newPos) {
        State().b2w.pos = newPos;
    }

    NEINLINE neV3 GetPos() {
        return State().b2w.pos;
    }

    NEINLINE neRigidBodyDerive &Derive() {
        return derive;
    }

    void RecalcInertiaTensor();

    void SetAngMom(const neV3 &am);

    void GravityEnable(bool yes);

    void CheckForIdle();

    void CheckForIdleNonJoint();

    void CheckForIdleJoint();

    void BecomeIdle();

    void ZeroMotion();

    void WakeUp();

    bool AddStackInfo(neRestRecord &rc);

    void FreeStackInfo();

    bool NewStackInfo(neRestRecord &rc);

    void MigrateNewHeader(neStackHeader *newHeader, neStackHeader *curHeader);

    void ResetRestOnRecords();

    void RemoveStackInfo();

    bool NewStackInfoTerminator(neStackHeader *header);

    s32 AddContactImpulseRecord(bool withConstraint);

    bool IsRestPointStillValid();

    void AddRestContact(neRestRecord &rc);

    s32 CheckContactValidity();

    void ResolveRestingPenetration();

    bool IsConstraintNeighbour(neRigidBodyBase *otherBody);

    neController *AddController(neRigidBodyControllerCallback *rbc, s32 period);

    void BeginIterateController();

    neController *GetNextController();

    void Free();

    void DrawCPointLine();

    void SetAngMomComponent(const neV3 &angMom, const neV3 &dir);

    void ShiftPosition(const neV3 &delta);

    void UpdateAABB();

    s32 CheckRestHull();

    bool ApplyCollisionImpulse(const neV3 &impulse, const neV3 &contactPoint, neImpulseType itype);

    neV3 GetCorrectRotation(neRigidBody_ *otherBody, neReal massOther, neV3 &pointThis, neV3 &pointOther);

    void CorrectPosition(neV3 &pointThis, neV3 &pointDest, s32 flag, s32 changeLast);

    void CorrectRotation(neReal massOther, neV3 &pointThis, neV3 &pointDest, neV3 &pointDest2, s32 flag, s32 changeLast);

    void CorrectPenetrationDrift();

    void CorrectPenetrationDrift2(s32 index, bool slide, s32 flag);

    neReal TestImpulse(neV3 &dir, neV3 &pt, neReal &linear, neReal &angular);

    void UpdateDerive();

    void AddContactConstraint();

    void CorrectPenetrationRotation();

    void CorrectPenetrationTranslation();

    void CorrectPenetrationRotation2(s32 index, bool slide);

    void CorrectPenetrationTranslation2(s32 index, bool slide);

    bool CheckStillIdle();

    bool CheckHighEnergy();

    bool TestWakeUpImpulse(const neV3 &impulse);

    void MidPointIntegration(const neV3 &totalTorque, neReal tStep);

    void ImprovedEulerIntegration(const neV3 &totalTorque, neReal tStep);

    void RungeKutta4Integration(const neV3 &totalTorque, neReal tStep);

    void WakeUpAllJoint();

    void ApplyLinearConstraint();

    void ApplyAngularConstraint();

    void ConstraintDoSleepCheck();

    bool CheckStationary();

    void SyncOldState();

    bool AllRestRecordInvalid();

    NEINLINE neQ GetRotationQ() {
        return State().q;
    }

protected:

    void AdvanceDynamic(neReal tStep);

    void AdvancePosition(neReal tStep);

    void IsCollideWith(neRigidBody_ &rb);

    void UpdateController();
};

NEINLINE neV3 neRigidBodyBase::GetLinearVelocity() {
    if (AsCollisionBody()) {
        neV3 v;

        v.SetZero();

        return v;
    } else {
        return AsRigidBody()->Derive().linearVel;
    }
}

NEINLINE neV3 neRigidBodyBase::GetAngularVelocity() {
    if (AsCollisionBody()) {
        neV3 v;

        v.SetZero();

        return v;
    } else {
        return ((neRigidBody_ *) this)->Derive().angularVel;
    }
}

NEINLINE neV3 neRigidBodyBase::VelocityAtPoint(const neV3 &pt) {
    neV3 ret;

    if (AsCollisionBody()) {
        ret.SetZero();

        return ret;
    } else {
        ret = ((neRigidBody_ *) this)->Derive().linearVel;

        ret += ((neRigidBody_ *) this)->Derive().angularVel.Cross(pt);

        return ret;
    }
}

/*
NEINLINE neV3 neRigidBodyBase::VelocityAtPoint(const neV3 & pt)
{
	neV3 ret;

	if (AsCollisionBody())
	{
		ret.SetZero();

		return ret;
	}
	else
	{
		//ret = ((neRigidBody_*)this)->Derive().linearVel;

		ret = (((neRigidBody_*)this)->State().pos() - ((neRigidBody_*)this)->correctionInfo.lastPos);

		ret *= (1.0f / sim->currentTimeStep);

		ret += ((neRigidBody_*)this)->Derive().angularVel.Cross(pt);

		return ret;
	}
}
*/
#define TILT_TOLERANCE 0.9f

#endif
