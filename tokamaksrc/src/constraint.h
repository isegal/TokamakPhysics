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

class _neConstraint;

class neRigidBodyBase;

class neRigidBody;

class neJointSolver;

class neSimulator;

/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neJointPoint
*
*	Desc:
*
****************************************************************************/

class neJointPoint {
public:
    neV3 points[2];

    neV3 &PtBody() { return points[0]; }

    neV3 &PtWorld() { return points[1]; }
};


/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: _neConstraint
*
*	Desc:
*
****************************************************************************/
class neController;

using neControllerItem = neFreeListItem<neController>;

using neConstraintHandle = neCollection<_neConstraint>::itemType;

struct neLimitState {
    s32 limitType;

    bool applyLimitImpulse;

    neV3 limitAxis;

    neV3 limitNormalA;

    neV3 limitNormalB;

    bool lowerLimitOn; // false means upper limit

    bool enableLimit;

    neReal limitAngularPenetration;

    neReal limitAngularPenetration2;

    neReal upperLimit;

    neReal lowerLimit;

    _neConstraint *constr;

    void Reset(_neConstraint *c, s32 ltype = 0) {
        constr = c;

        applyLimitImpulse = false;

        limitType = ltype;

        enableLimit = false;

        upperLimit = 0.0f;

        lowerLimit = 0.0f;
    }

    void CheckLimitPrimary();

    void CheckLimitSecondary();

    void CheckLimitPrimarySlider();
};

struct neMotor {
    bool enable;

    neJoint::MotorType motorType;

    neReal desireVelocity;

    neReal maxForce;

    neV3 axis;//for ball joint

    void Reset() {
        enable = false;

        motorType = neJoint::NE_MOTOR_SPEED;

        desireVelocity = 0.0f;

        maxForce = 0.0f;

        axis.Set(1.0f, 0.0f, 0.0f);
    }

    void PrepareForSolver(_neConstraint *constraint);
};

class _neConstraint {
public:
    enum {
        NE_CPOINT_TYPE_BODY,
        NE_CPOINT_TYPE_WORLD,
        NE_CPOINT_TYPE_RESULT,
    };
    neT3 frameA;
    neT3 frameB;

    neT3 frameAWorld;
    neT3 frameBWorld;

    neSimulator *sim;

    neController *controllers;

    neControllerItem *controllerCursor;

    neRigidBody *bodyA;

    neRigidBodyBase *bodyB;

    neConstraintHandle bodyAHandle;
    neConstraintHandle bodyBHandle;

    s32 pointCount;

    neJointPoint cpointsA[2];
    neJointPoint cpointsB[2];
/*	
	neV3 cpointResults[2][2];

	neReal clength[2];
	neReal clengthSq[2];
*/
    bool enable;

    bool infiniteMassB;

    neReal accuracy;

    s32 iteration;

    neReal jointLength;

    neReal pos;

    neReal pos2;

    neReal jointDampingFactor;

    bool alreadySetup;
    /*

      apply limit

    */
    neLimitState limitStates[2];

    neMotor motors[2];

    neJoint::ConstraintType type;

    void Enable(bool yes);

    void GeneratePointsFromFrame();

    void Reset();

    void SetType(neJoint::ConstraintType t);

//	neV3 * GetPoint(neRigidBodyBase * body, s32 index, s32 ptType);

//	void ChooseRigidConstraints();

    void UpdateConstraintPoint();

//	neReal ApplyConstraintImpulse(neSimulator & sim);

    void InfiniteMassB(bool yes);

    void AddToRigidBody();

    neController *AddController(neJointControllerCallback *jc, s32 period);

    void BeginIterateController();

    neController *GetNextController();

    void UpdateController();

    void FindGreatest();

    neRigidBodyBase *GetNotThisBody(neRigidBody *notThis) {
        if (bodyA == notThis) {
            return bodyB;
        } else {
            return (neRigidBodyBase *) bodyA;
        }
    }

    void CheckLimit();

    void DrawCPointLine();

    neT3 GetFrameBWorld();

    neT3 GetBodyB2W();

    void SetupLimitCollision();

    void UpdateCurrentPosition();

    void ApplyDamping();

//	void SolveAngularConstraint();
};

using neCBodyItem = neFreeListItem<neRigidBodyBase *>;

class neConstraintHeader {
public:
    enum {
        FLAG_NONE = 0,
        FLAG_NEED_SETUP = 1,
        FLAG_NEED_REORG = 2,
    };

    _neConstraint *head;
    _neConstraint *tail;

    bool solved;

    s32 flag;

    neCollection<neRigidBodyBase> bodies;

    neConstraintHeader() { Reset(); }

    void AddToSolver(neReal &epsilon, s32 &iteration);

    void Reset() {
        head = nullptr;

        tail = nullptr;

        flag = FLAG_NEED_SETUP;

        solved = false;

        bodies.Reset();
    }

    void Add(_neConstraint *c) {
        if (tail) {
            ((neFreeListItem<_neConstraint> *) tail)->Append((neFreeListItem<_neConstraint> *) c);

            tail = c;
        } else {
            head = tail = c;

            ((neFreeListItem<_neConstraint> *) c)->Solo();
        }
    }

    void Remove(_neConstraint *c) {
        neFreeListItem<_neConstraint> *item = (neFreeListItem<_neConstraint> *) c;

        if (c == head) {
            head = (_neConstraint *) item->next;
        }
        if (c == tail) {
            tail = (_neConstraint *) item->prev;
        }
        item->Remove();
    }
    //void Purge(neSimulator * sim);

    void TraverseApplyConstraint(bool autoSleep);

    bool StationaryCheck();

    void BecomeIdle(bool checkResting = false);

    void WakeUp();

    void RemoveAll();
};

