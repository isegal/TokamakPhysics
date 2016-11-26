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

#ifndef SIMULATOR_H
#define SIMULATOR_H

#define DEFAULT_CONSTRAINT_EPSILON 0.001f

#define DEFAULT_CONSTRAINT_ITERATION 4

//#define _DEBUG_REGION

#include "stack.h"
#include <functional>

///////////////////////////////////////////////////////////////////
//
//	Simulator
//
//
///////////////////////////////////////////////////////////////////
/*
class neGravityController: public neController
{
public:
	neV3 gravity;

	void ControllerUpdate(neRigidBody & rb)
	{
		neV3 gforce = gravity;

		gforce *= rb.GetMass();

		rb.ApplyForceCOG(gforce);
	}
};
*/

class neFastImpulse {
public:
    neRigidBody *bodyA;
    neRigidBody *bodyB;
    neV3 contactA;
    neV3 contactB;
    neM3 k;
    neM3 kInv;
    neV3 initRelVel;
    neM3 collisionFrame;
    neM3 w2c;
    neReal relativeSpeedSq;

public:
    void Init();

    void Update();

    void Apply(neReal scale);
};


struct nePhysicsMaterial {
    neReal friction;
    neReal resititution;
    neReal density;
};


/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neRegion
*
*	Desc:
*
****************************************************************************/
class neRegion;

class neCoordList {
public:
    neDLinkList<CCoordListEntry> coordList;

    void Add(neRigidBodyBase *bb, neRigidBodyBase *hint, s32 hintCoord);

    bool Reserve(s32 size, neAllocatorAbstract *all = nullptr) {
        return coordList.Reserve(size, all);
    }

    void Sort(bool sortOnly);

#ifdef _DEBUG

    void OuputDebug();

#endif

    s32 dim;

    neByte dimPower2;

    neRegion *region;
};

struct neOverlappedPair {
PLACEMENT_MAGIC

    neRigidBodyBase *bodyA;
    neRigidBodyBase *bodyB;
};

struct neOverlapped {
    neByte status;
    neOverlappedPair *pairItem;
};

struct neAddBodyInfo {
    neRigidBodyBase *body;

    neRigidBodyBase *hint;
};

class neRegion {
public:
    neRegion() {}

//	enum {
//		MAX_OVERLAPPED = 100000,
//	};
    enum {
        SORT_DIMENSION_X = 1,
        SORT_DIMENSION_Y = 2,
        SORT_DIMENSION_Z = 4,
    };

    void Initialise(neSimulator *s, neByte sortD = (SORT_DIMENSION_X | SORT_DIMENSION_Y));

    bool AddBody(neRigidBodyBase *bb, neRigidBodyBase *hint);

    void InsertCoordList(neRigidBodyBase *bb, neRigidBodyBase *hint);

    void RemoveBody(neRigidBodyBase *bb);

    void Update();

    void Rebuild();

    neOverlapped *GetOverlappedStatus(neRigidBodyBase *a, neRigidBodyBase *b);

    void ToggleOverlapStatus(neRigidBodyBase *a, neRigidBodyBase *b, neByte dimp2);

    void ResetOverlapStatus(neRigidBodyBase *a, neRigidBodyBase *b);

    void MakeTerrain(neTriangleMesh *tris);

    void FreeTerrain();

    neTriangleTree &GetTriangleTree() { return terrainTree; }

    ~neRegion();

public:
    neByte sortDimension;

    neSimulator *sim;

    s32 maxRigidBodies;

    s32 maxAnimBodies;

    s32 totalBodies;

    s32 maxParticle;

//	neArray<neOverlapped> rb2rb;

//	neArray<neOverlapped> rb2ab;

    neArray<neOverlapped> b2b;

    neArray<neOverlapped> b2p;

    neSimpleArray<neAddBodyInfo> newBodies;

    neDLinkList<neRigidBodyBase *> bodies;

    neDLinkList<neOverlappedPair> overlappedPairs;

    neCoordList coordLists[3];

    neTriangleTree terrainTree;

#ifdef _DEBUG_REGION
    bool debugOn;
#endif
};


/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neCollisionTable_
*
*	Desc:
*
****************************************************************************/

class neCollisionTable_ {
public:
    enum {
        NE_COLLISION_TABLE_MAX = neCollisionTable::NE_COLLISION_TABLE_MAX,
    };

    neCollisionTable_();

    ~neCollisionTable_();

    void Set(s32 collisionID1, s32 collisionID2, neCollisionTable::neReponseBitFlag flag);

    neCollisionTable::neReponseBitFlag Get(s32 collisionID1, s32 collisionID2);

    s32 GetMaxCollisionID() {
        return NE_COLLISION_TABLE_MAX;
    };

public:
    neCollisionTable::neReponseBitFlag table[NE_COLLISION_TABLE_MAX][NE_COLLISION_TABLE_MAX];

    neCollisionTable::neReponseBitFlag terrainTable[NE_COLLISION_TABLE_MAX];
};

class nePerformanceData {
public:
    static nePerformanceData *Create();

    nePerformanceData() {
        Reset();
    }

    void Reset() {
        dynamic = 0.0f;
        position = 0.0f;
        constrain_1 = 0.0f;
        constrain_2 = 0.0f;
        cd = 0.0f;
        cdCulling = 0.0f;
        terrain = 0.0f;
        terrainCulling = 0.0f;
        controllerCallback = 0.0f;
    }

    void Start();

    void Init();

    neReal GetCount();

    neReal GetTotalTime() {
        return dynamic +
               position +
               constrain_1 +
               constrain_2 +
               cd +
               cdCulling +
               terrain +
               terrainCulling +
               controllerCallback;
    };

    void UpdateDynamic();

    void UpdatePosition();

    void UpdateConstrain1();

    void UpdateConstrain2();

    void UpdateCD();

    void UpdateCDCulling();

    void UpdateTerrain();

    void UpdateTerrainCulling();

    void UpdateControllerCallback();

    neReal dynamic;

    neReal position;

    neReal controllerCallback;

    neReal constrain_1;

    neReal constrain_2;

    neReal cdCulling;

    neReal cd;

    neReal terrain;

    neReal terrainCulling;

    s32 perfFreqAdjust; // in case Freq is too big

    s32 overheadTicks;   // overhead  in calling timer
};

class neSimulator {
private:
    neSimulator(const neSimulatorSizeInfo &_sizeInfo, neAllocatorAbstract *alloc = nullptr,
                const neV3 *grav = nullptr);
    ~neSimulator();

public:
    friend class neRegion;

    enum {
        MAX_MATERIAL = 256,
    };

    typedef enum {
        LOG_OUTPUT_LEVEL_NONE = 0,
        LOG_OUTPUT_LEVEL_ONE,
        LOG_OUTPUT_LEVEL_FULL,
    } LOG_OUTPUT_LEVEL;

    static neSimulator *CreateSimulator(const neSimulatorSizeInfo &sizeInfo, neAllocatorAbstract *alloc = nullptr,
                                        const neV3 *grav = nullptr);
    static void DestroySimulator(neSimulator *sim);

    void Initialise(const neV3 &gravity);

    neRigidBody *CreateRigidBody(bool isParticle = false);

    neRigidBody *CreateRigidBodyFromConvex(TConvex *convex, neRigidBodyBase *originalBody);

    neCollisionBody *CreateCollisionBody();

    void Free(neRigidBodyBase *bb);

    void Advance(neReal time, size_t nStep, nePerformanceReport *_perfReport = nullptr);

    void Advance(neReal time, neReal minTimeStep, neReal maxTimeStep, nePerformanceReport *_perfReport = nullptr);

    void Advance(nePerformanceReport *_perfReport = nullptr);

    void ForeachActiveRB(const std::function<bool(neRigidBody&)>& rbCallback);

    bool SetMaterial(s32 index, neReal friction, neReal restitution, neReal density);

    bool GetMaterial(s32 index, neReal &friction, neReal &restitution, neReal &density);

    neReal HandleCollision(neRigidBodyBase *bodyA, neRigidBodyBase *bodyB, neCollisionResult &cresult,
                        neImpulseType impulseType, neReal scale = 0.0f);

    void CollisionRigidParticle(neRigidBody *ba, neRigidBody *bb, neCollisionResult &cresult);

    void SimpleShift(const neCollisionResult &cresult);

    void RegisterPenetration(neRigidBodyBase *bodyA, neRigidBodyBase *bodyB, neCollisionResult &cresult);

    void SetTerrainMesh(neTriangleMesh *tris);

    void FreeTerrainMesh();

    void
    CreatePoint2PointConstraint(neRigidBodyBase *bodyA, const neV3 &pointA, neRigidBodyBase *bodyB, const neV3 &pointB);

    neStackHeader *NewStackHeader(neStackInfo *);

    neConstraintHeader *NewConstraintHeader();

    void CheckStackHeader();

    neLogOutputCallback *SetLogOutputCallback(neLogOutputCallback *fn);

    neCollisionCallback *SetCollisionCallback(neCollisionCallback *fn);

    void LogOutput(LOG_OUTPUT_LEVEL);

    void SetLogOutputLevel(LOG_OUTPUT_LEVEL lvl);

    void UpdateConstraintControllers();

    void FreeAllBodies();

    void GetMemoryAllocated(s32 &memoryAllocated);

    bool CheckBreakage(neRigidBodyBase *originalBody, TConvex *convex, const neV3 &contactPoint, neV3 &impulse);

    void ResetTotalForce();

    void AdvanceDynamicRigidBodies();

    void AdvanceDynamicParticles();

    void AdvancePositionRigidBodies();

    void AdvancePositionParticles();

    void ApplyJointDamping();

    void ClearCollisionBodySensors();

    void UpdateAABB();

    //neReal SolveDynamicLocal(neCollisionResult * cr);

    neReal SolveLocal(neCollisionResult *cr);

    void AddContactConstraint(neReal &epsilon, s32 &iteration);

    void SetGravity(const neV3 &g);

    neV3 CalcNormalImpulse(neCollisionResult &cresult, bool isContact);

    void ResetStackHeaderFlag();

    void AddCollisionResult(neCollisionResult &cresult);

    neCollisionBody *GetTerrainBody() {
        return &fakeCollisionBody;
    }

public:
    neSimulatorSizeInfo sizeInfo;

    nePerformanceReport *perfReport;

    nePerformanceData *perf;

    neV3 gravity;

    neV3 gravityVector;

    neReal gravityMag;

    neReal restingSpeed;

    s32 stepSoFar;

    neReal _currentTimeStep;

    neReal oneOnCurrentTimeStep;

    neReal highEnergy;

//	neConstraintSolver solver;

    neDLinkList<neConstraintHeader> constraintHeaders;

    neDLinkList<_neConstraint> constraintHeap;

//	neDLinkList<neMiniConstraint> miniConstraintHeap;

    neDLinkList<neController> controllerHeap;

    neStackInfoHeap stackInfoHeap;

    neStackHeaderHeap stackHeaderHeap;

    neStackHeader stackHeaderX;

    neDLinkList<neSensor_> sensorHeap;

    neDLinkList<TConvex> geometryHeap;

    neSimpleArray<neByte *> pointerBuffer1;

    neSimpleArray<neByte *> pointerBuffer2;

    LOG_OUTPUT_LEVEL logLevel;

    s32 solverStage;

    bool solverLastIteration;

    static char logBuffer[256];

    neSimpleArray<neCollisionResult> cresultHeap;

    neSimpleArray<neCollisionResult> cresultHeap2;

    neConstraintHeader contactConstraintHeader;

    neReal magicNumber;

    s32 currentRecord;

    neReal timeFromLastFrame;

    neReal lastTimeStep;

protected:
    void CheckCollision();

    void CheckTerrainCollision();

    void SolveAllConstrain();

    void SolveOneConstrainChain(neReal epsilon, s32 iteration);

    void ResolvePenetration();

    void SolveContactConstrain();

    void CheckIfStationary();

    nePhysicsMaterial materials[MAX_MATERIAL];

public:

    neAllocatorAbstract *allocator;

    neAllocatorDefault allocDef;

//data

    size_t maxRigidBodies;

    size_t maxAnimBodies;

    size_t maxParticles;

    neDLinkList<neRigidBody> rigidBodyHeap;

    neDLinkList<neCollisionBody> collisionBodyHeap;

    neDLinkList<neRigidBody> rigidParticleHeap;

    neList<neRigidBody> activeRB;

    neList<neRigidBody> inactiveRB;

    neList<neCollisionBody> activeCB;

    neList<neCollisionBody> inactiveCB;

    neList<neRigidBody> activeRP;

    neList<neRigidBody> inactiveRP;

    neList<neCollisionResult> colResults;

    neRegion region;

    neCollisionTable_ colTable;

    neSimpleArray<neTreeNode *> treeNodes;

    neSimpleArray<s32> triangleIndex;

    neCollisionBody fakeCollisionBody;

//state
    bool buildCoordList;

//others
    neCollisionCallback *collisionCallback;

    neLogOutputCallback *logCallback;

    neBreakageCallback *breakageCallback;

    neTerrainTriangleQueryCallback *terrainQueryCallback;

    neCustomCDRB2RBCallback customCDRB2RBCallback;

    neCustomCDRB2ABCallback customCDRB2ABCallback;

    s32 idleBodyCount;
};

#endif
