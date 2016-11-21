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

#ifndef COLLISION_H
#define COLLISION_H

typedef enum {
    IMPULSE_IGNORE,
    IMPULSE_NORMAL,
    IMPULSE_CONTACT,
    IMPULSE_CONSTRAINT,
    IMPULSE_SLIDER,
    IMPULSE_SLIDER_LIMIT_PRIMARY,
//	IMPULSE_LIMIT,
            IMPULSE_ANGULAR_LIMIT_PRIMARY,
    IMPULSE_ANGULAR_LIMIT_SECONDARY,
    IMPULSE_ANGULAR_MOTOR_PRIMARY,
    IMPULSE_ANGULAR_MOTOR_SECONDARY,
    IMPULSE_RELATIVE_LINEAR_VELOCITY,
} neImpulseType;

class neRigidBodyBase;

class neRigidBody_;

///////////////////////////////////////////////////////////////////
//
//	Collision Model
//
//
///////////////////////////////////////////////////////////////////

struct TCollisionResult {
    neV3 point[2];        // closest point in world space, but relative to object centre
    neV3 normal;            // toward feature on body A

    neReal distance;
    bool penetrated;

    int matIndex[2];
};

///////////////////////////////////////////////////////////////////

//typedef struct neBox neBox; 

struct neBox {
    neV3 boxSize; //half of dimensions
    //neReal boxSize[4];
};

struct neTri {
    s32 indices[3];
};

struct neTriangleTerrain {
    neSimpleArray<s32> *triIndex;
    neArray<neTriangle_> *triangles;
};

struct neSphere {
    neReal radius;
    neReal radiusSq;
};

struct neCylinder {
    neReal radius;
    neReal radiusSq;
    neReal halfHeight;
};

struct neConvexMesh {
    neV3 *vertices;
    s32 *neighbours;
    s32 vertexCount;
};

struct neConvexDCD {
    neByte *convexData;
    neV3 *vertices;
    s32 numVerts;
};

#ifdef USE_OPCODE

struct neOPCMesh
{
    Opcode::OPCODE_Model * opmodel;
    IceMaths::Point * vertices;
    u32 vertCount;
    IndexedTriangle * triIndices;
    u32 triCount;
};

#endif


struct neBreakInfo {
    neV3 inertiaTensor;
    neV3 breakPlane;
    neReal mass;
    neReal breakMagnitude;
    neReal breakAbsorb;
    neReal neighbourRadius;
    neGeometry::neBreakFlag flag; //break all,
};


struct TConvex {
    enum {
        POINT,
        LINE,
        TRIANGLE,
        BOX,
        SPHERE,
        CYLINDER,
        TERRAIN,
        CONVEXITY,
        CONVEXDCD,
        OPCODE_MESH,
    };

    union {
        neBox box;
        neTri tri;
        neTriangleTerrain terrain;
        neSphere sphere;
        neCylinder cylinder;
        neConvexMesh convexMesh;
        neConvexDCD convexDCD;
#ifdef USE_OPCODE
        neOPCMesh opcodeMesh;
#endif
    } as;

    neT3 c2p;    // convex to physics object
    neReal boundingRadius;
    neReal envelope;
    uint32_t type;
    s32 matIndex;
    void * userData;
    neBreakInfo breakInfo;
    neV3 *vertices;

    void SetBoxSize(neReal width, neReal height, neReal depth);

    void SetSphere(neReal radius);

    void SetTriangle(s32 a, s32 b, s32 c, neV3 *vertices);

    void SetTerrain(neSimpleArray<s32> &triangleIndex, neArray<neTriangle_> &triangles, neV3 *vertices);

    void SetConvexMesh(neByte *convexData);

#ifdef USE_OPCODE

    void	SetOpcodeMesh(IndexedTriangle * triIndex, u32 triCount, IceMaths::Point * vertArray, u32 vertCount);

#endif

    void SetTransform(neT3 &t3);

    neT3 GetTransform();

    void SetUserData(void * ud) {
        userData = ud;
    }

    void * GetUserData() {
        return userData;
    }

    void SetMaterialId(s32 id);

    s32 GetMaterialId();

    neReal GetBoundRadius();

    uint32_t GetType();

    void Initialise();

    neM3 CalcInertiaTensor(neReal density, neReal &mass);

    void GetExtend(neV3 &minExt, neV3 &maxEnt);

    //quick access functions
    NEINLINE neReal BoxSize(s32 dir) {
        ASSERT(type == BOX);

        return as.box.boxSize[dir];
    }

    NEINLINE neReal Radius() {
        ASSERT(type == SPHERE);

        return as.sphere.radius;
    }

    NEINLINE neReal RadiusSq() {
        ASSERT(type == SPHERE);

        return as.sphere.radiusSq;
    }

    NEINLINE neReal CylinderRadius() {
        ASSERT(type == CYLINDER);

        return as.cylinder.radius;
    }

    NEINLINE neReal CylinderRadiusSq() {
        ASSERT(type == CYLINDER);

        return as.cylinder.radiusSq;
    }

    NEINLINE neReal CylinderHalfHeight() {
        ASSERT(type == CYLINDER);

        return as.cylinder.halfHeight;
    }
};

class neSensor_ {
public:
    neV3 pos;

    neV3 dir;

    neV3 dirNormal;

    neReal length;

    void * cookies;

    //results

    neV3 normal;

    neV3 contactPoint;

    neReal depth;

    s32 materialID;

    neRigidBodyBase *body;

public:

    neSensor_() {
        pos.SetZero();
        dir.SetZero();
        cookies = nullptr;
        normal.SetZero();
        depth = 0;
        materialID = 0;
        body = nullptr;
    }
};

/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neCollision
*
*	Desc:
*
****************************************************************************/

using TConvexItem = neFreeListItem<TConvex>;

class neCollision {
public:
    neCollision() {
        convex = nullptr;
        convexCount = 0;
        boundingRadius = 0.0f;
        obb.SetBoxSize(1.0f, 1.0f, 1.0f);
    }

//	void		SeTConvex(TConvex * con, s32 count)
//	{
//		convex = con;
//		convexCount = count+1;
//	}

    void CalcBB();

public:
    TConvex obb;
    TConvex *convex;
    s32 convexCount;
    neReal boundingRadius;
};

class neCollisionResult;

using neCollisionResultHandle = neCollection<neCollisionResult>::itemType;

class neCollisionResult {
PLACEMENT_MAGIC

public:
    //neCollisionResultHandle bodyAHandle;
    //neCollisionResultHandle bodyBHandle;

    neV3 contactA;
    neV3 contactB;

    neV3 contactABody; // or relative tangential velocity
    neV3 contactBBody; // or angular limit axis

    neV3 contactAWorld;
    neV3 contactBWorld;

    neM3 collisionFrame;

    neM3 w2c;

    neV3 initRelVelWorld;
    neV3 initRelVel;
    neV3 finaltRelVel;

    s32 materialIdA;
    s32 materialIdB;
    neReal depth; //+ve
    bool penetrate;

    neRigidBodyBase *bodyA;
    neRigidBodyBase *bodyB;

    neReal relativeSpeed;
    neReal finalRelativeSpeed;

    TConvex *convexA;
    TConvex *convexB;

    neM3 k;

    neM3 kInv;

    neReal impulseScale;

    neImpulseType impulseType;

    bool flag;

    void UpdateConstraintRelativeSpeed();

    void StartStage2();

    void PrepareForSolver(bool aIdle = false, bool bIdle = false);

    void CalcCollisionMatrix(neRigidBody_ *ba, neRigidBody_ *bb, bool isWorld);

    void CalcCollisionMatrix2(neRigidBody_ *ba, neRigidBody_ *bb);

    void CalcCollisionMatrix3(neRigidBody_ *ba, neRigidBody_ *bb);

    neReal SolveContact(neSimulator *sim);

    neReal SolveConstraint(neSimulator *sim);

    neReal SolveSlider(neSimulator *sim);

    neReal SolveSliderLimit(neSimulator *sim);

    neReal SolveAngularPrimary(neSimulator *sim);

    neReal SolveAngularSecondary(neSimulator *sim);

    neReal SolveAngularMotorPrimary(neSimulator *sim);

    neReal SolveRelativeLinear(neSimulator *sim);

    neReal SolveAngular(neReal depth, const neV3 &axis, neReal relAV, neSimulator *sim);

    neReal SolveAngular2(const neV3 &axisA, const neV3 &axisB, neReal relAV, neReal desireAV, neReal depth,
                      neSimulator *sim);

    neReal SolveAngular3(neReal depth, const neV3 &axis, neReal relAV, neSimulator *sim);

    void CalcError(neSimulator *sim);
//	void AddToBodies();

    neReal Value() {
        return relativeSpeed;
    };

    bool CheckIdle();

    void Swap() {
        collisionFrame[2] *= -1.0f;

        neSwap(contactA, contactB);

        neSwap(convexA, convexB);
    }
};

void CollisionTest(neCollisionResult &result, neCollision &colA, neT3 &transA, neCollision &colB, neT3 &transB,
                   const neV3 &backupVector);

void CollisionTestSensor(TConvex *obbA, neSensor_ *sensorsA, neT3 &transA, neCollision &colB, neT3 &transB,
                         neRigidBodyBase *body);

void ConvexCollisionTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB, neT3 &transB,
                         const neV3 &backupVector);

void Box2BoxTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB, neT3 &transB,
                 const neV3 &backupVector);

void Box2TriangleTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB, neT3 &transB);

void Box2TerrainTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB);

void Box2SphereTest(neCollisionResult &result, TConvex &boxA, neT3 &transA, TConvex &sphereB, neT3 &transB);

void Box2CylinderTest(neCollisionResult &result, TConvex &boxA, neT3 &transA, TConvex &sphereB, neT3 &transB);

void Sphere2TerrainTest(neCollisionResult &result, TConvex &sphereA, neT3 &transA, TConvex &terrainB);

void Sphere2SphereTest(neCollisionResult &result, TConvex &sphereA, neT3 &transA, TConvex &sphereB, neT3 &transB);

void Cylinder2CylinderTest(neCollisionResult &result, TConvex &cA, neT3 &transA, TConvex &cB, neT3 &transB);

void Cylinder2TerrainTest(neCollisionResult &result, TConvex &cylinderA, neT3 &transA, TConvex &terrainB);

void Cylinder2SphereTest(neCollisionResult &result, TConvex &cylinderA, neT3 &transA, TConvex &sphereB, neT3 &transB);

void Box2ConvexTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB, neT3 &transB,
                    const neV3 &backupVector);

void Convex2ConvexTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB, neT3 &transB,
                       const neV3 &backupVector);

void TranslateCOM(neM3 &I, neV3 &translate, neReal mass, neReal factor);

void DiagonalizeMassTensor(neM3 &I, neV3 &diagonal, neM3 &eigenVectors);

void SensorTest(neSensor_ &sensorA, TConvex &convexB, neT3 &transB);

#ifdef USE_OPCODE

void Box2OpcodeTest(neCollisionResult & result, TConvex & convexA, neT3 & transA, TConvex & convexB, neT3 & transB);

void Sphere2OpcodeTest(neCollisionResult & result, TConvex & convexA, neT3 & transA, TConvex & convexB, neT3 & transB);

void Cylinder2OpcodeTest(neCollisionResult & result, TConvex & convexA, neT3 & transA, TConvex & convexB, neT3 & transB);

void Opcode2TerrainTest(neCollisionResult & result, TConvex & convexA, neT3 & transA, TConvex & convexB);

void Opcode2OpcodeTest(neCollisionResult & result, TConvex & convexA, neT3 & transA, TConvex & convexB, neT3 & transB);

#endif //USE_OPCODE

#endif




















