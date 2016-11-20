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

#ifndef NE_MATH_H
#define NE_MATH_H

#ifdef USE_OPCODE

#include "Opcode.h"

#endif //USE_OPCODE

#include <math.h>
#include <float.h>
#include "ne_type.h"
#include "ne_debug.h"
#include "ne_smath.h"
//#include <xmmintrin.h>
/****************************************************************************
*
*	neV3
*
****************************************************************************/

static s32 neNextDim1[] = {1, 2, 0};
static s32 neNextDim2[] = {2, 0, 1};

using neQ = struct neQ;

using neM3 = struct neM3;

//struct __declspec(align(16)) neV3
struct neV3 {
public:

    neReal v[4];
/*
	union
	{
		neReal v[4];
		
		struct nTag
		{
			neReal X,Y,Z,W;
		}n;

		//__m128 m;
	};
*/
    NEINLINE neV3 &SetZero(void);

    NEINLINE neV3 &SetOne(void);

    NEINLINE neV3 &SetHalf(void);

    NEINLINE neV3 &Set(neReal value);

    NEINLINE neV3 &Set(neReal x, neReal y, neReal z);

    NEINLINE neV3 &Set(const neV3 &V);

    NEINLINE neV3 &Set(const neQ &Q);

    NEINLINE void Set(neReal val[3]);

    NEINLINE void Get(neReal val[3]);

    NEINLINE neReal &operator[](s32 I);

    NEINLINE neReal operator[](s32 I) const;

    NEINLINE neReal X() const;

    NEINLINE neReal Y() const;

    NEINLINE neReal Z() const;

    NEINLINE neReal W() const;

    NEINLINE void Normalize(void);

    NEINLINE neReal Length(void) const;

    NEINLINE neReal Dot(const neV3 &V) const;

    NEINLINE neV3 Cross(const neV3 &V) const;

    NEINLINE void RotateX(neRadian Rx);

    NEINLINE void RotateY(neRadian Ry);

    NEINLINE void RotateZ(neRadian Rz);

    NEINLINE neRadian GetPitch(void) const;

    NEINLINE neRadian GetYaw(void) const;

    NEINLINE void SetBoxTensor(neReal width, neReal height, neReal depth, neReal mass);

    NEINLINE void SetAbs(const neV3 &v);

    NEINLINE neReal GetDistanceFromLine(const neV3 &point1, const neV3 &point2);

    NEINLINE neReal GetDistanceFromLine2(neV3 &project, const neV3 &pointA, const neV3 &pointB);

    NEINLINE neReal GetDistanceFromLineAndProject(neV3 &result, const neV3 &startPoint, const neV3 &dir);

    NEINLINE bool GetIntersectPlane(neV3 &normal, neV3 &pointOnPlane, neV3 &point1, neV3 &point2);

    NEINLINE void SetMin(const neV3 &V1, const neV3 &V2);

    NEINLINE void SetMax(const neV3 &V1, const neV3 &V2);

    NEINLINE void RemoveComponent(const neV3 &V);

    NEINLINE bool IsConsiderZero() const;

    NEINLINE bool IsFinite() const;

    NEINLINE neV3 Project(const neV3 &v) const;

//	NEINLINE neV3 & operator = (const neV3& V);
    NEINLINE neV3 &operator/=(neReal S);

    NEINLINE neV3 &operator*=(neReal S);

    NEINLINE neV3 &operator+=(const neV3 &V);

    NEINLINE neV3 &operator-=(const neV3 &V);

    NEINLINE neV3 friend operator+(const neV3 &V1, const neV3 &V2);

    NEINLINE neV3 friend operator-(const neV3 &V1, const neV3 &V2);

    NEINLINE neV3 friend operator/(const neV3 &V, neReal S);

    NEINLINE neV3 friend operator*(const neV3 &V, neReal S);

    NEINLINE neV3 friend operator*(const neV3 &V1, const neV3 &V2);

    NEINLINE neV3 friend operator*(const neV3 &V, const neM3 &M);

    NEINLINE neM3 friend operator^(const neV3 &V, const neM3 &M); //cross product operator
    NEINLINE friend neV3 operator-(const neV3 &V);

    NEINLINE friend neV3 operator*(neReal S, const neV3 &V);

#ifdef USE_OPCODE

    NEINLINE neV3 & operator = (const IceMaths::Point & pt);
    NEINLINE IceMaths::Point& AssignIcePoint(IceMaths::Point & pt) const;

#endif //USE_OPCODE
};

/****************************************************************************
*
*	neV4
*
****************************************************************************/

struct neV4 {
    neReal X, Y, Z, W;

    // functions
    NEINLINE neV4(void);

    NEINLINE neV4(neReal x, neReal y, neReal z, neReal w);

    NEINLINE neV4(const neV3 &V, neReal w);

    NEINLINE neV4(const neV4 &V);

    NEINLINE void SetZero(void);

    NEINLINE void Set(neReal x, neReal y, neReal z, neReal w);

    NEINLINE neReal &operator[](s32 I);

    NEINLINE neV4 &operator/=(neReal S);

    NEINLINE neV4 &operator*=(neReal S);

    NEINLINE neV4 &operator+=(const neV4 &V);

    NEINLINE neV4 &operator-=(const neV4 &V);

    NEINLINE neV4 &Normalize(void);

    NEINLINE neReal Length(void) const;

    NEINLINE neReal Dot(const neV4 &V) const;

    NEINLINE friend neV4 operator-(const neV4 &V);

    NEINLINE friend neV4 operator*(neReal S, const neV4 &V);

    NEINLINE friend neV4 operator/(const neV4 &V, neReal S);

    NEINLINE friend neV4 operator*(const neV4 &V, neReal S);

    NEINLINE friend neV4 operator+(const neV4 &V1, const neV4 &V2);

    NEINLINE friend neV4 operator-(const neV4 &V1, const neV4 &V2);
};

/****************************************************************************
*
*	neM3
*
****************************************************************************/

struct neM3 {
    neV3 M[3];

    NEINLINE neV3 &operator[](s32 I);

    NEINLINE neV3 operator[](s32 I) const;

    NEINLINE void SetZero(void);

    NEINLINE void SetIdentity(void);

    NEINLINE bool SetInvert(const neM3 &rhs);

    NEINLINE neM3 &SetTranspose(neM3 &M);

    NEINLINE void GetColumns(neV3 &V1, neV3 &V2, neV3 &V3) const;

    NEINLINE void SetColumns(const neV3 &V1, const neV3 &V2, const neV3 &V3);

    NEINLINE neV3 GetDiagonal();

    NEINLINE neV3 TransposeMulV3(const neV3 &V);

    NEINLINE void RotateXYZ(const neV3 &rotate);

    NEINLINE neM3 &SkewSymmetric(const neV3 &V);

    NEINLINE bool IsIdentity() const;

    NEINLINE bool IsOrthogonalNormal() const;

    NEINLINE bool IsFinite() const;

    NEINLINE neM3 &operator+=(const neM3 &add);

    NEINLINE neM3 operator^(const neV3 &vec) const; //cross product
    NEINLINE neM3 operator*(neReal scalar) const;

    NEINLINE friend neM3 operator+(const neM3 &add1, const neM3 &add2);

    NEINLINE friend neM3 operator-(const neM3 &sub1, const neM3 &sub2);

    NEINLINE friend neM3 operator*(const neM3 &M1, const neM3 &M2);

    NEINLINE friend neV3 operator*(const neM3 &M1, const neV3 &V);

    NEINLINE friend neM3 operator*(neReal scalar, const neM3 &M);

    NEINLINE friend neM3 &operator*=(neM3 &M1, const neReal f);

    NEINLINE friend neM3 &operator/=(neM3 &M1, const neReal f);
};

/****************************************************************************
*
*	neM4
*
****************************************************************************/

struct neM4 {
    neReal M[4][4];

    // functions
    NEINLINE void
    Set(float row00, float row01, float row02, float row03, float row10, float row11, float row12, float row13,
        float row20, float row21, float row22, float row23, float row30, float row31, float row32, float row33);

    NEINLINE void Set(int row, int col, neReal val) { M[col][row] = val; }

    NEINLINE void SetZero(void);

    NEINLINE void SetIdentity(void);

    NEINLINE neV3 GetScale(void) const;

    NEINLINE neV3 GetTranslation(void) const;

    NEINLINE void SetTranslation(const neV3 &V);

    NEINLINE void SetScale(const neV3 &V);

    NEINLINE neReal &operator[](s32 I);

    NEINLINE neM4 &operator*=(const neM4 &M);

    NEINLINE neM4 &operator=(const neM3 &M);

    NEINLINE neV3 TransformAs3x3(const neV3 &V) const;

    NEINLINE void GetRows(neV3 &V1, neV3 &V2, neV3 &V3) const;

    NEINLINE void SetRows(const neV3 &V1, const neV3 &V2, const neV3 &V3);

    NEINLINE void GetColumns(neV3 &V1, neV3 &V2, neV3 &V3) const;

    NEINLINE void SetColumns(const neV3 &V1, const neV3 &V2, const neV3 &V3);

    NEINLINE void GetColumn(neV3 &V1, uint32_t col) const;

    NEINLINE void SetTranspose(const neM4 &M);

    NEINLINE void SetFastInvert(const neM4 &Src);

    NEINLINE friend neM4 operator*(const neM4 &M1, const neM4 &M2);

    NEINLINE friend neV3 operator*(const neM4 &M1, const neV3 &V);
};

/****************************************************************************
*
*	neQ
*
****************************************************************************/

struct neQ {
    neReal X, Y, Z, W;

    // functions
    NEINLINE neQ(void);

    NEINLINE neQ(neReal X, neReal Y, neReal Z, neReal W);

    NEINLINE neQ(const neM4 &M);

    NEINLINE void Zero(void);

    NEINLINE void Identity(void);

    NEINLINE void SetupFromMatrix(const neM4 &Matrix);

    NEINLINE void SetupFromMatrix3(const neM3 &Matrix);

    NEINLINE void GetAxisAngle(neV3 &Axis, neRadian &Angle) const;

    NEINLINE neM4 BuildMatrix(void) const;

    NEINLINE neM3 BuildMatrix3(void) const;

    NEINLINE neQ &Normalize(void);

    NEINLINE neReal Dot(const neQ &Q) const;

    NEINLINE neQ &Invert(void);

    NEINLINE bool IsFinite();

    NEINLINE neQ &operator*=(const neQ &Q);

    NEINLINE neQ &operator*=(neReal S);

    NEINLINE neQ &operator+=(const neQ &Q);

    NEINLINE neQ &operator-=(const neQ &Q);

    NEINLINE neQ &Set(neReal X, neReal Y, neReal Z, neReal W);

    NEINLINE neQ &Set(const neV3 &V, neReal W);

    NEINLINE neQ &Set(neReal angle, const neV3 &axis);

    NEINLINE friend neQ operator-(const neQ &V);

    NEINLINE friend neQ operator*(const neQ &Qa, const neQ &Qb);

    NEINLINE friend neV3 operator*(const neQ &Q, const neV3 &V);

    NEINLINE friend neQ operator*(const neQ &Q, neReal S);

    NEINLINE friend neQ operator*(neReal S, const neQ &Q);

    NEINLINE friend neQ operator+(const neQ &Qa, const neQ &Qb);

    NEINLINE friend neQ operator-(const neQ &Qa, const neQ &Qb);
};

/****************************************************************************
*
*	neT3
*
****************************************************************************/

struct neT3 {
public:
    neM3 rot;
    neV3 pos;

    NEINLINE neT3 FastInverse();

    NEINLINE neT3 operator*(const neT3 &t);

    NEINLINE neV3 operator*(const neV3 &v);

    NEINLINE bool IsFinite();

public:
    NEINLINE void MakeD3DCompatibleMatrix() {
        rot[0].v[3] = 0.0f;
        rot[1].v[3] = 0.0f;
        rot[2].v[3] = 0.0f;
        pos.v[3] = 1.0f;
    }

    NEINLINE void SetIdentity() {
        rot.SetIdentity();
        pos.SetZero();
/*
 *		additional code to make this binary compatible with rendering matrix
 */
        MakeD3DCompatibleMatrix();
    }

#ifdef USE_OPCODE

    NEINLINE neT3 & operator = (const IceMaths::Matrix4x4 & mat);
    NEINLINE IceMaths::Matrix4x4 & AssignIceMatrix(IceMaths::Matrix4x4 & mat) const;

#endif //USE_OPCODE
};

///////////////////////////////////////////////////////////////////////////
// INCLUDE INLINE HEADERS
///////////////////////////////////////////////////////////////////////////

#include "ne_math_misc_inline.h"
#include "ne_math_v3_inline.h"
#include "ne_math_v4_inline.h"
#include "ne_math_m4_inline.h"
#include "ne_math_m3_inline.h"
#include "ne_math_q_inline.h"
#include "ne_math_t3_inline.h"

///////////////////////////////////////////////////////////////////////////
// END
///////////////////////////////////////////////////////////////////////////
#endif //NE_MATH_H
