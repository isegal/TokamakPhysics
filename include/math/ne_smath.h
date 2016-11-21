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

#ifndef NE_SMATH_H
#define NE_SMATH_H

#include "ne_type.h"
#include <cmath>

#define NE_PI    (3.141592653589793238462643f)

#define NE_RAD_TO_DEG(A) ((neReal)(((A) * (180.0f / NE_PI))))
#define NE_DEG_TO_RAD(A) ((neReal)(((A) * (NE_PI / 180.0f))))
#define NE_RI              NE_DEG_TO_RAD(1)
#define NE_ZERO (1.0e-6f)

using neRadian = neReal;

///////////////////////////////////////////////////////////////////////////
//
// GENERAL
//
///////////////////////////////////////////////////////////////////////////

neReal neFRand(neReal Min, neReal Max);

neReal neSin(neRadian S);

neRadian neASin(neReal S);

neReal neCos(neRadian C);

neRadian neACos(neReal C);

neReal neTan(neRadian T);

neRadian neATan(neReal T);

neRadian neATan2(neReal y, neReal x);

bool neRealsEqual(neReal s1, neReal s2);

bool neIsConsiderZero(neReal f);

bool neIsFinite(neReal);

//template< class ta >                     NEINLINE ta      neAbs     ( const ta&  A )                               { return ( A < 0 ) ? -A : A;   }

NEINLINE neReal neAbs(neReal f) {
    return (neReal) fabs(f);
}

template<class ta, class tb, class tc>
NEINLINE bool neInRange(const ta &X, const tb &Min, const tc &Max) { return (Min <= X) && (X <= Max); }

template<class ta, class tb, class tc>
NEINLINE ta neRange(const ta &X, const tb &Min, const tc &Max) {
    if (X < Min) return Min;
    return (X > Max) ? Max : X;
}

template<class ta>
NEINLINE void neSwap(ta &X, ta &Y) {
    ta tmp = X;
    X = Y;
    Y = tmp;
}

template<class ta>
NEINLINE ta neSqr(const ta &A) { return A * A; }

template<class ta>
NEINLINE ta neMin(const ta &A, const ta &B) { return (A < B) ? A : B; }

template<class ta>
NEINLINE ta neMax(const ta &A, const ta &B) { return (A > B) ? A : B; }

NEINLINE neReal neMin(const s32 &A, const neReal &B) { return (A < B) ? A : B; }

NEINLINE neReal neMax(const s32 &A, const neReal &B) { return (A > B) ? A : B; }

NEINLINE neReal neMin(const neReal &A, const s32 &B) { return (A < B) ? A : B; }

NEINLINE neReal neMax(const neReal &A, const s32 &B) { return (A > B) ? A : B; }

NEINLINE bool neIsFinite(neReal n) { return neFinite((double) n); }

#endif
