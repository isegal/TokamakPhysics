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

#include <stdlib.h>

//=========================================================================

NEINLINE neReal neFRand(neReal Min, neReal Max) {
    ASSERT(Max >= Min);
    return ((((neReal) rand() / (neReal) RAND_MAX) * (Max - Min)) + Min);
}

//=========================================================================

NEINLINE neReal neSin(neRadian S) {
    return (neReal) sin(S);
}

//=========================================================================

NEINLINE neReal neCos(neRadian C) {
    return (neReal) cos(C);
}

//=========================================================================

NEINLINE neRadian neASin(neReal S) {
    return (neReal) asin(S);
}

//=========================================================================

NEINLINE neRadian neACos(neReal C) {
    return (neReal) acos(C);
}

//=========================================================================

NEINLINE neReal neTan(neRadian T) {
    return (neReal) tan(T);
}

//=========================================================================

NEINLINE neRadian neATan(neReal T) {
    return (neReal) atan(T);
}

//=========================================================================

NEINLINE neRadian neATan2(neReal y, neReal x) {
    return (neReal) atan2(y, x);
}

//=========================================================================

NEINLINE bool neRealsEqual(neReal s1, neReal s2) {
    if ((2.0f * neAbs(s1 - s2) / (s1 + s2)) < NE_ZERO) {
        return true;
    }

    return false;
}

//=========================================================================

NEINLINE bool neIsConsiderZero(neReal f) {
    return (neAbs(f) < NE_ZERO);
}