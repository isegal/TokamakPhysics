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

#ifndef NE_TYPE_H
#define NE_TYPE_H

#include <cstdint>
#include <cstdarg>
//#include <tchar.h>
//#include <strsafe.h>


///////////////////////////////////////////////////////////////////////////
// BASIC TYPES
///////////////////////////////////////////////////////////////////////////

using s8 = signed char;
using s16 = short;
using s32 = int;
using neReal = float;
using neByte = uint8_t;
using neErr = s32;

#if _MSC_VER
typedef signed   __int64    s64;
typedef unsigned __int64    u64;
#define neFinite _finite
#define inline   __forceinline       // Make sure that the compiler inlines when we tell him
#define NEINLINE __forceinline
const char PATH_SEP = '\\';
#elif defined __GNUC__
using s64 = long long;
using u64 = unsigned long long;
#define neFinite isfinite
#define NEINLINE inline
const char PATH_SEP = '/';
#endif

#endif //NE_TYPE_H
