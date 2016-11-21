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

///////////////////////////////////////////////////////////////////////////
// VECTOR4 MEMBER FUNCTIONS
///////////////////////////////////////////////////////////////////////////

//=========================================================================

NEINLINE neV4::neV4() {}

//=========================================================================

NEINLINE neV4::neV4(neReal x, neReal y, neReal z, neReal w) {
    X = x;
    Y = y;
    Z = z;
    W = w;
}

//=========================================================================

NEINLINE neV4::neV4(const neV4 &V) {
    (*this) = V;
}

//=========================================================================

NEINLINE neV4::neV4(const neV3 &V, neReal w) {
    X = V.X();
    Y = V.Y();
    Z = V.Z();
    W = w;
}

//=========================================================================

NEINLINE neReal neV4::Length() const {
    return (neReal) sqrt(this->Dot(*this));
}

//=========================================================================

NEINLINE neV4 &neV4::Normalize() {
    *this *= 1 / Length();
    return *this;
}

//=========================================================================

NEINLINE void neV4::SetZero() {
    X = Y = Z = W = 0;
}

//=========================================================================

NEINLINE void neV4::Set(neReal x, neReal y, neReal z, neReal w) {
    X = x;
    Y = y;
    Z = z;
    W = w;
}

//=========================================================================

NEINLINE neReal neV4::Dot(const neV4 &V) const {
    return X * V.X + Y * V.Y + Z * V.Z + W * V.W;
}

//=========================================================================

NEINLINE neReal &neV4::operator[](s32 I) {
    ASSERT(I >= 0);
    ASSERT(I <= 3);
    return ((neReal *) this)[I];
}

//=========================================================================

NEINLINE neV4 &neV4::operator+=(const neV4 &V) {
    X += V.X;
    Y += V.Y;
    Z += V.Z;
    W += V.W;
    return *this;
}

//=========================================================================

NEINLINE neV4 &neV4::operator-=(const neV4 &V) {
    X -= V.X;
    Y -= V.Y;
    Z -= V.Z;
    W -= V.W;
    return *this;
}

//=========================================================================

NEINLINE neV4 &neV4::operator/=(neReal S) {
    *this = *this / S;
    return *this;
}

//=========================================================================

NEINLINE neV4 &neV4::operator*=(neReal S) {
    *this = *this * S;
    return *this;
}


///////////////////////////////////////////////////////////////////////////
// VECTOR4 FRIEND FUNCTIONS
///////////////////////////////////////////////////////////////////////////

//=========================================================================

NEINLINE neV4 operator-(const neV4 &V) {
    return neV4(-V.X, -V.Y, -V.Z, -V.W);
}

//=========================================================================

NEINLINE neV4 operator+(const neV4 &V1, const neV4 &V2) {
    return neV4(V1.X + V2.X, V1.Y + V2.Y, V1.Z + V2.Z, V1.W + V2.W);
}

//=========================================================================

NEINLINE neV4 operator-(const neV4 &V1, const neV4 &V2) {
    return neV4(V1.X - V2.X, V1.Y - V2.Y, V1.Z - V2.Z, V1.W - V2.W);
}

//=========================================================================

NEINLINE neV4 operator/(const neV4 &V, neReal S) {
    return V * (1 / S);
}

//=========================================================================

NEINLINE neV4 operator*(const neV4 &V, const neReal S) {
    return neV4(V.X * S, V.Y * S, V.Z * S, V.W * S);
}

//=========================================================================

NEINLINE neV4 operator*(neReal S, const neV4 &V) {
    return V * S;
}

///////////////////////////////////////////////////////////////////////////
// END
///////////////////////////////////////////////////////////////////////////

