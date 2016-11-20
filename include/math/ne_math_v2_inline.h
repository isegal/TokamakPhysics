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
// VECTOR2 MEMBER FUNCTIONS
///////////////////////////////////////////////////////////////////////////

//=========================================================================

NEINLINE neV2::neV2(void) {}

//=========================================================================

NEINLINE neV2::neV2(neReal x, neReal y) {
    X = x;
    Y = y;
}

//=========================================================================

NEINLINE neV2::neV2(const neV2 &V) {
    *this = V;
}

//==============================================================================

NEINLINE neV2::neV2(const neReal &K) {
    X = K;//(neReal)cos( R );
    Y = K;//(neReal)sin( R );
}

//=========================================================================

NEINLINE void neV2::Zero(void) {
    X = Y = 0;
}

//=========================================================================

NEINLINE neReal
&

neV2::operator[](s32 I) {
    ASSERT(I >= 0);
    ASSERT(I <= 1);
    return ((neReal * )(this))[I];
}

//=========================================================================

NEINLINE neV2
&

neV2::operator+=(const neV2 &V) {
    X += V.X;
    Y += V.Y;
    return *this;
}

//=========================================================================

NEINLINE neV2
&

neV2::operator-=(const neV2 &V) {
    X -= V.X;
    Y -= V.Y;
    return *this;
}

//=========================================================================

NEINLINE neV2
&
neV2::operator/=(neReal
S )
{
*this = *this /
S;
return *this;
}

//=========================================================================

NEINLINE neV2
&
neV2::operator*=(neReal
S )
{
*this = *this *
S;
return *this;
}

//=========================================================================

NEINLINE void neV2::Set(neReal
x,
neReal y
)
{
X = x;
Y = y;
}

//=========================================================================

NEINLINE neReal

neV2::Length(void) const {
    return (neReal)
    sqrt(this->Dot(*this));
}

//=========================================================================

NEINLINE neV2
&

neV2::Normalize(void) {
    *this *= 1 / Length();
    return *this;
}

//=========================================================================

NEINLINE neReal

neV2::Dot(const neV2 &V) const {
    return X * V.X + Y * V.Y;
}

//==============================================================================

NEINLINE neV2

neV2::Cross(const neV2 &V) const {
    return neV2((X * V.Y) - (Y * V.X));
}

//==============================================================================

NEINLINE neReal

neV2::GetAngle(void) const {
    return (neReal)
    atan2(Y, X);
}

//==============================================================================

NEINLINE neV2
&
neV2::Rotate( neRadian
R )
{
neReal s = (neReal)
sin( R );
neReal c = (neReal)
cos( R );
neReal x = X;
neReal y = Y;

X = (c * x) - (s * y);
Y = (c * y) + (s * x);

return *this;
}

///////////////////////////////////////////////////////////////////////////
// VECTOR2 FRIEND FUNCTIONS
///////////////////////////////////////////////////////////////////////////

//=========================================================================

NEINLINE neV2

operator-(const neV2 &V) {
    return neV2(-V.X, -V.Y);
}

//=========================================================================

NEINLINE neV2

operator+(const neV2 &V1, const neV2 &V2) {
    return neV2(V1.X + V2.X, V1.Y + V2.Y);
}

//=========================================================================

NEINLINE neV2

operator-(const neV2 &V1, const neV2 &V2) {
    return neV2(V1.X - V2.X, V1.Y - V2.Y);
}

//=========================================================================

NEINLINE neV2

operator/(const neV2 &V, neReal S) {
    return V * (1 / S);
}

//=========================================================================

NEINLINE neV2

operator*(const neV2 &V, const neReal S) {
    return neV2(V.X * S, V.Y * S);
}

//=========================================================================

NEINLINE neV2
operator*(neReal
S,
const neV2 &V
)
{
return
V *S;
}

//=========================================================================

NEINLINE neRadian

neV2_AngleBetween(const neV2 &V1, const neV2 &V2) {
    neReal D, C;

    D = V1.Length() * V2.Length();

    if (D == 0.0f) return 0;

    C = V1.Dot(V2) / D;

    if (C > 1.0f) C = 1.0f;
    else if (C < -1.0f) C = -1.0f;

    return (neReal)
    acos(C);
}

///////////////////////////////////////////////////////////////////////////
// END
///////////////////////////////////////////////////////////////////////////
