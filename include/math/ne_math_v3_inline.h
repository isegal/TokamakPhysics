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

//=========================================================================

NEINLINE neReal &neV3::operator[](s32 I) {
    ASSERT(I >= 0);
    ASSERT(I <= 2);
    // return ((neReal*)this)[I];
    return v[I];
}

//=========================================================================

NEINLINE neReal neV3::operator[](s32 I) const {
    ASSERT(I >= 0);
    ASSERT(I <= 2);
    return v[I];
}

//=========================================================================

NEINLINE neV3 &neV3::Set(neReal x, neReal y, neReal z) {
    this->v[0] = x;
    this->v[1] = y;
    this->v[2] = z;
    //m = _mm_set_ps(x, y, z, 0);
    return (*this);
}
/*
NEINLINE neV3 & neV3::operator =(const neV3 & V)
{ 
	_mm_store_ps(&v[0], V.m); 
	return (*this);
};
*/
NEINLINE neV3 &neV3::Set(const neV3 &V) {
    (*this) = V;
    return (*this);
}

NEINLINE neV3 &neV3::Set(const neQ &Q) {
    v[0] = Q.X;
    v[1] = Q.Y;
    v[2] = Q.Z;

    return (*this);
}

NEINLINE void neV3::SetAbs(const neV3 &a) {
    v[0] = neAbs(a[0]);
    v[1] = neAbs(a[1]);
    v[2] = neAbs(a[2]);
}

//=========================================================================

NEINLINE void neV3::Set(neReal val[3]) {
    this->v[0] = val[0];
    this->v[1] = val[1];
    this->v[2] = val[2];
}

//=========================================================================

NEINLINE void neV3::Get(neReal val[3]) {
    val[0] = X();
    val[1] = Y();
    val[2] = Z();
}

//=========================================================================

NEINLINE neReal neV3::Length() const {
    neReal dot = this->Dot(*this);

    if (neIsConsiderZero(dot))
        return 0.0f;

    return (neReal) sqrtf(dot);
}

//=========================================================================

NEINLINE void neV3::Normalize() {
    neReal l = Length();

    if (l == 0.0f) {
        SetZero();
    } else {
        *this *= 1.0f / Length();
    }
}

//=========================================================================

NEINLINE void neV3::RotateX(neRadian Rx) {
    neReal s = (neReal) sinf(Rx);
    neReal c = (neReal) cosf(Rx);
    neReal y = v[1];
    neReal z = v[2];

    v[1] = (c * y) - (s * z);
    v[2] = (c * z) + (s * y);
}

//=========================================================================

NEINLINE void neV3::RotateY(neRadian Ry) {
    neReal s = (neReal) sinf(Ry);
    neReal c = (neReal) cosf(Ry);
    neReal x = X();
    neReal z = Z();

    this->v[0] = (c * x) + (s * z);
    this->v[2] = (c * z) - (s * x);
}

//=========================================================================

NEINLINE void neV3::RotateZ(neRadian Rz) {
    neReal s = (neReal) sinf(Rz);
    neReal c = (neReal) cosf(Rz);
    neReal x = X();
    neReal y = Y();

    this->v[0] = (c * x) - (s * y);
    this->v[1] = (c * y) + (s * x);
}

//=========================================================================

NEINLINE neV3 &neV3::SetZero() {
    this->v[0] = this->v[1] = this->v[2] = 0.0f;

    return (*this);
}

NEINLINE neV3 &neV3::SetOne() {
    this->v[0] = this->v[1] = this->v[2] = 1.0f;
    return (*this);
}

NEINLINE neV3 &neV3::SetHalf() {
    this->v[0] = this->v[1] = this->v[2] = 0.5f;
    return (*this);
}

NEINLINE neV3 &neV3::Set(neReal value) {
    this->v[0] = this->v[1] = this->v[2] = value;
    return (*this);
}

//=========================================================================

NEINLINE neReal neV3::Dot(const neV3 &V) const {
    return X() * V.X() + Y() * V.Y() + Z() * V.Z();
}

//=========================================================================

NEINLINE neV3 neV3::Cross(const neV3 &V) const {
    neV3 tmp;

    tmp.v[0] = Y() * V.Z() - Z() * V.Y();
    tmp.v[1] = Z() * V.X() - X() * V.Z();
    tmp.v[2] = X() * V.Y() - Y() * V.X();

    return tmp;
}

//=========================================================================

NEINLINE neV3 &neV3::operator+=(const neV3 &V) {
    v[0] += V.X();
    v[1] += V.Y();
    v[2] += V.Z();
    return *this;
}

//=========================================================================

NEINLINE neV3 &neV3::operator-=(const neV3 &V) {
    v[0] -= V.X();
    v[1] -= V.Y();
    v[2] -= V.Z();
    return *this;
}

//=========================================================================

NEINLINE neV3 &neV3::operator/=(neReal S) {
    *this = *this / S;
    return *this;
}

//=========================================================================

NEINLINE neV3 &neV3::operator*=(neReal S) {
    *this = *this * S;
    return *this;
}

//=========================================================================

NEINLINE neRadian neV3::GetPitch() const {
    return (neReal) -atan2(Y(), (neReal) sqrt(X() * X() + Z() * Z()));
}

//=========================================================================

NEINLINE neRadian neV3::GetYaw() const {
    return (neReal) atan2(X(), Z());
}

///////////////////////////////////////////////////////////////////////////
// VECTOR3 FRIEND FUNCTIONS
///////////////////////////////////////////////////////////////////////////

//=========================================================================

NEINLINE neV3 operator-(const neV3 &V) {
    neV3 tmp;
    return tmp.Set(-V.X(), -V.Y(), -V.Z());
}

//=========================================================================

NEINLINE neV3 operator+(const neV3 &V1, const neV3 &V2) {
    neV3 tmp;

    //tmp.m = _mm_add_ps(V1.m, V2.m);

    //return tmp;

    return tmp.Set(V1.X() + V2.X(), V1.Y() + V2.Y(), V1.Z() + V2.Z());
}

//=========================================================================

NEINLINE neV3 operator-(const neV3 &V1, const neV3 &V2) {
    neV3 tmp;

    //tmp.m = _mm_sub_ps(V1.m, V2.m);

    //return tmp;
    return tmp.Set(V1.X() - V2.X(), V1.Y() - V2.Y(), V1.Z() - V2.Z());
}

//=========================================================================

NEINLINE neV3 operator/(const neV3 &V, neReal S) {
    return V * (1.0f / S);
}

//=========================================================================

NEINLINE neV3 operator*(const neV3 &V, const neReal S) {
    neV3 tmp;
    return tmp.Set(V.X() * S, V.Y() * S, V.Z() * S);
}

//=========================================================================

NEINLINE neV3 operator*(neReal S, const neV3 &V) {
    return V * S;
}

//=========================================================================
NEINLINE void neV3::SetBoxTensor(neReal width, neReal height, neReal length, neReal mass) {
    v[0] = mass * (length * length + height * height) / 12.0f;
    v[1] = mass * (width * width + height * height) / 12.0f;
    v[2] = mass * (length * length + width * width) / 12.0f;
}

//=========================================================================
NEINLINE void neV3::SetMin(const neV3 &V1, const neV3 &V2) {
    (*this)[0] = (V1.X() < V2.X()) ? V1.X() : V2.X();
    (*this)[1] = (V1.Y() < V2.Y()) ? V1.Y() : V2.Y();
    (*this)[2] = (V1.Z() < V2.Z()) ? V1.Z() : V2.Z();
}
//=========================================================================
NEINLINE void neV3::SetMax(const neV3 &V1, const neV3 &V2) {
    (*this)[0] = (V1.X() > V2.X()) ? V1.X() : V2.X();
    (*this)[1] = (V1.Y() > V2.Y()) ? V1.Y() : V2.Y();
    (*this)[2] = (V1.Z() > V2.Z()) ? V1.Z() : V2.Z();
}
//=========================================================================
NEINLINE neV3 operator*(const neV3 &V, const neM3 &M) {
    neV3 ret;

    ret[0] = V.Dot(M[0]);
    ret[1] = V.Dot(M[1]);
    ret[2] = V.Dot(M[2]);
    return ret;
}
//=========================================================================
NEINLINE neV3 operator*(const neV3 &V1, const neV3 &V2) {
    neV3 ret;

    //ret.m = _mm_mul_ps(V1.m, V2.m);

    //return ret;

    ret[0] = V1[0] * V2[0];
    ret[1] = V1[1] * V2[1];
    ret[2] = V1[2] * V2[2];

    return ret;
}

NEINLINE bool neV3::IsConsiderZero() const {
    return (neIsConsiderZero(v[0]) &&
            neIsConsiderZero(v[1]) &&
            neIsConsiderZero(v[2]));
}

NEINLINE bool neV3::IsFinite() const {
    if (neFinite((double) v[0]) &&
        neFinite((double) v[1]) &&
        neFinite((double) v[2]))
        return true;
    return false;
}

NEINLINE neReal neV3::GetDistanceFromLine(const neV3 &pointA, const neV3 &pointB) {
    neV3 ba = pointB - pointA;

    neReal len = ba.Length();

    if (neIsConsiderZero(len))
        ba.SetZero();
    else
        ba *= 1.0f / len;

    neV3 pa = (*this) - pointA;

    neReal k = pa.Dot(ba);

    neV3 q = pointA + k * ba;

    neV3 diff = (*this) - q;

    return diff.Length();
}

NEINLINE neReal neV3::GetDistanceFromLine2(neV3 &project, const neV3 &pointA, const neV3 &pointB) {
    neV3 ba = pointB - pointA;

    neReal len = ba.Length();

    if (neIsConsiderZero(len))
        ba.SetZero();
    else
        ba *= 1.0f / len;

    neV3 pa = (*this) - pointA;

    neReal k = pa.Dot(ba);

    project = pointA + k * ba;

    neV3 diff = (*this) - project;

    return diff.Length();
}

NEINLINE neReal neV3::GetDistanceFromLineAndProject(neV3 &result, const neV3 &startPoint, const neV3 &dir) {
    neV3 pa = (*this) - startPoint;

    neReal k = pa.Dot(dir);

    result = startPoint + k * dir;

    neV3 diff = (*this) - result;

    return diff.Length();
}

NEINLINE void neV3::RemoveComponent(const neV3 &V) {
    neReal dot = (*this).Dot(V);

    (*this) = (*this) - V * dot;
}

NEINLINE neV3 neV3::Project(const neV3 &v) const {
    neReal dot = (*this).Dot(v);

    return (v * dot);
}

NEINLINE bool neV3::GetIntersectPlane(neV3 &normal, neV3 &pointOnPlane, neV3 &point1, neV3 &point2) {
    neV3 diff = point2 - point1;

    neReal d2 = normal.Dot(diff);

    if (neIsConsiderZero(d2))
        return false;

    neReal d1 = normal.Dot(pointOnPlane - point1);

    neReal u = d1 / d2;

    *this = point1 + diff * u;

    return true;
}

NEINLINE neReal neV3::X() const {
    return v[0];
}

NEINLINE neReal neV3::Y() const {
    return v[1];
}

NEINLINE neReal neV3::Z() const {
    return v[2];
}

NEINLINE neReal neV3::W() const {
    return v[3];
}

///////////////////////////////////////////////////////////////////////////
// END
///////////////////////////////////////////////////////////////////////////

