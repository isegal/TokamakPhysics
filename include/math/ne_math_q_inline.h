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
// neQ inlines
///////////////////////////////////////////////////////////////////////////

//=========================================================================

NEINLINE neQ::neQ() {
    Identity();
}

//=========================================================================

NEINLINE neQ::neQ(neReal x, neReal y, neReal z, neReal w) {
    X = x;
    Y = y;
    Z = z;
    W = w;
}

//==========================================================================

NEINLINE neQ::neQ(const neM4 &M) {
    SetupFromMatrix(M);
}

//==========================================================================

NEINLINE void neQ::Zero() {
    X = Y = Z = W = 0;
}

//==========================================================================

NEINLINE void neQ::Identity() {
    X = Y = Z = 0;
    W = 1;
}

//==========================================================================

NEINLINE neQ &neQ::Invert() {
    // This can be done also be negating (X,Y,Z) stead of W
    W = -W;
    return *this;
}

//=========================================================================

NEINLINE neM4 neQ::BuildMatrix() const {
    neM4 M;

    const neReal tx = 2.0f * X;
    const neReal ty = 2.0f * Y;
    const neReal tz = 2.0f * Z;
    const neReal twx = tx * W;
    const neReal twy = ty * W;
    const neReal twz = tz * W;
    const neReal txx = tx * X;
    const neReal txy = ty * X;
    const neReal txz = tz * X;
    const neReal tyy = ty * Y;
    const neReal tyz = tz * Y;
    const neReal tzz = tz * Z;

    M.M[0][0] = 1.0f - (tyy + tzz);
    M.M[1][0] = txy - twz;
    M.M[2][0] = txz + twy;
    M.M[0][1] = txy + twz;
    M.M[1][1] = 1.0f - (txx + tzz);
    M.M[2][1] = tyz - twx;
    M.M[0][2] = txz - twy;
    M.M[1][2] = tyz + twx;
    M.M[2][2] = 1.0f - (txx + tyy);

    M.M[3][0] = M.M[3][1] = M.M[3][2] =
    M.M[0][3] = M.M[1][3] = M.M[2][3] = 0.0f;
    M.M[3][3] = 1.0f;

    return M;
}

NEINLINE neM3 neQ::BuildMatrix3() const {
    neM3 M;

    const neReal tx = 2.0f * X;
    const neReal ty = 2.0f * Y;
    const neReal tz = 2.0f * Z;
    const neReal twx = tx * W;
    const neReal twy = ty * W;
    const neReal twz = tz * W;
    const neReal txx = tx * X;
    const neReal txy = ty * X;
    const neReal txz = tz * X;
    const neReal tyy = ty * Y;
    const neReal tyz = tz * Y;
    const neReal tzz = tz * Z;

    M.M[0][0] = 1.0f - (tyy + tzz);
    M.M[1][0] = txy - twz;
    M.M[2][0] = txz + twy;
    M.M[0][1] = txy + twz;
    M.M[1][1] = 1.0f - (txx + tzz);
    M.M[2][1] = tyz - twx;
    M.M[0][2] = txz - twy;
    M.M[1][2] = tyz + twx;
    M.M[2][2] = 1.0f - (txx + tyy);

    return M;
}

//=========================================================================

NEINLINE void neQ::SetupFromMatrix(const neM4 &Matrix) {
    // squared magniudes of quaternion components
    // first compute squared magnitudes of quaternion components - at least one
    // will be greater than 0 since quaternion is unit magnitude
    const neReal qs2 = 0.25f * (Matrix.M[0][0] + Matrix.M[1][1] + Matrix.M[2][2] + 1.0f);
    const neReal qx2 = qs2 - 0.5f * (Matrix.M[1][1] + Matrix.M[2][2]);
    const neReal qy2 = qs2 - 0.5f * (Matrix.M[2][2] + Matrix.M[0][0]);
    const neReal qz2 = qs2 - 0.5f * (Matrix.M[0][0] + Matrix.M[1][1]);


    // find maximum magnitude component
    const s32 i = (qs2 > qx2) ?
                  ((qs2 > qy2) ? ((qs2 > qz2) ? 0 : 3) : ((qy2 > qz2) ? 2 : 3)) :
                  ((qx2 > qy2) ? ((qx2 > qz2) ? 1 : 3) : ((qy2 > qz2) ? 2 : 3));

    // compute signed quaternion components using numerically stable method
    switch (i) {
        case 0: {
            W = (neReal) sqrt(qs2);
            const neReal tmp = 0.25f / W;
            X = (Matrix.M[1][2] - Matrix.M[2][1]) * tmp;
            Y = (Matrix.M[2][0] - Matrix.M[0][2]) * tmp;
            Z = (Matrix.M[0][1] - Matrix.M[1][0]) * tmp;
            break;
        }
        case 1: {
            X = (neReal) sqrt(qx2);
            const neReal tmp = 0.25f / X;
            W = (Matrix.M[1][2] - Matrix.M[2][1]) * tmp;
            Y = (Matrix.M[1][0] + Matrix.M[0][1]) * tmp;
            Z = (Matrix.M[2][0] + Matrix.M[0][2]) * tmp;
            break;
        }
        case 2: {
            Y = (neReal) sqrt(qy2);
            const neReal tmp = 0.25f / Y;
            W = (Matrix.M[2][0] - Matrix.M[0][2]) * tmp;
            Z = (Matrix.M[2][1] + Matrix.M[1][2]) * tmp;
            X = (Matrix.M[0][1] + Matrix.M[1][0]) * tmp;
            break;
        }
        case 3: {
            Z = (neReal) sqrt(qz2);
            const neReal tmp = 0.25f / Z;
            W = (Matrix.M[0][1] - Matrix.M[1][0]) * tmp;
            X = (Matrix.M[0][2] + Matrix.M[2][0]) * tmp;
            Y = (Matrix.M[1][2] + Matrix.M[2][1]) * tmp;
            break;
        }
    }

    // for consistency, force positive scalar component [ (s; v) = (-s; -v) ]
    if (W < 0) *this = -*this;

    // normalize, just to be safe
    Normalize();
}

NEINLINE void neQ::SetupFromMatrix3(const neM3 &Matrix) {
    neM4 m;

    m.SetIdentity();

    int i, j;

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            m.M[i][j] = Matrix.M[i][j];

    SetupFromMatrix(m);
}

//=========================================================================

NEINLINE neReal neQ::Dot(const neQ &Q) const {
    return Q.X * X + Q.Y * Y + Q.Z * Z + Q.W * W;
}

//=========================================================================

NEINLINE neQ &neQ::Normalize() {
    neReal t;
    neReal norm = Dot(*this);

//	if (neRealsEqual(norm, 1.0f))
//		return *this;

    ASSERT(norm >= 0.0f);

    t = 1.0f / (float) sqrt(norm);

    X *= t;
    Y *= t;
    Z *= t;
    W *= t;

    return *this;
}

//=========================================================================

NEINLINE neQ &neQ::operator*=(const neQ &Q) {
    *this = *this * Q;
    return *this;
}

//=========================================================================

NEINLINE neQ &neQ::operator*=(neReal S) {
    *this = *this * S;
    return *this;
}

//==========================================================================

NEINLINE neQ &neQ::operator+=(const neQ &Q) {
    *this = *this + Q;
    return *this;
}

//==========================================================================

NEINLINE neQ &neQ::operator-=(const neQ &Q) {
    *this = *this - Q;
    return *this;
}

//=========================================================================

NEINLINE neQ operator-(const neQ &Q) {
    return neQ(-Q.X, -Q.Y, -Q.Z, -Q.W);
}

//==========================================================================

NEINLINE neQ operator*(const neQ &q1, const neQ &q2) {
    neReal tmp_0,    //temporary variables
            tmp_1,
            tmp_2,
            tmp_3,
            tmp_4,
            tmp_5,
            tmp_6,
            tmp_7,
            tmp_8,
            tmp_9;

    neQ zq;

    tmp_0 = (q1.Z - q1.Y) * (q2.Y - q2.Z);
    tmp_1 = (q1.W + q1.X) * (q2.W + q2.X);
    tmp_2 = (q1.W - q1.X) * (q2.Y + q2.Z);
    tmp_3 = (q1.Y + q1.Z) * (q2.W - q2.X);
    tmp_4 = (q1.Z - q1.X) * (q2.X - q2.Y);
    tmp_5 = (q1.Z + q1.X) * (q2.X + q2.Y);
    tmp_6 = (q1.W + q1.Y) * (q2.W - q2.Z);
    tmp_7 = (q1.W - q1.Y) * (q2.W + q2.Z);
    tmp_8 = tmp_5 + tmp_6 + tmp_7;
    tmp_9 = 0.5f * (tmp_4 + tmp_8);


    zq.X = tmp_1 + tmp_9 - tmp_8;
    zq.Y = tmp_2 + tmp_9 - tmp_7;
    zq.Z = tmp_3 + tmp_9 - tmp_6;
    zq.W = tmp_0 + tmp_9 - tmp_5;

    return zq;

}

//==========================================================================

NEINLINE void neQ::GetAxisAngle(neV3 &Axis, neRadian &Angle) const {
    neReal sum = X * X + Y * Y + Z * Z;

    if (neIsConsiderZero(sum)) {
        Angle = 0.0f;

        Axis.Set(1.0f, 0.0f, 0.0f);

        return;
    }
    neReal OneOver = 1.0f / (neReal) sqrtf(sum);

    Axis.Set(OneOver * X, OneOver * Y, OneOver * Z);

    neReal w = W;

    if (neIsConsiderZero(W - 1.0f)) {
        Angle = 0.0f;
    } else {
        Angle = 2.0f * (neReal) acosf(w);
    }
}

NEINLINE neV3 operator*(const neQ &Q, const neV3 &V) {
    neV3 Axis;
    Axis.Set(Q.X, Q.Y, Q.Z);
    neV3 uv;
    uv.Set(Axis.Cross(V));
    neV3 uuv;
    uuv.Set(Axis.Cross(uv) * 2.0f);

    return V + uuv + (uv * Q.W * 2.0f);
}

//==========================================================================

NEINLINE neQ operator*(const neQ &Q, neReal S) {
    return neQ(Q.X * S, Q.Y * S, Q.Z * S, Q.W * S);
}

//==========================================================================

NEINLINE neQ operator*(neReal S, const neQ &Q) {
    return Q * S;
}

//==========================================================================

NEINLINE neQ operator+(const neQ &Qa, const neQ &Qb) {
    return neQ(Qa.X + Qb.X, Qa.Y + Qb.Y, Qa.Z + Qb.Z, Qa.W + Qb.W);
}

//==========================================================================

NEINLINE neQ operator-(const neQ &Qa, const neQ &Qb) {
    return neQ(Qa.X - Qb.X, Qa.Y - Qb.Y, Qa.Z - Qb.Z, Qa.W - Qb.W);
}

NEINLINE neQ &neQ::Set(neReal _X, neReal _Y, neReal _Z, neReal _W) {
    X = _X;
    Y = _Y;
    Z = _Z;
    W = _W;
    return (*this);
}

NEINLINE neQ &neQ::Set(const neV3 &V, neReal W) {
    return Set(V[0], V[1], V[2], W);
}

NEINLINE neQ &neQ::Set(neReal angle, const neV3 &axis) {
    neReal halfAngle = angle * 0.5f;

    neReal sinHalfAngle = sinf(halfAngle);

    neReal cosHalfAngle = cosf(halfAngle);

    neV3 tmp = axis * sinHalfAngle;

    Set(tmp, cosHalfAngle);

    return (*this);
}

NEINLINE bool neQ::IsFinite() {
    return (neFinite(X) &&
            neFinite(Y) &&
            neFinite(Z) &&
            neFinite(W));
}