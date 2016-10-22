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

#ifndef DCD_H
#define DCD_H

struct DCDFace {
    neByte *neighbourFaces;
    neByte *neighbourVerts;
    neByte *neighbourEdges;
};

struct DCDVert {
    neByte *neighbourEdges;
};

struct DCDEdge {
    neByte f1;
    neByte f2;
    neByte v1;
    neByte v2;
};

struct DCDMesh {
    size_t numFaces;
    size_t numVerts;
    size_t numEdges;
    neByte pad0;

    size_t numNeighbour;

    neV3 *normals;
    neV3 *vertices;

    DCDFace *faces;
    DCDVert *verts;
    DCDEdge *edges;

    void SetConvex(const TConvex &convex, neV3 *vertArray);

    neV3 GetVertOnFace(size_t faceIndex, size_t vertIndex);

    neV3 GetVert(size_t vertIndex);

    neV3 GetNormal(size_t faceIndex);

//	neByte FaceGetNumFaceNeighbour(s32 faceIndex);
    neByte FaceGetFaceNeighbour(size_t faceIndex, size_t neighbourIndex);

    neByte FaceGetEdgeNeighbour(size_t faceIndex, size_t neighbourIndex);

//	neByte VertGetNumEdgeNeighbour(s32 vertIndex);
    neByte VertGetEdgeNeighbour(size_t vertIndex, size_t neighbourIndex);

    neByte EdgeGetVert1(size_t edgeIndex);

    neByte EdgeGetVert2(size_t edgeIndex);
};

bool TestDCD(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB, neT3 &transB,
             const neV3 &backupVector);

void Convex2TerrainTest(neCollisionResult &result, TConvex &convexA, neT3 &transA, TConvex &convexB);

#endif