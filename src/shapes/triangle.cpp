#include "triangle.h"
#include "stats.h"
#include "paramset.h"
#include <set>

TriangleMesh::TriangleMesh(const Transform &objToWorld, int nTriangles, const int *vertexIndices,
                           int nVertices, const Point3f *P, const Vector3f *S, const Normal3f *N, const Point2f *UV,
                           const shared_ptr<Texture<Float>> &alphaMask)
       : nTriangles(nTriangles), nVertices(nVertices),
         vertexIndices(vertexIndices, vertexIndices + 3 * nTriangles),
         alphaMask(alphaMask)
{
    p.reset(new Point3f[nVertices]);
    for (unsigned i = 0; i < nVertices; i++)
        p[i] = objToWorld(p[i]);

    if (UV) {
        uv.reset(new Point2f[nVertices]);
        memcpy(uv.get(), UV, nVertices * sizeof(Point2f));
    }
    if (N) {
        n.reset(new Normal3f[nVertices]);
        for (int i = 0; i < nVertices; ++i) n[i] = objToWorld(N[i]);
    }
    if (S) {
        s.reset(new Vector3f[nVertices]);
        for (int i = 0; i < nVertices; ++i) s[i] = objToWorld(S[i]);
    }
}

vector<shared_ptr<Shape>>
TriangleMesh::create(const Transform *objToWorld, const Transform *worldToObj, bool revOrient,
                     int nTriangles, const int *vertexIndices, int nVertices,
                     const Point3f *p, const Vector3f *s, const Normal3f *n, const Point2f *uv,
                     const shared_ptr<Texture<Float>> &alphaMask)
{
    shared_ptr<TriangleMesh> mesh = make_shared<TriangleMesh>(*objToWorld, nTriangles, vertexIndices,
                                                              nVertices, p, s, n, uv, alphaMask);
    vector<shared_ptr<Shape>> tris;
    for (int i = 0; i < nTriangles; ++i)
        tris.push_back(make_shared<Triangle>(objToWorld, worldToObj, revOrient, mesh, i));
    return tris;
}

vector<shared_ptr<Shape>>
TriangleMesh::create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                     const ParamSet &params, TriangleMesh::FloatTextureMap *floatTextures)
{
    int nvi, npi, nuvi, nsi, nni;
    const int *vi = params.findInt("indices", &nvi);
    const Point3f *P = params.findPoint3f("P", &npi);
    const Point2f *uvs = params.findPoint2f("uv", &nuvi);
    if (!uvs) uvs = params.findPoint2f("st", &nuvi);
    vector<Point2f> tempUVs;
    if (!uvs) {
        const Float *fuv = params.findFloat("uv", &nuvi);
        if (!fuv) fuv = params.findFloat("st", &nuvi);
        if (fuv) {
            nuvi /= 2;
            tempUVs.reserve(nuvi);
            for (int i = 0; i < nuvi; ++i)
                tempUVs.push_back(Point2f(fuv[2 * i], fuv[2 * i + 1]));
            uvs = &tempUVs[0];
        }
    }
    if (uvs) {
        if (nuvi < npi) {
            ERROR("Not enough of \"uv\"s for triangle mesh.  Expected %d, "
                  "found %d.  Discarding.", npi, nuvi);
            uvs = nullptr;
        } else if (nuvi > npi)
            WARNING("More \"uv\"s provided than will be used for triangle "
                    "mesh.  (%d expcted, %d found)", npi, nuvi);
    }
    if (!vi) {
        ERROR("Vertex indices \"indices\" not provided with triangle mesh shape");
        return vector<shared_ptr<Shape>>();
    }
    if (!P) {
        ERROR("Vertex positions \"P\" not provided with triangle mesh shape");
        return vector<shared_ptr<Shape>>();
    }
    const Vector3f *S = params.findVector3f("S", &nsi);
    if (S && nsi != npi) {
        ERROR("Number of \"S\"s for triangle mesh must match \"P\"s");
        S = nullptr;
    }
    const Normal3f *N = params.findNormal3f("N", &nni);
    if (N && nni != npi) {
        ERROR("Number of \"N\"s for triangle mesh must match \"P\"s");
        N = nullptr;
    }
    for (int i = 0; i < nvi; ++i)
        if (vi[i] >= npi) {
            ERROR("trianglemesh has out of-bounds vertex index %d (%d \"P\" "
                  "values were given", vi[i], npi);
            return vector<shared_ptr<Shape>>();
        }

    int nfi;
    const int *faceIndices = params.findInt("faceIndices", &nfi);
    if (faceIndices && nfi != nvi / 3) {
        ERROR("Number of face indices, %d, doesn't match number of faces, %d", nfi, nvi / 3);
        faceIndices = nullptr;
    }

    shared_ptr<Texture<Float>> alphaTex;
    string alphaTexName = params.findTexture("alpha");
    if (alphaTexName != "") {
        if (floatTextures->find(alphaTexName) != floatTextures->end())
            alphaTex = (*floatTextures)[alphaTexName];
        else
            ERROR("Couldn't find float texture \"%s\" for \"alpha\" parameter",
                  alphaTexName.c_str());
    } else if (params.findOneFloat("alpha", 1.f) == 0.f)
        alphaTex.reset(new ConstantTexture<Float>(0.f));

    return create(o2w, w2o, reverseOrientation, nvi / 3, vi, npi, P, S, N, uvs, alphaTex);
}

Bounds3f Triangle::objectBound() const {
    const auto &p0 = mesh->p[v[0]];
    const auto &p1 = mesh->p[v[1]];
    const auto &p2 = mesh->p[v[2]];
    return unionOf( Bounds3f( (*worldToObject)(p0), (*worldToObject)(p1) ), (*worldToObject)(p2) );
}

Bounds3f Triangle::worldBound() const {
    const auto &p0 = mesh->p[v[0]];
    const auto &p1 = mesh->p[v[1]];
    const auto &p2 = mesh->p[v[2]];
    return unionOf(Bounds3f(p0, p1), p2);
}

STAT_PERCENT("Intersections/Ray-triangle intersection tests", nTriangleHits, nTriangleTests);

bool Triangle::intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const
{
    ProfilePhase phase(Profiler::Stage::TriIntersect);
    nTriangleTests++;

    // Get triangle vertices
    const auto &p0 = mesh->p[v[0]];
    const auto &p1 = mesh->p[v[1]];
    const auto &p2 = mesh->p[v[2]];

    // Perform ray-triangle intersection test
    // Transform vertices to ray coordinate space
    auto p0t = p0 - Vector3f(ray.o), p1t = p1 - Vector3f(ray.o), p2t = p2 - Vector3f(ray.o); // translate
    int kz = maxDim(abs(ray.d)); int kx = (kz + 1) % 3; int ky = (kx + 1) % 3; // permute
    auto d = permute(ray.d, kx, ky, kz);
    Float Sx = -d.x / d.z, Sy = -d.y / d.z, Sz = 1.f / d.z; // shear
    p0t.x += Sx * p0t.z; p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z; p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z; p2t.y += Sy * p2t.z;

    // Compute edge coefficients (P16 Equation 3.2)
    Float e0 = p1t.x * p2t.y - p1t.y * p2t.x;
    Float e1 = p2t.x * p0t.y - p2t.y * p0t.x;
    Float e2 = p0t.x * p1t.y - p0t.y * p1t.x;

    // Double precision test at edges
    if (sizeof(Float) == sizeof(float) &&
        (e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
        double p2txp1ty = double(p2t.x) * double(p1t.y);
        double p2typ1tx = double(p2t.y) * double(p1t.x);
        e0 = float(p2typ1tx - p2txp1ty);
        double p0txp2ty = double(p0t.x) * double(p2t.y);
        double p0typ2tx = double(p0t.y) * double(p2t.x);
        e1 = float(p0typ2tx - p0txp2ty);
        double p1txp0ty = double(p1t.x) * double(p0t.y);
        double p1typ0tx = double(p1t.y) * double(p0t.x);
        e2 = float(p1typ0tx - p1txp0ty);
    }

    // Perform triangle edge and determinant tests
    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0)) // signs are different
        return false;
    Float areaDet = e0 + e1 + e2; // the determinant to evaluate triangle area
    if (areaDet == 0)
        // ray hit triangle on the edge (the projection of triangle on the xOy plane is a line segment)
        return false;

    // Compute scaled hit distance to triangle and test againt ray t range
    p0t.z *= Sz; p1t.z *= Sz; p2t.z *= Sz;
    Float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z; // not divided by det (e0 + e1 + e2) yet
    if (areaDet < 0 && (tScaled >= 0 || tScaled < ray.tMax * areaDet))
        return false;
    else if (areaDet > 0 && (tScaled <= 0 || tScaled > ray.tMax * areaDet))
        return false;

    // Compute barycentric coordinates and t value for triangle intersection
    Float invAreaDet = 1 / areaDet;
    Float b0 = e0 * invAreaDet, b1 = e1 * invAreaDet, b2 = e2 * invAreaDet;
    Float t = tScaled * invAreaDet;

    // Ensure that computed triangle t is conservatively greater than zero
    Float maxZt = maxComp(abs(Vector3f(p0t.z, p1t.z, p2t.z)));
    Float deltaZ = gamma(3) * maxZt;
    Float maxXt = maxComp(abs(Vector3f(p0t.x, p1t.x, p2t.x)));
    Float maxYt = maxComp(abs(Vector3f(p0t.y, p1t.y, p2t.y)));
    Float deltaX = gamma(5) * (maxXt + maxZt);
    Float deltaY = gamma(5) * (maxYt + maxZt);
    Float deltaE = 2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);
    Float maxE = maxComp(abs(Vector3f(e0, e1, e2)));
    Float deltaT = 3 * (gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) * abs(invAreaDet);
    if (t <= deltaT) return false;

    // Compute triangle partial derivatives
    Vector3f dpdu, dpdv;
    Point2f uv[3];
    getUVs(uv);
    Vector2f duv02 = uv[0] - uv[2], duv12 = uv[1] - uv[2];
    Vector3f dp02 = p0 - p2, dp12 = p1 - p2;
    Float delDet = duv02[0] * duv12[1] - duv02[1] * duv12[0];
    if (delDet == 0)
        coordSystem(normalize(cross(p2 - p0, p1 - p0)), &dpdu, &dpdv);
    else {
        Float invDelDet = 1.0 / delDet;
        dpdu = ( duv12[1] * dp02 - duv02[1] * dp12) * invDelDet;
        dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * invDelDet;
    }

    // Compute error bounds for triangle intersection
    Float xAbsSum = (abs(b0 * p0.x) + abs(b1 * p1.x) + abs(b2 * p2.x));
    Float yAbsSum = (abs(b0 * p0.y) + abs(b1 * p1.y) + abs(b2 * p2.y));
    Float zAbsSum = (abs(b0 * p0.z) + abs(b1 * p1.z) + abs(b2 * p2.z));
    Vector3f pError = gamma(7) * Vector3f(xAbsSum, yAbsSum, zAbsSum);

    // Interpolate (u,v) parametric coordinates and hit point
    Point3f pHit = b0 * p0 + b1 * p1 + b2 * p2;
    Point2f uvHit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];

    // Test intersection against alpha texture, if present
    if (testAlphaTexture && mesh->alphaMask) {
        SurfaceInteraction isectLocal(pHit, Vector3f(), uvHit, -ray.d, dpdu, dpdv, Normal3f(), Normal3f(),
                                      ray.time, this);
        if (mesh->alphaMask->evaluate(isectLocal) == 0)
            return false;
    }

    // Fill in $SurfaceInteraction from triangle hit
    *isect = SurfaceInteraction(pHit, pError, uvHit, -ray.d, dpdu, dpdv, Normal3f(), Normal3f(), ray.time, this);
    // Override surface normal in case of incorrect orientation
    isect->n = isect->shading.n = Normal3f(normalize(cross(dp02, dp12)));
    if (mesh->n && mesh->s) {
        // Initialize shading geometry
        Normal3f ns;
        if (mesh->n) {
            ns = (b0 * mesh->n[v[0]] + b1 * mesh->n[v[1]] + b2 * mesh->n[v[2]]);
            if (ns.lengthSq() > 0)
                ns = normalize(ns);
        } else
            ns = isect->n;

        Vector3f ss;
        if (mesh->s) {
            ss = (b0 * mesh->s[v[0]] + b1 * mesh->s[v[1]] + b2 * mesh->s[v[2]]);
            if (ss.lengthSq() > 0)
                ss = normalize(ss);
        } else
            ss = normalize(isect->dpdu);

        Vector3f ts = cross(ss, ns);
        if (ts.lengthSq() > 0.f) {
            ts = normalize(ts);
            ss = cross(ts, ns); // adjust ss to ensure coordinate system is orthogonal
        } else
            coordSystem((Vector3f)ns, &ss, &ts);

        Normal3f dndu, dndv; // similar to dpdu and dpdv
        if (mesh->n) {
            Vector2f duv02 = uv[0] - uv[2];
            Vector2f duv12 = uv[1] - uv[2];
            Normal3f dn1 = mesh->n[v[0]] - mesh->n[v[2]];
            Normal3f dn2 = mesh->n[v[1]] - mesh->n[v[2]];
            Float nDelDet = duv02[0] * duv12[1] - duv02[1] * duv12[0];
            if (abs(nDelDet) < 1e-8) { // degenerate UVs
                // still compute dndu and dndv, with respect to the same arbitrary coordinate system
                // triangles with degenerate parameterizations are still reasonable
                Vector3f dn = cross(Vector3f(mesh->n[v[2]] - mesh->n[v[0]]),
                        Vector3f(mesh->n[v[1]] - mesh->n[v[0]]));
                if (dn.lengthSq() == 0)
                    dndu = dndv = Normal3f(0, 0, 0);
                else {
                    Vector3f dnu, dnv;
                    coordSystem(dn, &dnu, &dnv);
                    dndu = Normal3f(dnu);
                    dndv = Normal3f(dnv);
                }
            } else {
                Float invDet = 1 / nDelDet;
                dndu = (duv12[1] * dn1 - duv02[1] * dn2) * invDet;
                dndv = (-duv12[0] * dn1 + duv02[0] * dn2) * invDet;
            }
        } else
            dndu = dndv = Normal3f(0, 0, 0);

        isect->setShadingGeometry(ss, ts, dndu, dndv, true);
    }

    // Ensure correct orientation of normal
    if (mesh->n)
        isect->n = faceforward(isect->n, isect->shading.n);
    else if (reverseOrientation ^ transformSwapsHandedness)
        isect->n = isect->shading.n = -isect->n;

    *tHit = t;
    ++nTriangleHits;
    return true;
}

bool Triangle::intersectP(const Ray &ray, bool testAlphaTexture) const {
    ProfilePhase phase(Profiler::Stage::TriIntersectP);
    nTriangleTests++;

    // Get triangle vertices
    const auto &p0 = mesh->p[v[0]];
    const auto &p1 = mesh->p[v[1]];
    const auto &p2 = mesh->p[v[2]];

    // Perform ray-triangle intersection test
    // Transform vertices to ray coordinate space
    auto p0t = p0 - Vector3f(ray.o), p1t = p1 - Vector3f(ray.o), p2t = p2 - Vector3f(ray.o); // translate
    int kz = maxDim(abs(ray.d)); int kx = (kz + 1) % 3; int ky = (kx + 1) % 3; // permute
    auto d = permute(ray.d, kx, ky, kz);
    Float Sx = -d.x / d.z, Sy = -d.y / d.z, Sz = 1.f / d.z; // shear
    p0t.x += Sx * p0t.z; p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z; p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z; p2t.y += Sy * p2t.z;

    // Compute edge coefficients (P16 Equation 3.2)
    Float e0 = p1t.x * p2t.y - p1t.y * p2t.x;
    Float e1 = p2t.x * p0t.y - p2t.y * p0t.x;
    Float e2 = p0t.x * p1t.y - p0t.y * p1t.x;

    // Double precision test at edges
    if (sizeof(Float) == sizeof(float) &&
        (e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
        double p2txp1ty = double(p2t.x) * double(p1t.y);
        double p2typ1tx = double(p2t.y) * double(p1t.x);
        e0 = float(p2typ1tx - p2txp1ty);
        double p0txp2ty = double(p0t.x) * double(p2t.y);
        double p0typ2tx = double(p0t.y) * double(p2t.x);
        e1 = float(p0typ2tx - p0txp2ty);
        double p1txp0ty = double(p1t.x) * double(p0t.y);
        double p1typ0tx = double(p1t.y) * double(p0t.x);
        e2 = float(p1typ0tx - p1txp0ty);
    }

    // Perform triangle edge and determinant tests
    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
        return false;
    Float areaDet = e0 + e1 + e2;
    if (areaDet == 0)
        return false;

    // Compute scaled hit distance to triangle and test againt ray t range
    p0t.z *= Sz; p1t.z *= Sz; p2t.z *= Sz;
    Float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if (areaDet < 0 && (tScaled >= 0 || tScaled < ray.tMax * areaDet))
        return false;
    else if (areaDet > 0 && (tScaled <= 0 || tScaled > ray.tMax * areaDet))
        return false;

    // Compute barycentric coordinates and t value for triangle intersection
    Float invAreaDet = 1 / areaDet;
    Float b0 = e0 * invAreaDet, b1 = e1 * invAreaDet, b2 = e2 * invAreaDet;
    Float t = tScaled * invAreaDet;

    // Ensure that computed triangle t is conservatively greater than zero
    Float maxZt = maxComp(abs(Vector3f(p0t.z, p1t.z, p2t.z)));
    Float deltaZ = gamma(3) * maxZt;
    Float maxXt = maxComp(abs(Vector3f(p0t.x, p1t.x, p2t.x)));
    Float maxYt = maxComp(abs(Vector3f(p0t.y, p1t.y, p2t.y)));
    Float deltaX = gamma(5) * (maxXt + maxZt);
    Float deltaY = gamma(5) * (maxYt + maxZt);
    Float deltaE = 2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);
    Float maxE = maxComp(abs(Vector3f(e0, e1, e2)));
    Float deltaT = 3 * (gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) * abs(invAreaDet);
    if (t <= deltaT) return false;

    // Compute triangle partial derivatives
    Vector3f dpdu, dpdv;
    Point2f uv[3];
    getUVs(uv);
    Vector2f duv02 = uv[0] - uv[2], duv12 = uv[1] - uv[2];
    Vector3f dp02 = p0 - p2, dp12 = p1 - p2;
    Float delDet = duv02[0] * duv12[1] - duv02[1] * duv12[0];
    if (delDet == 0)
        coordSystem(normalize(cross(p2 - p0, p1 - p0)), &dpdu, &dpdv);
    else {
        Float invDelDet = 1.0 / delDet;
        dpdu = ( duv12[1] * dp02 - duv02[1] * dp12) * invDelDet;
        dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * invDelDet;
    }

    // Interpolate (u,v) parametric coordinates and hit point
    Point3f pHit = b0 * p0 + b1 * p1 + b2 * p2;
    Point2f uvHit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];

    // Test intersection against alpha texture, if present
    if (testAlphaTexture && mesh->alphaMask) {
        SurfaceInteraction isectLocal(pHit, Vector3f(), uvHit, -ray.d, dpdu, dpdv, Normal3f(), Normal3f(),
                                      ray.time, this);
        if (mesh->alphaMask->evaluate(isectLocal) == 0)
            return false;
    }

    ++nTriangleHits;
    return true;
}

Float Triangle::area() const {
    const auto &p0 = mesh->p[v[0]];
    const auto &p1 = mesh->p[v[1]];
    const auto &p2 = mesh->p[v[2]];
    return 0.5 * cross(p1 - p0, p2 - p0).length();
}

int Subdivision::SDVertex::valence() {
    auto f = startFace;
    unsigned nFace = 1;
    if (!boundary) { // interior vertex
        while ((f = f->nextFace(this)) != startFace) nFace++;
        return nFace;
    } else {
        while ((f = f->nextFace(this)) != nullptr) nFace++;
        f = startFace;
        while ((f = f->prevFace(this)) != nullptr) nFace++;
        return nFace + 1;
    }
}

void Subdivision::SDVertex::oneRing(Point3f *p) {
    if (!boundary) {
        auto face = startFace;
        do {
            *p++ = face->nextVert(this)->p;
            face = face->nextFace(this);
        } while (face != startFace);
    } else {
        SDFace *face  = startFace, *f2;
        while ((f2 = face->nextFace(this)) != nullptr)
            face = f2;
        *p++ = face->nextVert(this)->p;
        do {
            *p++ = face->prevVert(this)->p;
            face = face->prevFace(this);
        } while (face != nullptr);
    }
}

Point3f Subdivision::weightOneRing(SDVertex *vert, Float beta) {
    int valence = vert->valence();
    Point3f *pRing = ALLOCA(Point3f, valence);
    vert->oneRing(pRing);
    Point3f p = (1 - valence * beta) * vert->p;
    for (int i = 0; i < valence; i++)
        p += beta * pRing[i];
    return p;
}

Point3f Subdivision::weightBoundary(SDVertex *vert, Float beta) {
    int valence = vert->valence();
    Point3f *pRing = ALLOCA(Point3f, valence);
    vert->oneRing(pRing);
    Point3f p = (1 - 2 * beta) * vert->p;
    p += beta * pRing[0];
    p += beta * pRing[valence - 1];
    return p;
}

vector<shared_ptr<Shape>> Subdivision::subdivide(const Transform *objToWorld, const Transform *worldToObj,
                                                 bool reverseOrientation, int nLevels, int nIndices,
                                                 const int *vertexIndices, int nVertices, const Point3f *p)
{
    vector<SDVertex *> vertices;
    vector<SDFace *> faces;

    // Allocate vertices and faces
    unique_ptr<SDVertex[]> vertArr(new SDVertex[nVertices]);
    for (int i = 0; i < nVertices; ++i) {
        vertArr[i] = SDVertex(p[i]);
        vertices.push_back(&vertArr[i]);
    }
    int nFaces = nIndices / 3;
    unique_ptr<SDFace[]> faceArr(new SDFace[nFaces]);
    for (int i = 0; i < nFaces; ++i)
        faces.push_back(&faceArr[i]);

    // Set face to vertex pointers
    const int *vp = vertexIndices;
    for (int i = 0; i < nFaces; i++, vp += 3) {
        auto f = faces[i];
        for (unsigned j = 0; j < 3; j++) {
            auto v = vertices[vp[j]];
            f->v[j] = v;
            v->startFace = f;
        }
    }

    // Set neighbor pointers in faces
    set<SDEdge> edges;
    for (int i = 0; i < nFaces; i++) {
        auto f = faces[i];
        for (int edgeNum = 0; edgeNum < 3; edgeNum++) {
            int v0 = edgeNum, v1 = NEXT(edgeNum);
            SDEdge e(f->v[v0], f->v[v1]);
            if (edges.find(e) == edges.end()) { // new edge
                e.f[0] = f;
                e.f0EdgeNum = edgeNum;
                edges.insert(e);
            } else { // preexisting edge
                e = *edges.find(e);
                e.f[0]->f[e.f0EdgeNum] = f;
                f->f[edgeNum] = e.f[0];
                edges.erase(e);
            }
        }
    }

    // Finish vertex initialization
    for (int i = 0; i < nVertices; i++) {
        auto v = vertices[i];
        auto f = v->startFace;
        do
            f = f->nextFace(v);
        while (f && f != v->startFace);
        v->boundary = (f == nullptr);
        v->regular = (!v->boundary && v->valence() == 6) || (v->boundary && v->valence() == 4);
    }

    // Refine subdivision mesh into triangles
    auto f = faces;
    auto v = vertices;
    MemoryArena arena;

    for (int i = 0; i < nLevels; i++) {
        // Update f and v for next level of subdivision
        vector<SDFace *> newFaces;
        vector<SDVertex *> newVertices;

        // Allocate next level of children in mesh tree
        for (auto vertex : v) {
            vertex->child = arena.alloc<SDVertex>();
            vertex->child->regular = vertex->regular;
            vertex->child->boundary = vertex->boundary;
            newVertices.push_back(vertex->child);
        }
        for (auto face : f)
            for (int k = 0; k < 4; k++) {
                face->children[k] = arena.alloc<SDFace>();
                newFaces.push_back(face->children[k]);
            }

        // Update vertex positions and create new edge vertices
        for (auto vertex : v)
            vertex->child->p = vertex->boundary ? weightBoundary(vertex, 1.0 / 8.0)
                                                : vertex->regular ? weightOneRing(vertex, 1.0 / 16.0)
                                                                  : weightOneRing(vertex, beta(vertex->valence()));
        map<SDEdge, SDVertex *> edgeVerts;
        for (auto face : f)
            for (int k = 0; k < 3; k++) {
                SDEdge edge(face->v[k], face->v[NEXT(k)]);
                auto vert = edgeVerts[edge];
                if (!vert) {
                    vert = arena.alloc<SDVertex>();
                    newVertices.push_back(vert);
                    vert->regular = true;
                    vert->boundary = (face->f[k] == nullptr);
                    vert->startFace = face->children[3]; // newly added face in the center of triangle
                    if (vert->boundary) {
                           vert->p =  0.5 * edge.v[0]->p;
                           vert->p += 0.5 * edge.v[1]->p;
                       } else {
                           vert->p =  3.0 / 8.0 * edge.v[0]->p;
                           vert->p += 3.0 / 8.0 * edge.v[1]->p;
                           vert->p += 1.0 / 8.0 * face->otherVert(edge.v[0], edge.v[1])->p;
                           vert->p += 1.0 / 8.0 * face->f[k]->otherVert(edge.v[0], edge.v[1])->p;
                    }
                }
                edgeVerts[edge] = vert;
            }

        // Update new mesh topology
        // Update even vertex face pointers
        for (auto vertex : v) {
            int vertNum = vertex->startFace->vNum(vertex);
            vertex->child->startFace = vertex->startFace->children[vertNum];
        }
        // Update face neighbor pointers
        for (auto face : f)
            for (int j = 0; j < 3; j++) {
                face->children[3]->f[j] = face->children[NEXT(j)]; // neighor faces of the same parent
                face->children[j]->f[NEXT(j)] = face->children[3];
                auto f2 = face->f[j]; // of other parents
                face->children[j]->f[j] = f2 ? f2->children[f2->vNum(face->v[j])] : nullptr;
                f2 = face->f[PREV(j)];
                face->children[j]->f[PREV(j)] = f2 ? f2->children[f2->vNum(face->v[j])] : nullptr;
            }
        // Update face vertex pointers
        for (auto face : f)
            for (int j = 0; j < 3; j++) {
                face->children[j]->v[j] = face->v[j]->child;
                SDVertex *vert = edgeVerts[SDEdge(face->v[j], face->v[NEXT(j)])];
                face->children[j]->v[NEXT(j)] = vert;
                face->children[NEXT(j)]->v[j] = vert;
                face->children[3]->v[j] = vert;
            }

        // Prepare for next level of subdivision
        f = newFaces;
        v = newVertices;
    }

    // Push vertices to limit surface
    unique_ptr<Point3f[]> pLimit(new Point3f[v.size()]);
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i]->boundary)
            pLimit[i] = weightBoundary(v[i], 1.f / 5.f);
        else
            pLimit[i] = weightOneRing(v[i], loopGamma(v[i]->valence()));
    }
    for (size_t i = 0; i < v.size(); ++i) v[i]->p = pLimit[i];

    // Compute vertex tangents on limit surface
    vector<Normal3f> Ns;
    Ns.reserve(v.size());
    vector<Point3f> pRing(16, Point3f());
    for (auto vertex : v) {
        Vector3f S(0, 0, 0), T(0, 0, 0);
        int valence = vertex->valence();
        if (valence > (int)pRing.size()) pRing.resize(valence);
        vertex->oneRing(&pRing[0]);
        if (!vertex->boundary) {
            // Compute tangents of interior face
            for (int j = 0; j < valence; ++j) {
                S += cos(2 * PI * j / valence) * Vector3f(pRing[j]);
                T += sin(2 * PI * j / valence) * Vector3f(pRing[j]);
            }
        } else {
            // Compute tangents of boundary face
            S = pRing[valence - 1] - pRing[0];
            if (valence == 2)
                T = Vector3f(pRing[0] + pRing[1] - 2 * vertex->p);
            else if (valence == 3)
                T = pRing[1] - vertex->p;
            else if (valence == 4)  // regular
                T = Vector3f(-1 * pRing[0] + 2 * pRing[1] + 2 * pRing[2] +
                             -1 * pRing[3] + -2 * vertex->p);
            else {
                Float theta = PI / float(valence - 1);
                T = Vector3f(sin(theta) * (pRing[0] + pRing[valence - 1]));
                for (int k = 1; k < valence - 1; ++k) {
                    Float wt = (2 * cos(theta) - 2) * sin((k)*theta);
                    T += Vector3f(wt * pRing[k]);
                }
                T = -T;
            }
        }
        Ns.push_back(Normal3f(cross(S, T)));
    }

    // Create triangle mesh from subdivision mesh
    {
        size_t ntris = f.size();
        unique_ptr<int[]> verts(new int[3 * ntris]);
        int *vp = verts.get();
        size_t totVerts = v.size();
        map<SDVertex *, int> usedVerts;
        for (unsigned i = 0; i < totVerts; ++i) usedVerts[v[i]] = i;
        for (unsigned i = 0; i < ntris; ++i)
            for (int j = 0; j < 3; ++j) {
                *vp = usedVerts[f[i]->v[j]];
                ++vp;
            }

        return TriangleMesh::create(objToWorld, worldToObj, reverseOrientation, ntris, verts.get(),
                                    totVerts, pLimit.get(), nullptr, &Ns[0], nullptr, nullptr);
    }
}
