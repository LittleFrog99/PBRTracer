#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "core/shape.h"
#include "log.h"

struct TriangleMesh {
    TriangleMesh(const Transform &objToWorld, int nTriangles, const int *vertexIndices, int nVertices,
                 const Point3f *P, const Vector3f *S, const Normal3f *N, const Point2f *UV,
                 const shared_ptr<Texture<Float>> &alphaMask);

    static vector<shared_ptr<Shape>> create(const Transform *objToWorld, const Transform *worldToObj,
                                            bool revOrient, int nTriangles, const int *vertexIndices, int nVertices,
                                            const Point3f *p, const Vector3f *s, const Normal3f *n, const Point2f *uv,
                                            const shared_ptr<Texture<Float>> &alphaMask);

    const int nTriangles, nVertices;
    vector<int> vertexIndices;
    unique_ptr<Point3f[]> p;
    unique_ptr<Normal3f[]> n;
    unique_ptr<Vector3f[]> s;
    unique_ptr<Point2f[]> uv;
    shared_ptr<Texture<Float>> alphaMask;
};

class Triangle : public Shape {
public:
    Triangle(const Transform *objToWorld, const Transform *worldToObj, bool revOrient,
             const shared_ptr<TriangleMesh> &mesh, int triNumber)
        : Shape(objToWorld, worldToObj, revOrient), mesh(mesh)
    {
        v = &mesh->vertexIndices[3 * triNumber];
    }

    Bounds3f objectBound() const;
    Bounds3f worldBound() const;

    bool intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const;
    bool intersectP(const Ray &ray, bool testAlphaTexture = true) const;

    Float area() const;

private:
    void getUVs(Point2f uv[3]) const {
        if (mesh->uv) {
            uv[0] = mesh->uv[v[0]];
            uv[1] = mesh->uv[v[1]];
            uv[2] = mesh->uv[v[2]];
        } else {
            uv[0] = Point2f(0, 0);
            uv[1] = Point2f(1, 0);
            uv[2] = Point2f(1, 1);
        }
    }

    shared_ptr<TriangleMesh> mesh;
    const int *v;
    // int faceIndex;
};

class Subdivision {
public:
    static vector<shared_ptr<Shape>> subdivide(const Transform *objToWorld, const Transform *worldToObj,
                                               bool reverseOrientation, int nLevels, int nIndices,
                                               const int *vertexIndices, int nVertices, const Point3f *p);

private:
#define NEXT(i) ((i + 1) % 3)
#define PREV(i) ((i + 2) % 3)

    struct SDFace;

    struct SDVertex {
        SDVertex(const Point3f &p = Point3f()) : p(p) {}
        int valence();
        void oneRing(Point3f *p);

        Point3f p;
        SDFace *startFace = nullptr;
        SDVertex *child = nullptr;
        bool regular = false, boundary = false;
    };

    struct SDFace  {
        SDFace() {}

        int vNum(SDVertex *vert) const {
            for (int i = 0; i < 3; i++)
                if (v[i] == vert) return i;
            SEVERE("Basic logic error in SDFace::vNum()");
            return -1;
        }

        SDVertex * otherVert(SDVertex *v0, SDVertex *v1) {
            for (int i = 0; i < 3; i++)
                if (v[i] != v0 && v[i] != v1)
                    return v[i];
            SEVERE("Basic logic error in SDVertex::otherVert()");
            return nullptr;
        }

        SDFace * nextFace(SDVertex *vert) { return f[vNum(vert)]; }
        SDFace * prevFace(SDVertex *vert) { return f[PREV(vNum(vert))]; }
        SDVertex * nextVert(SDVertex *vert) { return v[NEXT(vNum(vert))]; }
        SDVertex * prevVert(SDVertex *vert) { return v[PREV(vNum(vert))]; }

        SDVertex *v[3] = { nullptr, nullptr, nullptr };
        SDFace *f[3] = { nullptr, nullptr, nullptr }; // neighboring faces
        SDFace *children[4] = { nullptr, nullptr, nullptr, nullptr };
    };

    struct SDEdge {
        SDEdge(SDVertex *v0 = nullptr, SDVertex *v1 = nullptr) {
            v[0] = min(v0, v1);
            v[1] = max(v0, v1);
        }

        bool operator < (const SDEdge &e2) const {
            if (v[0] == e2.v[0]) return v[1] < e2.v[1];
            return v[0] < e2.v[0];
        }

        SDVertex *v[2];
        SDFace *f[2] = { nullptr, nullptr };
        int f0EdgeNum = -1;
    };

    static Float beta(int valence) { // used to weigh nearby vertices
        if (valence == 3) return 3.0 / 16.0;
        else return 3.0 / (8.0 * valence);
    }

    static Float loopGamma(int valence) {
        return 1.f / (valence + 3.f / (8.f * beta(valence)));
    }

    static Point3f weightOneRing(SDVertex *vert, Float beta);
    static Point3f weightBoundary(SDVertex *vert, Float beta);

};

#endif // TRIANGLE_H
