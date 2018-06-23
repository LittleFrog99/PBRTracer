#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "core/shape.h"

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
    shared_ptr<Texture<Float>> shadowAlphaMask; // newly added in pbrt
    vector<int> faceIndices; // newly added in pbrt
};

class Triangle : public Shape {
public:
    Triangle(const Transform *objToWorld, const Transform *worldToObj, bool revOrient,
             const shared_ptr<TriangleMesh> &mesh, int triNumber)
        : Shape(objToWorld, worldToObj, revOrient), mesh(mesh)
    { v = &mesh->vertexIndices[3 * triNumber]; }

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
};

#endif // TRIANGLE_H
