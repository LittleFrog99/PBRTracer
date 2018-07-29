#include "texture.h"
#include "paramset.h"

Point2f SphericalMapping2D::map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const
{
    Point2f st = sphere(si.p);
    // Compute texture coordinate differentials for sphere (u, v) mapping
    const float delta = .1f;
    Point2f stDeltaX = sphere(si.p + delta * si.dpdx);
    *dstdx = (stDeltaX - st) / delta;
    Point2f stDeltaY = sphere(si.p + delta * si.dpdy);
    *dstdy = (stDeltaY - st) / delta;
    // Handle sphere mapping discontinuity for coordinate differentials
    if ((*dstdx)[1] > 0.5f) (*dstdx)[1] = 1 - (*dstdx)[1];
    else if ((*dstdx)[1] < -.5f) (*dstdx)[1] = -((*dstdx)[1] + 1);
    if ((*dstdy)[1] > 0.5f) (*dstdy)[1] = 1 - (*dstdy)[1];
    else if ((*dstdy)[1] < -.5f) (*dstdy)[1] = -((*dstdy)[1] + 1);
    return st;
}

Point2f CylindricalMapping2D::map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const
{
    Point2f st = cylinder(si.p);
    // Compute texture coordinate differentials for sphere (u, v) mapping
    const float delta = .1f;
    Point2f stDeltaX = cylinder(si.p + delta * si.dpdx);
    *dstdx = (stDeltaX - st) / delta;
    Point2f stDeltaY = cylinder(si.p + delta * si.dpdy);
    *dstdy = (stDeltaY - st) / delta;
    // Handle sphere mapping discontinuity for coordinate differentials
    if ((*dstdx)[1] > 0.5f) (*dstdx)[1] = 1 - (*dstdx)[1];
    else if ((*dstdx)[1] < -.5f) (*dstdx)[1] = -((*dstdx)[1] + 1);
    if ((*dstdy)[1] > 0.5f) (*dstdy)[1] = 1 - (*dstdy)[1];
    else if ((*dstdy)[1] < -.5f) (*dstdy)[1] = -((*dstdy)[1] + 1);
    return st;
}

unique_ptr<TextureMapping2D> TextureMapping2D::create(const Transform &tex2world, const TextureParams &tp)
{
    TextureMapping2D *map;
    string type = tp.findString("mapping", "uv");
    if (type == "uv") {
        float su = tp.findFloat("uscale", 1.);
        float sv = tp.findFloat("vscale", 1.);
        float du = tp.findFloat("udelta", 0.);
        float dv = tp.findFloat("vdelta", 0.);
        map = new UVMapping2D(su, sv, du, dv);
    } else if (type == "spherical")
        map = new SphericalMapping2D(tex2world.inverse());
    else if (type == "cylindrical")
        map = new CylindricalMapping2D(tex2world.inverse());
    else if (type == "planar")
        map = new PlanarMapping2D(tp.findVector3f("v1", Vector3f(1, 0, 0)),
                                  tp.findVector3f("v2", Vector3f(0, 1, 0)),
                                  tp.findFloat("udelta", 0.f), tp.findFloat("vdelta", 0.f));
    else {
        ERROR("2D texture mapping \"%s\" unknown", type.c_str());
        map = new UVMapping2D;
    }
    return unique_ptr<TextureMapping2D>(map);
}

template <class T>
ConstantTexture<T> * ConstantTexture<T>::create(const Transform &tex2world, const TextureParams &tp) {
    return new ConstantTexture<T>(tp.findFloat("value", 1.f));
}

template class ConstantTexture<float>;
template class ConstantTexture<Spectrum>;

namespace Noise {

inline static float gradient(int x, int y, int z, float dx, float dy, float dz) {
    int h = NOISE_PERM[NOISE_PERM[NOISE_PERM[x] + y] + z];
    h &= 15; // lower 4 bits determine associated gradient vectors
    float u = (h < 8 || h == 12 || h == 13) ? dx : dy;
    float v = (h < 4 || h == 12 || h == 13) ? dy : dz;
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

inline static float noiseWeight(float t) {
    return 6 * POW5(t) - 15 * QUAD(t) + 10 * CUB(t);
}

float perlin(float x, float y, float z) {
    // Compute noise cell coordinates and offsets
    int ix = floorf(x), iy = floorf(y), iz = floorf(z);
    float dx = x - ix, dy = y - iy, dz = z - iz;

    // Compute gradient weights
    ix %= (NOISE_PERM_SIZE - 1);
    iy %= (NOISE_PERM_SIZE - 1);
    iz %= (NOISE_PERM_SIZE - 1);
    float w000 = gradient(ix, iy, iz, dx, dy, dz);
    float w001 = gradient(ix, iy, iz + 1, dx, dy, dz - 1);
    float w010 = gradient(ix, iy + 1, iz, dx, dy - 1, dz);
    float w011 = gradient(ix, iy + 1, iz + 1, dx, dy - 1, dz - 1);
    float w100 = gradient(ix + 1, iy, iz, dx - 1, dy, dz);
    float w101 = gradient(ix + 1, iy, iz + 1, dx - 1, dy, dz - 1);
    float w110 = gradient(ix + 1, iy + 1, iz, dx - 1, dy - 1, dz);
    float w111 = gradient(ix + 1, iy + 1, iz + 1, dx - 1, dy - 1, dz - 1);

    // Compute trilinear interpolation of weights
    float wx = noiseWeight(dx), wy = noiseWeight(dy), wz = noiseWeight(dz);
    float x00 = lerp(wx, w000, w100), x01 = lerp(wx, w001, w101);
    float x10 = lerp(wx, w010, w110), x11 = lerp(wx, w011, w111);
    float y0 = lerp(wy, x00, x10), y1 = lerp(wy, x01, x11);
    return lerp(wz, y0, y1);
}

inline static float smoothStep(float min, float max, float value) {
    float v = clamp((value - min) / (max - min), 0, 1);
    return -2 * CUB(v) + 3 * SQ(v);
}

float fBm(const Point3f &p, const Vector3f &dpdx, const Vector3f &dpdy, float omega, int maxOctaves)
{
    // Compute number of octaves
    float len2 = max(dpdx.lengthSq(), dpdy.lengthSq());
    float n = clamp(-1 - 0.5f * log2f(len2), 0, maxOctaves);
    int nInt = floorf(n);

    // Compute sum of octaves
    float sum = 0, lambda = 1, weight = 1;
    for (int i = 0; i < nInt; i++) {
        sum += weight * perlin(lambda * p);
        lambda *= 1.99f;
        weight *= omega;
    }
    float nPartial = n - nInt;
    sum += weight * smoothStep(0.3f, 0.7f, nPartial) * perlin(lambda * p);

    return sum;
}

float turbulence(const Point3f &p, const Vector3f &dpdx, const Vector3f &dpdy, float omega, int maxOctaves)
{
    // Compute number of octaves
    float len2 = max(dpdx.lengthSq(), dpdy.lengthSq());
    float n = clamp(-1 - 0.5f * log2f(len2), 0, maxOctaves);
    int nInt = floorf(n);

    // Compute sum of octaves
    float sum = 0, lambda = 1, weight = 1;
    for (int i = 0; i < nInt; i++) {
        sum += weight * perlin(lambda * p);
        lambda *= 1.99f;
        weight *= omega;
    }

    // Account for contributions of clamped octaves
    float nPartial = n - nInt;
    sum += weight * lerp(smoothStep(0.3f, 0.7f, nPartial), 0.2f, abs(perlin(lambda * p)));
    for (int i = nInt; i < maxOctaves; i++) {
        sum += weight * 0.2f;
        weight *= omega;
    }

    return sum;
}

int NOISE_PERM[2 * NOISE_PERM_SIZE] = {
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
    36, 103, 30, 69, 142,
    8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
    94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174,
    20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77,
    146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55,
    46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76,
    132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100,
    109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147,
    118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28,
    42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101,
    155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232,
    178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12,
    191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31,
    181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
    138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66,
    215, 61, 156, 180, // end first copy
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194,
    233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6,
    148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
    57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74,
    165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60,
    211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
    63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135,
    130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226,
    250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59,
    227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2,
    44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19,
    98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251,
    34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115,
    121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72,
    243, 141, 128, 195, 78, 66, 215, 61, 156, 180
};

}
