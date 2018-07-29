#include "dots.h"
#include "paramset.h"

template <class T>
T DotsTexture<T>::evaluate(const SurfaceInteraction &si) const {
    // Compute cell indices for dots
    Vector2f dstdx, dstdy;
    Point2f st = mapping->map(si, &dstdx, &dstdy);
    int sCell = int(floorf(st[0] + 0.5f)), tCell = int(floorf(st[1] + 0.5f));

    // Return inside texture results
    if (Noise::perlin(sCell + 0.5f, tCell + 0.5f) > 0) {
        constexpr float radius = 0.35f;
        constexpr float maxShift = 0.5f - radius;
        float sCenter = sCell + maxShift * Noise::perlin(sCell + 1.5f, tCell + 2.8f);
        float tCenter = tCell + maxShift * Noise::perlin(sCell + 4.5f, tCell + 9.8f);
        Vector2f dst = st - Point2f(sCenter, tCenter);
        if (dst.lengthSq() < SQ(radius))
            return inside->evaluate(si);
    }
    return outside->evaluate(si);
}

template <class T>
DotsTexture<T> * DotsTexture<T>::create(const Transform &tex2world, const TextureParams &tp) {
    auto map = TextureMapping2D::create(tex2world, tp);
    return new DotsTexture<T>(move(map), tp.getTexture("inside", T(0.0f)), tp.getTexture("outside", T(1.0f)));
}

template class DotsTexture<float>;
template class DotsTexture<Spectrum>;
