#ifndef TEXTURE_DOTS
#define TEXTURE_DOTS

#include "core/texture.h"

template <class T>
class DotsTexture : public Texture<T> {
public:
    DotsTexture(unique_ptr<TextureMapping2D> mapping, const shared_ptr<Texture<T>> &inside,
                const shared_ptr<Texture<T>> &outside)
        : mapping(move(mapping)), inside(inside), outside(outside) {}

    static DotsTexture<T> * create(const Transform &tex2world, const TextureParams &tp);

    T evaluate(const SurfaceInteraction &si) const;

private:
    unique_ptr<TextureMapping2D> mapping;
    shared_ptr<Texture<T>> inside, outside;
};

extern template class DotsTexture<float>;
extern template class DotsTexture<Spectrum>;

#endif // TEXTURE_DOTS
