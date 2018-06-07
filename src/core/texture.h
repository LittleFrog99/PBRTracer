#ifndef TEXTURE_H
#define TEXTURE_H


template <class T>
class Texture {
public:
    Texture();
};

template <class T>
class ConstantTexture : public Texture<T> {
public:
};

#endif // TEXTURE_H
