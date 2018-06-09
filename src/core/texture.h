#ifndef TEXTURE_H
#define TEXTURE_H


template <class T>
class Texture {
public:
    Texture() {}
};

template <class T>
class ConstantTexture : public Texture<T> {
public:
    ConstantTexture(const T &value) : value(value) {}

private:
    T value;
};

#endif // TEXTURE_H
