#ifndef TEXTURE_IMAGE
#define TEXTURE_IMAGE

#include "core/texture.h"
#include "mipmap.h"
#include "imageio.h"

struct TexInfo {
    TexInfo(const string &filename, bool doTrilinear, float maxAniso, ImageWrap wrap, float scale, bool gamma)
        : filename(filename), doTrilinear(doTrilinear), maxAniso(maxAniso), wrapMode(wrap), scale(scale),
          gamma(gamma) {}

    bool operator < (const TexInfo &t2) const {
        if (filename != t2.filename) return filename < t2.filename;
        if (doTrilinear != t2.doTrilinear) return doTrilinear < t2.doTrilinear;
        if (maxAniso != t2.maxAniso) return maxAniso < t2.maxAniso;
        if (scale != t2.scale) return scale < t2.scale;
        if (gamma != t2.gamma) return !gamma;
        return wrapMode < t2.wrapMode;
    }

    string filename;
    bool doTrilinear;
    float maxAniso;
    ImageWrap wrapMode;
    float scale;
    bool gamma;
};

template <class Tmem, class Tret>
class ImageTexture : public Texture<Tret> {
public:
    ImageTexture(unique_ptr<TextureMapping2D> mapping, const string &filename, bool doTrilinear,
                 float maxAniso, ImageWrap wrap, float scale, bool gamma) 
        : mapping(move(mapping)) {
        mipmap = getTexture(filename, doTrilinear, maxAniso, wrap, scale, gamma);
    }

    static Mipmap<Tmem> * getTexture(const string &filename, bool doTrilinear, float maxAniso, ImageWrap wrap,
                                     float scale, bool gamma);

    Tret evaluate(const SurfaceInteraction &si) const;

    static void clearCache() {
        textures.erase(textures.begin(), textures.end());
    }

private:
    static ImageTexture<Tmem, Tret> * create(const Transform &tex2world, const TextureParams &tp);

    static void convertIn(const RGBSpectrum &from, RGBSpectrum *to, float scale, bool gamma) {
        for (int i = 0; i < RGBSpectrum::NUM_SAMPLES; i++)
            (*to)[i] = scale * (gamma ? ImageIO::inverseGammaCorrect(from[i]) : from[i]);
    }

    static void convertIn(const RGBSpectrum &from, float *to, float scale, bool gamma) {
        *to = scale * (gamma ? ImageIO::inverseGammaCorrect(from.luminance()) : from.luminance());
    }

    static void convertOut(const RGBSpectrum &from, Spectrum *to) {
        float rgb[3];
        from.toRGB(rgb);
        *to = Spectrum::fromRGB(rgb);
    }

    static void convertOut(float from, float *to) { *to = from; }

    unique_ptr<TextureMapping2D> mapping;
    Mipmap<Tmem> *mipmap;
    static map<TexInfo, unique_ptr<Mipmap<Tmem>>> textures;

    template <class T>
    friend class ImageTextureCreator;
};

template <class T>
class ImageTextureCreator {
public:
    static Texture<T> * create(const Transform &tex2world, const TextureParams &tp) {
        Texture<T> *ptr;
        create(tex2world, tp, &ptr);
        return ptr;
    }

private:
    static void create(const Transform &tex2world, const TextureParams &tp, Texture<float> **tex) {
        *tex = ImageTexture<float, float>::create(tex2world, tp);
    }

    static void create(const Transform &tex2world, const TextureParams &tp, Texture<Spectrum> **tex) {
        *tex = ImageTexture<RGBSpectrum, Spectrum>::create(tex2world, tp);
    }
};

extern template class ImageTexture<float, float>;
extern template class ImageTexture<RGBSpectrum, Spectrum>;

#endif // TEXTURE_IMAGE
