#include "imagemap.h"
#include "log.h"
#include "paramset.h"
#include "file.h"

template <class Tmem, class Tret>
Mipmap<Tmem> * ImageTexture<Tmem, Tret>::getTexture(const string &filename, bool doTrilinear, float maxAniso,
                                                    ImageWrap wrap, float scale, bool gamma)
{
    TexInfo info(filename, doTrilinear, maxAniso, wrap, scale, gamma);
    if (textures.find(info) != textures.end()) // texture cache is present
        return textures[info].get();

    // Create Mipmap for _filename_
    Point2i resolution;
    auto texels = ImageIO::readImage(filename, &resolution);
    Mipmap<Tmem> *mipmap = nullptr;

    if (texels) {
        // Flip image in y
        for (int y = 0; y < resolution.y / 2; ++y)
            for (int x = 0; x < resolution.x; ++x) {
                int o1 = y * resolution.x + x;
                int o2 = (resolution.y - 1 - y) * resolution.x + x;
                swap(texels[o1], texels[o2]);
            }

        // Convert texels to type _Tmem_ and create _Mipmap
        unique_ptr<Tmem[]> convertedTexels(new Tmem[resolution.x * resolution.y]);
        for (int i = 0; i < resolution.x * resolution.y; i++)
            convertIn(texels[i], &convertedTexels[i], scale, gamma);
        mipmap = new Mipmap<Tmem>(resolution, convertedTexels.get(), doTrilinear, maxAniso, wrap);
    } else {
        // Create one-valued _Mipmap_
        WARNING("Creating a constant grey texture to replace \"%s\".", filename.c_str());
        Tmem oneVal = scale;
        mipmap = new Mipmap<Tmem>(Point2i(1, 1), &oneVal);
    }

    textures[info].reset(mipmap);
    return mipmap;
}

template <class Tmem, class Tret>
Tret ImageTexture<Tmem, Tret>::evaluate(const SurfaceInteraction &si) const {
    Vector2f dstdx, dstdy;
    Point2f st = mapping->map(si, &dstdx, &dstdy);
    Tmem mem = mipmap->lookup(st, dstdx, dstdy);
    Tret ret;
    convertOut(mem, &ret);
    return ret;
}

template <class Tmem, class Tret>
ImageTexture<Tmem, Tret> *ImageTexture<Tmem, Tret>::create(const Transform &tex2world, const TextureParams &tp)
{
    // Initialize 2D texture mapping _map_ from _tp_
    unique_ptr<TextureMapping2D> map;
    string type = tp.findString("mapping", "uv");
    if (type == "uv") {
        float su = tp.findFloat("uscale", 1.);
        float sv = tp.findFloat("vscale", 1.);
        float du = tp.findFloat("udelta", 0.);
        float dv = tp.findFloat("vdelta", 0.);
        map.reset(new UVMapping2D(su, sv, du, dv));
    } else if (type == "spherical")
        map.reset(new SphericalMapping2D(tex2world.inverse()));
    else if (type == "cylindrical")
        map.reset(new CylindricalMapping2D(tex2world.inverse()));
    else if (type == "planar")
        map.reset(new PlanarMapping2D(tp.findVector3f("v1", Vector3f(1, 0, 0)),
                                      tp.findVector3f("v2", Vector3f(0, 1, 0)),
                                      tp.findFloat("udelta", 0.f), tp.findFloat("vdelta", 0.f)));
    else {
        ERROR("2D texture mapping \"%s\" unknown", type.c_str());
        map.reset(new UVMapping2D);
    }

    // Initialize _ImageTexture_ parameters
    float maxAniso = tp.findFloat("maxanisotropy", 8.f);
    bool trilerp = tp.findBool("trilinear", false);
    string wrap = tp.findString("wrap", "repeat");
    ImageWrap wrapMode = ImageWrap::Repeat;
    if (wrap == "black")
        wrapMode = ImageWrap::Black;
    else if (wrap == "clamp")
        wrapMode = ImageWrap::Clamp;
    float scale = tp.findFloat("scale", 1.f);
    std::string filename = tp.findFilename("filename");
    bool gamma = tp.findBool("gamma", File::hasExtension(filename, ".tga") ||
                             File::hasExtension(filename, ".png"));
    return new ImageTexture<Tmem, Tret>(move(map), filename, trilerp, maxAniso, wrapMode, scale, gamma);
}


template class ImageTexture<float, float>;
template class ImageTexture<RGBSpectrum, Spectrum>;
