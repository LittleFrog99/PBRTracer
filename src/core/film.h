#ifndef CORE_FILM
#define CORE_FILM

#include "spectrum.h"
#include "bounds.h"
#include "filter.h"
#include "stats.h"
#include "parallel.h"
#include "paramset.h"

struct FilmTilePixel {
    Spectrum contribSum = 0.f;
    Float filterWeightSum = 0.f;
};

class FilmTile {
public:
    FilmTile(const Bounds2i &pixelBounds, const Vector2f &filterRadius,
             const Float *filterTable, int filterTableSize,
             Float maxSampleLuminance)
        : pixelBounds(pixelBounds), filterRadius(filterRadius),
          invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
          filterTable(filterTable), filterTableSize(filterTableSize),
          maxSampleLuminance(maxSampleLuminance) {
        pixels = vector<FilmTilePixel>(max(0, pixelBounds.area()));
    }

    void addSample(const Point2f &pFilm, Spectrum L, Float sampleWeight = 1.0);

    FilmTilePixel & getPixel(const Point2i &p) {
        CHECK(insideExclusive(p, pixelBounds));
        int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
        int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
        return pixels[offset];
    }

    const FilmTilePixel & getPixel(const Point2i &p) const {
        CHECK(insideExclusive(p, pixelBounds));
        int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
        int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
        return pixels[offset];
    }

    Bounds2i getPixelBounds() const { return pixelBounds; }

private:
    const Bounds2i pixelBounds;
    const Vector2f filterRadius, invFilterRadius;
    const Float *filterTable;
    const int filterTableSize;
    vector<FilmTilePixel> pixels;
    const Float maxSampleLuminance;
    friend class Film;

};

class Film {
public:
    Film(const Point2i &resolution, const Bounds2f &cropWindow, unique_ptr<Filter> filter,
         Float diagonal, const string &filename, Float scale, Float maxSampleLuminance = INFINITY);
    Bounds2i getSampleBounds() const;
    Bounds2f getPhysicalExtent() const;
    unique_ptr<FilmTile> getFilmTile(const Bounds2i &sampleBounds);
    void mergeFilmTile(unique_ptr<FilmTile> tile);
    void setImage(const Spectrum *img) const;
    void addSplat(const Point2f &p, Spectrum v);
    void writeImage(Float splatScale = 1);
    void clear();

    Film * create(const ParamSet &params, unique_ptr<Filter> filter);

    const Point2i fullResolution;
    const Float diagonal; // NDC space, in meters
    unique_ptr<Filter> filter;
    const string filename;
    Bounds2i croppedPixelBounds;

private:
    struct Pixel {
        Pixel() { xyz[0] = xyz[1] = xyz[2] = filterWeightSum = 0; }
        Float xyz[3];
        Float filterWeightSum;
        AtomicFloat splatXYZ[3];
        Float pad; // padding to ensure 32 bytes for float or 64 bytes for double
    };

    unique_ptr<Pixel[]> pixels;
    static constexpr int FILTER_TABLE_WIDTH = 16;
    Float filterTable[SQ(FILTER_TABLE_WIDTH)];
    mutex mutex;
    const Float scale;
    const Float maxSampleLuminance;

    Pixel & getPixel(const Point2i &p) {
        CHECK(insideExclusive(p, croppedPixelBounds));
        int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
        int offset = (p.x - croppedPixelBounds.pMin.x) +
                     (p.y - croppedPixelBounds.pMin.y) * width;
        return pixels[offset];
    }
};



#endif // CORE_FILM
