#ifndef CORE_FILM
#define CORE_FILM

#include "spectrum.h"
#include "bounds.h"
#include "filter.h"
#include "parallel.h"

struct FilmTilePixel {
    Spectrum contribSum = 0.f;
    float filterWeightSum = 0.f;
};

class FilmTile {
public:
    FilmTile(const Bounds2i &pixelBounds, const Vector2f &filterRadius,
             const float *filterTable, int filterTableSize,
             float maxSampleLuminance)
        : pixelBounds(pixelBounds), filterRadius(filterRadius),
          invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
          filterTable(filterTable), filterTableSize(filterTableSize),
          maxSampleLuminance(maxSampleLuminance) {
        pixels = vector<FilmTilePixel>(max(0, pixelBounds.area()));
    }

    void addSample(const Point2f &pFilm, Spectrum L, float sampleWeight = 1.0);

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
    const float *filterTable;
    const int filterTableSize;
    vector<FilmTilePixel> pixels;
    const float maxSampleLuminance;
    friend class Film;
};

class Film {
public:
    Film(const Point2i &resolution, const Bounds2f &cropWindow, unique_ptr<Filter> filter,
         float diagonal, const string &filename, float scale, float maxSampleLuminance = INFINITY);
    static Film * create(const ParamSet &params, unique_ptr<Filter> filter);

    Bounds2i getSampleBounds() const;
    Bounds2f getPhysicalExtent() const;
    unique_ptr<FilmTile> getFilmTile(const Bounds2i &sampleBounds);
    void mergeFilmTile(unique_ptr<FilmTile> tile);
    void setImage(const Spectrum *img) const;
    void addSplat(const Point2f &p, Spectrum v);
    void writeImage(float splatScale = 1);
    void clear();

    const Point2i fullResolution;
    const float diagonal; // NDC space, in meters
    unique_ptr<Filter> filter;
    const string filename;
    Bounds2i croppedPixelBounds;

private:
    struct Pixel {
        Pixel() {
            xyz[0] = xyz[1] = xyz[2] = 0;
            splatXYZ[0] = splatXYZ[1] = splatXYZ[2] = 0;
            filterWeightSum = 0;
        }

        float xyz[3];
        float filterWeightSum;
        AtomicFloat splatXYZ[3];
        float pad; // padding to ensure 32 bytes for float or 64 bytes for double
    };

    unique_ptr<Pixel[]> pixels;
    static constexpr int FILTER_TABLE_WIDTH = 16;
    float filterTable[SQ(FILTER_TABLE_WIDTH)];
    mutex mutex;
    const float scale;
    const float maxSampleLuminance;

    Pixel & getPixel(const Point2i &p) {
        CHECK(insideExclusive(p, croppedPixelBounds));
        int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
        int offset = (p.x - croppedPixelBounds.pMin.x) +
                     (p.y - croppedPixelBounds.pMin.y) * width;
        return pixels[offset];
    }
};



#endif // CORE_FILM
