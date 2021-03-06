#include "film.h"
#include "core/renderer.h"
#include "imageio.h"
#include "paramset.h"
#include "stats.h"

void FilmTile::addSample(const Point2f &pFilm, Spectrum L, float sampleWeight) {
    ProfilePhase pp(Stage::AddFilmSample);
    if (L.luminance() > maxSampleLuminance)
        L *= maxSampleLuminance / L.luminance();
    // Compute sample's raster bounds
    Point2f pFilmDiscrete = pFilm - Vector2f(0.5f, 0.5f);
    Point2i p0 = Point2i(ceil(pFilmDiscrete - filterRadius));
    Point2i p1 = Point2i(floor(pFilmDiscrete + filterRadius)) + Point2i(1, 1);
    p0 = max(p0, pixelBounds.pMin);
    p1 = min(p1, pixelBounds.pMax);

    // Loop over filter support and add sample to pixel arrays
    // Precompute $x$ and $y$ filter table offsets
    int *ifx = ALLOCA(int, p1.x - p0.x);
    for (int x = p0.x; x < p1.x; ++x) {
        float fx = abs((x - pFilmDiscrete.x) * invFilterRadius.x * filterTableSize);
        ifx[x - p0.x] = min(int(floor(fx)), filterTableSize - 1);
    }
    int *ify = ALLOCA(int, p1.y - p0.y);
    for (int y = p0.y; y < p1.y; ++y) {
        float fy = abs((y - pFilmDiscrete.y) * invFilterRadius.y * filterTableSize);
        ify[y - p0.y] = min(int(floor(fy)), filterTableSize - 1);
    }
    for (int y = p0.y; y < p1.y; ++y) {
        for (int x = p0.x; x < p1.x; ++x) {
            // Evaluate filter value at $(x,y)$ pixel
            int offset = ify[y - p0.y] * filterTableSize + ifx[x - p0.x];
            float filterWeight = filterTable[offset];
            // Update pixel values with filtered sample contribution
            FilmTilePixel &pixel = getPixel(Point2i(x, y));
            pixel.contribSum += L * sampleWeight * filterWeight;
            pixel.filterWeightSum += filterWeight;
        }
    }
}

const int Film::FILTER_TABLE_WIDTH = 16;

Film::Film(const Point2i &resolution, const Bounds2f &cropWindow, Filter *filter,
           float diagonal, const string &filename, float scale, float maxSampleLuminance)
    : fullResolution(resolution), diagonal(diagonal * 0.001f), filter(filter),
      filename(filename), filterTable(new float[SQ(FILTER_TABLE_WIDTH)]), scale(scale),
      maxSampleLuminance(maxSampleLuminance)
{
    // Compute film image bounds
    croppedPixelBounds = Bounds2i(Point2i(ceil(fullResolution.x * cropWindow.pMin.x),
                                          ceil(fullResolution.y * cropWindow.pMin.y)),
                                  Point2i(ceil(fullResolution.x * cropWindow.pMax.x),
                                          ceil(fullResolution.y * cropWindow.pMax.y)));
    // Allocate image storage
    pixels = unique_ptr<Pixel[]>(new Pixel[croppedPixelBounds.area()]);
    // Procompute filter weight table
    int offset = 0;
    for (int y = 0; y < FILTER_TABLE_WIDTH; y++) {
        for (int x = 0; x < FILTER_TABLE_WIDTH; x++, offset++) {
            Point2f p;
            p.x = (x + 0.5f) * filter->radius.x / FILTER_TABLE_WIDTH;
            p.y = (y + 0.5f) * filter->radius.y / FILTER_TABLE_WIDTH;
            filterTable[offset] = filter->evaluate(p);
        }
    }
} 

Bounds2i Film::getSampleBounds() const {
    Vector2f halfPixel = Vector2f(0.5f);
    Bounds2f floatBounds(floor(Point2f(croppedPixelBounds.pMin) + halfPixel - filter->radius),
                         ceil(Point2f(croppedPixelBounds.pMax) - halfPixel + filter->radius));
    return Bounds2i(floatBounds);
}

Bounds2f Film::getPhysicalExtent() const {
    float aspect = float(fullResolution.y) / float(fullResolution.x);
    float x = sqrt(SQ(diagonal) / (1 + SQ(aspect)));
    float y = aspect * x;
    return Bounds2f(Point2f(-x / 2, -y / 2), Point2f(x / 2, y / 2));
}

unique_ptr<FilmTile> Film::getFilmTile(const Bounds2i &sampleBounds) {
    Vector2f halfPixel = Vector2f(0.5f);
    Bounds2f floatBounds(sampleBounds);
    Point2i p0(ceil(floatBounds.pMin - halfPixel - filter->radius));
    Point2i p1(floor(floatBounds.pMax - halfPixel + filter->radius) + Point2f(1, 1));
    Bounds2i tilePixelBounds = intersect(Bounds2i(p0, p1), croppedPixelBounds);
    return make_unique<FilmTile>(tilePixelBounds, filter->radius, filterTable.get(), FILTER_TABLE_WIDTH,
                                 maxSampleLuminance);
}

void Film::mergeFilmTile(unique_ptr<FilmTile> tile) {
    lock_guard<std::mutex> lock(mutex);
    for (Point2i pixel : tile->getPixelBounds()) {
        const FilmTilePixel &tilePixel = tile->getPixel(pixel);
        Pixel &mergePixel = getPixel(pixel);
        float xyz[3];
        tilePixel.contribSum.toXYZ(xyz);
        for (int i = 0; i < 3; i++)
            mergePixel.xyz[i] += xyz[i];
        mergePixel.filterWeightSum += tilePixel.filterWeightSum;
    }
}

void Film::setImage(const Spectrum *img) const {
    int nPixels = croppedPixelBounds.area();
    for (int i = 0; i < nPixels; i++) {
        Pixel &p = pixels[i];
        img[i].toXYZ(p.xyz);
        p.filterWeightSum = 1;
        p.splatXYZ[0] = p.splatXYZ[1] = p.splatXYZ[2] = 0;
    }
}

void Film::addSplat(const Point2f &p, Spectrum v) {
    if (!insideExclusive(Point2i(p), croppedPixelBounds))
        return;
    float xyz[3];
    v.toXYZ(xyz);
    Pixel &pixel = getPixel(Point2i(p));
    for (int i = 0; i < 3; i++)
        pixel.splatXYZ[i].add(xyz[i]);
}

void Film::writeImage(float splatScale) {
    // Convert image to RGB and compute final pixel values
    unique_ptr<float[]> rgb(new float[3 * croppedPixelBounds.area()]);
    int offset = 0;
    for (Point2i p : croppedPixelBounds) {
        // Convert XYZ color to RGB
        auto &pixel = getPixel(p);
        XYZToRGB(pixel.xyz, &rgb[3 * offset]);
        // Normalize pixel with weighted sum
        float filterWeightSum = pixel.filterWeightSum;
        if (filterWeightSum != 0) {
            float invWt = 1.0f / filterWeightSum;
            rgb[3 * offset] = max(0.0f, rgb[3 * offset] * invWt);
            rgb[3 * offset + 1] = max(0.0f, rgb[3 * offset + 1] * invWt);
            rgb[3 * offset + 2] = max(0.0f, rgb[3 * offset + 2] * invWt);
        }
        // Add splat value at pixel
        float splatRGB[3];
        float splatXYZ[3] = { pixel.splatXYZ[0], pixel.splatXYZ[1], pixel.splatXYZ[2] };
        XYZToRGB(splatXYZ, splatRGB);
        rgb[3 * offset] += splatScale * splatRGB[0];
        rgb[3 * offset + 1] += splatScale * splatRGB[1];
        rgb[3 * offset + 2] += splatScale * splatRGB[2];
        // Scale pixel value
        rgb[3 * offset] *= scale;
        rgb[3 * offset + 1] *= scale;
        rgb[3 * offset + 2] *= scale;
        ++offset;
    }
    // Write RGB image
    ImageIO::writeImage(filename, &rgb[0], croppedPixelBounds, fullResolution);
}

void Film::clear() {
    for (Point2i p : croppedPixelBounds) {
        auto &pixel = getPixel(p);
        pixel.xyz[0] = pixel.xyz[1] = pixel.xyz[2] = 0;
        pixel.splatXYZ[0] = pixel.splatXYZ[1] = pixel.splatXYZ[2] = 0;
        pixel.filterWeightSum = 0;
    }
}


Film * Film::create(const ParamSet &params, Filter *filter) {
    string filename;
    if (Renderer::options.imageFile != "") {
        filename = Renderer::options.imageFile;
        string paramsFilename = params.findOneString("filename", "");
        if (paramsFilename != "")
            WARNING("Output filename supplied on command line, \"%s\" is overriding "
                    "filename provided in scene description file, \"%s\".",
                    Renderer::options.imageFile.c_str(), paramsFilename.c_str());
    } else
        filename = params.findOneString("filename", "pbrt.exr");

    int xres = params.findOneInt("xresolution", 1280);
    int yres = params.findOneInt("yresolution", 720);
    if (Renderer::options.quickRender) xres = max(1, xres / 4);
    if (Renderer::options.quickRender) yres = max(1, yres / 4);
    Bounds2f crop(Point2f(0, 0), Point2f(1, 1));
    int cwi;
    const float *cr = params.findFloat("cropwindow", &cwi);
    if (cr && cwi == 4) {
        crop.pMin.x = clamp(min(cr[0], cr[1]), 0.f, 1.f);
        crop.pMax.x = clamp(max(cr[0], cr[1]), 0.f, 1.f);
        crop.pMin.y = clamp(min(cr[2], cr[3]), 0.f, 1.f);
        crop.pMax.y = clamp(max(cr[2], cr[3]), 0.f, 1.f);
    } else if (cr)
        ERROR("%d values supplied for \"cropwindow\". Expected 4.", cwi);

    float scale = params.findOneFloat("scale", 1.);
    float diagonal = params.findOneFloat("diagonal", 35.);
    float maxSampleLuminance = params.findOneFloat("maxsampleluminance", INFINITY);

    return new Film(Point2i(xres, yres), crop, filter, diagonal, filename, scale,
                    maxSampleLuminance);
}
