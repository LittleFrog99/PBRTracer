#include "film.h"

void FilmTile::addSample(const Point2f &pFilm, Spectrum L, Float sampleWeight) {
    ProfilePhase pp(Profiler::Stage::AddFilmSample);
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
        Float fx = abs((x - pFilmDiscrete.x) * invFilterRadius.x * filterTableSize);
        ifx[x - p0.x] = min(int(floor(fx)), filterTableSize - 1);
    }
    int *ify = ALLOCA(int, p1.y - p0.y);
    for (int y = p0.y; y < p1.y; ++y) {
        Float fy = abs((y - pFilmDiscrete.y) * invFilterRadius.y * filterTableSize);
        ify[y - p0.y] = min(int(floor(fy)), filterTableSize - 1);
    }
    for (int y = p0.y; y < p1.y; ++y) {
        for (int x = p0.x; x < p1.x; ++x) {
            // Evaluate filter value at $(x,y)$ pixel
            int offset = ify[y - p0.y] * filterTableSize + ifx[x - p0.x];
            Float filterWeight = filterTable[offset];
            // Update pixel values with filtered sample contribution
            FilmTilePixel &pixel = getPixel(Point2i(x, y));
            pixel.contribSum += L * sampleWeight * filterWeight;
            pixel.filterWeightSum += filterWeight;
        }
    }
}

Film::Film(const Point2i &resolution, const Bounds2f &cropWindow, unique_ptr<Filter> filter,
           Float diagonal, const string &filename, Float scale, Float maxSampleLuminance)
    : fullResolution(resolution), diagonal(diagonal * 0.001f), filter(move(filter)),
      filename(filename), scale(scale), maxSampleLuminance(maxSampleLuminance)
{
    // Compute film image bounds
    croppedPixelBounds = Bounds2i(Point2i(ceil(fullResolution.x * cropWindow.pMin.x),
                                          ceil(fullResolution.y * cropWindow.pMin.y)),
                                  Point2i(ceil(fullResolution.x * cropWindow.pMax.x),
                                          ceil(fullResolution.y * cropWindow.pMax.y)));
    // Allocate image storage
    pixels = unique_ptr<Pixel[]>(new Pixel[croppedPixelBounds.area()]);
    // Procompute filter weight table
}
