#include "tests/gtest/gtest.h"
#include "samplers/halton.h"
#include "core/camera.h"

TEST(HaltonCameraSample, Basics) {
    Bounds2i bounds(Point2i(0, 0), Point2i(2, 2));
    HaltonSampler sampler(16, bounds);
    for (auto pixel : bounds) {
        sampler.startPixel(pixel);
        cout << "Start pixel: \n";
        do {
            auto camSample = sampler.getCameraSample(Point2i());
            cout << camSample.pFilm << camSample.pLens << camSample.time << endl;
            for (int i = 0; i < 10; i++)
                cout << sampler.get1D() << endl;
            for (int i = 0; i < 10; i++)
                cout << sampler.get2D() << endl;
        } while (sampler.startNextSample());
    }
}
