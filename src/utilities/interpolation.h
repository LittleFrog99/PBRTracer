#ifndef UTILITY_INTERPOLATION
#define UTILITY_INTERPOLATION

namespace Interpolation {

float catmullRom(int size, const float *nodes, const float *values, float x);
bool catmullRomWeights(int size, const float *nodes, float x, int *offset, float *weights);
float sampleCatmullRom(int size, const float *nodes, const float *f, const float *cdf, float sample,
                       float *fval = nullptr,float *pdf = nullptr);
float sampleCatmullRom2D(int size1, int size2, const float *nodes1, const float *nodes2, const float *values,
                         const float *cdf, float alpha, float sample, float *fval = nullptr, float *pdf = nullptr);
float integrateCatmullRom(int n, const float *nodes, const float *values, float *cdf);
float invertCatmullRom(int n, const float *x, const float *values, float u);

}

#endif // UTILITY_INTERPOLATION
