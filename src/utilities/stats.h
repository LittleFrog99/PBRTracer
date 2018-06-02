#ifndef STATS_H
#define STATS_H

#include "utilities.h"
#include <map>
#include <chrono>

class StatsAccumulator;

class Stats {
public:
    static void print(FILE *dest);
    static void clear();
    static void reportThread();

private:

};

class Profiler {
public:
    enum class Stage;
    static const char *stageNames[];

    static void init();
    static void suspend();
    static void resume();
    static void workerThreadInit();
    static void reportResults(FILE *dest);
    static void clear();
    static void cleanup();

    inline static uint64_t profToBits(Stage p) {
        return 1ull << int(p);
    }

    thread_local static uint64_t state;

private:

};

thread_local uint64_t Profiler::state = uint64_t();

class StatRegisterer {
public:
    StatRegisterer(function<void(StatsAccumulator &)> func) {
        if (!funcs)
            funcs = new std::vector<function<void(StatsAccumulator &)>>;
        funcs->push_back(func);
    }
    static void callCallbacks(StatsAccumulator &accum);

private:
    static vector<function<void(StatsAccumulator &)>> *funcs;
};

class StatsAccumulator {
public:
    inline void reportCounter(const string &name, int64_t val) {
        counters[name] += val;
    }

    inline void reportMemoryCounter(const string &name, int64_t val) {
        memoryCounters[name] += val;
    }

    inline void reportIntDistrib(const string &name, int64_t sum, int64_t count,
                                 int64_t min, int64_t max) {
        intmapSums[name] += sum;
        intmapCounts[name] += count;
        if (intmapMins.find(name) == intmapMins.end())
            intmapMins[name] = min;
        else
            intmapMins[name] = std::min(intmapMins[name], min);
        if (intmapMaxs.find(name) == intmapMaxs.end())
            intmapMaxs[name] = max;
        else
            intmapMaxs[name] = std::max(intmapMaxs[name], max);
    }

    inline void reportFloatDistrib(const string &name, double sum, int64_t count,
                                   double min, double max) {
        floatmapSums[name] += sum;
        floatmapCounts[name] += count;
        if (floatmapMins.find(name) == floatmapMins.end())
            floatmapMins[name] = min;
        else
            floatmapMins[name] = std::min(floatmapMins[name], min);
        if (floatmapMaxs.find(name) == floatmapMaxs.end())
            floatmapMaxs[name] = max;
        else
            floatmapMaxs[name] = std::max(floatmapMaxs[name], max);
    }

    inline void reportPercentage(const string &name, int64_t num, int64_t denom) {
        percentages[name].first += num;
        percentages[name].second += denom;
    }

    inline void reportRatio(const string &name, int64_t num, int64_t denom) {
        ratios[name].first += num;
        ratios[name].second += denom;
    }

    void print(FILE *file);
    void clear();

private:
    map<string, int64_t> counters;
    map<string, int64_t> memoryCounters;
    map<string, int64_t> intmapSums;
    map<string, int64_t> intmapCounts;
    map<string, int64_t> intmapMins;
    map<string, int64_t> intmapMaxs;
    map<string, double> floatmapSums;
    map<string, int64_t> floatmapCounts;
    map<string, double> floatmapMins;
    map<string, double> floatmapMaxs;
    map<string, std::pair<int64_t, int64_t>> percentages;
    map<string, std::pair<int64_t, int64_t>> ratios;
};

class ProfilePhase {
public:
    ProfilePhase(Profiler::Stage p) {
        categoryBit = Profiler::profToBits(p);
        reset = (Profiler::state & categoryBit) == 0;
        Profiler::state |= categoryBit;
    }

    ~ProfilePhase() {
        if (reset) Profiler::state &= ~categoryBit;
    }

    ProfilePhase(const ProfilePhase &) = delete;
    ProfilePhase & operator = (const ProfilePhase &) = delete;

private:
    bool reset;
    uint64_t categoryBit;
};

// Statistics Macros
#define STAT_COUNTER(title, var)                           \
    static thread_local int64_t var;                  \
    static void STATS_FUNC##var(StatsAccumulator &accum) { \
        accum.ReportCounter(title, var);                   \
        var = 0;                                           \
    }                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)
#define STAT_MEMORY_COUNTER(title, var)                    \
    static thread_local int64_t var;                  \
    static void STATS_FUNC##var(StatsAccumulator &accum) { \
        accum.ReportMemoryCounter(title, var);             \
        var = 0;                                           \
    }                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)


#define STATS_INT64_T_MIN std::numeric_limits<int64_t>::max()
#define STATS_INT64_T_MAX std::numeric_limits<int64_t>::lowest()
#define STATS_DBL_T_MIN std::numeric_limits<double>::max()
#define STATS_DBL_T_MAX std::numeric_limits<double>::lowest()

#define STAT_INT_map(title, var)                                  \
    static thread_local int64_t var##sum;                             \
    static thread_local int64_t var##count;                           \
    static thread_local int64_t var##min = (STATS_INT64_T_MIN);       \
    static thread_local int64_t var##max = (STATS_INT64_T_MAX);       \
    static void STATS_FUNC##var(StatsAccumulator &accum) {                 \
        accum.ReportIntmap(title, var##sum, var##count, var##min, \
                                    var##max);                             \
        var##sum = 0;                                                      \
        var##count = 0;                                                    \
        var##min = std::numeric_limits<int64_t>::max();                    \
        var##max = std::numeric_limits<int64_t>::lowest();                 \
    }                                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

#define STAT_FLOAT_map(title, var)                                  \
    static thread_local double var##sum;                                \
    static thread_local int64_t var##count;                             \
    static thread_local double var##min = (STATS_DBL_T_MIN);            \
    static thread_local double var##max = (STATS_DBL_T_MAX);            \
    static void STATS_FUNC##var(StatsAccumulator &accum) {                   \
        accum.ReportFloatmap(title, var##sum, var##count, var##min, \
                                      var##max);                             \
        var##sum = 0;                                                        \
        var##count = 0;                                                      \
        var##min = std::numeric_limits<double>::max();                       \
        var##max = std::numeric_limits<double>::lowest();                    \
    }                                                                        \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

#define REPORT_VALUE(var, value)                                   \
    do {                                                          \
        var##sum += value;                                        \
        var##count += 1;                                          \
        var##min = std::min(var##min, decltype(var##min)(value)); \
        var##max = std::max(var##max, decltype(var##min)(value)); \
    } while (0)

#define STAT_PERCENT(title, numVar, denomVar)                 \
    static thread_local int64_t numVar, denomVar;        \
    static void STATS_FUNC##numVar(StatsAccumulator &accum) { \
        accum.ReportPercentage(title, numVar, denomVar);      \
        numVar = denomVar = 0;                                \
    }                                                         \
    static StatRegisterer STATS_REG##numVar(STATS_FUNC##numVar)

#define STAT_RATIO(title, numVar, denomVar)                   \
    static thread_local int64_t numVar, denomVar;        \
    static void STATS_FUNC##numVar(StatsAccumulator &accum) { \
        accum.ReportRatio(title, numVar, denomVar);           \
        numVar = denomVar = 0;                                \
    }                                                         \
    static StatRegisterer STATS_REG##numVar(STATS_FUNC##numVar)

enum class Profiler::Stage {
    SceneConstruction,
    AccelConstruction,
    TextureLoading,
    MIPMapCreation,
    IntegratorRender,
    SamplerIntegratorLi,
    SPPMCameraPass,
    SPPMGridConstruction,
    SPPMPhotonPass,
    SPPMStatsUpdate,
    BDPTGenerateSubpath,
    BDPTConnectSubpaths,
    LightDistribLookup,
    LightDistribSpinWait,
    LightDistribCreation,
    DirectLighting,
    BSDFEvaluation,
    BSDFSampling,
    BSDFPdf,
    BSSRDFEvaluation,
    BSSRDFSampling,
    PhaseFuncEvaluation,
    PhaseFuncSampling,
    AccelIntersect,
    AccelIntersectP,
    LightSample,
    LightPdf,
    MediumSample,
    MediumTr,
    TriIntersect,
    TriIntersectP,
    CurveIntersect,
    CurveIntersectP,
    ShapeIntersect,
    ShapeIntersectP,
    ComputeScatteringFuncs,
    GenerateCameraRay,
    MergeFilmTile,
    SplatFilm,
    AddFilmSample,
    StartPixel,
    GetSample,
    TexFiltTrilerp,
    TexFiltEWA,
    TexFiltPtex,
    NumProfCategories
};

const char * Profiler::stageNames[] = {
    "Scene parsing and creation",
    "Acceleration structure creation",
    "Texture loading",
    "MIP map generation",
    "Integrator::Render()",
    "SamplerIntegrator::Li()",
    "SPPM camera pass",
    "SPPM grid construction",
    "SPPM photon pass",
    "SPPM photon statistics update",
    "BDPT subpath generation",
    "BDPT subpath connections",
    "SpatialLightmap lookup",
    "SpatialLightmap spin wait",
    "SpatialLightmap creation",
    "Direct lighting",
    "BSDF::f()",
    "BSDF::Sample_f()",
    "BSDF::PDF()",
    "BSSRDF::f()",
    "BSSRDF::Sample_f()",
    "PhaseFunction::p()",
    "PhaseFunction::Sample_p()",
    "Accelerator::Intersect()",
    "Accelerator::IntersectP()",
    "Light::Sample_*()",
    "Light::Pdf()",
    "Medium::Sample()",
    "Medium::Tr()",
    "Triangle::Intersect()",
    "Triangle::IntersectP()",
    "Curve::Intersect()",
    "Curve::IntersectP()",
    "Other Shape::Intersect()",
    "Other Shape::IntersectP()",
    "Material::ComputeScatteringFunctions()",
    "Camera::GenerateRay[Differential]()",
    "Film::MergeTile()",
    "Film::AddSplat()",
    "Film::AddSample()",
    "Sampler::StartPixelSample()",
    "Sampler::GetSample[12]D()",
    "MIPMap::Lookup() (trilinear)",
    "MIPMap::Lookup() (EWA)",
    "Ptex lookup",
};

static_assert(int(Profiler::Stage::NumProfCategories) <= 64,
              "No more than 64 profiling categories may be defined.");

static_assert(int(Profiler::Stage::NumProfCategories) ==
              sizeof(Profiler::stageNames) / sizeof(Profiler::stageNames[0]),
              "ProfNames[] array and Prof enumerant have different numbers of entries!");

#endif // STATS_H
