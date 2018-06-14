#ifndef STATS_H
#define STATS_H

#include "utilities.h"
#include <array>

class StatsAccumulator;

class StatsRegisterer {
public:
    StatsRegisterer(function<void(StatsAccumulator &)> func) {
        if (!funcs)
            funcs = new std::vector<function<void(StatsAccumulator &)>>;
        funcs->push_back(func);
    }

    static void callCallbacks(StatsAccumulator &accum) {
        for (auto func : *funcs) func(accum);
    }

private:
    static vector<function<void(StatsAccumulator &)>> *funcs;
};

class StatsAccumulator {
public:
    void reportCounter(const string &name, int64_t val) {
        counters[name] += val;
    }

    void reportMemoryCounter(const string &name, int64_t val) {
        memoryCounters[name] += val;
    }

    void reportIntDistrib(const string &name, int64_t sum, int64_t count,
                                 int64_t min, int64_t max) {
        intDistribSums[name] += sum;
        intDistribCounts[name] += count;
        if (intDistribMins.find(name) == intDistribMins.end())
            intDistribMins[name] = min;
        else
            intDistribMins[name] = std::min(intDistribMins[name], min);
        if (intDistribMaxs.find(name) == intDistribMaxs.end())
            intDistribMaxs[name] = max;
        else
            intDistribMaxs[name] = std::max(intDistribMaxs[name], max);
    }

    void reportFloatDistrib(const string &name, double sum, int64_t count,
                                   double min, double max) {
        floatDistribSums[name] += sum;
        floatDistribCounts[name] += count;
        if (floatDistribMins.find(name) == floatDistribMins.end())
            floatDistribMins[name] = min;
        else
            floatDistribMins[name] = std::min(floatDistribMins[name], min);
        if (floatDistribMaxs.find(name) == floatDistribMaxs.end())
            floatDistribMaxs[name] = max;
        else
            floatDistribMaxs[name] = std::max(floatDistribMaxs[name], max);
    }

    void reportPercentage(const string &name, int64_t num, int64_t denom) {
        percentages[name].first += num;
        percentages[name].second += denom;
    }

    void reportRatio(const string &name, int64_t num, int64_t denom) {
        ratios[name].first += num;
        ratios[name].second += denom;
    }

    void print(FILE *file);
    void clear();

private:
    map<string, int64_t> counters;
    map<string, int64_t> memoryCounters;
    map<string, int64_t> intDistribSums;
    map<string, int64_t> intDistribCounts;
    map<string, int64_t> intDistribMins;
    map<string, int64_t> intDistribMaxs;
    map<string, double> floatDistribSums;
    map<string, int64_t> floatDistribCounts;
    map<string, double> floatDistribMins;
    map<string, double> floatDistribMaxs;
    map<string, pair<int64_t, int64_t>> percentages;
    map<string, pair<int64_t, int64_t>> ratios;

    static void getCategoryAndTitle(const string &str, string *category, string *title);
};

class Stats {
public:
    static void print(FILE *dest) { statsAccum.print(dest); }
    static void clear() { statsAccum.clear(); }
    static void reportThread();

private:
    static StatsAccumulator statsAccum;
};

class Profiler {
public:
    enum class Stage;
    static const char *stageNames[];

    static void init();
    static void workerThreadInit();
    static void reportResults(FILE *dest);
    static void clear();
    static void cleanup();

    static void suspend() { suspendCount++; }
    static void resume() { suspendCount--; }

    static uint64_t profToBits(Stage p) { return 1ull << int(p); }

    thread_local static uint64_t state;
    static atomic<bool> isRunning;

private:
    struct ProfileSample {
        atomic<uint64_t> profilerState{0};
        atomic<uint64_t> count{0};
    };

    static constexpr int PROFILE_HASH_SIZE = 256;
    static chrono::system_clock::time_point startTime;
    static array<ProfileSample, PROFILE_HASH_SIZE> profileSamples;
    static atomic<int> suspendCount;

    static void reportProfileSample(int, siginfo_t *, void *);
    static string timeString(float pct, chrono::system_clock::time_point now);

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
        accum.reportCounter(title, var);                   \
        var = 0;                                           \
    }                                                      \
    static StatsRegisterer STATS_REG##var(STATS_FUNC##var)

#define STAT_MEMORY_COUNTER(title, var)                    \
    static thread_local int64_t var;                  \
    static void STATS_FUNC##var(StatsAccumulator &accum) { \
        accum.reportMemoryCounter(title, var);             \
        var = 0;                                           \
    }                                                      \
    static StatsRegisterer STATS_REG##var(STATS_FUNC##var)

#define STATS_INT64_T_MIN std::numeric_limits<int64_t>::max()
#define STATS_INT64_T_MAX std::numeric_limits<int64_t>::lowest()
#define STATS_DBL_T_MIN std::numeric_limits<double>::max()
#define STATS_DBL_T_MAX std::numeric_limits<double>::lowest()

#define STAT_INT_DISTRIB(title, var)                                  \
    static thread_local int64_t var##sum;                             \
    static thread_local int64_t var##count;                           \
    static thread_local int64_t var##min = (STATS_INT64_T_MIN);       \
    static thread_local int64_t var##max = (STATS_INT64_T_MAX);       \
    static void STATS_FUNC##var(StatsAccumulator &accum) {                 \
        accum.reportIntmap(title, var##sum, var##count, var##min, \
                                    var##max);                             \
        var##sum = 0;                                                      \
        var##count = 0;                                                    \
        var##min = std::numeric_limits<int64_t>::max();                    \
        var##max = std::numeric_limits<int64_t>::lowest();                 \
    }                                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

#define STAT_FLOAT_DISTRIB(title, var)                                  \
    static thread_local double var##sum;                                \
    static thread_local int64_t var##count;                             \
    static thread_local double var##min = (STATS_DBL_T_MIN);            \
    static thread_local double var##max = (STATS_DBL_T_MAX);            \
    static void STATS_FUNC##var(StatsAccumulator &accum) {                   \
        accum.reportFloatmap(title, var##sum, var##count, var##min, \
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
        accum.reportPercentage(title, numVar, denomVar);      \
        numVar = denomVar = 0;                                \
    }                                                         \
    static StatRegisterer STATS_REG##numVar(STATS_FUNC##numVar)

#define STAT_RATIO(title, numVar, denomVar)                   \
    static thread_local int64_t numVar, denomVar;        \
    static void STATS_FUNC##numVar(StatsAccumulator &accum) { \
        accum.reportRatio(title, numVar, denomVar);           \
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

#endif // STATS_H
