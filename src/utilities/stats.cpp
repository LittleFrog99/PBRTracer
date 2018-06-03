#include "stats.h"
#include "stringprint.h"
#include <signal.h>
#include <sys/time.h>

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
              "stageNames[] array and Stage enumerant have different numbers of entries!");

StatsAccumulator Stats::statsAccum;
vector<function<void(StatsAccumulator &)>> * StatsRegisterer::funcs = nullptr;

void StatsAccumulator::print(FILE *dest) {
    fprintf(dest, "Statistics:\n");
    map<string, vector<string>> toPrint;

    for (auto &counter : counters) {
        if (counter.second == 0) continue;
        string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        toPrint[category].push_back(StringPrint::printf(
            "%-42s               %12" PRIu64, title.c_str(), counter.second));
    }
    for (auto &counter : memoryCounters) {
        if (counter.second == 0) continue;
        string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        double kb = double(counter.second) / 1024.;
        if (kb < 1024.)
            toPrint[category].push_back(StringPrint::printf(
                "%-42s                  %9.2f kB", title.c_str(), kb));
        else {
            float mib = kb / 1024.;
            if (mib < 1024.)
                toPrint[category].push_back(StringPrint::printf(
                    "%-42s                  %9.2f MiB", title.c_str(), mib));
            else {
                float gib = mib / 1024.;
                toPrint[category].push_back(StringPrint::printf(
                    "%-42s                  %9.2f GiB", title.c_str(), gib));
            }
        }
    }
    for (auto &DistribSum : intDistribSums) {
        const string &name = DistribSum.first;
        if (intDistribCounts[name] == 0) continue;
        string category, title;
        getCategoryAndTitle(name, &category, &title);
        double avg = (double)DistribSum.second /
                     (double)intDistribCounts[name];
        toPrint[category].push_back(
            StringPrint::printf("%-42s                      %.3f avg [range %" PRIu64
                         " - %" PRIu64 "]",
                         title.c_str(), avg, intDistribMins[name],
                         intDistribMaxs[name]));
    }
    for (auto &DistribSum : floatDistribSums) {
        const string &name = DistribSum.first;
        if (floatDistribCounts[name] == 0) continue;
        string category, title;
        getCategoryAndTitle(name, &category, &title);
        double avg = (double)DistribSum.second /
                     (double)floatDistribCounts[name];
        toPrint[category].push_back(
            StringPrint::printf("%-42s                      %.3f avg [range %f - %f]",
                         title.c_str(), avg, floatDistribMins[name],
                         floatDistribMaxs[name]));
    }
    for (auto &percentage : percentages) {
        if (percentage.second.second == 0) continue;
        int64_t num = percentage.second.first;
        int64_t denom = percentage.second.second;
        string category, title;
        getCategoryAndTitle(percentage.first, &category, &title);
        toPrint[category].push_back(
            StringPrint::printf("%-42s%12" PRIu64 " / %12" PRIu64 " (%.2f%%)",
                         title.c_str(), num, denom, (100.f * num) / denom));
    }
    for (auto &ratio : ratios) {
        if (ratio.second.second == 0) continue;
        int64_t num = ratio.second.first;
        int64_t denom = ratio.second.second;
        string category, title;
        getCategoryAndTitle(ratio.first, &category, &title);
        toPrint[category].push_back(StringPrint::printf(
            "%-42s%12" PRIu64 " / %12" PRIu64 " (%.2fx)", title.c_str(), num,
            denom, (double)num / (double)denom));
    }

    for (auto &categories : toPrint) {
        fprintf(dest, "  %s\n", categories.first.c_str());
        for (auto &item : categories.second)
            fprintf(dest, "    %s\n", item.c_str());
    }
}

void StatsAccumulator::clear() {
    counters.clear();
    memoryCounters.clear();
    intDistribSums.clear();
    intDistribCounts.clear();
    intDistribMins.clear();
    intDistribMaxs.clear();
    floatDistribSums.clear();
    floatDistribCounts.clear();
    floatDistribMins.clear();
    floatDistribMaxs.clear();
    percentages.clear();
    ratios.clear();
}

void StatsAccumulator::getCategoryAndTitle(const string &str, string *category, string *title)
{
    const char *s = str.c_str();
    const char *slash = strchr(s, '/');
    if (!slash)
        *title = str;
    else {
        *category = string(s, slash - s);
        *title = string(slash + 1);
    }
}

void Stats::reportThread() {
    static mutex mut;
    lock_guard<mutex> lock(mut);
    StatsRegisterer::callCallbacks(Stats::statsAccum);
}

thread_local uint64_t Profiler::state;
chrono::system_clock::time_point Profiler::startTime;
array<Profiler::ProfileSample, Profiler::PROFILE_HASH_SIZE> Profiler::profileSamples;
atomic<bool> Profiler::isRunning{false};
atomic<int> Profiler::suspendCount;

void Profiler::init() {
    state = profToBits(Stage::SceneConstruction);
    clear();

    startTime = std::chrono::system_clock::now();

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = reportProfileSample;
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPROF, &sa, nullptr);

    static itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 1000000 / 100;  // 100 Hz sampling
    timer.it_value = timer.it_interval;

    isRunning = true;
    setitimer(ITIMER_PROF, &timer, nullptr);
}

void Profiler::workerThreadInit() {
    state = profToBits(Stage::SceneConstruction);
}

void Profiler::clear() {
    for (ProfileSample &ps : profileSamples) {
        ps.profilerState = 0;
        ps.count = 0;
    }
}

void Profiler::cleanup() {
    static struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    timer.it_value = timer.it_interval;

    setitimer(ITIMER_PROF, &timer, nullptr);
}

void Profiler::reportProfileSample(int, siginfo_t *, void *) {
    if (suspendCount > 0) return;
    if (state == 0) return;  // A ProgressReporter thread, most likely.

    uint64_t h = std::hash<uint64_t>{}(state) % (PROFILE_HASH_SIZE - 1);
    int count = 0;
    while (count < PROFILE_HASH_SIZE &&
           profileSamples[h].profilerState != state &&
           profileSamples[h].profilerState != 0) {
        // Wrap around to the start if we hit the end.
        if (++h == PROFILE_HASH_SIZE) h = 0;
        ++count;
    }
    profileSamples[h].profilerState = state;
    ++profileSamples[h].count;
}

string Profiler::timeString(float pct, chrono::system_clock::time_point now)
{
    pct /= 100.;  // remap passed value to to [0,1]
    int64_t ns = chrono::duration_cast<chrono::nanoseconds>(now - startTime).count();
    // milliseconds for this category
    int64_t ms = int64_t(ns * pct / 1000000.);
    // Peel off hours, minutes, seconds, and remaining milliseconds.
    int h = ms / (3600 * 1000);
    ms -= h * 3600 * 1000;
    int m = ms / (60 * 1000);
    ms -= m * (60 * 1000);
    int s = ms / 1000;
    ms -= s * 1000;
    ms /= 10;  // only printing 2 digits of fractional seconds
    return StringPrint::printf("%4d:%02d:%02d.%02d", h, m, s, ms);
}

void Profiler::reportResults(FILE *dest) {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    constexpr int NumProfCategories = int(Stage::NumProfCategories);
    uint64_t overallCount = 0;
    int used = 0;
    for (const ProfileSample &ps : profileSamples) {
        if (ps.count > 0) {
            overallCount += ps.count;
            ++used;
        }
    }

    map<string, uint64_t> flatResults;
    map<string, uint64_t> hierarchicalResults;
    for (const ProfileSample &ps : profileSamples) {
        if (ps.count == 0) continue;

        string s;
        for (int b = 0; b < NumProfCategories; ++b) {
            if (ps.profilerState & (1ull << b)) {
                if (s.size() > 0) {
                    // contribute to the parents...
                    hierarchicalResults[s] += ps.count;
                    s += "/";
                }
                s += stageNames[b];
            }
        }
        hierarchicalResults[s] += ps.count;

        int nameIndex = Math::log2Int(ps.profilerState);
        flatResults[stageNames[nameIndex]] += ps.count;
    }

    fprintf(dest, "  Profile\n");
    for (const auto &r : hierarchicalResults) {
        float pct = (100.f * r.second) / overallCount;
        int indent = 4;
        int slashIndex = r.first.find_last_of("/");
        if (slashIndex == string::npos)
            slashIndex = -1;
        else
            indent += 2 * std::count(r.first.begin(), r.first.end(), '/');
        const char *toPrint = r.first.c_str() + slashIndex + 1;
        fprintf(dest, "%*c%s%*c %5.2f%% (%s)\n", indent, ' ', toPrint,
                max(0, int(67 - strlen(toPrint) - indent)), ' ', pct,
                timeString(pct, now).c_str());
    }

    // Sort the flattened ones by time, longest to shortest.
    vector<pair<string, uint64_t>> flatVec;
    for (const auto &r : flatResults)
        flatVec.push_back(std::make_pair(r.first, r.second));
    sort(flatVec.begin(), flatVec.end(),
         [](pair<string, uint64_t> a, pair<string, uint64_t> b) { return a.second > b.second; });

    fprintf(dest, "  Profile (flattened)\n");
    for (const auto &r : flatVec) {
        float pct = (100.f * r.second) / overallCount;
        int indent = 4;
        const char *toPrint = r.first.c_str();
        fprintf(dest, "%*c%s%*c %5.2f%% (%s)\n", indent, ' ', toPrint,
                max(0, int(67 - strlen(toPrint) - indent)), ' ', pct,
                timeString(pct, now).c_str());
    }
    fprintf(dest, "\n");
}
