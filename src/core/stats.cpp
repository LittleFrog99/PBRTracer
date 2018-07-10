#include "core/stats.h"
#include "stringprint.h"
#include "core/renderer.h"
#include <signal.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

const char * Profiler::stageNames[] = {
    "Scene Parsing and Creation",
    "Acceleration Structure Creation",
    "Texture Loading",
    "Mipmap Generation",
    "Integrator::render()",
    "SamplerIntegrator::computeLi()",
    "SPPM Camera Pass",
    "SPPM Grid Construction",
    "SPPM Photon Pass",
    "SPPM Photon Statistics Update",
    "BDPT Subpath Generation",
    "BDPT Subpath Connections",
    "SpatialLightmap Lookup",
    "SpatialLightmap Spin wait",
    "SpatialLightmap Creation",
    "Direct lighting",
    "BSDF::computeF()",
    "BSDF::sampleF()",
    "BSDF::pdf()",
    "BSSRDF::computeS()",
    "BSSRDF::sampleS()",
    "PhaseFunction::computeP()",
    "PhaseFunction::sampleP()",
    "Accelerator::intersect()",
    "Accelerator::intersectP()",
    "Light::sample()",
    "Light::pdf()",
    "Medium::sample()",
    "Medium::computeTr()",
    "Triangle::intersect()",
    "Triangle::intersectP()",
    "Curve::intersect()",
    "Curve::intersectP()",
    "Other Shape::intersect()",
    "Other Shape::intersectP()",
    "Material::computeScatteringFuncs()",
    "Camera::generateRay()",
    "Film::mergeTile()",
    "Film::addSplat()",
    "Film::addSample()",
    "Sampler::startPixelSample()",
    "Sampler::getSample[12]D()",
    "MIPMap::lookup() (trilinear)",
    "MIPMap::lookup() (EWA)",
    "Ptex Lookup",
};

static_assert(int(Profiler::Stage::NumProfCategories) <= 64,
              "No more than 64 profiling categories may be defined.");

static_assert(int(Profiler::Stage::NumProfCategories) ==
              sizeof(Profiler::stageNames) / sizeof(Profiler::stageNames[0]),
              "stageNames[] array and Stage enumerant have different numbers of entries!");

vector<function<void(StatsAccumulator &)>> * StatsRegisterer::funcs = nullptr;

void StatsAccumulator::print(FILE *dest) {
    fprintf(dest, "Statistics:\n");
    map<string, vector<string>> toPrint;

    for (auto &counter : counters) {
        if (counter.second == 0) continue;
        string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        toPrint[category].push_back(STRING_PRINTF(
            "%-42s               %12" PRIu64, title.c_str(), counter.second));
    }
    for (auto &counter : memoryCounters) {
        if (counter.second == 0) continue;
        string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        double kb = double(counter.second) / 1024.;
        if (kb < 1024.)
            toPrint[category].push_back(STRING_PRINTF(
                "%-42s                  %9.2f kB", title.c_str(), kb));
        else {
            float mib = kb / 1024.;
            if (mib < 1024.)
                toPrint[category].push_back(STRING_PRINTF(
                    "%-42s                  %9.2f MiB", title.c_str(), mib));
            else {
                float gib = mib / 1024.;
                toPrint[category].push_back(STRING_PRINTF(
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
            STRING_PRINTF("%-42s                      %.3f avg [range %" PRIu64
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
            STRING_PRINTF("%-42s                      %.3f avg [range %f - %f]",
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
            STRING_PRINTF("%-42s%12" PRIu64 " / %12" PRIu64 " (%.2f%%)",
                         title.c_str(), num, denom, (100.f * num) / denom));
    }
    for (auto &ratio : ratios) {
        if (ratio.second.second == 0) continue;
        int64_t num = ratio.second.first;
        int64_t denom = ratio.second.second;
        string category, title;
        getCategoryAndTitle(ratio.first, &category, &title);
        toPrint[category].push_back(STRING_PRINTF(
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

StatsAccumulator Statistics::statsAccum;

void Statistics::reportThread() {
    static mutex mut;
    lock_guard<mutex> lock(mut);
    StatsRegisterer::callCallbacks(statsAccum);
}

thread_local uint64_t Profiler::state;
chrono::system_clock::time_point Profiler::startTime;
array<Profiler::ProfileSample, Profiler::PROFILE_HASH_SIZE> Profiler::profileSamples;
atomic<bool> Profiler::isRunning{false};
atomic<int> Profiler::suspendCount;

void Profiler::init() {
    CHECK(!Profiler::isRunning);
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
    CHECK_EQ(setitimer(ITIMER_PROF, &timer, nullptr), 0)
            << "Timer could not be initialized: " << strerror(errno);
}

void Profiler::workerThreadInit() {
    CHECK(!Profiler::isRunning || Profiler::suspendCount > 0);
    state = profToBits(Stage::SceneConstruction);
}

void Profiler::clear() {
    for (ProfileSample &ps : profileSamples) {
        ps.profilerState = 0;
        ps.count = 0;
    }
}

void Profiler::cleanup() {
    CHECK(Profiler::isRunning);
    static struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    timer.it_value = timer.it_interval;

    CHECK_EQ(setitimer(ITIMER_PROF, &timer, nullptr), 0)
            << "Timer could not be disabled: " << strerror(errno);
}

void Profiler::reportProfileSample(int, siginfo_t *, void *) {
    if (suspendCount > 0) return;
    if (state == 0) return;  // A ProgressReporter thread, most likely.

    uint64_t h = std::hash<uint64_t>{}(state) % (PROFILE_HASH_SIZE - 1);
    int count = 0;
    while (count < PROFILE_HASH_SIZE && profileSamples[h].profilerState != state &&
           profileSamples[h].profilerState != 0) {
        // Wrap around to the start if we hit the end.
        if (++h == PROFILE_HASH_SIZE) h = 0;
        ++count;
    }
    CHECK_NE(count, PROFILE_HASH_SIZE) << "Profiler hash table filled up!";
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
    return STRING_PRINTF("%4d:%02d:%02d.%02d", h, m, s, ms);
}

void Profiler::reportResults(FILE *dest) {
    auto now = chrono::system_clock::now();

    constexpr int NumProfCategories = int(Stage::NumProfCategories);
    uint64_t overallCount = 0;
    int used = 0;
    for (const ProfileSample &ps : profileSamples) {
        if (ps.count > 0) {
            overallCount += ps.count;
            ++used;
        }
    }

    LOG(INFO) << "Used " << used << " / " << PROFILE_HASH_SIZE
              << " entries in profiler hash table";

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
        CHECK_LT(nameIndex, NumProfCategories);
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

ProgressReporter::ProgressReporter(int64_t totalWork, const string &title)
    : totalWork(max((int64_t)1, totalWork)), title(title),
      startTime(chrono::system_clock::now()) {
    workDone = 0;
    exitThread = false;
    // Launch thread to periodically update progress bar
    if (!Renderer::options.quiet) {
        Profiler::suspend();
        shared_ptr<Barrier> barrier = make_shared<Barrier>(2);
        updateThread = thread([this, barrier]() {
            Profiler::workerThreadInit();
            Profiler::state = 0;
            barrier->wait();
            printBar();
        });
        // Wait for the thread to get past the ProfilerWorkerThreadInit() call.
        barrier->wait();
        Profiler::resume();
    }
}

void ProgressReporter::update(int64_t num) {
    if (num == 0 || Renderer::options.quiet) return;
    workDone += num;
}

ProgressReporter::~ProgressReporter() {
    if (!Renderer::options.quiet) {
        workDone = totalWork;
        exitThread = true;
        updateThread.join();
        printf("\n");
    }
}

void ProgressReporter::printBar() {
    int barLength = terminalWidth() - 28;
    int totalPlusses = max(2, barLength - (int)title.size());
    int plussesPrinted = 0;

    // Initialize progress string
    const int bufLen = title.size() + totalPlusses + 64;
    unique_ptr<char[]> buf(new char[bufLen]);
    snprintf(buf.get(), bufLen, "\r%s: [", title.c_str());
    char *curSpace = buf.get() + strlen(buf.get());
    char *s = curSpace;
    for (int i = 0; i < totalPlusses; ++i) *s++ = ' ';
    *s++ = ']';
    *s++ = ' ';
    *s++ = '\0';
    fputs(buf.get(), stdout);
    fflush(stdout);

    chrono::milliseconds sleepDuration(250);
    int iterCount = 0;
    while (!exitThread) {
        this_thread::sleep_for(sleepDuration);

        // Periodically increase sleepDuration to reduce overhead of
        // updates.
        ++iterCount;
        if (iterCount == 10)
            // Up to 0.5s after ~2.5s elapsed
            sleepDuration *= 2;
        else if (iterCount == 70)
            // Up to 1s after an additional ~30s have elapsed.
            sleepDuration *= 2;
        else if (iterCount == 520)
            // After 15m, jump up to 5s intervals
            sleepDuration *= 5;

        Float percentDone = Float(workDone) / Float(totalWork);
        int plussesNeeded = round(totalPlusses * percentDone);
        while (plussesPrinted < plussesNeeded) {
            *curSpace++ = '+';
            ++plussesPrinted;
        }
        fputs(buf.get(), stdout);

        // Update elapsed time and estimated time to completion
        Float seconds = elapsedMS() / 1000.f;
        Float estRemaining = seconds / percentDone - seconds;
        if (percentDone == 1.f)
            printf(" (%.1fs)       ", seconds);
        else if (!isinf(estRemaining))
            printf(" (%.1fs|%.1fs)  ", seconds,
                   max((Float)0., estRemaining));
        else
            printf(" (%.1fs|?s)  ", seconds);
        fflush(stdout);
    }
}

void ProgressReporter::done() {
    workDone = totalWork;
}

int ProgressReporter::terminalWidth() {
    struct winsize w;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) < 0) {
        // ENOTTY is fine and expected, e.g. if output is being piped to a file.
        if (errno != ENOTTY) {
            static bool warned = false;
            if (!warned) {
                warned = true;
                fprintf(stderr, "Error in ioctl() in TerminalWidth(): %d\n", errno);
            }
        }
        return 80;
    }
    return w.ws_col;
}
