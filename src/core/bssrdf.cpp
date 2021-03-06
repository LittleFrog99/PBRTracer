#include "bssrdf.h"
#include "core/primitive.h"
#include "stats.h"

Spectrum SeparableBSSRDF::sample_S(const Scene &scene, float u1, const Point2f &u2, MemoryArena &arena,
                                   SurfaceInteraction *pi, float *pdf) const
{
    Spectrum Sp = sample_Sp(scene, u1, u2, arena, pi, pdf);
    if (!Sp.isBlack()) {
        pi->bsdf = ARENA_ALLOC(arena, BSDF)(*pi);
        pi->bsdf->add(ARENA_ALLOC(arena, SeparableBSSRDFAdpater)(this));
        pi->wo = Vector3f(pi->shading.n);
    }
    return Sp;
}

Spectrum SeparableBSSRDF::sample_Sp(const Scene &scene, float u1, const Point2f &u2, MemoryArena &arena,
                                    SurfaceInteraction *pi, float *pdf) const
{
    // Choose projection axis for BSSRDF sampling
    Vector3f vx, vy, vz;
    if (u1 < .5f) {
        vx = ss;
        vy = ts;
        vz = Vector3f(ns);
        u1 *= 2;
    } else if (u1 < .75f) {
        // Prepare for sampling rays with respect to _ss_
        vx = ts;
        vy = Vector3f(ns);
        vz = ss;
        u1 = (u1 - .5f) * 4;
    } else {
        // Prepare for sampling rays with respect to _ts_
        vx = Vector3f(ns);
        vy = ss;
        vz = ts;
        u1 = (u1 - .75f) * 4;
    }

    // Choose spectral channel for BSSRDF sampling
    int ch = clamp(int(u1 * Spectrum::nSamples), 0, Spectrum::nSamples - 1);
    u1 = u1 * Spectrum::nSamples - ch;

    // Sample BSSRDF profile in polar coordinates
    float r = sample_Sr(ch, u2[0]);
    if (r < 0) return Spectrum(0.f);
    float phi = 2 * PI * u2[1];

    // Compute BSSRDF profile bounds and intersection height
    float rMax = sample_Sr(ch, 0.999f);
    if (r >= rMax) return Spectrum(0.f);
    float l = 2 * sqrt(rMax * rMax - r * r);

    // Compute BSSRDF sampling ray segment
    Interaction base;
    base.p = po.p + r * (vx * cos(phi) + vy * sin(phi)) - l * vz * 0.5f;
    base.time = po.time;
    Point3f pTarget = base.p + l * vz;

    // Intersect BSSRDF sampling ray against the scene geometry
    struct IntersectionChain {
        SurfaceInteraction si;
        IntersectionChain *next = nullptr;
    };
    IntersectionChain *chain = ARENA_ALLOC(arena, IntersectionChain)();

    // Accumulate chain of intersections along ray
    IntersectionChain *ptr = chain;
    int nFound = 0;
    while (true) {
        Ray r = base.spawnRayTo(pTarget);
        if (r.d == Vector3f(0, 0, 0) || !scene.intersect(r, &ptr->si))
            break;

        base = ptr->si;
        // Append admissible intersection to _IntersectionChain_
        if (ptr->si.primitive->getMaterial() == material) {
            IntersectionChain *next = ARENA_ALLOC(arena, IntersectionChain)();
            ptr->next = next;
            ptr = next;
            nFound++;
        }
    }

    // Randomly choose one of several intersections during BSSRDF sampling
    if (nFound == 0) return Spectrum(0.0f);
    int selected = clamp(int(u1 * nFound), 0, nFound - 1);
    while (selected-- > 0) chain = chain->next;
    *pi = chain->si;

    // Compute sample PDF and return the spatial term
    *pdf = pdf_Sp(*pi) / nFound;
    return compute_Sp(*pi);
}

float SeparableBSSRDF::pdf_Sp(const SurfaceInteraction &pi) const {
    // Express difference vector and surface normal at pi with local coordinates at po
    Vector3f d = po.p - pi.p;
    Vector3f dLocal(dot(ss, d), dot(ts, d), dot(ns, d));
    Normal3f nLocal(dot(ss, pi.n), dot(ts, pi.n), dot(ns, pi.n));

    // Compute BSSRDF profile radius under projection along each axis
    float rProj[3] = {sqrt(dLocal.y * dLocal.y + dLocal.z * dLocal.z),
                      sqrt(dLocal.z * dLocal.z + dLocal.x * dLocal.x),
                      sqrt(dLocal.x * dLocal.x + dLocal.y * dLocal.y)};

    // Return combined probability from all BSSRDF sampling strategies
    float pdf = 0, axisProb[3] = {.25f, .25f, .5f};
    constexpr float chProb = 1.0f / Spectrum::nSamples;
    for (int axis = 0; axis < 3; ++axis)
        for (int ch = 0; ch < Spectrum::nSamples; ++ch)
            pdf += pdf_Sr(ch, rProj[axis]) * abs(nLocal[axis]) * chProb * axisProb[axis];

    return pdf;
}

struct BSSRDF::MeasuredSS {
    const char *name;
    float sigma_prime_s[3], sigma_a[3];  // mm^-1
};

BSSRDF::MeasuredSS BSSRDF::SubsurfaceParameterTable[] = {
    // From "A Practical Model for Subsurface Light Transport"
    // Jensen, Marschner, Levoy, Hanrahan
    { "Apple", {2.29, 2.39, 1.97}, {0.0030, 0.0034, 0.046}, },
    { "Chicken1", {0.15, 0.21, 0.38}, {0.015, 0.077, 0.19}, },
    { "Chicken2", {0.19, 0.25, 0.32}, {0.018, 0.088, 0.20}, },
    { "Cream", {7.38, 5.47, 3.15}, {0.0002, 0.0028, 0.0163}, },
    { "Ketchup", {0.18, 0.07, 0.03}, {0.061, 0.97, 1.45}, },
    { "Marble", {2.19, 2.62, 3.00}, {0.0021, 0.0041, 0.0071}, },
    { "Potato", {0.68, 0.70, 0.55}, {0.0024, 0.0090, 0.12}, },
    { "Skimmilk", {0.70, 1.22, 1.90}, {0.0014, 0.0025, 0.0142}, },
    { "Skin1", {0.74, 0.88, 1.01}, {0.032, 0.17, 0.48}, },
    { "Skin2", {1.09, 1.59, 1.79}, {0.013, 0.070, 0.145}, },
    { "Spectralon", {11.6, 20.4, 14.9}, {0.00, 0.00, 0.00}, },
    { "Wholemilk", {2.55, 3.21, 3.77}, {0.0011, 0.0024, 0.014}, },

    // From "Acquiring Scattering Properties of Participating Media by Dilution",
    // Narasimhan, Gupta, Donner, Ramamoorthi, Nayar, Jensen
    {"Lowfat Milk", {0.89187, 1.5136, 2.532}, {0.002875, 0.00575, 0.0115}},
    {"Reduced Milk",
     {2.4858, 3.1669, 4.5214},
     {0.0025556, 0.0051111, 0.012778}},
    {"Regular Milk", {4.5513, 5.8294, 7.136}, {0.0015333, 0.0046, 0.019933}},
    {"Espresso", {0.72378, 0.84557, 1.0247}, {4.7984, 6.5751, 8.8493}},
    {"Mint Mocha Coffee", {0.31602, 0.38538, 0.48131}, {3.772, 5.8228, 7.82}},
    {"Lowfat Soy Milk",
     {0.30576, 0.34233, 0.61664},
     {0.0014375, 0.0071875, 0.035937}},
    {"Regular Soy Milk",
     {0.59223, 0.73866, 1.4693},
     {0.0019167, 0.0095833, 0.065167}},
    {"Lowfat Chocolate Milk",
     {0.64925, 0.83916, 1.1057},
     {0.0115, 0.0368, 0.1564}},
    {"Regular Chocolate Milk",
     {1.4585, 2.1289, 2.9527},
     {0.010063, 0.043125, 0.14375}},
    {"Coke", {8.9053e-05, 8.372e-05, 0}, {0.10014, 0.16503, 0.2468}},
    {"Pepsi", {6.1697e-05, 4.2564e-05, 0}, {0.091641, 0.14158, 0.20729}},
    {"Sprite",
     {6.0306e-06, 6.4139e-06, 6.5504e-06},
     {0.001886, 0.0018308, 0.0020025}},
    {"Gatorade",
     {0.0024574, 0.003007, 0.0037325},
     {0.024794, 0.019289, 0.008878}},
    {"Chardonnay",
     {1.7982e-05, 1.3758e-05, 1.2023e-05},
     {0.010782, 0.011855, 0.023997}},
    {"White Zinfandel",
     {1.7501e-05, 1.9069e-05, 1.288e-05},
     {0.012072, 0.016184, 0.019843}},
    {"Merlot", {2.1129e-05, 0, 0}, {0.11632, 0.25191, 0.29434}},
    {"Budweiser Beer",
     {2.4356e-05, 2.4079e-05, 1.0564e-05},
     {0.011492, 0.024911, 0.057786}},
    {"Coors Light Beer",
     {5.0922e-05, 4.301e-05, 0},
     {0.006164, 0.013984, 0.034983}},
    {"Clorox",
     {0.0024035, 0.0031373, 0.003991},
     {0.0033542, 0.014892, 0.026297}},
    {"Apple Juice",
     {0.00013612, 0.00015836, 0.000227},
     {0.012957, 0.023741, 0.052184}},
    {"Cranberry Juice",
     {0.00010402, 0.00011646, 7.8139e-05},
     {0.039437, 0.094223, 0.12426}},
    {"Grape Juice", {5.382e-05, 0, 0}, {0.10404, 0.23958, 0.29325}},
    {"Ruby Grapefruit Juice",
     {0.011002, 0.010927, 0.011036},
     {0.085867, 0.18314, 0.25262}},
    {"White Grapefruit Juice",
     {0.22826, 0.23998, 0.32748},
     {0.0138, 0.018831, 0.056781}},
    {"Shampoo",
     {0.0007176, 0.0008303, 0.0009016},
     {0.014107, 0.045693, 0.061717}},
    {"Strawberry Shampoo",
     {0.00015671, 0.00015947, 1.518e-05},
     {0.01449, 0.05796, 0.075823}},
    {"Head & Shoulders Shampoo",
     {0.023805, 0.028804, 0.034306},
     {0.084621, 0.15688, 0.20365}},
    {"Lemon Tea Powder",
     {0.040224, 0.045264, 0.051081},
     {2.4288, 4.5757, 7.2127}},
    {"Orange Powder",
     {0.00015617, 0.00017482, 0.0001762},
     {0.001449, 0.003441, 0.007863}},
    {"Pink Lemonade Powder",
     {0.00012103, 0.00013073, 0.00012528},
     {0.001165, 0.002366, 0.003195}},
    {"Cappuccino Powder", {1.8436, 2.5851, 2.1662}, {35.844, 49.547, 61.084}},
    {"Salt Powder", {0.027333, 0.032451, 0.031979}, {0.28415, 0.3257, 0.34148}},
    {"Sugar Powder",
     {0.00022272, 0.00025513, 0.000271},
     {0.012638, 0.031051, 0.050124}},
    {"Suisse Mocha Powder", {2.7979, 3.5452, 4.3365}, {17.502, 27.004, 35.433}},
    {"Pacific Ocean Surface Water",
     {0.0001764, 0.00032095, 0.00019617},
     {0.031845, 0.031324, 0.030147}}};

bool BSSRDF::getScatteringProperties(const string &name, Spectrum *sigma_a, Spectrum *sigma_prime_s) {
    for (auto &mss : SubsurfaceParameterTable) {
        if (name == mss.name) {
            *sigma_a = Spectrum::fromRGB(mss.sigma_a);
            *sigma_prime_s = Spectrum::fromRGB(mss.sigma_prime_s);
            return true;
        }
    }
    return false;
}
