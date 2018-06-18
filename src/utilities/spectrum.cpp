#include "spectrum.h"


bool SpectrumUtil::samplesAreSorted(const float *lambda, const float *vals, int n)
{
    for (int i = 0; i < n - 1; ++i)
            if (lambda[i] > lambda[i + 1]) return false;
        return true;
}

void SpectrumUtil::sortSamples(float *lambda, float *vals, int n) {
    vector<pair<float, float>> sortVec;
    sortVec.reserve(n);
    for (int i = 0; i < n; ++i)
        sortVec.push_back(make_pair(lambda[i], vals[i]));
    sort(sortVec.begin(), sortVec.end());
    for (int i = 0; i < n; ++i) {
        lambda[i] = sortVec[i].first;
        vals[i] = sortVec[i].second;
    }
}

float SpectrumUtil::interpolateSamples(const float *lambda, const float *vals, int n, float l)
{
    if (l <= lambda[0]) return vals[0];
    if (l >= lambda[n - 1]) return vals[n - 1];
    int offset = Math::findInterval(n, [&](int index) { return lambda[index] <= l; });
    float t = (l - lambda[offset]) / (lambda[offset + 1] - lambda[offset]);
    return Math::lerp(t, vals[offset], vals[offset + 1]);
}

void SpectrumUtil::blackbody(const Float *lambda, int n, Float T, Float *Le)
{
    if (T <= 0) {
        for (int i = 0; i < n; ++i) Le[i] = 0.f;
        return;
    }
    const Float c = 299792458;
    const Float h = 6.62606957e-34;
    const Float kb = 1.3806488e-23;
    for (int i = 0; i < n; ++i) {
        // Compute emitted radiance for blackbody at wavelength _lambda[i]_
        Float l = lambda[i] * 1e-9;
        Float lambda5 = pow(l, 5);
        Le[i] = (2 * h * c * c) / (lambda5 * (exp((h * c) / (l * kb * T)) - 1));
    }
}

void SpectrumUtil::blackbodyNormalized(const Float *lambda, int n, Float T, Float *Le) {
    blackbody(lambda, n, T, Le);
    // Normalize _Le_ values based on maximum blackbody radiance
    Float lambdaMax = 2.8977721e-3 / T * 1e9;
    Float maxL;
    blackbody(&lambdaMax, 1, T, &maxL);
    for (int i = 0; i < n; ++i) Le[i] /= maxL;
}

RGBSpectrum RGBSpectrum::fromSampled(const float *lambda, const float *v, int n)
{
    // Sort samples if unordered, use sorted for returned spectrum
    if (!samplesAreSorted(lambda, v, n)) {
        vector<float> slambda(&lambda[0], &lambda[n]);
        vector<float> sv(&v[0], &v[n]);
        sortSamples(&slambda[0], &sv[0], n);
        return fromSampled(&slambda[0], &sv[0], n);
    }
    float xyz[3] = {0, 0, 0};
    for (int i = 0; i < NUM_CIE_SAMPLES; ++i) {
        float val = interpolateSamples(lambda, v, n, CIE_LAMBDA[i]);
        xyz[0] += val * CIE_X[i];
        xyz[1] += val * CIE_Y[i];
        xyz[2] += val * CIE_Z[i];
    }
    float scale = float(CIE_LAMBDA[NUM_CIE_SAMPLES - 1] - CIE_LAMBDA[0]) /
                  float(CIE_Y_INTEGRAL * NUM_CIE_SAMPLES);
    xyz[0] *= scale;
    xyz[1] *= scale;
    xyz[2] *= scale;
    return fromXYZ(xyz);
}

const float SpectrumUtil::CIE_LAMBDA[NUM_CIE_SAMPLES] = {
    360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374,
    375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389,
    390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404,
    405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419,
    420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434,
    435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449,
    450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464,
    465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479,
    480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494,
    495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509,
    510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524,
    525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539,
    540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554,
    555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569,
    570, 571, 572, 573, 574, 575, 576, 577, 578, 579, 580, 581, 582, 583, 584,
    585, 586, 587, 588, 589, 590, 591, 592, 593, 594, 595, 596, 597, 598, 599,
    600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 611, 612, 613, 614,
    615, 616, 617, 618, 619, 620, 621, 622, 623, 624, 625, 626, 627, 628, 629,
    630, 631, 632, 633, 634, 635, 636, 637, 638, 639, 640, 641, 642, 643, 644,
    645, 646, 647, 648, 649, 650, 651, 652, 653, 654, 655, 656, 657, 658, 659,
    660, 661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674,
    675, 676, 677, 678, 679, 680, 681, 682, 683, 684, 685, 686, 687, 688, 689,
    690, 691, 692, 693, 694, 695, 696, 697, 698, 699, 700, 701, 702, 703, 704,
    705, 706, 707, 708, 709, 710, 711, 712, 713, 714, 715, 716, 717, 718, 719,
    720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 731, 732, 733, 734,
    735, 736, 737, 738, 739, 740, 741, 742, 743, 744, 745, 746, 747, 748, 749,
    750, 751, 752, 753, 754, 755, 756, 757, 758, 759, 760, 761, 762, 763, 764,
    765, 766, 767, 768, 769, 770, 771, 772, 773, 774, 775, 776, 777, 778, 779,
    780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794,
    795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805, 806, 807, 808, 809,
    810, 811, 812, 813, 814, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824,
    825, 826, 827, 828, 829, 830
};

const float SpectrumUtil::CIE_X[NUM_CIE_SAMPLES] = {
    // CIE X function values
    0.0001299000f,   0.0001458470f,   0.0001638021f,   0.0001840037f,
    0.0002066902f,   0.0002321000f,   0.0002607280f,   0.0002930750f,
    0.0003293880f,   0.0003699140f,   0.0004149000f,   0.0004641587f,
    0.0005189860f,   0.0005818540f,   0.0006552347f,   0.0007416000f,
    0.0008450296f,   0.0009645268f,   0.001094949f,    0.001231154f,
    0.001368000f,    0.001502050f,    0.001642328f,    0.001802382f,
    0.001995757f,    0.002236000f,    0.002535385f,    0.002892603f,
    0.003300829f,    0.003753236f,    0.004243000f,    0.004762389f,
    0.005330048f,    0.005978712f,    0.006741117f,    0.007650000f,
    0.008751373f,    0.01002888f,     0.01142170f,     0.01286901f,
    0.01431000f,     0.01570443f,     0.01714744f,     0.01878122f,
    0.02074801f,     0.02319000f,     0.02620736f,     0.02978248f,
    0.03388092f,     0.03846824f,     0.04351000f,     0.04899560f,
    0.05502260f,     0.06171880f,     0.06921200f,     0.07763000f,
    0.08695811f,     0.09717672f,     0.1084063f,      0.1207672f,
    0.1343800f,      0.1493582f,      0.1653957f,      0.1819831f,
    0.1986110f,      0.2147700f,      0.2301868f,      0.2448797f,
    0.2587773f,      0.2718079f,      0.2839000f,      0.2949438f,
    0.3048965f,      0.3137873f,      0.3216454f,      0.3285000f,
    0.3343513f,      0.3392101f,      0.3431213f,      0.3461296f,
    0.3482800f,      0.3495999f,      0.3501474f,      0.3500130f,
    0.3492870f,      0.3480600f,      0.3463733f,      0.3442624f,
    0.3418088f,      0.3390941f,      0.3362000f,      0.3331977f,
    0.3300411f,      0.3266357f,      0.3228868f,      0.3187000f,
    0.3140251f,      0.3088840f,      0.3032904f,      0.2972579f,
    0.2908000f,      0.2839701f,      0.2767214f,      0.2689178f,
    0.2604227f,      0.2511000f,      0.2408475f,      0.2298512f,
    0.2184072f,      0.2068115f,      0.1953600f,      0.1842136f,
    0.1733273f,      0.1626881f,      0.1522833f,      0.1421000f,
    0.1321786f,      0.1225696f,      0.1132752f,      0.1042979f,
    0.09564000f,     0.08729955f,     0.07930804f,     0.07171776f,
    0.06458099f,     0.05795001f,     0.05186211f,     0.04628152f,
    0.04115088f,     0.03641283f,     0.03201000f,     0.02791720f,
    0.02414440f,     0.02068700f,     0.01754040f,     0.01470000f,
    0.01216179f,     0.009919960f,    0.007967240f,    0.006296346f,
    0.004900000f,    0.003777173f,    0.002945320f,    0.002424880f,
    0.002236293f,    0.002400000f,    0.002925520f,    0.003836560f,
    0.005174840f,    0.006982080f,    0.009300000f,    0.01214949f,
    0.01553588f,     0.01947752f,     0.02399277f,     0.02910000f,
    0.03481485f,     0.04112016f,     0.04798504f,     0.05537861f,
    0.06327000f,     0.07163501f,     0.08046224f,     0.08973996f,
    0.09945645f,     0.1096000f,      0.1201674f,      0.1311145f,
    0.1423679f,      0.1538542f,      0.1655000f,      0.1772571f,
    0.1891400f,      0.2011694f,      0.2133658f,      0.2257499f,
    0.2383209f,      0.2510668f,      0.2639922f,      0.2771017f,
    0.2904000f,      0.3038912f,      0.3175726f,      0.3314384f,
    0.3454828f,      0.3597000f,      0.3740839f,      0.3886396f,
    0.4033784f,      0.4183115f,      0.4334499f,      0.4487953f,
    0.4643360f,      0.4800640f,      0.4959713f,      0.5120501f,
    0.5282959f,      0.5446916f,      0.5612094f,      0.5778215f,
    0.5945000f,      0.6112209f,      0.6279758f,      0.6447602f,
    0.6615697f,      0.6784000f,      0.6952392f,      0.7120586f,
    0.7288284f,      0.7455188f,      0.7621000f,      0.7785432f,
    0.7948256f,      0.8109264f,      0.8268248f,      0.8425000f,
    0.8579325f,      0.8730816f,      0.8878944f,      0.9023181f,
    0.9163000f,      0.9297995f,      0.9427984f,      0.9552776f,
    0.9672179f,      0.9786000f,      0.9893856f,      0.9995488f,
    1.0090892f,      1.0180064f,      1.0263000f,      1.0339827f,
    1.0409860f,      1.0471880f,      1.0524667f,      1.0567000f,
    1.0597944f,      1.0617992f,      1.0628068f,      1.0629096f,
    1.0622000f,      1.0607352f,      1.0584436f,      1.0552244f,
    1.0509768f,      1.0456000f,      1.0390369f,      1.0313608f,
    1.0226662f,      1.0130477f,      1.0026000f,      0.9913675f,
    0.9793314f,      0.9664916f,      0.9528479f,      0.9384000f,
    0.9231940f,      0.9072440f,      0.8905020f,      0.8729200f,
    0.8544499f,      0.8350840f,      0.8149460f,      0.7941860f,
    0.7729540f,      0.7514000f,      0.7295836f,      0.7075888f,
    0.6856022f,      0.6638104f,      0.6424000f,      0.6215149f,
    0.6011138f,      0.5811052f,      0.5613977f,      0.5419000f,
    0.5225995f,      0.5035464f,      0.4847436f,      0.4661939f,
    0.4479000f,      0.4298613f,      0.4120980f,      0.3946440f,
    0.3775333f,      0.3608000f,      0.3444563f,      0.3285168f,
    0.3130192f,      0.2980011f,      0.2835000f,      0.2695448f,
    0.2561184f,      0.2431896f,      0.2307272f,      0.2187000f,
    0.2070971f,      0.1959232f,      0.1851708f,      0.1748323f,
    0.1649000f,      0.1553667f,      0.1462300f,      0.1374900f,
    0.1291467f,      0.1212000f,      0.1136397f,      0.1064650f,
    0.09969044f,     0.09333061f,     0.08740000f,     0.08190096f,
    0.07680428f,     0.07207712f,     0.06768664f,     0.06360000f,
    0.05980685f,     0.05628216f,     0.05297104f,     0.04981861f,
    0.04677000f,     0.04378405f,     0.04087536f,     0.03807264f,
    0.03540461f,     0.03290000f,     0.03056419f,     0.02838056f,
    0.02634484f,     0.02445275f,     0.02270000f,     0.02108429f,
    0.01959988f,     0.01823732f,     0.01698717f,     0.01584000f,
    0.01479064f,     0.01383132f,     0.01294868f,     0.01212920f,
    0.01135916f,     0.01062935f,     0.009938846f,    0.009288422f,
    0.008678854f,    0.008110916f,    0.007582388f,    0.007088746f,
    0.006627313f,    0.006195408f,    0.005790346f,    0.005409826f,
    0.005052583f,    0.004717512f,    0.004403507f,    0.004109457f,
    0.003833913f,    0.003575748f,    0.003334342f,    0.003109075f,
    0.002899327f,    0.002704348f,    0.002523020f,    0.002354168f,
    0.002196616f,    0.002049190f,    0.001910960f,    0.001781438f,
    0.001660110f,    0.001546459f,    0.001439971f,    0.001340042f,
    0.001246275f,    0.001158471f,    0.001076430f,    0.0009999493f,
    0.0009287358f,   0.0008624332f,   0.0008007503f,   0.0007433960f,
    0.0006900786f,   0.0006405156f,   0.0005945021f,   0.0005518646f,
    0.0005124290f,   0.0004760213f,   0.0004424536f,   0.0004115117f,
    0.0003829814f,   0.0003566491f,   0.0003323011f,   0.0003097586f,
    0.0002888871f,   0.0002695394f,   0.0002515682f,   0.0002348261f,
    0.0002191710f,   0.0002045258f,   0.0001908405f,   0.0001780654f,
    0.0001661505f,   0.0001550236f,   0.0001446219f,   0.0001349098f,
    0.0001258520f,   0.0001174130f,   0.0001095515f,   0.0001022245f,
    0.00009539445f,  0.00008902390f,  0.00008307527f,  0.00007751269f,
    0.00007231304f,  0.00006745778f,  0.00006292844f,  0.00005870652f,
    0.00005477028f,  0.00005109918f,  0.00004767654f,  0.00004448567f,
    0.00004150994f,  0.00003873324f,  0.00003614203f,  0.00003372352f,
    0.00003146487f,  0.00002935326f,  0.00002737573f,  0.00002552433f,
    0.00002379376f,  0.00002217870f,  0.00002067383f,  0.00001927226f,
    0.00001796640f,  0.00001674991f,  0.00001561648f,  0.00001455977f,
    0.00001357387f,  0.00001265436f,  0.00001179723f,  0.00001099844f,
    0.00001025398f,  0.000009559646f, 0.000008912044f, 0.000008308358f,
    0.000007745769f, 0.000007221456f, 0.000006732475f, 0.000006276423f,
    0.000005851304f, 0.000005455118f, 0.000005085868f, 0.000004741466f,
    0.000004420236f, 0.000004120783f, 0.000003841716f, 0.000003581652f,
    0.000003339127f, 0.000003112949f, 0.000002902121f, 0.000002705645f,
    0.000002522525f, 0.000002351726f, 0.000002192415f, 0.000002043902f,
    0.000001905497f, 0.000001776509f, 0.000001656215f, 0.000001544022f,
    0.000001439440f, 0.000001341977f, 0.000001251141f};

const float SpectrumUtil::CIE_Y[NUM_CIE_SAMPLES] = {
    // CIE Y function values
    0.000003917000f,  0.000004393581f,  0.000004929604f,  0.000005532136f,
    0.000006208245f,  0.000006965000f,  0.000007813219f,  0.000008767336f,
    0.000009839844f,  0.00001104323f,   0.00001239000f,   0.00001388641f,
    0.00001555728f,   0.00001744296f,   0.00001958375f,   0.00002202000f,
    0.00002483965f,   0.00002804126f,   0.00003153104f,   0.00003521521f,
    0.00003900000f,   0.00004282640f,   0.00004691460f,   0.00005158960f,
    0.00005717640f,   0.00006400000f,   0.00007234421f,   0.00008221224f,
    0.00009350816f,   0.0001061361f,    0.0001200000f,    0.0001349840f,
    0.0001514920f,    0.0001702080f,    0.0001918160f,    0.0002170000f,
    0.0002469067f,    0.0002812400f,    0.0003185200f,    0.0003572667f,
    0.0003960000f,    0.0004337147f,    0.0004730240f,    0.0005178760f,
    0.0005722187f,    0.0006400000f,    0.0007245600f,    0.0008255000f,
    0.0009411600f,    0.001069880f,     0.001210000f,     0.001362091f,
    0.001530752f,     0.001720368f,     0.001935323f,     0.002180000f,
    0.002454800f,     0.002764000f,     0.003117800f,     0.003526400f,
    0.004000000f,     0.004546240f,     0.005159320f,     0.005829280f,
    0.006546160f,     0.007300000f,     0.008086507f,     0.008908720f,
    0.009767680f,     0.01066443f,      0.01160000f,      0.01257317f,
    0.01358272f,      0.01462968f,      0.01571509f,      0.01684000f,
    0.01800736f,      0.01921448f,      0.02045392f,      0.02171824f,
    0.02300000f,      0.02429461f,      0.02561024f,      0.02695857f,
    0.02835125f,      0.02980000f,      0.03131083f,      0.03288368f,
    0.03452112f,      0.03622571f,      0.03800000f,      0.03984667f,
    0.04176800f,      0.04376600f,      0.04584267f,      0.04800000f,
    0.05024368f,      0.05257304f,      0.05498056f,      0.05745872f,
    0.06000000f,      0.06260197f,      0.06527752f,      0.06804208f,
    0.07091109f,      0.07390000f,      0.07701600f,      0.08026640f,
    0.08366680f,      0.08723280f,      0.09098000f,      0.09491755f,
    0.09904584f,      0.1033674f,       0.1078846f,       0.1126000f,
    0.1175320f,       0.1226744f,       0.1279928f,       0.1334528f,
    0.1390200f,       0.1446764f,       0.1504693f,       0.1564619f,
    0.1627177f,       0.1693000f,       0.1762431f,       0.1835581f,
    0.1912735f,       0.1994180f,       0.2080200f,       0.2171199f,
    0.2267345f,       0.2368571f,       0.2474812f,       0.2586000f,
    0.2701849f,       0.2822939f,       0.2950505f,       0.3085780f,
    0.3230000f,       0.3384021f,       0.3546858f,       0.3716986f,
    0.3892875f,       0.4073000f,       0.4256299f,       0.4443096f,
    0.4633944f,       0.4829395f,       0.5030000f,       0.5235693f,
    0.5445120f,       0.5656900f,       0.5869653f,       0.6082000f,
    0.6293456f,       0.6503068f,       0.6708752f,       0.6908424f,
    0.7100000f,       0.7281852f,       0.7454636f,       0.7619694f,
    0.7778368f,       0.7932000f,       0.8081104f,       0.8224962f,
    0.8363068f,       0.8494916f,       0.8620000f,       0.8738108f,
    0.8849624f,       0.8954936f,       0.9054432f,       0.9148501f,
    0.9237348f,       0.9320924f,       0.9399226f,       0.9472252f,
    0.9540000f,       0.9602561f,       0.9660074f,       0.9712606f,
    0.9760225f,       0.9803000f,       0.9840924f,       0.9874812f,
    0.9903128f,       0.9928116f,       0.9949501f,       0.9967108f,
    0.9980983f,       0.9991120f,       0.9997482f,       1.0000000f,
    0.9998567f,       0.9993046f,       0.9983255f,       0.9968987f,
    0.9950000f,       0.9926005f,       0.9897426f,       0.9864444f,
    0.9827241f,       0.9786000f,       0.9740837f,       0.9691712f,
    0.9638568f,       0.9581349f,       0.9520000f,       0.9454504f,
    0.9384992f,       0.9311628f,       0.9234576f,       0.9154000f,
    0.9070064f,       0.8982772f,       0.8892048f,       0.8797816f,
    0.8700000f,       0.8598613f,       0.8493920f,       0.8386220f,
    0.8275813f,       0.8163000f,       0.8047947f,       0.7930820f,
    0.7811920f,       0.7691547f,       0.7570000f,       0.7447541f,
    0.7324224f,       0.7200036f,       0.7074965f,       0.6949000f,
    0.6822192f,       0.6694716f,       0.6566744f,       0.6438448f,
    0.6310000f,       0.6181555f,       0.6053144f,       0.5924756f,
    0.5796379f,       0.5668000f,       0.5539611f,       0.5411372f,
    0.5283528f,       0.5156323f,       0.5030000f,       0.4904688f,
    0.4780304f,       0.4656776f,       0.4534032f,       0.4412000f,
    0.4290800f,       0.4170360f,       0.4050320f,       0.3930320f,
    0.3810000f,       0.3689184f,       0.3568272f,       0.3447768f,
    0.3328176f,       0.3210000f,       0.3093381f,       0.2978504f,
    0.2865936f,       0.2756245f,       0.2650000f,       0.2547632f,
    0.2448896f,       0.2353344f,       0.2260528f,       0.2170000f,
    0.2081616f,       0.1995488f,       0.1911552f,       0.1829744f,
    0.1750000f,       0.1672235f,       0.1596464f,       0.1522776f,
    0.1451259f,       0.1382000f,       0.1315003f,       0.1250248f,
    0.1187792f,       0.1127691f,       0.1070000f,       0.1014762f,
    0.09618864f,      0.09112296f,      0.08626485f,      0.08160000f,
    0.07712064f,      0.07282552f,      0.06871008f,      0.06476976f,
    0.06100000f,      0.05739621f,      0.05395504f,      0.05067376f,
    0.04754965f,      0.04458000f,      0.04175872f,      0.03908496f,
    0.03656384f,      0.03420048f,      0.03200000f,      0.02996261f,
    0.02807664f,      0.02632936f,      0.02470805f,      0.02320000f,
    0.02180077f,      0.02050112f,      0.01928108f,      0.01812069f,
    0.01700000f,      0.01590379f,      0.01483718f,      0.01381068f,
    0.01283478f,      0.01192000f,      0.01106831f,      0.01027339f,
    0.009533311f,     0.008846157f,     0.008210000f,     0.007623781f,
    0.007085424f,     0.006591476f,     0.006138485f,     0.005723000f,
    0.005343059f,     0.004995796f,     0.004676404f,     0.004380075f,
    0.004102000f,     0.003838453f,     0.003589099f,     0.003354219f,
    0.003134093f,     0.002929000f,     0.002738139f,     0.002559876f,
    0.002393244f,     0.002237275f,     0.002091000f,     0.001953587f,
    0.001824580f,     0.001703580f,     0.001590187f,     0.001484000f,
    0.001384496f,     0.001291268f,     0.001204092f,     0.001122744f,
    0.001047000f,     0.0009765896f,    0.0009111088f,    0.0008501332f,
    0.0007932384f,    0.0007400000f,    0.0006900827f,    0.0006433100f,
    0.0005994960f,    0.0005584547f,    0.0005200000f,    0.0004839136f,
    0.0004500528f,    0.0004183452f,    0.0003887184f,    0.0003611000f,
    0.0003353835f,    0.0003114404f,    0.0002891656f,    0.0002684539f,
    0.0002492000f,    0.0002313019f,    0.0002146856f,    0.0001992884f,
    0.0001850475f,    0.0001719000f,    0.0001597781f,    0.0001486044f,
    0.0001383016f,    0.0001287925f,    0.0001200000f,    0.0001118595f,
    0.0001043224f,    0.00009733560f,   0.00009084587f,   0.00008480000f,
    0.00007914667f,   0.00007385800f,   0.00006891600f,   0.00006430267f,
    0.00006000000f,   0.00005598187f,   0.00005222560f,   0.00004871840f,
    0.00004544747f,   0.00004240000f,   0.00003956104f,   0.00003691512f,
    0.00003444868f,   0.00003214816f,   0.00003000000f,   0.00002799125f,
    0.00002611356f,   0.00002436024f,   0.00002272461f,   0.00002120000f,
    0.00001977855f,   0.00001845285f,   0.00001721687f,   0.00001606459f,
    0.00001499000f,   0.00001398728f,   0.00001305155f,   0.00001217818f,
    0.00001136254f,   0.00001060000f,   0.000009885877f,  0.000009217304f,
    0.000008592362f,  0.000008009133f,  0.000007465700f,  0.000006959567f,
    0.000006487995f,  0.000006048699f,  0.000005639396f,  0.000005257800f,
    0.000004901771f,  0.000004569720f,  0.000004260194f,  0.000003971739f,
    0.000003702900f,  0.000003452163f,  0.000003218302f,  0.000003000300f,
    0.000002797139f,  0.000002607800f,  0.000002431220f,  0.000002266531f,
    0.000002113013f,  0.000001969943f,  0.000001836600f,  0.000001712230f,
    0.000001596228f,  0.000001488090f,  0.000001387314f,  0.000001293400f,
    0.000001205820f,  0.000001124143f,  0.000001048009f,  0.0000009770578f,
    0.0000009109300f, 0.0000008492513f, 0.0000007917212f, 0.0000007380904f,
    0.0000006881098f, 0.0000006415300f, 0.0000005980895f, 0.0000005575746f,
    0.0000005198080f, 0.0000004846123f, 0.0000004518100f
};

const float SpectrumUtil::CIE_Z[NUM_CIE_SAMPLES] = {
    0.0006061f,     0.000680879f,     0.000765146f,     0.000860012f,
  0.000966593f,        0.001086f,      0.00122059f,      0.00137273f,
   0.00154358f,      0.00173429f,        0.001946f,      0.00217778f,
   0.00243581f,      0.00273195f,      0.00307806f,        0.003486f,
   0.00397523f,      0.00454088f,      0.00515832f,      0.00580291f,
      0.00645f,      0.00708322f,      0.00774549f,      0.00850115f,
   0.00941454f,         0.01055f,       0.0119658f,       0.0136559f,
     0.015588f,       0.0177302f,         0.02005f,       0.0225114f,
    0.0252029f,       0.0282797f,        0.031897f,         0.03621f,
    0.0414377f,       0.0475037f,       0.0541199f,        0.060998f,
      0.06785f,       0.0744863f,       0.0813616f,       0.0891536f,
    0.0985405f,          0.1102f,        0.124613f,        0.141702f,
     0.161304f,        0.183257f,          0.2074f,        0.233692f,
     0.262611f,        0.294775f,        0.330799f,          0.3713f,
     0.416209f,        0.465464f,        0.519695f,         0.57953f,
       0.6456f,        0.718484f,        0.796713f,        0.877846f,
     0.959439f,         1.03905f,         1.11537f,          1.1885f,
      1.25812f,         1.32393f,          1.3856f,         1.44264f,
       1.4948f,         1.54219f,         1.58488f,         1.62296f,
       1.6564f,          1.6853f,         1.70987f,         1.73038f,
      1.74706f,         1.76004f,         1.76962f,         1.77626f,
      1.78043f,          1.7826f,         1.78297f,          1.7817f,
       1.7792f,         1.77587f,         1.77211f,         1.76826f,
      1.76404f,         1.75894f,         1.75247f,          1.7441f,
      1.73356f,         1.72086f,         1.70594f,         1.68874f,
       1.6692f,         1.64753f,         1.62341f,         1.59602f,
      1.56453f,          1.5281f,         1.48611f,         1.43952f,
      1.38988f,         1.33874f,         1.28764f,         1.23742f,
      1.18782f,         1.13876f,         1.09015f,          1.0419f,
     0.994198f,        0.947347f,        0.901453f,        0.856619f,
      0.81295f,        0.770517f,        0.729445f,        0.689914f,
     0.652105f,          0.6162f,        0.582329f,        0.550416f,
     0.520338f,        0.491967f,         0.46518f,        0.439925f,
     0.416184f,        0.393882f,        0.372946f,          0.3533f,
     0.334858f,        0.317552f,        0.301338f,        0.286169f,
        0.272f,        0.258817f,        0.246484f,        0.234772f,
     0.223453f,          0.2123f,        0.201169f,         0.19012f,
     0.179225f,        0.168561f,          0.1582f,        0.148138f,
     0.138376f,        0.128994f,        0.120075f,          0.1117f,
     0.103905f,       0.0966675f,       0.0899827f,       0.0838453f,
      0.07825f,        0.073209f,       0.0686782f,       0.0645678f,
    0.0607883f,         0.05725f,       0.0539044f,       0.0507466f,
    0.0477528f,       0.0448986f,         0.04216f,       0.0395073f,
    0.0369356f,       0.0344584f,       0.0320887f,         0.02984f,
    0.0277118f,       0.0256944f,       0.0237872f,       0.0219893f,
       0.0203f,       0.0187181f,       0.0172404f,       0.0158636f,
    0.0145846f,          0.0134f,       0.0123072f,       0.0113019f,
    0.0103779f,      0.00952931f,         0.00875f,       0.0080352f,
    0.0073816f,       0.0067854f,       0.0062428f,         0.00575f,
    0.0053036f,       0.0048998f,       0.0045342f,       0.0042024f,
       0.0039f,       0.0036232f,       0.0033706f,       0.0031414f,
    0.0029348f,         0.00275f,       0.0025852f,       0.0024386f,
    0.0023094f,       0.0021968f,          0.0021f,      0.00201773f,
    0.0019482f,       0.0018898f,      0.00184093f,          0.0018f,
   0.00176627f,       0.0017378f,       0.0017112f,      0.00168307f,
      0.00165f,      0.00161013f,       0.0015644f,       0.0015136f,
   0.00145853f,          0.0014f,      0.00133667f,         0.00127f,
     0.001205f,      0.00114667f,          0.0011f,       0.0010688f,
    0.0010494f,       0.0010356f,       0.0010212f,           0.001f,
   0.00096864f,      0.00092992f,      0.00088688f,      0.00084256f,
       0.0008f,      0.00076096f,      0.00072368f,      0.00068592f,
   0.00064544f,          0.0006f,     0.000547867f,       0.0004916f,
    0.0004354f,     0.000383467f,         0.00034f,     0.000307253f,
   0.00028316f,      0.00026544f,     0.000251813f,         0.00024f,
  0.000229547f,      0.00022064f,      0.00021196f,     0.000202187f,
      0.00019f,     0.000174213f,      0.00015564f,      0.00013596f,
  0.000116853f,          0.0001f,     8.61333e-05f,        7.46e-05f,
      6.5e-05f,     5.69333e-05f,           5e-05f,       4.416e-05f,
    3.948e-05f,       3.572e-05f,       3.264e-05f,           3e-05f,
  2.76533e-05f,       2.556e-05f,       2.364e-05f,     2.18133e-05f,
        2e-05f,     1.81333e-05f,        1.62e-05f,        1.42e-05f,
  1.21333e-05f,           1e-05f,     7.73333e-06f,         5.4e-06f,
      3.2e-06f,     1.33333e-06f,           0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f,               0.0f,
    0.0f,               0.0f,               0.0f
};
