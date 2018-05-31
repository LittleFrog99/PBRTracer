#include "utilities.h"

bool Math::solveQuadratic(Float a, Float b, Float c, Float *t0, Float *t1)
{
    Float discr = SQ(b) - 4.0 * a * c;
    if (discr < 0) return false;
    Float rtDiscr = sqrt(discr);
    Float q;
    if (b < 0) q = -.5 * (b - rtDiscr);
    else q = -.5 * (b + rtDiscr);
    *t0 = q / a;
    *t1 = c / q;
    if (*t0 > *t1) swap(*t0, *t1);
    return true;
}

bool Math::solveLinear2x2(const Float A[2][2], const Float B[2], Float *x0, Float *x1)
{
    Float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (std::abs(det) < 1e-10f) return false;
    *x0 = (A[1][1] * B[0] - A[0][1] * B[1]) / det;
    *x1 = (A[0][0] * B[1] - A[1][0] * B[0]) / det;
    if (isnan(*x0) || isnan(*x1)) return false;
    return true;
}
