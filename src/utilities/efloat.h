#ifndef UTILITY_EFLOAT
#define UTILITY_EFLOAT

#include "stringprint.h"

class EFloat {
public:
    EFloat() {}
    EFloat(float v, float err = 0.f) : v(v) {
        if (err == 0.)
            low = high = v;
        else {
            low = nextFloatDown(v - err);
            high = nextFloatUp(v + err);
        }
// Store high precision reference value in _EFloat_
/* #ifndef NDEBUG
        vPrecise = v;
        check();
#endif  // NDEBUG*/
    }

    EFloat(const EFloat &ef) {
        ef.check();
        v = ef.v;
        low = ef.low;
        high = ef.high;
#ifndef NDEBUG
        // vPrecise = ef.vPrecise;
#endif
    }

#ifndef NDEBUG
    EFloat(float v, long double lD, float err) : EFloat(v, err) {
        vPrecise = lD;
        check();
    }
#endif  // DEBUG

    void check() const {
        if (!isinf(low) && !isnan(low) && !isinf(high) && !isnan(high))
            CHECK_LE(low, high);
/* #ifndef NDEBUG
        if (!isinf(v) && !isnan(v)) {
            CHECK_LE(lowerBound(), vPrecise);
            CHECK_LE(vPrecise, upperBound());
        }
#endif */
    }

    float getAbsoluteError() const { return high - low; }
    float upperBound() const { return high; }
    float lowerBound() const { return low; }

#ifndef NDEBUG
    float getRelativeError() const {
        return abs((vPrecise - v) / vPrecise);
    }
    long double preciseValue() const { return vPrecise; }
#endif

    explicit operator float() const { return v; }
    explicit operator double() const { return v; }

    EFloat operator + (EFloat ef) const {
        EFloat r;
        r.v = v + ef.v;
#ifndef NDEBUG
        // r.vPrecise = vPrecise + ef.vPrecise;
#endif  // DEBUG
        // Interval arithemetic addition, with the result rounded away from
        // the value r.v in order to be conservative
        r.low = nextFloatDown(lowerBound() + ef.lowerBound());
        r.high = nextFloatUp(upperBound() + ef.upperBound());
        r.check();
        return r;
    }

    EFloat operator - (EFloat ef) const {
        EFloat r;
        r.v = v - ef.v;
#ifndef NDEBUG
        // r.vPrecise = vPrecise - ef.vPrecise;
#endif
        r.low = nextFloatDown(lowerBound() - ef.upperBound());
        r.high = nextFloatUp(upperBound() - ef.lowerBound());
        r.check();
        return r;
    }

    EFloat operator * (EFloat ef) const {
        EFloat r;
        r.v = v * ef.v;
#ifndef NDEBUG
        // r.vPrecise = vPrecise * ef.vPrecise;
#endif
        float prod[4] = {
            lowerBound() * ef.lowerBound(), upperBound() * ef.lowerBound(),
            lowerBound() * ef.upperBound(), upperBound() * ef.upperBound()};
        r.low = nextFloatDown(
            min(min(prod[0], prod[1]), min(prod[2], prod[3])));
        r.high = nextFloatUp(
            max(max(prod[0], prod[1]), max(prod[2], prod[3])));
        r.check();
        return r;
    }

    EFloat operator / (EFloat ef) const {
        EFloat r;
        r.v = v / ef.v;
#ifndef NDEBUG
        // r.vPrecise = vPrecise / ef.vPrecise;
#endif
        if (ef.low < 0 && ef.high > 0) {
            // Bah. The interval we're dividing by straddles zero, so just
            // return an interval of everything.
            r.low = -INFINITY;
            r.high = INFINITY;
        } else {
            float div[4] = {
                lowerBound() / ef.lowerBound(), upperBound() / ef.lowerBound(),
                lowerBound() / ef.upperBound(), upperBound() / ef.upperBound()};
            r.low = nextFloatDown(min(min(div[0], div[1]), min(div[2], div[3])));
            r.high = nextFloatUp(max(max(div[0], div[1]), max(div[2], div[3])));
        }
        r.check();
        return r;
    }

    EFloat operator - () const {
        EFloat r;
        r.v = -v;
#ifndef NDEBUG
        // r.vPrecise = -vPrecise;
#endif
        r.low = -high;
        r.high = -low;
        r.check();
        return r;
    }

    inline bool operator == (EFloat fe) const { return v == fe.v; }

    EFloat &operator = (const EFloat &ef) {
        ef.check();
        if (&ef != this) {
            v = ef.v;
            low = ef.low;
            high = ef.high;
#ifndef NDEBUG
            // vPrecise = ef.vPrecise;
#endif
        }
        return *this;
    }

    friend ostream & operator << (ostream &os, const EFloat &ef) {
        os << STRING_PRINTF("v=%f (%a) - [%f, %f]",
                           ef.v, ef.v, ef.low, ef.high);
#ifndef NDEBUG
        // os << STRING_PRINTF(", precise=%.30Lf", ef.vPrecise);
#endif // !NDEBUG
        return os;
    }

    friend inline EFloat sqrt(EFloat fe) {
        EFloat r;
        r.v = sqrt(fe.v);
#ifndef NDEBUG
        // r.vPrecise = sqrt(fe.vPrecise);
#endif
        r.low = nextFloatDown(sqrt(fe.low));
        r.high = nextFloatUp(sqrt(fe.high));
        r.check();
        return r;
    }

    friend inline EFloat abs(EFloat fe) {
        if (fe.low >= 0)
            // The entire interval is greater than zero, so we're all set.
            return fe;
        else if (fe.high <= 0) {
            // The entire interval is less than zero.
            EFloat r;
            r.v = -fe.v;
#ifndef NDEBUG
            // r.vPrecise = -fe.vPrecise;
#endif
            r.low = -fe.high;
            r.high = -fe.low;
            r.check();
            return r;
        } else {
            // The interval straddles zero.
            EFloat r;
            r.v = abs(fe.v);
#ifndef NDEBUG
            // r.vPrecise = abs(fe.vPrecise);
#endif
            r.low = 0;
            r.high = max(-fe.low, fe.high);
            r.check();
            return r;
        }
    }

    friend inline bool solveQuadratic(EFloat A, EFloat B, EFloat C, EFloat *t0, EFloat *t1) {
        // Find quadratic discriminant
        double discrim = SQ(double(B.v)) - 4. * double(A.v) * double(C.v);
        if (discrim < 0.) return false;
        double rootDiscrim = sqrt(discrim);

        EFloat floatRootDiscrim(rootDiscrim, MACHINE_EPSILON * rootDiscrim);

        // Compute quadratic _t_ values
        EFloat q;
        if ((float)B < 0)
            q = (B - floatRootDiscrim) * (-0.5);
        else
            q = (B + floatRootDiscrim) * (-0.5);
        *t0 = q / A;
        *t1 = C / q;
        if (float(*t0) > float(*t1)) swap(*t0, *t1);
        return true;
    }

private:
    float v, low, high;
#ifndef NDEBUG
    long double vPrecise;
#endif  // NDEBUG

};

inline EFloat operator * (float f, EFloat fe) { return EFloat(f) * fe; }
inline EFloat operator / (float f, EFloat fe) { return EFloat(f) / fe; }
inline EFloat operator + (float f, EFloat fe) { return EFloat(f) + fe; }
inline EFloat operator - (float f, EFloat fe) { return EFloat(f) - fe; }


#endif // UTILITY_EFLOAT
