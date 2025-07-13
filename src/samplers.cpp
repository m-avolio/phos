#include "samplers.h"
#include "rkcommon/math/vec.h"

// Andrew Kensler 2013
inline unsigned permute(unsigned i, unsigned l, unsigned p) {
    unsigned w = l - 1;
    w |= w >> 1;
    w |= w >> 2;
    w |= w >> 4;
    w |= w >> 8;
    w |= w >> 16;

    do {
        i ^= p;
        i *= 0xe170893d;
        i ^= p >> 16;
        i ^= (i & w) >> 4;
        i ^= p >> 8;
        i *= 0x0929eb3f;
        i ^= p >> 23;
        i ^= (i & w) >> 1;
        i *= (1 | p >> 27);
        i *= 0x6935fa69;
        i ^= (i & w) >> 11;
        i *= 0x74dcb303;
        i ^= (i & w) >> 2;
        i *= 0x9e501cc3;
        i ^= (i & w) >> 2;
        i *= 0xc860a3df;
        i &= w;
        i ^= i >> 5;
    } while (i >= l);

    return (i + p) % l;
}

// Correlated Multi-Jittered Sampling
// Andrew Kensler 2013
inline float randfloat(unsigned i, unsigned p) {
    i ^= p;
    i ^= i >> 17;
    i ^= i >> 10;  i *= 0xb36534e5;
    i ^= i >> 12;  i *= 0x93fc4795;
    i ^= i >> 21;  i ^= 0xdf6e307f;
    i ^= i >> 17;  i *= 1 | (p >> 18);
    return i * (1.0f / 4294967808.0f);
}

// Andrew Kensler 2013
rkcommon::math::vec2f cmj(int s, int N, int p, float a) {
    int m = static_cast<int>(sqrtf(N * a));
    int n = (N + m - 1) / m;

    s = permute(s, N, p * 0x51633e2d);
    int sx = permute(s % m, m, p * 0x68bc21eb);
    int sy = permute(s / m, n, p * 0x02e5be93);

    float jx = randfloat(s, p * 0x967a889b);
    float jy = randfloat(s, p * 0x368cc8b7);

    rkcommon::math::vec2f r = rkcommon::math::vec2f(
        (sx + (sy + jx) / n) / m,
        (s + jy) / N
    );

    return r;
}