"""Helpers: engineering value -> AGC octal words (sign-agreeing)."""
import math
def _w(n):  # signed int in [-16383,16383] -> ones complement octal
    return '%05o' % (n if n >= 0 else 0o77777 + n)
def sp(x, scale=0):
    n = int(round(x / 2.0**scale * 16384))
    n = max(-16383, min(16383, n))
    return _w(n)
def dpi(x, scale=0):
    n = int(round(x / 2.0**scale * 2**28))
    lim = 2**28 - 1
    return max(-lim, min(lim, n))
def dp(x, scale=0):
    n = dpi(x, scale)
    s = -1 if n < 0 else 1
    a = abs(n)
    hi, lo = a >> 14, a & 0o37777
    return '%s,%s' % (_w(s*hi), _w(s*lo))
def vec(v, scale=0):
    return ','.join(dp(c, scale) for c in v)
def fromdp(hi, lo, scale=0):
    def sw(w): return w if w < 0o40000 else -(0o77777 - w)
    return (sw(hi)*16384 + sw(lo)) / 2**28 * 2.0**scale
def tp(x, scale=0):
    n = int(round(x / 2.0**scale * 2**42))
    s = -1 if n < 0 else 1
    a = abs(n)
    return '%s,%s,%s' % (_w(s*(a >> 28)), _w(s*((a >> 14) & 0o37777)), _w(s*(a & 0o37777)))
