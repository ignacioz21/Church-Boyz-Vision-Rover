// Sustituto mínimo de Arduino.h para compilar la lógica del rover en la PC (sim/).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

using std::max;
using std::min;
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

uint32_t millis();          // Reloj simulado (sim.cpp)
void delay(uint32_t ms);

class String {
    std::string s;
public:
    String() {}
    String(const char *c) : s(c) {}
    String(const std::string &c) : s(c) {}
    String(int v) : s(std::to_string(v)) {}
    int length() const { return (int)s.size(); }
    char charAt(int i) const { return (i >= 0 && i < (int)s.size()) ? s[i] : 0; }
    int indexOf(char c, int from = 0) const { auto p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
    int indexOf(const String &t, int from = 0) const { auto p = s.find(t.s, from); return p == std::string::npos ? -1 : (int)p; }
    String substring(int a, int b = -1) const { return String(b < 0 ? s.substr(a) : s.substr(a, b - a)); }
    long toInt() const { return atol(s.c_str()); }
    float toFloat() const { return (float)atof(s.c_str()); }
    const char *c_str() const { return s.c_str(); }
    friend String operator+(const String &a, const String &b) { return String(a.s + b.s); }
    friend String operator+(const char *a, const String &b) { return String(std::string(a) + b.s); }
};

struct SerialShim {
    void printf(const char *fmt, ...) { va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); }
    void println(const char *m) { puts(m); }
};
extern SerialShim Serial;
