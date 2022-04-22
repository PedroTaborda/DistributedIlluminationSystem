#pragma once
#include <memory>

#define clip(x, min, max) (x < min ? min : (x > max ? max : x))

struct StraightLine{
    double slope;
    double intercept;
};

StraightLine linearRegression(int n, double x[], double y[]);

double mean(int n, double x[]);

double interpolate(int n, double vecx[], double vecy[], double x);

void range(int n, double *range, double start, double step);
