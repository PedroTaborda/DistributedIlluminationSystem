#include "math_utils.hpp"
#include <math.h>

StraightLine linearRegression(int n, double x[], double y[])
{
    double x_mean = 0.0;
    double y_mean = 0.0;
    double sum_xy = 0.0;
    double sum_x2 = 0.0;

    for (int i = 0; i < n; i++) {
        x_mean += x[i];
        y_mean += y[i];
    }

    x_mean /= (double)n;
    y_mean /= (double)n;

    for (int i = 0; i < n; i++) {
        sum_xy += (x[i] - x_mean) * (y[i] - y_mean);
        sum_x2 += (x[i] - x_mean) * (x[i] - x_mean);
    }

    double slope = sum_xy / sum_x2; 
    double intercept = y_mean - slope * x_mean;

    return StraightLine{slope, intercept};
}

double mean(int n, double x[])
{
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += x[i];
    }
    return sum / (double)n;
}

double interpolate(int n, double vecx[], double vecy[], double x)
{
    int i = 0;
    while (i < n && vecx[i] < x)
        i++;
    if (i == 0)
        return vecy[0];
    if (i == n)
        return vecy[n - 1];
    double dx = vecx[i] - vecx[i - 1];
    double dy = vecy[i] - vecy[i - 1];
    return vecy[i - 1] + (x - vecx[i - 1]) * dy / dx;
}

void range(int n, double *range, double start, double step)
{
    for (int i = 0; i < n; i++)
        range[i] = start + i * step;
}

double dot(int n, double *x, double *y)
{
    double sum = 0.0;
    for (int i = 0; i < n; i++)
        sum += x[i] * y[i];
    return sum;
}

unsigned int argmin(int n, double *x)
{
    unsigned int min_idx = 0;
    double min = x[0];
    for (int i = 1; i < n; i++)
    {
        if (x[i] < min)
        {
            min = x[i];
            min_idx = i;
        }
    }
    return min_idx;
}
