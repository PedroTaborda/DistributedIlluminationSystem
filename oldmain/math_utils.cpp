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
