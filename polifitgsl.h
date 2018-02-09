/**
 * Provides function to fit a polynomial curve to a set of data.
 * Source: https://rosettacode.org/wiki/Polynomial_regression#C
 */

#ifndef _POLIFITGSL_H
#define _POLIFITGSL_H
#include <gsl/gsl_multifit.h>
#include <stdbool.h>
#include <math.h>
bool polynomialfit(int obs, int degree, 
        const double *dx, const double *dy, double *store); /* n, p */
#endif
 
