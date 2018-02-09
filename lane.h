#ifndef LANE_H
#define LANE_H

#include <vector>
#include <cmath>

class Lane
{
private:
    int n; //number of parameters that define the lane (degree + 1)
    std::vector<double> params; //array of size degree. Defines coefficients for left lane curve.
    double filter; //filter for curve to remove jitter. lane = old_lane*filter + new_lane*(1-filter).
    double curvature; //positive curvature is right, negative is left
    double width;

public:
    Lane(int degree, double filter);
    
    int getN();
    std::vector<double> getParams();
    double getFilter();
    double getCurvature();
    double getWidth();
    
    void update(std::vector<double> l, std::vector<double> r);
    
};

#endif
