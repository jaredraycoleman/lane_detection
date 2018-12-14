#ifndef LANE_H
#define LANE_H

#include <vector>
#include <cmath>
#include <string>

class Lane
{
private:
    std::vector<double> params; //array of size degree. Defines coefficients for left lane curve.
    std::vector<double> lparams;
    std::vector<double> rparams;

    double filter; //filter for curve to remove jitter. lane = old_lane*filter + new_lane*(1-filter).
    double curvature; //positive curvature is right, negative is left
    double width;
    double camera_height;
    double vehicle_length;
    double vehicle_width;

public:
    Lane(std::string config_path, std::vector<double> lparams, std::vector<double> rparams);
    
    int getDegree();
    std::vector<double> getParams() const;
    std::vector<double> getLParams() const;
    std::vector<double> getRParams() const;

    double getFilter();
    double getCurvature();
    double getWidth();
    
    void update(std::vector<double> l, std::vector<double> r);
    
};

#endif
