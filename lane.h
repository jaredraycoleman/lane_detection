#ifndef LANE_H
#define LANE_H

class Lane
{
private:
	int n; //number of parameters that define the lane (degree + 1)
	vector<double> params; //array of size degree. Defines coefficients for left lane curve.
	double filter; //filter for curve to remove jitter. lane = old_lane*filter + new_lane*(1-filter).
	double curvature; //positive curvature is right, negative is left
	double width;

public:
	/**
	 * Only provided constructor for Lane.
	 * @param degree Degree of polynomial that defines lanes.
	 * @param filter Filter for curve to remove jitter.
	 */ 
	Lane(int degree, double filter);
	
	
	//Getters
	int getN() { return this->n; }
	vector<double> getParams();
	double getFilter();
	double getCurvature();
	double getWidth();
	
	/**
	 * Updates left and right lane polynomial coefficients.
	 * @param l_new Array of size degree. Defines coefficients for left lane curve.
	 * @param r_new Array of size degree. Defines coefficients for right lane curve.
	 */
	void update(vector<double> l, vector<double> r);
	
}

#endif
