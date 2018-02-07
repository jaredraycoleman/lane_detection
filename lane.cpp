#include "lane.h"

Lane::Lane(int degree, double filter = 0.9)
	: degree(degree)
	, l_params(vector<double>(degree, (double)nan("1")))
	, r_params(vector<double>(degree, (double)nan("1")))
	, filter(filter)
{ 
	if (this->filter < 0.0 || this->filter > 1.0) this->filter = 0.9;
}

void Lane::update(vector<double> l, vector<double> r)
{
	if (params[0] != params[0])
	{
		for (int i = 0; i < n; i++)
		{
			this->params[i] = (l[i] + r[i]) / 2;
		}
	}
	else
	{
		for (int i = 0; i < n; i++)
		{
			double temp = (l[i] + r[i]) / 2;
			this->params[i] = filter * params[i] + (1 - filter) * temp;
		}
	}
}

int Lane::getN() { return this->n; }

vector<double> Lane::getParams() { return this->params; }

double Lane::getFilter() { return this->filter; }

double Lane::getCurvature() { return this->params[2]; }

double Lane::getWidth() { return this->width; }
