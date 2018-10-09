#ifndef interpolate_h
#define interpolate_h

#include <iostream>
#include <vector>
#include <algorithm>

#include "../lib/Eigen/Core"

using namespace std;
using namespace Eigen;


class Interp1d
{
    private:
    VectorXd x_base;
    VectorXd y_base;

    bool needsSort(VectorXd x);
    static bool less(pair<double,double> a, pair<double,double> b);
    MatrixXd XSort(MatrixXd mat);

    public:
    Interp1d(VectorXd x, VectorXd y, string fill_value);
    double operator()(double x);
};

#endif