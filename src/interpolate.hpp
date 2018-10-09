#ifndef interpolate_hpp
#define interpolate_hpp

#include <iostream>
#include <vector>
#include <algorithm>

#include "../lib/Eigen/Core"

namespace ForRocket
{
    
class Interp1d
{
    private:
    Eigen::VectorXd x_base;
    Eigen::VectorXd y_base;
    std::string fille_value_config;

    bool needsSort(Eigen::VectorXd x);
    static bool less(std::pair<double,double> a, std::pair<double,double> b);
    Eigen::MatrixXd XSort(Eigen::MatrixXd mat);

    public:
    Interp1d(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value);
    double operator()(double x);
};

}
#endif