#include "interpolate.hpp"

bool ForRocket::Interp1d::needsSort(Eigen::VectorXd x)
{
    double slope;
    for (int i=1; i < x.size(); i++)
    {
        slope = x[i] - x[i-1];
        if (slope < 0)
        {
            return true;
        }
    }
    return false;
}

bool ForRocket::Interp1d::less(std::pair<double,double> a, std::pair<double,double> b)
{
    return (bool)((a.first < b.first) ? 1 : 0);
};

Eigen::MatrixXd ForRocket::Interp1d::XSort(Eigen::MatrixXd mat)
{
    std::vector<std::pair<double, double> > v1;
    for(int i=0; i < mat.col(0).size(); i++)
    {
        v1.push_back(std::pair<double, double>(mat(i, 0), mat(i, 1)));
    }
    stable_sort(v1.begin(), v1.end(), less);
    for (int i=0; i < v1.size(); i++)
    {
        mat(i, 0) = v1[i].first;
        mat(i, 1) = v1[i].second;
    }
    return mat;
}

ForRocket::Interp1d::Interp1d(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value)
{
    x_base = x;
    y_base = y;
    fille_value_config = fill_value;

    if (x_base.size() != y_base.size())
    {
        std::cerr << "Error! Different number of array : " << x_base.size() << '/' << y_base.size() << std::endl;
        exit(EXIT_FAILURE);
    }

    if (!(fill_value == "zero" || fill_value == "same" || fill_value == "extrapolate"))
    {
        std::cerr << "Error! undefined reference to fill value of extrapolate type : " << fill_value << std::endl;
        std::cerr << "zero / same / extrapolate" << fill_value << std::endl;
        exit(EXIT_FAILURE);
    }

    if (needsSort(x_base))
    {
        Eigen::MatrixXd mat(x_base.size(), 2);
        for(int i=0; i < x_base.size(); i++)
        {
            mat(i, 0) = x_base[i];
            mat(i, 1) = y_base[i];
        }
        mat = XSort(mat);
        for(int i=0; i < x_base.size(); i++)
        {
            x_base[i] = mat(i, 0);
            y_base[i] = mat(i, 1);
        }
    }
}

double ForRocket::Interp1d::operator()(double x)
{
    if (x < x_base[0])
    {
        if (fille_value_config == "zero")
        {
            return 0;
        }
        else if (fille_value_config == "same")
        {
            return y_base[0];
        }
        else
        {
            double slope = (y_base[1] - y_base[0]) / (x_base[1] - x_base[0]);
            return y_base[0] + slope * (x - x_base[0]);
        }
    }
    if (x > x_base[x_base.size()-1])
    {
        if (fille_value_config == "zero")
        {
            return 0;
        }
        else if (fille_value_config == "same")
        {
            return y_base[y_base.size()-1];
        }
        else
        {
            int i_end = x_base.size()-1;
            double slope = (y_base[i_end] - y_base[i_end-1]) / (x_base[i_end] - x_base[i_end-1]);
            return y_base[i_end] + slope * (x - x_base[i_end]);
        }
    }
    for (int i=0; i < x_base.size(); i++)
    {
        if (x >= x_base[i])
        {
            double slope = (y_base[i+1] - y_base[i]) / (x_base[i+1] - x_base[i]);
            return y_base[i] + slope * (x - x_base[i]);
        }
    }
}

