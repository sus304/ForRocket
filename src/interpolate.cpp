#include "interpolate.h"


bool Interp1d::needsSort(VectorXd x)
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

bool Interp1d::less(pair<double,double> a, pair<double,double> b)
{
    return (bool)((a.first < b.first) ? 1 : 0);
};

MatrixXd Interp1d::XSort(MatrixXd mat)
{
    vector<pair<double, double> > v1;
    for(int i=0; i < mat.col(0).size(); i++)
    {
        v1.push_back(pair<double, double>(mat(i, 0), mat(i, 1)));
    }
    stable_sort(v1.begin(), v1.end(), less);
    for (int i=0; i < v1.size(); i++)
    {
        mat(i, 0) = v1[i].first;
        mat(i, 1) = v1[i].second;
    }
    return mat;
}

Interp1d::Interp1d(VectorXd x, VectorXd y, string fill_value)
{
    x_base = x;
    y_base = y;

    if (x_base.size() != y_base.size())
    {
        cerr << "Error! Different number of array : " << x_base.size() << '/' << y_base.size() << endl;
        exit(EXIT_FAILURE);
    }

    if (needsSort(x_base))
    {
        MatrixXd mat(x_base.size(), 2);
        for(int i=0; i < x_base.col(0).size(); i++)
        {
            mat(i, 0) = x_base[i];
            mat(i, 1) = y_base[i];
        }
    }

    if (fill_value == "same")
    {
        cout << "same" << endl;
    }
    else if (fill_value == "zero")
    {
        cout << "zero" << endl;
    }

}

double Interp1d::operator()(double x)
{
    return x;
}

