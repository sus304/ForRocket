// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "interpolate.hpp"

#include <iostream>
#include <cmath>


bool forrocket::interpolate::Interp1d::need_sort(const std::vector<double>& x) {
    double slope;
    for (int i=1; i < x.size(); ++i) {
        slope = x[i] - x[i-1];
        if (slope < 0) return true;
    }
    return false;
};


// bool forrocket::interpolate::Interp1d::less(const std::pair<double, double>& a, const std::pair<double, double>& b) {
//     return (bool)((a.first < b.first) ? 1 : 0);
// };


void forrocket::interpolate::Interp1d::ascending_order_sort(std::vector<double>& x, std::vector<double>& y) {
    std::vector<std::pair<double, double> > v1;
    for (int i=0; i < x.size(); ++i) {
        v1.push_back(std::make_pair(x[i], y[i]));
    }
    // stable_sort(v1.begin(), v1.end(), less);
    stable_sort(v1.begin(), v1.end());
    for (int i=0; i < v1.size(); ++i) {
        x[i] = v1[i].first;
        y[i] = v1[i].second;
    }
};


forrocket::interpolate::Interp1d::Interp1d(const std::vector<double> x, const std::vector<double> y, const std::string kind, const std::string fill_value) {
    if (x.size() != y.size()) {
        std::cerr << "Error! Diffirent number of array: " << x.size() << "/" << y.size() << std::endl;
        exit(EXIT_FAILURE);
    }
    x_src = x;
    y_src = y;

    if (fill_value == "zero") this->fill_value = 0;
    else if (fill_value == "same") this->fill_value = 1;
    else if (fill_value == "extrapolate") this->fill_value = 2;
    else {
        std::cerr << "Error! Undefined reference to fill value type: " << fill_value << std::endl;
        std::cerr << "zero / same / extrapolate" << std::endl;;
        exit(EXIT_FAILURE);
    }

    if (need_sort(x_src)) ascending_order_sort(x_src, y_src);

    if (kind == "linear") {
        polator = new Linear1D();
    }
    else if (kind == "cubic" || kind == "spline") {
        polator = new CubicSpline1D(x, y);
    } else {
        std::cerr << "Error! Undifined polate kind" << std::endl;
        exit(EXIT_FAILURE);
    }
};

forrocket::interpolate::Interp1d::Interp1d(const Interp1d& from) {
    x_src = from.x_src;
    y_src = from.y_src;
    fill_value = from.fill_value;

    if (typeid(*from.polator) == typeid(Linear1D)) {
        polator = new Linear1D();
    } else if (typeid(*from.polator) == typeid(CubicSpline1D)) {
        polator = new CubicSpline1D(from.x_src, from.y_src);
    }
};

forrocket::interpolate::Interp1d& forrocket::interpolate::Interp1d::operator=(const Interp1d& from) {
    if (this != &from) {
        x_src = from.x_src;
        y_src = from.y_src;
        fill_value = from.fill_value;

        if (typeid(*from.polator) == typeid(Linear1D)) {
            polator = new Linear1D();
        } else if (typeid(*from.polator) == typeid(CubicSpline1D)) {
            polator = new CubicSpline1D(from.x_src, from.y_src);
        }  
    }
    return *this;
};


double forrocket::interpolate::Interp1d::operator()(const double x) {
    return polator->polate(x, x_src, y_src, fill_value);
};


std::vector<double> forrocket::interpolate::Interp1d::operator()(const std::vector<double> x) {
    std::vector<double> res;
    for(int i=0; i < x.size(); ++i) {
        res.push_back(polator->polate(x[i], x_src, y_src, fill_value));
    }
    return res;
};


Eigen::VectorXd forrocket::interpolate::Interp1d::operator()(const Eigen::VectorXd x) {
    Eigen::VectorXd res(x.size());
    for(int i=0; i < x.size(); ++i) {
        res[i] = polator->polate(x[i], x_src, y_src, fill_value);
    }
    return res;
};




double forrocket::interpolate::Linear1D::polate(const double& x, const std::vector<double>& x_src, const std::vector<double>& y_src, const int& fill_value) {
    if (x >= x_src[0] && x <= x_src.back()) {  // 内挿
        // TODO: binary search
        for (int i=0; i < x_src.size(); ++i) {
            if (x == x_src[i]) {
                return y_src[i];
            }
            if (x < x_src[i]) {
                double slope = (y_src[i] - y_src[i-1]) / (x_src[i] - x_src[i-1]);
                return y_src[i-1] + slope * (x - x_src[i-1]);
            }
        }
    } else if (x < x_src[0]) {  // 左外挿
        switch (fill_value) {
        case 0:  // zero
            return 0;
            break;
        
        case 1:  // same
            return y_src[0];
            break;

        case 2:  // extrapolate
            double slope = (y_src[1] - y_src[0]) / (x_src[1] - x_src[0]);
            return y_src[0] + slope * (x - x_src[0]);
            break;
        }
    } else if (x > x_src[x_src.size()-1]) {  // 右外挿
        switch (fill_value) {
        case 0:  // zero
            return 0;
            break;
        
        case 1:  // same
            return y_src[y_src.size()-1];
            break;

        case 2:  // extrapolate
            int index_end = y_src.size()-1;
            double slope = (y_src[index_end] - y_src[index_end-1]) / (x_src[index_end] - x_src[index_end-1]);
            return y_src[index_end] + slope * (x - x_src[index_end]);
            break;
        }
    }
};


forrocket::interpolate::CubicSpline1D::CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) {
    SolveSplineCoefficient(x, y);
};


forrocket::interpolate::CubicSpline1D::~CubicSpline1D() {
    delete[] a_spline;
    delete[] b_spline;
    delete[] c_spline;
    delete[] d_spline;
};

forrocket::interpolate::CubicSpline1D::CubicSpline1D(const CubicSpline1D& from) { 
    array_size = from.array_size;
    a_spline = new double[from.array_size];
    for (int i=0; i < from.array_size; ++i) {
        a_spline[i] = from.a_spline[i];
    }
    b_spline = new double[from.array_size];
    for (int i=0; i < from.array_size; ++i) {
        b_spline[i] = from.b_spline[i];
    }
    c_spline = new double[from.array_size];
    for (int i=0; i < from.array_size; ++i) {
        c_spline[i] = from.c_spline[i];
    }
    d_spline = new double[from.array_size];
    for (int i=0; i < from.array_size; ++i) {
        d_spline[i] = from.d_spline[i];
    }
};

forrocket::interpolate::CubicSpline1D& forrocket::interpolate::CubicSpline1D::operator=(const CubicSpline1D& from) {
    if (this != &from) {
        array_size = from.array_size;
        a_spline = new double[from.array_size];
        for (int i=0; i < from.array_size; ++i) {
            a_spline[i] = from.a_spline[i];
        }
        b_spline = new double[from.array_size];
        for (int i=0; i < from.array_size; ++i) {
            b_spline[i] = from.b_spline[i];
        }
        c_spline = new double[from.array_size];
        for (int i=0; i < from.array_size; ++i) {
            c_spline[i] = from.c_spline[i];
        }
        d_spline = new double[from.array_size];
        for (int i=0; i < from.array_size; ++i) {
            d_spline[i] = from.d_spline[i];
        }
    }
    return *this;
};

void forrocket::interpolate::CubicSpline1D::SolveSplineCoefficient(const std::vector<double>& x, const std::vector<double>& y) {
    // Tri-Diagonal Matrix Algorithm
    array_size = x.size();
    double* h_spline = new double[array_size];
    double* P_tdma = new double[array_size];
    double* Q_tdma = new double[array_size];

    a_spline = new double[array_size];
    b_spline = new double[array_size];
    c_spline = new double[array_size];
    d_spline = new double[array_size];

    for (int i=1; i < x.size(); ++i) {
        h_spline[i] = x[i+1] - x[i];
        a_spline[i] = y[i];
    }
    a_spline[array_size-1] = y[array_size-1];
    c_spline[0] = 0.0;
    c_spline[array_size-1] = 0.0;

    P_tdma[0] = 0.0;
    Q_tdma[0] = 0.0;
    for (int i=1; i < array_size; ++i) { 
        P_tdma[i] = h_spline[i] / (2.0 * (h_spline[i] + h_spline[i-1]) + h_spline[i] * P_tdma[i-1]);
        double d_tdma = (3.0 / h_spline[i]) * (a_spline[i+1] - a_spline[i]) - (3.0 / h_spline[i-1]) * (a_spline[i] - a_spline[i-1]);
        Q_tdma[i] = (d_tdma + h_spline[i] * Q_tdma[i-1]) / (2.0 * (h_spline[i] + h_spline[i-1]) + h_spline[i] * Q_tdma[i-1]);
    }
    for (int i=array_size-2; i > 0; --i) {
        c_spline[i] = P_tdma[i] * c_spline[i+1] + Q_tdma[i];
    }
    for (int i=0; i < array_size; ++i) {
        b_spline[i] = (a_spline[i+1] - a_spline[i]) / h_spline[i] - h_spline[i] / 3.0 * (c_spline[i+1] + 2.0 * c_spline[i]);
        d_spline[i] = (c_spline[i+1] - c_spline[i] / (3.0 * h_spline[i]));
    }

    delete[] h_spline;
    delete[] P_tdma;
    delete[] Q_tdma;
};


double forrocket::interpolate::CubicSpline1D::polate(const double& x, const std::vector<double>& x_src, const std::vector<double>& y_src, const int& fill_value) {
    int index_target;
    if (x < x_src[0]) {  // 左外挿
        switch (fill_value) {
        case 0:  // zero
            return 0;
            break;
        
        case 1:  // same
            return y_src[0];
            break;

        case 2:  // extrapolate
            index_target = 0;
            break;
        }
    }
    else if (x >= x_src[0] && x <= x_src[x_src.size()-1]) {  // 内挿
        for (int i=0; i < x_src.size(); ++i) {
            if (x <= x_src[i]) {
                index_target = i - 1;
                break;
            }
        }
    }
    else if (x > x_src[x_src.size()-1]) {  // 右外挿
        switch (fill_value) {
        case 0:  // zero
            return 0;
            break;
        
        case 1:  // same
            return y_src[y_src.size()-1];
            break;

        case 2:  // extrapolate
            index_target = x_src.size() - 1;
            break;
        }
    }

    double h = x - x_src[index_target];
    return a_spline[index_target] + b_spline[index_target] * h + c_spline[index_target] * std::pow(h, 2) + d_spline[index_target] * std::pow(h, 3);
};
