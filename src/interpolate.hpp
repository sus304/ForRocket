// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate.hpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef INTERPOLATE_HPP_
#define INTERPOLATE_HPP_

#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    namespace interpolate {
        class _Polator1D {
            public:
                virtual double polate(const double& x, const std::vector<double>& x_src, const std::vector<double>& y_src, const int& fill_value) = 0;

        };


        class _Linear1D : public _Polator1D {
            public:
                _Linear1D() {};
                _Linear1D(const std::vector<double>& x, const std::vector<double>& y);

                double polate(const double& x, const std::vector<double>& x_src, const std::vector<double>& y_src, const int& fill_value);
        };


        class _CubicSpline1D : public _Polator1D {
            public:
                _CubicSpline1D() {};
                _CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) {};
                ~_CubicSpline1D();

                double polate(const double& x, const std::vector<double>& x_src, const std::vector<double>& y_src, const int& fill_value);
            
            private:
                void solve_spline_coefficient(const std::vector<double>& x, const std::vector<double>& y);
                double* a_spline;
                double* b_spline;
                double* c_spline;
                double* d_spline;
        };


        class Interp1d {
            public:
                Interp1d() {};
                Interp1d(const std::vector<double> x, const std::vector<double> y, const std::string kind, const std::string fill_value);

                double operator()(const double x);
                std::vector<double> operator()(const std::vector<double> x);
                Eigen::VectorXd operator()(const Eigen::VectorXd x);
            
                std::vector<double> x_src;
                std::vector<double> y_src;
                int fill_value;

            private:
                _Polator1D* polator;

                bool need_sort(const std::vector<double>& x);
                bool less(const std::pair<double, double>& a, const std::pair<double, double>& b);
                void ascending_order_sort(std::vector<double>& x, std::vector<double>& y);

        };


        class Interp2d {
            public:
                Interp2d() {};
                Interp2d(const std::vector<double> x, const std::vector<double> y, const std::vector<double> z, const std::string fill_value);

                double operator()(const double x, const double y);
                //TODO: method coding

            private:
                std::vector<double> x_src;
                std::vector<double> y_src;
                Eigen::MatrixXd z_src;
                int fill_value;
        };
    }
}

#endif
