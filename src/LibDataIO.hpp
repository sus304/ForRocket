// ******************************************************
// Project Name    : ForRocket
// File Name       : LibDataIO.hpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef LibDataIO_hpp_
#define LibDataIO_hpp_

#include <iostream>
#include <fstream>
#include <vector>

#include "../lib/Eigen/Core"

namespace lib_fileio {
    void save_csv(std::string filename, const Eigen::MatrixXd& X);
    // void save_csv(std::string filename, const Eigen::MatrixXd& X, std::string header);
    void save_csv(std::string filename, const Eigen::MatrixXd& X, std::vector<std::string> header_list);

    std::vector<double> str2double_split(std::string& str, char delimiter=',');
    Eigen::MatrixXd read_csv(std::string filename, int skiprows=0);
}

namespace lib_array {
    template <class T> std::vector<T> arange(const T start, const T stop, const T step=1);
    template <class T> std::vector<T> linspace(const T start, const T stop, const int num);
}

namespace lib_interpolate {
    // abstruct class
    class Interpolator {
        public:
            Interpolator() {};
            Interpolator(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value);
            virtual double polate(double x) = 0;
        
        protected:
            Eigen::VectorXd x_base;
            Eigen::VectorXd y_base;
            std::string type_polate;
            std::string fill_value_config;

            bool need_sort(Eigen::VectorXd x);
            static bool less(std::pair<double, double> a, std::pair<double, double> b);
            Eigen::MatrixXd XSort(Eigen::MatrixXd mat);

            virtual double function(int i, double x) = 0;
    };

    class CubicSpline1D : public Interpolator {
        public:
            CubicSpline1D() {};
            CubicSpline1D(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value);
            double polate(double x);
        
        protected:
            double function(int i, double x);

            double *a_spline;
            double *b_spline;
            double *c_spline;
            double *d_spline;

            void solve_spline_coefficient();
    };

    class Linear1D : public Interpolator {
        public:
            Linear1D() {};
            Linear1D(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value);
            double polate(double x);

        protected:
            double function(int i, double x);
    };

    class Interp1d {
        public:
            Interp1d() {};
            Interp1d(Eigen::VectorXd x, Eigen::VectorXd y, std::string kind, std::string fill_value);
            double operator()(double x);
            std::vector<double> operator()(std::vector<double> x);
            Eigen::VectorXd operator()(Eigen::VectorXd x);
        
        protected:
            Linear1D linear_polator;
            CubicSpline1D spline_polator;
            std::string type_polate;
            double select_polator(double x);
    };

}


#endif