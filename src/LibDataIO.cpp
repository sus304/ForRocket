// ******************************************************
// Project Name    : ForRocket
// File Name       : LibDataIO.cpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "LibDataIO.hpp"

// lib_fileio/////
/////////////////////////////////////////
void lib_fileio::save_csv(std::string filename, const Eigen::MatrixXd& X) {
    const Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "Error! No such file or directory: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    ofs << X.format(csv_format);
}

// void lib_fileio::save_csv(std::string filename, const Eigen::MatrixXd& X, std::string header) {

// };

void lib_fileio::save_csv(std::string filename, const Eigen::MatrixXd& X, std::vector<std::string> header_list) {
    std::string header = header_list[0];
    for (const auto& e : header_list) {
        header = header + e;
    }
    header = header + "\n";

    const Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "Error! No such file or directory: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    ofs << header << X.format(csv_format);
}

std::vector<double> lib_fileio::str2double_split(std::string& str, char delimiter=',') {
    std::istringstream stream(str);
    std::string member;
    std::vector<double> splited_value;
    while (getline(stream, member, delimiter)) {
        splited_value.push_back(stod(member));
    }
    return splited_value;
}

Eigen::MatrixXd lib_fileio::read_csv(std::string filename, int skiprows=0) {
    std::ifstream ifs(filename.c_str());
    if (!ifs) {
        std::cerr << "Error! No such file or directory: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    Eigen::MatrixXd read_result = Eigen::MatrixXd::Zero(1,2);

    std::string line;
    int n_row = 1;
    while (getline(ifs, line)) {
        if (n_row > skiprows) {
            std::vector<double> value_vector = lib_fileio::str2double_split(line);
            int max_col = value_vector.size();
            read_result.conservativeResize(n_row - skiprows, max_col);
            for (int i=0; i < value_vector.size(); i++) {
                read_result(n_row - skiprows - 1, i) = value_vector[i];
            }
        }
        n_row++;
    }
    return read_result;
}

// lib_array/////
/////////////////////////////////////////
template <class T> std::vector<T> lib_array::arange(const T start, const T stop, const T step) {
    std::vector<T> array;
    T value = start;
    while (value <= stop) {
        array.push_back(value);
        value = value + step;
    }
    return array;
}

template <class T> std::vector<T> lib_array::linspace(const T start, const T stop, const int num) {
    std::vector<T> array;
    T step = (start - stop) / num;
    for (int i=0; i <= num; i++) {
        array.push_back(start + i * step);
    }
    return array;
}

// lib_interpolate/////
///////////////////////////////////////////
lib_interpolate::Interpolator::Interpolator(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value) {
    x_base = x;
    y_base = y;
    fill_value_config = fill_value;

    if (x_base.size() != y_base.size()) {
        std::cerr << "Error! Diffirent number of array: " << x_base.size() << "/" << y_base.size() << std::endl;
        exit(EXIT_FAILURE);
    }

    if (!(fill_value == "zero" || fill_value == "same" || fill_value == "extrapolate")) { 
        std::cerr << "Error! Undefined reference to fill value type: " << fill_value << std::endl;
        std::cerr << "zero / same / extrapolate" << std::endl;;
        exit(EXIT_FAILURE);
    }

    // x軸が昇順になっているか=sortが必要か
    if (need_sort(x_base)) {
        Eigen::MatrixXd mat(x_base.size(), 2);
        for (int i=0; i < x_base.size(); i++) {
            mat(i, 0) = x_base[i];
            mat(i, 1) = y_base[i];
        }
        mat = XSort(mat);
        for (int i=0; i < x_base.size(); i++) {
            x_base[i] = mat(i, 0);
            y_base[i] = mat(i, 1);
        }
    }
}

bool lib_interpolate::Interpolator::need_sort(Eigen::VectorXd x) {
    double slope;
    for (int i=1; i < x.size(); i++) {
        slope = x[i] - x[i-1];
        if (slope < 0) return true;
    }
    return false;
}

bool lib_interpolate::Interpolator::less(std::pair<double, double> a, std::pair<double, double> b) {
    return (bool)((a.first < b.first) ? 1 : 0);
}

Eigen::MatrixXd lib_interpolate::Interpolator::XSort(Eigen::MatrixXd mat) {
    std::vector<std::pair<double, double>> v1;
    for (int i=0; i < mat.col(0).size(); i++) {
        v1.push_back(std::pair<double, double>(mat(i, 0), mat(i, 1)));
    }
    stable_sort(v1.begin(), v1.end(), lib_interpolate::Interpolator::less);
    for (int i=0; i < v1.size(); i++) {
        mat(i, 0) = v1[i].first;
        mat(i, 1) = v1[i].second;
    }
    return mat;
}


lib_interpolate::CubicSpline1D::CubicSpline1D(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value) : lib_interpolate::Interpolator::Interpolator(x, y, fill_value) {
    lib_interpolate::CubicSpline1D::solve_spline_coefficient();
}

double lib_interpolate::CubicSpline1D::polate(double x) {
    if (x <= x_base[0]) {  // 左側外
        if (fill_value_config == "zero") return 0;
        else {
            if (fill_value_config == "same") return y_base[0];
            else return lib_interpolate::CubicSpline1D::function(0, x);
        }
    }
    if (x >= x_base[x_base.size() - 1]) {  // 右側外
        if (fill_value_config == "zero") return 0;
        else {
            if (fill_value_config == "same") return y_base[y_base.size() - 1];
            else return lib_interpolate::CubicSpline1D::function(x_base.size()-1, x);
        }
    }
    for (int i=0; i < x_base.size(); i++) { 
        if (x <= x_base[i]) return lib_interpolate::CubicSpline1D::function(i-1, x);
    }
}

double lib_interpolate::CubicSpline1D::function(int i, double x) {
    double h = x - x_base[i];
    return a_spline[i] + b_spline[i] * h + c_spline[i] * pow(h, 2) + d_spline[i] * pow(h, 3);
}

void lib_interpolate::CubicSpline1D::solve_spline_coefficient() {
    // Tri-Diagonal Matrix Algorithm
    int array_size = x_base.size();
    double *h_spline = new double[array_size];
    double *P_tdma = new double[array_size];
    double *Q_tdma = new double[array_size];

    a_spline = new double[array_size];
    b_spline = new double[array_size];
    c_spline = new double[array_size];
    d_spline = new double[array_size];
    for (int i=1; i < array_size; i++) {
        h_spline[i] = x_base[i+1] - x_base[i];
        a_spline[i] = y_base[i];
    }
    a_spline[array_size-1] = y_base[array_size-1];
    c_spline[0] = 0.0;
    c_spline[array_size-1] = 0.0;

    P_tdma[0] = 0.0;
    Q_tdma[0] = 0.0;
    for (int i=1; i < array_size; i++) { 
        P_tdma[i] = h_spline[i] / (2.0 * (h_spline[i] + h_spline[i-1]) + h_spline[i] * P_tdma[i-1]);
        double d_tdma = (3.0 / h_spline[i]) * (a_spline[i+1] - a_spline[i]) - (3.0 / h_spline[i-1]) * (a_spline[i] - a_spline[i-1]);
        Q_tdma[i] = (d_tdma + h_spline[i] * Q_tdma[i-1]) / (2.0 * (h_spline[i] + h_spline[i-1]) + h_spline[i] * Q_tdma[i-1]);
    }
    for (int i=array_size-2; i > 0; i--) {
        c_spline[i] = P_tdma[i] * c_spline[i+1] + Q_tdma[i];
    }
    for (int i=0; i < array_size; i++) {
        b_spline[i] = (a_spline[i+1] - a_spline[i]) / h_spline[i] - h_spline[i] / 3.0 * (c_spline[i+1] + 2.0 * c_spline[i]);
        d_spline[i] = (c_spline[i+1] - c_spline[i] / (3.0 * h_spline[i]));
    }
}

lib_interpolate::Linear1D::Linear1D(Eigen::VectorXd x, Eigen::VectorXd y, std::string fill_value) {}

double lib_interpolate::Linear1D::polate(double x) {
    if (x <= x_base[0]) {  // 左側外
        if (fill_value_config == "zero") return 0;
        else {
            if (fill_value_config == "same") return y_base[0];
            else return lib_interpolate::Linear1D::function(0, x);
        }
    }
    if (x >= x_base[x_base.size() - 1]) {  // 右側外
        if (fill_value_config == "zero") return 0;
        else {
            if (fill_value_config == "same") return y_base[y_base.size() - 1];
            else {
                int i_end = x_base.size() - 1;
                double slope = (y_base[i_end] - y_base[i_end-1]) / (x_base[i_end] - x_base[i_end-1]);
                return y_base[i_end] + slope * (x - x_base[i_end]);
            }
        }
    }
    for (int i=0; i < x_base.size(); i++) { 
        if (x <= x_base[i]) return lib_interpolate::Linear1D::function(i-1, x);
    }
}

double lib_interpolate::Linear1D::function(int i, double x) {
    double slope = (y_base[i+1] - y_base[i]) / (x_base[i+1] - x_base[i]);
    return y_base[i] + slope * (x - x_base[i]);
}


lib_interpolate::Interp1d::Interp1d(Eigen::VectorXd x, Eigen::VectorXd y, std::string kind, std::string fill_value) {
    if (!(kind == "linear" || kind == "spline" || kind == "cubic")) {
        std::cerr << "Error! Undefined reference to type of interpolate type: " << kind << std::endl;
        exit(EXIT_FAILURE);
    }
    type_polate = kind;
    if (kind == "linear") {
        lib_interpolate::Linear1D obj(x, y, fill_value);
        linear_polator = obj;
    }
    else if (kind == "spline" || kind == "cubic") {
        lib_interpolate::CubicSpline1D obj(x, y ,fill_value);
        spline_polator = obj;
    }
}

double lib_interpolate::Interp1d::operator()(double x) {
    return lib_interpolate::Interp1d::select_polator(x);
}

std::vector<double> lib_interpolate::Interp1d::operator()(std::vector<double> x) {
    std::vector<double> y;
    for (const auto& e : x) {
        y.push_back(lib_interpolate::Interp1d::select_polator(e));
    }
    return y;
}

Eigen::VectorXd lib_interpolate::Interp1d::operator()(Eigen::VectorXd x) {
    Eigen::VectorXd y = Eigen::VectorXd::Zero(x.size());
    for (int i=0; i < x.size(); i++) {
        y[i] = lib_interpolate::Interp1d::select_polator(x[i]);
    }
    return y;
}

double lib_interpolate::Interp1d::select_polator(double x) {
    if (type_polate == "linear") return linear_polator.polate(x);
    if (type_polate == "spline" || type_polate == "cubic") return spline_polator.polate(x);
}
