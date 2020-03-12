// ******************************************************
// Project Name    : ForRocket
// File Name       : fileio.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "fileio.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

std::vector<std::vector<double>> forrocket::LoadCsvLog(std::string file_path, int skip_rows) {
    std::ifstream ifs(file_path);
    std::vector<std::vector<double>> res_vectors;
    std::string line;
    
    std::getline(ifs, line);
    auto str = split(line, ',');
    int n_cols = str.size();
    for (int i=0; i < n_cols; ++i) {
        std::vector<double> temp1;
        res_vectors.push_back(temp1);
    }
    ifs.seekg(0, std::ios_base::beg);

    // skip rows
    for (int i=0; i < skip_rows; ++i) std::getline(ifs, line);

    while (std::getline(ifs, line)) {
        auto str = split(line, ',');
        for (int i=0; i < n_cols; ++i) {
            res_vectors[i].push_back(std::stod(str[i]));
        }
    }

    // if (n_cols == 2) {
    //     std::vector<double> temp1;
    //     std::vector<double> temp2;
    //     while (std::getline(ifs, line)) {
    //         auto str = split(line, ',');
    //         temp1.push_back(std::stod(str[0]));
    //         temp1.push_back(std::stod(str[1]));
    //     }
    //     res_vectors.push_back(temp1);
    //     res_vectors.push_back(temp2);
    // } else if (n_cols == 3) {
    //     std::vector<double> temp1;
    //     std::vector<double> temp2;
    //     std::vector<double> temp3;
    //     while (std::getline(ifs, line)) {
    //         auto str = split(line, ',');
    //         temp1.push_back(std::stod(str[0]));
    //         temp1.push_back(std::stod(str[1]));
    //         temp1.push_back(std::stod(str[2]));
    //     }
    //     res_vectors.push_back(temp1);
    //     res_vectors.push_back(temp2);
    //     res_vectors.push_back(temp3);
    // } else if (n_cols == 4) {
    //     std::vector<double> temp1;
    //     std::vector<double> temp2;
    //     std::vector<double> temp3;
    //     std::vector<double> temp4;
    //     while (std::getline(ifs, line)) {
    //         auto str = split(line, ',');
    //         temp1.push_back(std::stod(str[0]));
    //         temp1.push_back(std::stod(str[1]));
    //         temp1.push_back(std::stod(str[2]));
    //         temp1.push_back(std::stod(str[3]));
    //     }
    //     res_vectors.push_back(temp1);
    //     res_vectors.push_back(temp2);
    //     res_vectors.push_back(temp3);
    //     res_vectors.push_back(temp4);
    // }
    return res_vectors;
};

std::vector<std::string> forrocket::split(std::string& line, char delimiter) {
    std::istringstream stream(line);
    std::string field;
    std::vector<std::string> res;
    while (getline(stream, field, delimiter)) {
        res.push_back(field);
    }
    return res;
};

