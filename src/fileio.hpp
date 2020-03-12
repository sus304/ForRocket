// ******************************************************
// Project Name    : ForRocket
// File Name       : fileio.hpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef FILEIO_HPP_
#define FILEIO_HPP_

#include <string>
#include <vector>

namespace forrocket {
    std::vector<std::vector<double>> LoadCsvLog(std::string file_path, int skip_rows=1);
    std::vector<std::string> split(std::string& line, char delimiter);
}

#endif
