// ******************************************************
// Project Name    : ForRocket
// File Name       : commandline_option.cpp
// Creation Date   : 2020/03/13
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "commandline_option.hpp"

#include <iostream>

void forrocket::DisplayHelp() {
    std::cout << "Usage: ForRocket [option] config.json" << std::endl;
    std::cout << "Option:" << std::endl;
    std::cout << "  -m                 Decrease column of trajectory log" << std::endl;
    std::cout << "  -q or --quite      Disable announce message." << std::endl;
    std::cout << "  -h or --help       Display this information and exit." << std::endl;
    std::cout << "  -v or --version    Display solver version and exit." << std::endl;
};