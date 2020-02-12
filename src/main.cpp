// ******************************************************
// Project Name    : ForRocket
// File Name       : main.cpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include <iostream>
#include <fstream>
#include <vector>

#include "solver/trajectory_solver.hpp"

const std::string program_ver = "1.0.0 alpha";

int main(int argc, char* argv[]) {
    std::cout << "Welcome to ForRocket" << std::endl;

    // system json read ////////
    ////////////////////////////////////////////////////////////////////
    //argument file errer check
    // if (argc != 2) {
    //     std::cout << "Errer! Argument file." << std::endl;
    //     std::cout << "Usage: ./ForRocket.exe [system config filename]" << std::endl;
    //     return 1;
    // }

    forrocket::TrajectorySolver solver("stage_config.json", "stage_config.json");
    solver.Solve();

    std::cout << "Bood-bye" << std::endl;
    return 0;
}