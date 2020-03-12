// ******************************************************
// Project Name    : ForRocket
// File Name       : ForRocket.cpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include <iostream>

#include "solver/trajectory_solver.hpp"

const std::string program_ver = "4.0.0 beta";

int main(int argc, char* argv[]) {
    std::cout << "ForRocket v" << program_ver << " Contact." << std::endl;

    // system json read ////////
    ////////////////////////////////////////////////////////////////////
    // argument file errer check
    if (argc != 2) {
        std::cout << "Error! Argument file." << std::endl;
        std::cout << "Usage: ForRocket.exe [solver config file path]" << std::endl;
        return 1;
    }

    std::cout << "Solver Start" << std::endl;
    forrocket::TrajectorySolver solver(argv[1]);
    solver.Solve();
    std::cout << "Solver Terminate" << std::endl;

    std::cout << "Good Day." << std::endl;
    return 0;
}