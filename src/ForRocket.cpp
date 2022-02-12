// ******************************************************
// Project Name    : ForRocket
// File Name       : ForRocket.cpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include "commandline_option.hpp"
#include "solver/trajectory_solver.hpp"

const std::string program_ver = "4.1.9";
const std::string release_date = "2022 Feb 13";

int main(int argc, char* argv[]) {
    // arg parse
    //////////////////////////////////////////////
    bool enable_msg = true;
    bool enable_minimum_damp = false;

    std::vector<int> json_index;
    for (int i=1; i < argc; ++i) {
        std::string str = argv[i];
        if (str.rfind(".json") != std::string::npos) {
            json_index.push_back(i);
            continue;
        }
        if (str.find("--") != std::string::npos) {
            if (str == "--help") {
                std::cout << "ForRocket v" << program_ver << " (release: " << release_date << ")"  << std::endl;
                forrocket::DisplayHelp();
                return 0;
            }
            if (str == "--version") {
                std::cout << "ForRocket version:" << program_ver << " (release: " << release_date << ")" << std::endl;
                return 0;
            }
            if (str == "--quite") {
                enable_msg = false;
            }
        }
        if (str.find("-") != std::string::npos) {
            if (str.find('h') != std::string::npos) {
                std::cout << "ForRocket v" << program_ver << " (release: " << release_date << ")"  << std::endl;
                forrocket::DisplayHelp();
                return 0;
            }
            if (str.find('v') != std::string::npos) {
                std::cout << "ForRocket version:" << program_ver << " (release: " << release_date << ")"  << std::endl;
                return 0;
            }
            if (str.find('q') != std::string::npos) {
                enable_msg = false;
            }
            if (str.find('m') != std::string::npos) {
                enable_minimum_damp = true;
            }
        }
    }
    
    if (json_index.size() == 0) {
        std::cerr << "Error! missing argument json file.";
        return 1;
    }
    if (json_index.size() != 1) {
        std::cerr << "Error! too many argument json file.";
        return 1;
    }

    if (enable_msg) std::cout << "ForRocket v" << program_ver << " Contact." << std::endl;
    
    // Timer Start
    auto start = std::chrono::system_clock::now();

    // Solver Run
    //////////////////////////////////////////////
    if (enable_msg) std::cout << "Solver Start." << std::endl;
    forrocket::TrajectorySolver solver(argv[json_index[0]]);
    solver.Solve();
    if (enable_msg) std::cout << "Solver Terminate." << std::endl;
    if (enable_msg) std::cout << "Export Result ..." << std::endl;
    solver.DumpResult(enable_minimum_damp);
    if (enable_msg) std::cout << "Export Compleate." << std::endl;

    // Timer Stop
    auto end = std::chrono::system_clock::now();
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (enable_msg) {
        if (msec > 10000) {
            std::cout << "Running Time: " << msec / 1000 << " sec" << std::endl;
        } else {
            std::cout << "Running Time: " << msec << " msec" << std::endl;
        }
    }

    if (enable_msg) std::cout << "Good Day." << std::endl;
    return 0;
}