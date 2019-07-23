// ******************************************************
// Project Name    : ForRocket
// File Name       : main.cpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include <iostream>
#include <fstream>
#include <vector>

#include "../lib/picojson.h"

#include "Rocket.hpp"
#include "DynamicsSolver.hpp"

const std::string program_ver = "1.0.0 alpha";

int main(int argc, char* argv[]) {
    std::cout << "Welcome to ForRocket" << std::endl;

    // system json read ////////
    ////////////////////////////////////////////////////////////////////
    //argument file errer check
    if (argc != 2) {
        std::cout << "Errer! Argument file." << std::endl;
        std::cout << "Usage: ./ForRocket.exe [system config filename]" << std::endl;
        return 1;
    }

    // json read
    std::ifstream ifs_json(argv[1], std::ios::in);
    if (ifs_json.fail()) {
        std::cerr << "Failed to read json " << argv[1] << std::endl;
        return 1;
    }
    const std::string buf_json((std::istreambuf_iterator<char>(ifs_json)), std::istreambuf_iterator<char>());
    ifs_json.close();

    // json parse
    picojson::value value_json;
    const std::string err = picojson::parse(value_json, buf_json);
    if (err.empty() == false) {
        std::cerr << err << std::endl;
        return 1;
    }
    picojson::object& obj_json = value_json.get<picojson::object>();

    std::string project_name = obj_json["Project Name"].get<std::string>();
    bool flag_multi_mode = obj_json["Running Mode"].get<picojson::object>()["Multi"].get<bool>();
    bool flag_fast_mode = obj_json["Running Mode"].get<picojson::object>()["Fast"].get<bool>();

    std::vector<std::string> stage_json_file;
    picojson::array& ary = obj_json["Stage Config json"].get<picojson::array>();
    for (const auto& e : ary) {
        stage_json_file.push_back(e.get<std::string>());
    }

    // make instance ////////
    ////////////////////////////////////////////////////////////////////
    // ForRocket::Rocket rocket(stage_json_file);
    ForRocket::Rocket rocket;
    ForRocket::solve_trajectory(rocket);

    return 0;
}