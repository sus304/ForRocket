// ******************************************************
// Project Name    : ForRocket
// File Name       : json_control.cpp
// Creation Date   : 2020/02/07
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "json_control.hpp"

forrocket::JsonControl::JsonControl(std::string json_file_path) {
    using json = nlohmann::json;
    std::ifstream json_ifs(json_file_path, std::ios::in);
    if (json_ifs.fail()) {
        std::cerr << "failed to open: " << json_file_path << std::endl;
        std::exit(1);
    }
    json_ifs >> json_obj;
    json_ifs.close();
};

forrocket::JsonControl::JsonControl(nlohmann::json json_obj) {
    this->json_obj = json_obj;
};


std::string forrocket::JsonControl::getString(std::string key) {
    return json_obj[key].get<std::string>();
};

int forrocket::JsonControl::getInt(std::string key) {
    return json_obj[key].get<int>();
};

double forrocket::JsonControl::getDouble(std::string key) {
    return json_obj[key].get<double>();
};

bool forrocket::JsonControl::getBool(std::string key) {
    return json_obj[key].get<bool>();
};

forrocket::JsonControl forrocket::JsonControl::getSubItem(std::string key) {
    JsonControl jc(json_obj[key]);
    return jc;
};
