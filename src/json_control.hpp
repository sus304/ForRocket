// ******************************************************
// Project Name    : ForRocket
// File Name       : json_control.hpp
// Creation Date   : 2020/02/07
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef JSONCONTROL_HPP_
#define JSONCONTROL_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "json.hpp"

namespace forrocket {
class JsonControl {
    public:
        JsonControl(std::string json_file_path);
        JsonControl(nlohmann::json json_obj);

        std::string getString(std::string key);
        int getInt(std::string key);
        double getDouble(std::string key);
        bool getBool(std::string key);
        JsonControl getSubItem(std::string key);
    
    private:
        nlohmann::json json_obj;
};
}  // namespace forrocket

#endif
