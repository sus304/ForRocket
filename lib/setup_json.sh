#!/bin/bash

JSON_VER_DOT="3.10.2"
JSON_VER_US="3_10_2"

rm -f json.hpp

wget --local-encoding=UTF-8 https://github.com/nlohmann/json/releases/download/v${JSON_VER_DOT}/json.hpp


