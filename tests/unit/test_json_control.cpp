// ******************************************************
// Unit tests for src/json_control.cpp
//   - JsonControl(std::string path): success + missing-file death path
//   - JsonControl(nlohmann::json): direct-wrap constructor
//   - getString / getInt / getDouble / getBool
//   - contains (true and false branches)
//   - getSubItem (returns a wrapped sub-object)
//
// Branch points covered (C1):
//   - json_ifs.fail() == true  -> std::cerr + std::exit(1)  (death test)
//   - json_ifs.fail() == false -> normal parse path
//   - both public constructors
// ******************************************************

#include <gtest/gtest.h>

#include <cstdio>     // std::remove
#include <cstdlib>    // EXIT_FAILURE
#include <fstream>
#include <string>

#include "json_control.hpp"
#include "json.hpp"

using forrocket::JsonControl;

namespace {

// Absolute path under /tmp: the test CWD is the build dir, so any path passed
// to JsonControl / written into a JSON must be absolute.
const char* kTmpJson = "/tmp/forrocket_test_json_control.json";

// Writes a small but representative JSON document covering every getter type.
void WriteValidJson(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"name\": \"forrocket\",\n"   // getString
        << "    \"count\": 7,\n"              // getInt
        << "    \"ratio\": 2.5,\n"            // getDouble
        << "    \"enabled\": true,\n"         // getBool
        << "    \"nested\": {\n"              // getSubItem
        << "        \"inner_value\": 42.0\n"
        << "    }\n"
        << "}\n";
    ofs.close();
}

}  // namespace

// --- missing-file branch: ifstream.fail() -> std::cerr + std::exit(1) -------
TEST(JsonControl, MissingFileExitsWithCode1) {
    // The factory exits with code 1 and prints "failed to open: <path>".
    // Wrap in std::string: a bare string literal is ambiguous between the
    // std::string and the nlohmann::json constructors.
    EXPECT_EXIT(JsonControl(std::string("/no/such/path/definitely_missing.json")),
                ::testing::ExitedWithCode(1),
                "failed to open");
}

// --- path constructor + all getters (success branch) ------------------------
TEST(JsonControl, PathConstructorAndGetters) {
    WriteValidJson(kTmpJson);
    // std::string disambiguates the path vs. json-object constructors.
    JsonControl jc{std::string(kTmpJson)};

    EXPECT_EQ(jc.getString("name"), "forrocket");
    EXPECT_EQ(jc.getInt("count"), 7);
    EXPECT_DOUBLE_EQ(jc.getDouble("ratio"), 2.5);
    EXPECT_TRUE(jc.getBool("enabled"));

    // contains: present key (true branch) and absent key (false branch).
    EXPECT_TRUE(jc.contains("name"));
    EXPECT_FALSE(jc.contains("missing_key"));

    // getSubItem returns a JsonControl wrapping the nested object.
    JsonControl sub = jc.getSubItem("nested");
    EXPECT_DOUBLE_EQ(sub.getDouble("inner_value"), 42.0);  // value set above

    std::remove(kTmpJson);
}

// --- nlohmann::json-object constructor --------------------------------------
TEST(JsonControl, JsonObjectConstructor) {
    nlohmann::json j;
    j["alpha"] = "beta";
    j["n"] = 3;
    j["x"] = 1.25;
    j["flag"] = false;

    JsonControl jc(j);  // exercises JsonControl(nlohmann::json)
    EXPECT_EQ(jc.getString("alpha"), "beta");
    EXPECT_EQ(jc.getInt("n"), 3);
    EXPECT_DOUBLE_EQ(jc.getDouble("x"), 1.25);
    EXPECT_FALSE(jc.getBool("flag"));
    EXPECT_TRUE(jc.contains("alpha"));
    EXPECT_FALSE(jc.contains("nope"));
}
