#ifndef csv_operator_hpp
#define csv_operator_hpp

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "../lib/Eigen/Core"

namespace ForRocket
{

std::vector<double> Str2DoubleSplitter(std::string& str, char delimiter = ',');

Eigen::MatrixXd ReadCsv(std::string file_name, int skiprows = 0);

}
#endif