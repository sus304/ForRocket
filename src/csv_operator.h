#ifndef csv_operator_h
#define csv_operator_h

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "../lib/Eigen/Core"

using namespace std;
using namespace Eigen;

vector<double> Str2DoubleSplitter(string& str, char delimiter = ',');

MatrixXd ReadCsv(string file_name, int skiprows = 0);

#endif