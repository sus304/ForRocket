#include "csv_operator.hpp"

std::vector<double> ForRocket::Str2DoubleSplitter(std::string& str, char delimiter)
{
    std::istringstream stream(str);
    std::string member;
    std::vector<double> splited_value;
    while (getline(stream, member, delimiter))
    {
        splited_value.push_back(stod(member));
    }
    return splited_value;
}

Eigen::MatrixXd ForRocket::ReadCsv(std::string file_name, int skiprows)
{
    std::ifstream ifs(file_name.c_str());
    if (!ifs) {
        std::cerr << "Error! No such file or directory : " << file_name << std::endl;
        exit(EXIT_FAILURE);
    }

    Eigen::MatrixXd read_result = Eigen::MatrixXd::Zero(1,2);

    std::string line;
    int n_row = 1;
    while (getline(ifs, line))
    {
        if (n_row > skiprows)
        {
            std::vector<double> value_vector = Str2DoubleSplitter(line);
            int max_col = value_vector.size();
            read_result.conservativeResize(n_row - skiprows, max_col);
            for (int i=0; i < value_vector.size(); i++)
            {
                read_result(n_row - skiprows - 1, i) = value_vector[i];
            }
        }
        n_row++;
    }
    return read_result;
}

