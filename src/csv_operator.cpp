#include "csv_operator.h"

vector<double> Str2DoubleSplitter(string& str, char delimiter)
{
    istringstream stream(str);
    string member;
    vector<double> splited_value;
    while (getline(stream, member, delimiter))
    {
        splited_value.push_back(stod(member));
    }
    return splited_value;
}

MatrixXd ReadCsv(string file_name, int skiprows)
{
    ifstream ifs(file_name.c_str());
    if (!ifs) {
        cerr << "Error! No such file or directory : " << file_name << endl;
        exit(EXIT_FAILURE);
    }

    MatrixXd read_result = MatrixXd::Zero(1,2);

    string line;
    int n_row = 1;
    while (getline(ifs, line))
    {
        if (n_row > skiprows)
        {
            vector<double> value_vector = Str2DoubleSplitter(line);
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

