#include "../src/csv_operator.hpp"


int main()
{
    namespace FR = ForRocket;

    std::string Cd_file_name = "../Cd.csv";
    Eigen::MatrixXd Cd_log = FR::ReadCsv(Cd_file_name, 1);

    std::cout << Cd_log(0, 0) << std::endl;

    return 0;
}


