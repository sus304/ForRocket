#include "../src/csv_operator.h"


int main()
{
    string Cd_file_name = "../Cd.csv";
    MatrixXd Cd_log = read_csv(Cd_file_name, 1);

    cout << Cd_log(0, 0) << endl;

    return 0;
}


