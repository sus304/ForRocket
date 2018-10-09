#include "../src/interpolate.hpp"


int main()
{
    namespace FR = ForRocket;

    Eigen::Vector3d x = Eigen::Vector3d(1,3,5);
    Eigen::Vector3d y = Eigen::Vector3d(2,6,10);
    FR::Interp1d inter(x, y, "same");
    std::cout << inter(6) << std::endl;

    return 0;
}


