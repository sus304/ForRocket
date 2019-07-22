#include <iostream>
#include <fstream>
#include <vector>

#include "lib/Eigen/Core"
#include <boost/numeric/odeint.hpp>


int main()
{
    Eigen::Matrix3f m1;
    m1 << 0.0f, 0.0f, 1.0f,
          0.0f, 0.0f, 1.0f,
          0.0f, 1.0f, 0.0f;
    
    Eigen::Vector3f v1, v3;
    v1 << 1.0, 2.0, 3.0;
    v3 << 3.0, 4.5, 6.0;

    Eigen::Matrix3f m2;
    Eigen::Vector3f v2;

    m2 = m1.array() + 2.0;  // 各要素への加算
    m2 = m1 * 2.0;  // 全体への乗算

    v2 = m1 * v1;  // 行列*ベクトルの内積

    v2 = v1 + v3;
    v2 = v1.array() + v3.array();

    std::cout << v2 << std::endl;

    return 0;    
};

