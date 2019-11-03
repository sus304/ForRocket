#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

int main()
{
    Eigen::Matrix3d m1;
    m1 << 0.0, 0.0, 1.0,
          0.0, 0.0, 1.0,
          0.0, 1.0, 0.0;
    
    Eigen::Vector3d v1, v3;
    v1 << 1.0, 2.0, 3.0;
    v3 << 3.0, 4.5, 6.0;

    Eigen::Vector3d v2;
    Eigen::Matrix3d m2;

    // 各要素への加算
    v2 = v1.array() + 2.0;
    m2 = m1.array() + 2.0;

    // 全体(各要素)への乗算
    v2 = v1 * 2.0;
    v2 = v1.array() * 2.0;
    m2 = m1 * 2.0;

    // ベクトル+ベクトル
    v2 = v1 + v3;
    std::cout << "v3 + v1" << std::endl;
    std::cout << v2 << std::endl;

    // ベクトルの要素毎の乗算
    v2 = v1.array() * v3.array();
    std::cout << "v3 * v1" << std::endl;
    std::cout << v2 << std::endl;

    // ベクトル*ベクトルの内積
    double v21 = v3.dot(v1);
    std::cout << "v3.dot(v1" << std::endl;
    std::cout << v21 << std::endl;

    // ベクトルxベクトルの外積
    v2 = v3.cross(v1);
    std::cout << "v3.cross(v1" << std::endl;
    std::cout << v2 << std::endl;

    // 行列*ベクトルの内積(座標変換行列)
    v2 = m1 * v1;
    std::cout << "m1 * v1" << std::endl;
    std::cout << v2 << std::endl;

    

    return 0;    
};

