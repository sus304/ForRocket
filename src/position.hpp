// ******************************************************
// Project Name    : ForRocket
// File Name       : position.hpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef POSITION_HPP_
#define POSITION_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Position {
        public:
            Eigen::Vector3d ECI;
            Eigen::Vector3d ECEF;
            Eigen::Vector3d NED;
            Eigen::Vector3d LLH;
            
        private:
            

    };

}


#endif