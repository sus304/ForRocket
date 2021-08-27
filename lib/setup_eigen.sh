#!/bin/bash

EIGEN_VER_DOT="3.4.0"
EIGEN_VER_US="3_4_0"

rm -rf Eigen/

mkdir setup
cd setup

wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VER_DOT}/eigen-${EIGEN_VER_DOT}.tar.gz
tar xzvf eigen-${EIGEN_VER_DOT}.tar.gz

cd ../
cp -r -a setup/eigen-${EIGEN_VER_DOT}/Eigen/ Eigen/
rm -rf setup/


