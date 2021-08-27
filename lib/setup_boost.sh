#!/bin/bash

BOOST_VER_DOT="1.77.0"
BOOST_VER_US="1_77_0"

rm -rf boost/

mkdir setup
cd setup

wget -O boost_${BOOST_VER_US}.tar.gz https://sourceforge.net/projects/boost/files/boost/${BOOST_VER_DOT}/boost_${BOOST_VER_US}.tar.gz/download
tar xzvf boost_${BOOST_VER_US}.tar.gz
cd boost_${BOOST_VER_US}/
./bootstrap.sh
./b2.exe headers

cd ../../
cp -r -a setup/boost_${BOOST_VER_US}/boost/ boost/
rm -rf setup/