#!/bin/bash
set -e

echo "Creating build folder ..."
mkdir -p build
cd build

echo "CMake ..."
cmake ..

echo "Compiliin project ..."
make

echo "Compilation finished"
