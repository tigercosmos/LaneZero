#!/usr/bin/env bash
# Build script for LaneZero with pybind11

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Building LaneZero...${NC}"

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo -e "${YELLOW}Configuring with CMake...${NC}"
cmake ..

# Build
echo -e "${YELLOW}Compiling...${NC}"
make -j$(sysctl -n hw.ncpu)

# Copy the Python module to the LaneZero package
echo -e "${YELLOW}Copying Python module...${NC}"
cp _core*.so ../LaneZero/

echo -e "${GREEN}Build completed successfully!${NC}"
echo -e "${YELLOW}LaneZero viewer executable:${NC}"
echo -e "  build/bin/LaneZeroView"
echo -e "${YELLOW}To install in development mode, run:${NC}"
echo -e "  pip install -e ."
echo -e "${YELLOW}To run tests, run:${NC}"
echo -e "  pytest test/"

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
