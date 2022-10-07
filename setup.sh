git submodule update --init --recursive

# Build vision_geometry 
cd share/vision_geometry
mkdir -p build && cd build
cmake .. && make -j12
cd ../..

# Build apriltags
cd apriltag
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_WRAPPER=OFF && make -j12
