#!/bin/bash
# Clean previous build
rm -rf build

# Create build directory and navigate into it
mkdir -p build && cd build

# Configure and build
make VERBOSE=1
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. && make -j$(nproc)


# Run the executable if build succeeds
if [ $? -eq 0 ]; then
    ./network_control_av
else
    echo "Build failed!"
fi
