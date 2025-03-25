# #!/bin/bash
# # Clean previous build
# rm -rf build

# # Create build directory and navigate into it
# mkdir -p build && cd build

# # Configure and build
# make VERBOSE=1
# cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. && make -j$(nproc)


# # Run the executable if build succeeds
# if [ $? -eq 0 ]; then
#     ./network_control_av
# else
#     echo "Build failed!"
# fi
#!/bin/bash
set -e  # Exit on error

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure only if necessary
if [ ! -f CMakeCache.txt ]; then
    cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
fi

# Build with verbose output
cmake --build . -- -j$(nproc) VERBOSE=1

# Run the executable if build succeeds
if [ $? -eq 0 ]; then
    echo "Build succeeded. Running the executable..."
    ./network_control_av
else
    echo "Build failed!"
fi
