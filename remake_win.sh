rm -rf build
cmake -S . -B ./build/ -G "MinGW Makefiles"
cd build
make
./circle
