# Comprehensive readme to follow.

# Build Instructions

git clone https://github.com/raspberrypi/pico-sdk.git
mkdir build
cmake -B ./build -DCMAKE_BUILD_TYPE=Release -DPICO_EXTRAS_PATH=./firmware/pico-extras/ -DPICO_SDK_PATH=./pico-sdk/ firmware/
cmake --build ./build --config Release