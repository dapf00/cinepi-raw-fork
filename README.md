![cp_raw_banner](https://github.com/cinepi/cinepi-raw/assets/25234407/71591abc-f9b2-467e-806f-30557bcd1491)
# Introduction

This is a fork of [CinePi-RAW](https://github.com/cinepi/cinepi-raw) that works with [cinepi-gd]() and [cine-cam-godot]().

# Build-Instructions
## Dependencies
### Raspberry Pi Cam App Dependencies (see [here](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps))
```bash
sudo apt install -y libcamera-dev libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y meson ninja-build
sudo apt install -y libasound2-dev libudev-dev
```
## CinePi-RAW specific dependencies
```bash
sudo apt-get install -y redis-server libhiredis-dev libspdlog-dev libjsoncpp-dev
```

Install [Redis++](https://github.com/sewenew/redis-plus-plus)

```bash
git clone https://github.com/sewenew/redis-plus-plus.git
cd redis-plus-plus
mkdir build
cd build
cmake ..
make
sudo make install
```

Install [cpp-mjpeg-streamer](https://github.com/nadjieb/cpp-mjpeg-streamer)

```bash
git clone https://github.com/nadjieb/cpp-mjpeg-streamer.git
cd cpp-mjpeg-streamer
mkdir build
cd build
cmake ..
make
sudo make install
```

## Build
```bash
meson setup build -Denable_libav=false -Denable_drm=false -Denable_egl=false -Denable_qt=false -Denable_opencv=false -Denable_tflite=false 
meson compile -C build
```

# Usage

Make run-raw.sh executable
```bash
sudo chmod +x run-raw.sh
```

Use run-raw.sh to run the application.
```bash
./run-raw.sh
```

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).
