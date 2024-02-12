![cp_raw_banner](https://github.com/cinepi/cinepi-raw/assets/25234407/71591abc-f9b2-467e-806f-30557bcd1491)

*fork of rpicam-apps that builds upon the rpicam-raw app, offering cinema dng recording capabillities and integration with REDIS offering an abstract "API" like layer for custom integrations / controls.*

>[!WARNING]
>These applications and libraries have been renamed from `libcamera-*` to `rpicam-*`. Symbolic links are installed to allow users to keep using the old application names, but these will be deprecated soon. Users are encouraged to adopt the new application and library names as soon as possible.

Requirements
-----
Please install the below requirements before continuing with the rest of the build process:

[Redis](https://github.com/redis/redis)
`sudo apt-get install redis-server`

[Hiredis](https://github.com/redis/hiredis)
`sudo apt-get install libhiredis-dev`

[Redis++](https://github.com/sewenew/redis-plus-plus)
`requires manual build`

[spdlog](https://github.com/gabime/spdlog)
`sudo apt install libspdlog-dev`

[JsonCpp](https://github.com/open-source-parsers/jsoncpp)
`sudo apt-get install libjsoncpp-dev`

[cpp-mjpeg-streamer](https://github.com/nadjieb/cpp-mjpeg-streamer.git)
`requires manual build`

Build
-----
For usage and build instructions, see the official Raspberry Pi documentation pages [here.](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps)

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).
