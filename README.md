# RS41OGN
Reprogram the RS-41 radiosonde for OGN position transmission/

To compile on Linux:

install git and the embedded ARM compiler:

...
sudo apt-get install git cmake gcc-arm-none-eabi
...

unpack and compile:

...
git clone https://github.com/pjalocha/RS41OGN.git
mkdir build
cd build
cmake ..
make
...

to upload with the ST-Linkv2:

...
sudo st-flash --reset --format ihex write RS41OGN.hex
...

to install the st-flash and other utilities for ST-Link:

...
sudo apt-get install build-essential libusb-1.0-0-dev
git clone https://github.com/texane/stlink.git
cd stlink.git
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
...

