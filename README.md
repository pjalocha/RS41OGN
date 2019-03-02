# RS41OGN
Reprogram the RS-41 radiosonde for OGN position transmission/

## To compile on Linux:

### install git and the embedded ARM compiler:

```
sudo apt-get install git cmake gcc-arm-none-eabi
```

### unpack and compile:

```
git clone https://github.com/pjalocha/RS41OGN.git
mkdir build
cd build
cmake ..
make
```

### to upload with the ST-Linkv2:

```
sudo st-flash --reset --format ihex write RS41OGN.hex
```

### to install the st-flash and other utilities for ST-Link:

```
sudo apt-get install build-essential libusb-1.0-0-dev
git clone https://github.com/texane/stlink.git
cd stlink.git
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

## Issues:

1. The sonde does not turn on when on batteries.

It does turn on on external power, then pressing the power button turns on the internal power
and the sonde cab then be disconnected from external power.

This has possibly to do with PA12 which control the internal power.
This pin is set LOW just after the code startup.

1. The sonde can only transmit in the 433MHz band, not on 868MHz.

This has to do with the Xtal which is 26MHz instead of the standard 30MHz for Si4032,
and so all frequencies programmed into the Si4032 are effectively scaled by 26/30
and so the top limit falls below 868MHz.

To use it on OGN requires deploying the network on 433MHz. It is not clear whether this is worth doing.

To make the sonde transmitting on 868MHz would require to change the crystal and the RF coupling circuit,
thus it is not a trivial thing.

1. The Si4032 RF chip used in the sonde can not receive, thus the system is transmit only.
Seeing other traffic and relay function are not possible.

