nRF51-DK + GMC306 example code
==============================

Requirements
-----------
- nRF51-DK
- nRF51 SDK: tested with Version 10.0.0
- GCC ARM embedded compiler: tested with gcc-arm-none-eabi-4_9-2015q1-20150306
- GMC306

I2C Connections
---------------
- Use I2C0
  - SCL: P0.07
  - SDA: P0.30
- I2C 7-bit slave address: 0x0C

Makefile
-------
- Modify the location of the SDK to your actual directory
```
#
# The base directory for nRF51 SDK
#
nRF51_SDK_ROOT := /C/Data/nRF51_SDK/V10_0_0
```

- Modify the location of the GNU tools to your actual directory
```
GNU_INSTALL_ROOT := C:/Program Files (x86)/GNU Tools ARM Embedded/4.9 2015q1
```

- Execute `make all` to build and flash the program

- Exectute `make help` for available targets
