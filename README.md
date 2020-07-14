# BME680 sensor data collection and display using Bosch API — ATSAML21 Xplained Pro MCU Embedded C
Environment Monitoring System Using BME680 Sensor API

Programmed on Atmel Studio 8 for the ATSAML21J18B Atmel 32-bit microcontroller

There are two different software packages, provided by Bosch, for use with the BME680. The first is the BME680 Sensor API. It provides basic sensor communication and data compensation functions. This software is provided as open source C code. This is the software package that you will be using in this laboratory. This software is provided on Blackboard in a zip file named BME680_driver-master.zip. The second software package is the Bosch Software Environmental Cluster (BSEC). This software features intelligent algorithms that enable use cases such as indoor-air-quality monitoring using the BME680. This software is available as closed source binary for several different compilers, including the GCC compiler on which Studio 7 is based. Use of this software is not a part of this laboratory. The BME680 Sensor API consists of three C code files: bme680.c, bme680.h, and bme680_def.h. These files are in the BME680_driver-master.zip. file. Since we are creating multifile-modular programs it is easy to integrate these files into this project.

